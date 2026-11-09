// XIAO ESP32S3 — ISOBUS implement (Virtual Terminal client)
//
// Based on the AgIsoStack++ example "esp32_platformio_object_pool".
//
// CAN hardware (choose with CAN_USE_MCP2515 below):
//   0 = SN65HVD230 transceiver on the ESP32's built-in CAN controller (TWAI)  <- default
//   1 = MCP2515 module over SPI (the old setup with the resistor divider)
//
// What it does:
//   1. Claims an ISOBUS address (you'll see 0x18EEFFxx frames on the bus)
//   2. Looks for a Virtual Terminal (VT) on the bus
//   3. When a VT is found: uploads the Norén screen (src/object_pool, made by tools/make_pool.py)
//   4. Shows the tractor's speed, rear hitch and rear PTO, read from the tractor's ISOBUS
//      messages. With no tractor on the bus (the bench) it simulates them.
//   5. Three buttons switch the implement's lights: outputs D0, D1, D3 (3.3 V, max ~10 mA each:
//      an LED with a 330 ohm resistor, or the input of a relay/MOSFET module). The XIAO's own
//      orange LED follows the work light, so you can see it without wiring anything.
//
// SN65HVD230 wiring:
//   XIAO 3V3 -> 3V3      XIAO GND -> GND
//   XIAO D4  -> CTX (TX)  XIAO D2 -> CRX (RX)
//   CANH / CANL -> the bus (no termination resistor here if both bus ends are already terminated)

#define CAN_USE_MCP2515 0

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#if CAN_USE_MCP2515
#include "isobus/hardware_integration/mcp2515_can_interface.hpp"
#include "isobus/hardware_integration/spi_interface_esp.hpp"
#else
#include "isobus/hardware_integration/twai_plugin.hpp"
#endif
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"

#include "console_logger.cpp"
#include "driver/gpio.h"
#if CAN_USE_MCP2515
#include "driver/spi_master.h"
#endif
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "noren_pool.hpp"

#include <atomic>
#include <cmath>

#include <memory>
#include <thread>

static const char *TAG = "implement";

// ---------------- Pins (XIAO label -> ESP32-S3 GPIO) ----------------
// D0=1 D1=2 D2=3 D3=4 D4=5 D5=6 D6=43 D7=44 D8=7 D9=8 D10=9
// SN65HVD230. Not D6/D7: those are UART0, which prints boot messages and would put junk on the bus.
#define PIN_CAN_TX GPIO_NUM_5 // D4
#define PIN_CAN_RX GPIO_NUM_3 // D2 (D5 on this XIAO appears damaged)

// Implement light outputs (high = on) and the XIAO's own LED (low = on)
#define PIN_LIGHT_WORK   GPIO_NUM_1  // D0
#define PIN_LIGHT_BEACON GPIO_NUM_2  // D1
#define PIN_LIGHT_REAR   GPIO_NUM_4  // D3
#define PIN_ONBOARD_LED  GPIO_NUM_21

// MCP2515 (only used when CAN_USE_MCP2515 is 1; its CS would clash with the work light on D0)
#define PIN_SCK   7     // D8
#define PIN_MISO  8     // D9  (through the 1k/2k divider)
#define PIN_MOSI  9     // D10
#define PIN_CS    1     // D0

// ---------------- MCP2515 bit timing: 8 MHz crystal, 250 kbit/s ----------------
// 16 time quanta per bit, sample point 14/16 = 87.5 % (what ISO 11783-2 asks for)
//   CNF1: SJW=1TQ, BRP=0          -> TQ = 250 ns
//   CNF2: BTLMODE, PS1=7TQ, PROP=6TQ
//   CNF3: PS2=2TQ
static constexpr std::uint8_t MCP_CNF1 = 0x00;
static constexpr std::uint8_t MCP_CNF2 = 0xB5;
static constexpr std::uint8_t MCP_CNF3 = 0x01;

static std::shared_ptr<isobus::VirtualTerminalClient> vtClient = nullptr;
static std::shared_ptr<isobus::VirtualTerminalClientUpdateHelper> vtHelper = nullptr;

// ---------------- Tractor data from ISOBUS (ISO 11783-7 tractor ECU messages) ----------------
// Filled in by the CAN stack's thread, read by the main loop.
static std::atomic<std::uint32_t> groundSpeedMmS{ 0xFFFFFFFF }; // mm/s, 0xFFFFFFFF = not available
static std::atomic<std::uint32_t> wheelSpeedMmS{ 0xFFFFFFFF };
static std::atomic<std::uint32_t> hitchTenths{ 0xFFFFFFFF };    // rear hitch position, 0.1 %
static std::atomic<std::uint32_t> ptoRpm{ 0xFFFFFFFF };         // rear PTO shaft speed
static std::atomic<std::uint32_t> lastTractorMessageMs{ 0 };
static std::atomic<bool> tractorSeen{ false };

static std::uint32_t now_ms()
{
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void tractor_heard()
{
	lastTractorMessageMs = now_ms();
	tractorSeen = true;
}

// Ground-based (radar) and wheel-based speed: bytes 0-1 = speed, 0.001 m/s per bit
static void on_ground_speed(const isobus::CANMessage &message, void *)
{
	std::uint16_t raw = message.get_uint16_at(0);
	groundSpeedMmS = (raw < 0xFB00) ? raw : 0xFFFFFFFF;
	tractor_heard();
}

static void on_wheel_speed(const isobus::CANMessage &message, void *)
{
	std::uint16_t raw = message.get_uint16_at(0);
	wheelSpeedMmS = (raw < 0xFB00) ? raw : 0xFFFFFFFF;
	tractor_heard();
}

// Primary (rear) hitch status: byte 0 = position, 0.4 % per bit
static void on_rear_hitch(const isobus::CANMessage &message, void *)
{
	std::uint8_t raw = message.get_uint8_at(0);
	hitchTenths = (raw <= 250) ? raw * 4u : 0xFFFFFFFF;
	tractor_heard();
}

// Primary (rear) PTO output shaft: bytes 0-1 = speed, 0.125 rpm per bit
static void on_rear_pto(const isobus::CANMessage &message, void *)
{
	std::uint16_t raw = message.get_uint16_at(0);
	ptoRpm = (raw < 0xFB00) ? (raw / 8u) : 0xFFFFFFFF;
	tractor_heard();
}

constexpr std::uint32_t PGN_WHEEL_SPEED = 0xFE48;  // 65096
constexpr std::uint32_t PGN_GROUND_SPEED = 0xFE49; // 65097
constexpr std::uint32_t PGN_REAR_HITCH = 0xFE45;   // 65093
constexpr std::uint32_t PGN_REAR_PTO = 0xFE43;     // 65091

// ---------------- Implement lights ----------------
struct Light
{
	const char *name;
	gpio_num_t pin;
	std::uint16_t button;
	std::uint16_t stateText;
	bool on;
};

static Light lights[] = {
	{ "Work", PIN_LIGHT_WORK, pool::BTN_WORK, pool::STR_WORK_STATE, false },
	{ "Beacon", PIN_LIGHT_BEACON, pool::BTN_BEACON, pool::STR_BEACON_STATE, false },
	{ "Rear", PIN_LIGHT_REAR, pool::BTN_REAR, pool::STR_REAR_STATE, false },
};

// Show a light's state on the terminal: button colour + ON/OFF text
static void show_light(const Light &light)
{
	if ((nullptr != vtClient) && vtClient->get_is_connected())
	{
		vtClient->send_change_background_colour(light.button, light.on ? pool::COLOUR_LIGHT_ON : pool::COLOUR_LIGHT_OFF);
		vtClient->send_change_background_colour(light.stateText, light.on ? pool::COLOUR_LIGHT_ON : pool::COLOUR_LIGHT_OFF);
		vtClient->send_change_string_value(light.stateText, light.on ? "ON " : "OFF");
	}
}

static void set_light(Light &light, bool on)
{
	light.on = on;
	gpio_set_level(light.pin, on ? 1 : 0);
	if (light.pin == PIN_LIGHT_WORK)
	{
		gpio_set_level(PIN_ONBOARD_LED, on ? 0 : 1); // the XIAO's LED is on when low
	}
	ESP_LOGI(TAG, "%s light %s", light.name, on ? "ON" : "OFF");
	show_light(light);
}

static void setup_light_outputs()
{
	gpio_config_t config = {};
	config.pin_bit_mask = (1ULL << PIN_LIGHT_WORK) | (1ULL << PIN_LIGHT_BEACON) | (1ULL << PIN_LIGHT_REAR) | (1ULL << PIN_ONBOARD_LED);
	config.mode = GPIO_MODE_OUTPUT;
	gpio_config(&config);
	for (auto &light : lights)
	{
		gpio_set_level(light.pin, 0);
	}
	gpio_set_level(PIN_ONBOARD_LED, 1);
}

// ---------------- Buttons on the terminal ----------------
// Events arrive in the VT client's thread; they're queued as "toggle requests" for the main loop.
static std::atomic<std::uint32_t> toggleRequests{ 0 }; // bit per light
static std::atomic<bool> allOffRequest{ false };

static void handle_button_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	if (event.keyEvent != isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
	{
		return; // switch on release, like a real switch panel
	}
	for (std::size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
	{
		if (event.objectID == lights[i].button)
		{
			toggleRequests |= (1u << i);
		}
	}
}

static void handle_softkey_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	if ((event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased) &&
	    (event.objectID == pool::KEY_ALL_OFF))
	{
		allOffRequest = true;
	}
}

extern "C" const std::uint8_t object_pool_start[] asm("_binary_object_pool_iop_start");
extern "C" const std::uint8_t object_pool_end[] asm("_binary_object_pool_iop_end");

// These must outlive app_main's setup code, so they're static
#if CAN_USE_MCP2515
static spi_device_interface_config_t spiDeviceConfig = {};
static std::unique_ptr<isobus::SPIInterfaceESP> spiInterface;
#else
static twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
static twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS(); // ISOBUS: 250 kbit/s
static twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#endif

// Everything runs in a std::thread (a pthread): this AgIsoStack++ version calls
// std::this_thread::get_id() in VirtualTerminalClient::initialize(), and on ESP-IDF that
// only works inside a pthread. Called from the plain FreeRTOS main task it aborts with
// "assert failed: pthread_self ... Failed to find current thread ID!".
static void implement_main()
{
	vTaskDelay(pdMS_TO_TICKS(2000)); // give the USB serial monitor time to connect
	ESP_LOGI(TAG, "=== XIAO ISOBUS implement (Norén demo) starting ===");
	setup_light_outputs();

#if CAN_USE_MCP2515
	// ---- SPI bus for the MCP2515 ----
	spi_bus_config_t busConfig = {};
	busConfig.mosi_io_num = PIN_MOSI;
	busConfig.miso_io_num = PIN_MISO;
	busConfig.sclk_io_num = PIN_SCK;
	busConfig.quadwp_io_num = -1;
	busConfig.quadhd_io_num = -1;
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_DISABLED));

	spiDeviceConfig.mode = 0;                    // MCP2515 uses SPI mode 0
	spiDeviceConfig.clock_speed_hz = 1000000;    // 1 MHz, forgiving with the resistor divider
	spiDeviceConfig.spics_io_num = PIN_CS;
	spiDeviceConfig.queue_size = 1;

	spiInterface = std::make_unique<isobus::SPIInterfaceESP>(&spiDeviceConfig, SPI2_HOST);
	if (!spiInterface->init())
	{
		ESP_LOGE(TAG, "SPI device init FAILED");
	}

	// ---- CAN stack with the MCP2515 driver ----
	auto canDriver = std::make_shared<isobus::MCP2515CANInterface>(spiInterface.get(), MCP_CNF1, MCP_CNF2, MCP_CNF3);
	const char *canName = "MCP2515";
#else
	// ---- Built-in CAN controller + SN65HVD230 ----
	twaiConfig.rx_queue_len = 32;
	twaiConfig.tx_queue_len = 32; // object pool upload sends long bursts
	auto canDriver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);
	const char *canName = "Built-in CAN (SN65HVD230)";
#endif

	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Info); // Debug for more detail
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid())
	{
		ESP_LOGE(TAG, "CAN start FAILED (%s). Check the wiring.", canName);
	}
	else
	{
		ESP_LOGI(TAG, "%s running at 250 kbit/s", canName);
	}

	// ---- Our identity on the bus (ISO NAME) ----
	isobus::NAME implementName(0);
	implementName.set_arbitrary_address_capable(true);
	implementName.set_industry_group(2);   // 2 = agricultural and forestry
	implementName.set_device_class(0);     // 0 = non-specific system
	implementName.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::MachineControl));
	implementName.set_identity_number(42); // must be unique per device you build
	implementName.set_ecu_instance(0);
	implementName.set_function_instance(0);
	implementName.set_device_class_instance(0);
	implementName.set_manufacturer_code(1407); // test value from the library example — fine on the bench

	auto implementCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(implementName, 0, 0x80);

	// The VT we're looking for: anything on the bus whose NAME says "Virtual Terminal"
	const isobus::NAMEFilter vtFilter(isobus::NAME::NAMEParameters::FunctionCode,
	                                  static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	auto partnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, { vtFilter });

	// ---- VT client with the example object pool ----
	vtClient = std::make_shared<isobus::VirtualTerminalClient>(partnerVT, implementCF);
	// The label tells the terminal which version of the screen this is: change it whenever the
	// pool changes, or the terminal shows its stored copy of the old one.
	vtClient->set_object_pool(0, object_pool_start, (object_pool_end - object_pool_start), "noren1");
	// The screen is drawn for a 480 x 480 data mask and 60 px soft keys. Real terminals differ
	// (the tractor's own terminal is bigger): the library rescales the pool to fit before uploading.
	vtClient->set_object_pool_scaling(0, 480, 60);
	vtClient->get_vt_soft_key_event_dispatcher().add_listener(handle_softkey_event);
	vtClient->get_vt_button_event_dispatcher().add_listener(handle_button_event);
	vtClient->initialize(true);

	vtHelper = std::make_shared<isobus::VirtualTerminalClientUpdateHelper>(vtClient);
	vtHelper->add_tracked_numeric_value(pool::VAR_SPEED, 0);
	vtHelper->add_tracked_numeric_value(pool::VAR_HITCH, 0);
	vtHelper->add_tracked_numeric_value(pool::VAR_PTO, 0);
	vtHelper->initialize();

	// Tractor ECU broadcasts
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(PGN_GROUND_SPEED, on_ground_speed, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(PGN_WHEEL_SPEED, on_wheel_speed, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(PGN_REAR_HITCH, on_rear_hitch, nullptr);
	isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(PGN_REAR_PTO, on_rear_pto, nullptr);

	// ---- Main loop: 5 updates per second ----
	bool wasConnected = false;
	bool showingTractor = false;
	std::uint32_t lastStatusLog = 0;
	while (true)
	{
		const std::uint32_t now = now_ms();
		const bool connected = vtClient->get_is_connected();

		// Buttons pressed on the terminal
		std::uint32_t toggles = toggleRequests.exchange(0);
		for (std::size_t i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
		{
			if (toggles & (1u << i))
			{
				set_light(lights[i], !lights[i].on);
			}
		}
		if (allOffRequest.exchange(false))
		{
			for (auto &light : lights)
			{
				if (light.on)
				{
					set_light(light, false);
				}
			}
		}

		// Tractor data, or a simulation when no tractor is on the bus
		const bool tractor = tractorSeen.load() && ((now - lastTractorMessageMs.load()) < 2000);
		std::uint32_t speedTenthsKmh = 0;
		std::uint32_t hitchPercent = 0;
		std::uint32_t pto = 0;
		if (tractor)
		{
			std::uint32_t mmPerS = (groundSpeedMmS.load() != 0xFFFFFFFF) ? groundSpeedMmS.load() : wheelSpeedMmS.load();
			speedTenthsKmh = (mmPerS != 0xFFFFFFFF) ? ((mmPerS * 36u + 500u) / 1000u) : 0; // mm/s -> 0.1 km/h
			hitchPercent = (hitchTenths.load() != 0xFFFFFFFF) ? ((hitchTenths.load() + 5u) / 10u) : 0;
			pto = (ptoRpm.load() != 0xFFFFFFFF) ? ptoRpm.load() : 0;
		}
		else
		{
			const float t = now / 1000.0f;
			speedTenthsKmh = static_cast<std::uint32_t>(80.0f + 60.0f * std::sin(t / 6.0f));      // 2-14 km/h
			const float phase = std::fmod(t, 20.0f);                                             // hitch up and down
			hitchPercent = static_cast<std::uint32_t>(phase < 10.0f ? phase * 10.0f : (20.0f - phase) * 10.0f);
			pto = (std::fmod(t, 30.0f) < 10.0f) ? 0 : ((std::fmod(t, 60.0f) < 30.0f) ? 540 : 1000);
		}

		if (connected)
		{
			if (!wasConnected || (tractor != showingTractor))
			{
				// Freshly connected (the terminal's copy has the pool's defaults) or the source changed
				vtClient->send_change_string_value(pool::STR_SOURCE, tractor ? "TRACTOR  " : "SIMULATED");
				showingTractor = tractor;
				ESP_LOGI(TAG, "Data source: %s", tractor ? "TRACTOR (ISOBUS)" : "SIMULATED (no tractor on the bus)");
			}
			if (!wasConnected)
			{
				// The terminal starts from the pool's defaults: send everything once
				for (const auto &light : lights)
				{
					show_light(light);
				}
				vtClient->send_change_numeric_value(pool::VAR_SPEED, speedTenthsKmh);
				vtClient->send_change_numeric_value(pool::VAR_HITCH, hitchPercent);
				vtClient->send_change_numeric_value(pool::VAR_PTO, pto);
			}
			vtHelper->set_numeric_value(pool::VAR_SPEED, speedTenthsKmh);
			vtHelper->set_numeric_value(pool::VAR_HITCH, hitchPercent);
			vtHelper->set_numeric_value(pool::VAR_PTO, pto);
		}
		wasConnected = connected;

		if (now - lastStatusLog >= 5000)
		{
			lastStatusLog = now;
			ESP_LOGI(TAG, "address %s 0x%02X | VT %s | %s: %lu.%lu km/h, hitch %lu %%, PTO %lu rpm",
			         implementCF->get_address_valid() ? "claimed" : "not yet", implementCF->get_address(),
			         connected ? "connected" : (partnerVT->get_address_valid() ? "found" : "not found"),
			         tractor ? "tractor" : "simulated", static_cast<unsigned long>(speedTenthsKmh / 10),
			         static_cast<unsigned long>(speedTenthsKmh % 10), static_cast<unsigned long>(hitchPercent),
			         static_cast<unsigned long>(pto));
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

extern "C" void app_main()
{
	std::thread implementThread(implement_main);
	implementThread.join(); // never returns: implement_main loops forever
}
