// XIAO ESP32S3 + MCP2515 — ISOBUS implement (Virtual Terminal client)
//
// Based on the AgIsoStack++ example "esp32_platformio_object_pool", with the
// built-in TWAI driver swapped for the library's MCP2515 driver.
//
// What it does:
//   1. Claims an ISOBUS address (you'll see 0x18EEFFxx frames on the bus)
//   2. Looks for a Virtual Terminal (VT) on the bus
//   3. When a VT is found: uploads the example object pool and handles its buttons
//
// Wiring is unchanged from the xiao-can-test project.

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/mcp2515_can_interface.hpp"
#include "isobus/hardware_integration/spi_interface_esp.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"

#include "console_logger.cpp"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "objectPoolObjects.h"

#include <memory>

static const char *TAG = "implement";

// ---------------- Pins (XIAO label -> ESP32-S3 GPIO) ----------------
// D0=1 D1=2 D2=3 D3=4 D4=5 D5=6 D6=43 D7=44 D8=7 D9=8 D10=9
#define PIN_SCK   7     // D8
#define PIN_MISO  8     // D9  (through the 1k/2k divider)
#define PIN_MOSI  9     // D10
#define PIN_CS    1    // D7  <-- your CS pin as a plain GPIO number

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

// ---------------- Button / soft key handling (same as the library example) ----------------
static void handle_softkey_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	if (event.keyNumber == 0)
	{
		vtHelper->set_active_data_or_alarm_mask(example_WorkingSet, mainRunscreen_DataMask);
	}
	if (event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased)
	{
		switch (event.objectID)
		{
			case alarm_SoftKey:
				vtHelper->set_active_data_or_alarm_mask(example_WorkingSet, example_AlarmMask);
				break;
			case acknowledgeAlarm_SoftKey:
				vtHelper->set_active_data_or_alarm_mask(example_WorkingSet, mainRunscreen_DataMask);
				break;
			default:
				break;
		}
	}
}

static void handle_button_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
	if ((event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased) ||
	    (event.keyEvent == isobus::VirtualTerminalClient::KeyActivationCode::ButtonStillHeld))
	{
		switch (event.objectID)
		{
			case Plus_Button:
				vtHelper->increase_numeric_value(ButtonExampleNumber_VarNum);
				break;
			case Minus_Button:
				vtHelper->decrease_numeric_value(ButtonExampleNumber_VarNum);
				break;
			default:
				break;
		}
	}
}

extern "C" const std::uint8_t object_pool_start[] asm("_binary_object_pool_iop_start");
extern "C" const std::uint8_t object_pool_end[] asm("_binary_object_pool_iop_end");

// These must outlive app_main's setup code, so they're static
static spi_device_interface_config_t spiDeviceConfig = {};
static std::unique_ptr<isobus::SPIInterfaceESP> spiInterface;

extern "C" void app_main()
{
	vTaskDelay(pdMS_TO_TICKS(2000)); // give the USB serial monitor time to connect
	ESP_LOGI(TAG, "=== XIAO ISOBUS implement starting ===");

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

	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Info); // Debug for more detail
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid())
	{
		ESP_LOGE(TAG, "CAN start FAILED - the MCP2515 did not answer over SPI. Check wiring/CS pin.");
	}
	else
	{
		ESP_LOGI(TAG, "MCP2515 running at 250 kbit/s");
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
	vtClient->set_object_pool(0, object_pool_start, (object_pool_end - object_pool_start), "ais1");
	vtClient->get_vt_soft_key_event_dispatcher().add_listener(handle_softkey_event);
	vtClient->get_vt_button_event_dispatcher().add_listener(handle_button_event);
	vtClient->initialize(true);

	vtHelper = std::make_shared<isobus::VirtualTerminalClientUpdateHelper>(vtClient);
	vtHelper->add_tracked_numeric_value(ButtonExampleNumber_VarNum, 214748364); // pool offset, = 0 on screen
	vtHelper->initialize();

	// ---- Status every 3 s ----
	while (true)
	{
		ESP_LOGI(TAG, "address %s 0x%02X | VT found: %s | VT connected: %s",
		         implementCF->get_address_valid() ? "claimed" : "not yet",
		         implementCF->get_address(),
		         partnerVT->get_address_valid() ? "yes" : "no",
		         vtClient->get_is_connected() ? "yes" : "no");
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}
