#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include <string>
#include <vector>

namespace esphome {
namespace wattcycle_ble {

// BLE GATT UUIDs
static const char *SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb";
static const char *WRITE_UUID   = "0000fff2-0000-1000-8000-00805f9b34fb";
static const char *NOTIFY_UUID  = "0000fff1-0000-1000-8000-00805f9b34fb";
static const char *AUTH_UUID    = "0000fffa-0000-1000-8000-00805f9b34fb";


// Protocol constants
static const uint8_t  FRAME_HEAD_TX = 0x1E;
static const uint8_t  FRAME_HEAD_RX = 0x7E;
static const uint8_t  FRAME_TAIL = 0x0D;
static const uint8_t  FUNC_READ  = 0x03;
static const uint16_t DP_ANALOG  = 0x008C;
static const uint16_t DP_PRODUCT = 0x0092;

class WattCycleBLE : public Component, public ble_client::BLEClientNode {
public:
	void setup() override;
	void loop() override;
	void dump_config() override;
	float get_setup_priority() const override {
		return setup_priority::DATA;
	}

	void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
	                         esp_ble_gattc_cb_param_t *param) override;

	void set_parent(ble_client::BLEClient *parent) {
		this->parent_ = parent;
		parent->register_ble_node(this);
	}

	void set_update_interval(uint32_t interval) {
		this->update_interval_ = interval;
	}
	void set_poll_offset(uint32_t offset) {
		this->poll_offset_ = offset;
	}

	void set_battery_voltage_sensor(sensor::Sensor *sensor) {
		this->battery_voltage_sensor_ = sensor;
	}
	void set_battery_current_sensor(sensor::Sensor *sensor) {
		this->battery_current_sensor_ = sensor;
	}
	void set_battery_power_sensor(sensor::Sensor *sensor)   {
		this->battery_power_sensor_ = sensor;
	}
	void set_mos_temperature_sensor(sensor::Sensor *sensor) {
		this->mos_temperature_sensor_ = sensor;
	}
	void set_pcb_temperature_sensor(sensor::Sensor *sensor) {
		this->pcb_temperature_sensor_ = sensor;
	}
	void set_soc_sensor(sensor::Sensor *sensor) {
		this->soc_sensor_ = sensor;
	}
	void set_remaining_capacity_sensor(sensor::Sensor *sensor) {
		this->remaining_capacity_sensor_ = sensor;
	}
	void set_total_capacity_sensor(sensor::Sensor *sensor) {
		this->total_capacity_sensor_ = sensor;
	}
	void set_design_capacity_sensor(sensor::Sensor *sensor) {
		this->design_capacity_sensor_ = sensor;
	}
	void set_cycles_sensor(sensor::Sensor *sensor) {
		this->cycles_sensor_ = sensor;
	}
	void set_cell_spread_sensor(sensor::Sensor *sensor) {
		this->cell_spread_sensor_ = sensor;
	}
	void add_cell_voltage_sensor(sensor::Sensor *sensor) {
		this->cell_voltage_sensors_.push_back(sensor);
	}
	void add_cell_temperature_sensor(sensor::Sensor *sensor) {
		this->cell_temperature_sensors_.push_back(sensor);
	}
	void set_model_or_fw_text_sensor(text_sensor::TextSensor *sensor) {
		this->model_or_fw_text_sensor_ = sensor;
	}
	void set_manufacturer_text_sensor(text_sensor::TextSensor *sensor) {
		this->manufacturer_text_sensor_ = sensor;
	}
	void set_serial_text_sensor(text_sensor::TextSensor *sensor) {
		this->serial_text_sensor_ = sensor;
	}

protected:
	// Non-blocking delays used after GATT operations.
	bool auth_pending_{false};
	uint32_t auth_pending_since_{0};
	bool post_auth_pending_{false};
	uint32_t post_auth_pending_since_{0};


	// Sensors
	sensor::Sensor *battery_voltage_sensor_{nullptr};
	sensor::Sensor *battery_current_sensor_{nullptr};
	sensor::Sensor *battery_power_sensor_{nullptr};
	sensor::Sensor *mos_temperature_sensor_{nullptr};
	sensor::Sensor *pcb_temperature_sensor_{nullptr};
	sensor::Sensor *soc_sensor_{nullptr};
	sensor::Sensor *remaining_capacity_sensor_{nullptr};
	sensor::Sensor *total_capacity_sensor_{nullptr};
	sensor::Sensor *design_capacity_sensor_{nullptr};
	sensor::Sensor *cycles_sensor_{nullptr};
	sensor::Sensor *cell_spread_sensor_{nullptr};
	std::vector<sensor::Sensor *> cell_voltage_sensors_;
	std::vector<sensor::Sensor *> cell_temperature_sensors_;
	text_sensor::TextSensor *model_or_fw_text_sensor_{nullptr};
	text_sensor::TextSensor *manufacturer_text_sensor_{nullptr};
	text_sensor::TextSensor *serial_text_sensor_{nullptr};

	// Timing
	uint32_t update_interval_{10000};
	uint32_t poll_offset_{0};
	uint32_t last_poll_{0};

	// Connection state
	bool authenticated_{false};
	bool auth_sent_{false};
	bool product_read_{false};

	// Handle GATT
	uint16_t notify_handle_{0};
	uint16_t write_handle_{0};

	// Receive buffer
	std::vector<uint8_t> rx_buffer_;
	size_t expected_length_{0};

	// Internal helpers
	void authenticate_();
	void send_read_command_(uint16_t dp_address);
	void handle_rx_bytes_(const uint8_t *data, size_t len);
	void parse_frame_(const std::vector<uint8_t> &frame);
	void parse_analog_data_(const uint8_t *data, size_t len);
	void parse_product_data_(const uint8_t *data, size_t len);

	// Utility
	uint16_t calculate_crc16_(const uint8_t *data, size_t len);
	float parse_temperature_(uint16_t raw);
	float parse_current_negative_(const uint8_t *buffer);
	std::string hex_encode_(const uint8_t *data, size_t len);
	std::string trim_ascii_(const uint8_t *data, size_t len);
};

}  // namespace wattcycle_ble
}  // namespace esphome
