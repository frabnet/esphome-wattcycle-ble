#include "wattcycle_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace espbt = esphome::esp32_ble_tracker;

namespace esphome {
namespace wattcycle_ble {

static const char *const TAG = "wattcycle_ble";
static const size_t MAX_FRAME_LENGTH = 256;

// Modbus CRC16 lookup tables
static const uint8_t CRC_HI_TABLE[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t CRC_LO_TABLE[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
	0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
	0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
	0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
	0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
	0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
	0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
	0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
	0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

void WattCycleBLE::setup() {
	ESP_LOGCONFIG(TAG, "Setting up WattCycle BLE...");
	this->rx_buffer_.reserve(256);
}

void WattCycleBLE::dump_config() {
	ESP_LOGCONFIG(TAG, "WattCycle BLE:");
	ESP_LOGCONFIG(TAG, "  Update interval: %us", this->update_interval_ / 1000);
	ESP_LOGCONFIG(TAG, "  Poll offset: %ums", this->poll_offset_);
	LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
	LOG_SENSOR("  ", "Battery Current", this->battery_current_sensor_);
	LOG_SENSOR("  ", "Battery Power", this->battery_power_sensor_);
	LOG_SENSOR("  ", "MOS Temperature", this->mos_temperature_sensor_);
	LOG_SENSOR("  ", "PCB Temperature", this->pcb_temperature_sensor_);
	LOG_SENSOR("  ", "SOC", this->soc_sensor_);
	LOG_SENSOR("  ", "Remaining Capacity", this->remaining_capacity_sensor_);
	LOG_SENSOR("  ", "Total Capacity", this->total_capacity_sensor_);
	LOG_SENSOR("  ", "Design Capacity", this->design_capacity_sensor_);
	LOG_SENSOR("  ", "Cycles", this->cycles_sensor_);
	LOG_SENSOR("  ", "Cell Spread", this->cell_spread_sensor_);
	ESP_LOGCONFIG(TAG, "  Cell voltage sensors: %zu", this->cell_voltage_sensors_.size());
	ESP_LOGCONFIG(TAG, "  Cell temperature sensors: %zu", this->cell_temperature_sensors_.size());
	LOG_TEXT_SENSOR("  ", "Model/Firmware", this->model_or_fw_text_sensor_);
	LOG_TEXT_SENSOR("  ", "Manufacturer", this->manufacturer_text_sensor_);
	LOG_TEXT_SENSOR("  ", "Serial", this->serial_text_sensor_);
}

void WattCycleBLE::loop() {
	// Wait briefly after CCCD setup before writing the auth key.
	if (this->auth_pending_) {
		if (millis() - this->auth_pending_since_ >= 200) {
			this->auth_pending_ = false;
			this->authenticate_();
		}
		return;
	}

	// Match the reference flow: after "HiLink", give the BMS time before the first read.
	if (this->post_auth_pending_) {
		if (millis() - this->post_auth_pending_since_ >= 300 + this->poll_offset_) {
			this->post_auth_pending_ = false;
			this->last_poll_ = millis();
			this->send_read_command_(DP_PRODUCT);
		}
		return;
	}

	// Wait until the BLE client is authenticated and connected.
	if (!this->authenticated_ || this->node_state != espbt::ClientState::ESTABLISHED) {
		return;
	}

	uint32_t now = millis();
	if (now - this->last_poll_ < this->update_interval_) {
		return;
	}

	this->last_poll_ = now;
	this->send_read_command_(DP_ANALOG);
}

void WattCycleBLE::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
	switch (event) {
	case ESP_GATTC_OPEN_EVT:
		if (param->open.status == ESP_GATT_OK) {
			ESP_LOGI(TAG, "Connected to WattCycle BMS");
			// Reset state for a new connection.
			this->authenticated_      = false;
			this->auth_sent_          = false;
			this->product_read_       = false;
			this->auth_pending_       = false;
			this->post_auth_pending_  = false;
			this->rx_buffer_.clear();
			this->expected_length_    = 0;
		}
		break;

	case ESP_GATTC_DISCONNECT_EVT:
		ESP_LOGW(TAG, "Disconnected from WattCycle BMS");
		this->authenticated_      = false;
		this->auth_sent_          = false;
		this->product_read_       = false;
		this->auth_pending_       = false;
		this->post_auth_pending_  = false;
		this->notify_handle_      = 0;
		this->write_handle_       = 0;
		this->rx_buffer_.clear();
		this->expected_length_    = 0;
		break;

	case ESP_GATTC_SEARCH_CMPL_EVT: {
		auto *service = this->parent_->get_service(espbt::ESPBTUUID::from_raw(SERVICE_UUID));
		if (service == nullptr) {
			ESP_LOGE(TAG, "Service not found");
			break;
		}

		auto *write_char = service->get_characteristic(espbt::ESPBTUUID::from_raw(WRITE_UUID));
		if (write_char == nullptr) {
			ESP_LOGE(TAG, "Write char not found");
			break;
		}
		this->write_handle_ = write_char->handle;

		auto *notify_char = service->get_characteristic(espbt::ESPBTUUID::from_raw(NOTIFY_UUID));
		if (notify_char == nullptr) {
			ESP_LOGE(TAG, "Notify char not found");
			break;
		}
		this->notify_handle_ = notify_char->handle;

		ESP_LOGI(TAG, "Write handle: 0x%04X, Notify handle: 0x%04X",
		         static_cast<unsigned>(this->write_handle_),
		         static_cast<unsigned>(this->notify_handle_));

		// Register the ESP32-side notification callback.
		esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(),
		                                  this->parent_->get_remote_bda(),
		                                  notify_char->handle);
		break;
	}

	case ESP_GATTC_READ_CHAR_EVT: {
		if (param->read.status != ESP_GATT_OK) {
			ESP_LOGW(TAG, "Read char failed: status=%d handle=0x%04X",
			         param->read.status, static_cast<unsigned>(param->read.handle));
			break;
		}
		ESP_LOGD(TAG, "READ_CHAR handle=0x%04X len=%d",
		         static_cast<unsigned>(param->read.handle), param->read.value_len);

		std::string hex;
		for (int i = 0; i < param->read.value_len; i++) {
			char buf[4];
			snprintf(buf, sizeof(buf), "%02X ", param->read.value[i]);
			hex += buf;
		}
		ESP_LOGD(TAG, "READ data: %s", hex.c_str());

		this->handle_rx_bytes_(param->read.value, param->read.value_len);
		break;
	}

	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		if (param->reg_for_notify.status == ESP_GATT_OK) {
			ESP_LOGI(TAG, "Registered for notifications");

			// Write CCCD after register_for_notify completes.
			auto *service = this->parent_->get_service(espbt::ESPBTUUID::from_raw(SERVICE_UUID));
			if (service != nullptr) {
				auto *notify_char = service->get_characteristic(espbt::ESPBTUUID::from_raw(NOTIFY_UUID));
				if (notify_char != nullptr) {
					auto *cccd = notify_char->get_descriptor(espbt::ESPBTUUID::from_uint16(0x2902));
					if (cccd != nullptr) {
						uint8_t val[] = {0x01, 0x00};
						esp_ble_gattc_write_char_descr(
						    this->parent_->get_gattc_if(),
						    this->parent_->get_conn_id(),
						    cccd->handle, sizeof(val), val,
						    ESP_GATT_WRITE_TYPE_RSP,
						    ESP_GATT_AUTH_REQ_NONE);
						ESP_LOGI(TAG, "CCCD written");
					}
				}
			}

			// The device needs a short pause before accepting the auth command.
			this->auth_pending_      = true;
			this->auth_pending_since_ = millis();
		}
		break;
	}

	case ESP_GATTC_NOTIFY_EVT: {
		if (param->notify.handle != this->notify_handle_) {
			break;
		}

		ESP_LOGD(TAG, "NOTIFY received: handle=0x%04X expected=0x%04X len=%d",
		         static_cast<unsigned>(param->notify.handle),
		         static_cast<unsigned>(this->notify_handle_), param->notify.value_len);
		std::string hex;
		for (int i = 0; i < param->notify.value_len; i++) {
			char buf[4];
			snprintf(buf, sizeof(buf), "%02X ", param->notify.value[i]);
			hex += buf;
		}
		ESP_LOGD(TAG, "NOTIFY data: %s", hex.c_str());

		this->handle_rx_bytes_(param->notify.value, param->notify.value_len);
		break;
	}

	default:
		break;
	}
}

void WattCycleBLE::authenticate_() {
	// Avoid sending the auth key more than once for the same connection.
	if (this->auth_sent_) {
		ESP_LOGD(TAG, "Auth already sent, skipping");
		return;
	}

	ESP_LOGI(TAG, "Authenticating...");

	auto *service = this->parent_->get_service(espbt::ESPBTUUID::from_raw(SERVICE_UUID));
	if (service == nullptr) {
		ESP_LOGE(TAG, "Service not found during auth");
		return;
	}

	auto *auth_char = service->get_characteristic(espbt::ESPBTUUID::from_raw(AUTH_UUID));
	if (auth_char == nullptr) {
		ESP_LOGE(TAG, "Auth characteristic not found");
		return;
	}

	// Write "HiLink" to the auth characteristic.
	uint8_t auth_data[] = {'H', 'i', 'L', 'i', 'n', 'k'};
	auto status = esp_ble_gattc_write_char(
	                  this->parent_->get_gattc_if(),
	                  this->parent_->get_conn_id(),
	                  auth_char->handle,
	                  sizeof(auth_data),
	                  auth_data,
	                  ESP_GATT_WRITE_TYPE_RSP,
	                  ESP_GATT_AUTH_REQ_NONE);

	if (status == ESP_GATT_OK) {
		ESP_LOGI(TAG, "Authentication command sent");
		this->auth_sent_     = true;
		this->authenticated_ = true;
		this->node_state     = espbt::ClientState::ESTABLISHED;
		// Delay the first read until the BMS has accepted the auth key.
		this->post_auth_pending_ = true;
		this->post_auth_pending_since_ = millis();
	} else {
		ESP_LOGE(TAG, "Failed to send auth: %d", status);
	}
}

void WattCycleBLE::send_read_command_(uint16_t dp_address) {
	if (this->write_handle_ == 0) {
		ESP_LOGW(TAG, "Write handle not set");
		return;
	}

	// Build the read frame used by the reference implementation.
	uint8_t frame[11];
	frame[0] = FRAME_HEAD_TX;     // 0x1E
	frame[1] = 0x00;              // VER
	frame[2] = 0x01;              // ADDR
	frame[3] = FUNC_READ;         // FUNC 0x03
	frame[4] = (dp_address >> 8) & 0xFF;
	frame[5] = dp_address & 0xFF;
	// READ_COUNT = 0x0000 is intentional for this device: the BMS responds
	// with the full dataset for the requested datapoint.
	frame[6] = 0x00;              // READ_COUNT high
	frame[7] = 0x00;              // READ_COUNT low

	uint16_t crc = this->calculate_crc16_(frame, 8);
	frame[8]  = (crc >> 8) & 0xFF;
	frame[9]  = crc & 0xFF;
	frame[10] = FRAME_TAIL;

	ESP_LOGD(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
	         frame[0], frame[1], frame[2], frame[3], frame[4],
	         frame[5], frame[6], frame[7], frame[8], frame[9], frame[10]);

	auto status = esp_ble_gattc_write_char(
	                  this->parent_->get_gattc_if(),
	                  this->parent_->get_conn_id(),
	                  this->write_handle_,
	                  sizeof(frame),
	                  frame,
	                  ESP_GATT_WRITE_TYPE_NO_RSP,
	                  ESP_GATT_AUTH_REQ_NONE);

	if (status != ESP_GATT_OK) {
		ESP_LOGW(TAG, "Failed to send read command: %d", status);
		return;
	}

	ESP_LOGD(TAG, "Read command sent for DP 0x%04X", static_cast<unsigned>(dp_address));
}

void WattCycleBLE::handle_rx_bytes_(const uint8_t *data, size_t len) {
	for (size_t i = 0; i < len; i++) {
		this->rx_buffer_.push_back(data[i]);
	}

	ESP_LOGV(TAG, "RX buffer size: %zu, expected: %zu",
	         this->rx_buffer_.size(), this->expected_length_);

	if (this->expected_length_ == 0 && this->rx_buffer_.size() >= 8) {
		if (this->rx_buffer_[0] != FRAME_HEAD_RX) {
			ESP_LOGW(TAG, "Invalid frame head: 0x%02X, discarding buffer", this->rx_buffer_[0]);
			this->rx_buffer_.clear();
			return;
		}

		uint16_t data_len = (this->rx_buffer_[6] << 8) | this->rx_buffer_[7];
		this->expected_length_ = data_len + 11;
		ESP_LOGD(TAG, "Expected frame length: %zu (data_len=%u)",
		         this->expected_length_, static_cast<unsigned>(data_len));

		if (this->expected_length_ > MAX_FRAME_LENGTH) {
			ESP_LOGW(TAG, "Frame too large: %zu bytes, discarding buffer", this->expected_length_);
			this->rx_buffer_.clear();
			this->expected_length_ = 0;
			return;
		}
	}

	if (this->expected_length_ > 0 && this->rx_buffer_.size() >= this->expected_length_) {
		if (this->rx_buffer_[this->expected_length_ - 1] == FRAME_TAIL) {
			ESP_LOGD(TAG, "Complete frame received (%zu bytes)", this->expected_length_);

			std::vector<uint8_t> frame(this->rx_buffer_.begin(),
			                           this->rx_buffer_.begin() + this->expected_length_);
			this->parse_frame_(frame);
		} else {
			ESP_LOGW(TAG, "Invalid frame tail: 0x%02X at pos %zu",
			         this->rx_buffer_[this->expected_length_ - 1], this->expected_length_ - 1);
		}

		this->rx_buffer_.clear();
		this->expected_length_ = 0;
	}
}

void WattCycleBLE::parse_frame_(const std::vector<uint8_t> &frame) {
	if (frame.size() < 11) {
		ESP_LOGW(TAG, "Frame too short: %zu bytes", frame.size());
		return;
	}

	// CRC is calculated on all bytes except CRC_HI, CRC_LO and TAIL.
	uint16_t calculated_crc = this->calculate_crc16_(frame.data(), frame.size() - 3);
	uint16_t received_crc   = (frame[frame.size() - 3] << 8) | frame[frame.size() - 2];

	if (calculated_crc != received_crc) {
		ESP_LOGW(TAG, "CRC mismatch: calc=0x%04X recv=0x%04X", calculated_crc, received_crc);
		std::string hex;
		for (size_t i = 0; i < frame.size(); i++) {
			char buf[4];
			snprintf(buf, sizeof(buf), "%02X ", frame[i]);
			hex += buf;
		}
		ESP_LOGW(TAG, "Frame: %s", hex.c_str());
		return;
	}

	uint8_t  func       = frame[3];
	uint16_t start_addr = (frame[4] << 8) | frame[5];
	uint16_t data_len   = (frame[6] << 8) | frame[7];

	ESP_LOGD(TAG, "Frame OK: FUNC=0x%02X ADDR=0x%04X LEN=%u",
	         func, static_cast<unsigned>(start_addr), static_cast<unsigned>(data_len));

	if (func == 0x86) {
		ESP_LOGW(TAG, "Error response from device");
		return;
	}

	const uint8_t *data      = &frame[8];
	size_t         data_size = data_len;

	if (start_addr == DP_ANALOG) {
		this->parse_analog_data_(data, data_size);
	} else if (start_addr == DP_PRODUCT) {
		if (!this->product_read_) {
			this->parse_product_data_(data, data_size);
			this->product_read_ = true;
		}
		this->last_poll_ = millis() - this->update_interval_;
	}
}

void WattCycleBLE::parse_analog_data_(const uint8_t *data, size_t len) {
	if (len < 1) {
		ESP_LOGW(TAG, "Analog data too short: %zu bytes", len);
		return;
	}

	size_t offset = 0;

	uint8_t cell_count = data[offset++];
	ESP_LOGD(TAG, "Cell count: %d", cell_count);

	if (offset + cell_count * 2 > len) {
		ESP_LOGW(TAG, "Buffer overflow in cell voltages");
		return;
	}

	std::vector<float> cell_voltages;
	cell_voltages.reserve(cell_count);
	for (uint8_t i = 0; i < cell_count; i++) {
		uint16_t cell_raw = (data[offset] << 8) | data[offset + 1];
		offset += 2;
		cell_voltages.push_back(cell_raw / 1000.0f);
	}
	for (size_t i = 0; i < cell_voltages.size() && i < this->cell_voltage_sensors_.size(); i++) {
		if (this->cell_voltage_sensors_[i] != nullptr) {
			this->cell_voltage_sensors_[i]->publish_state(cell_voltages[i]);
		}
	}

	if (offset + 1 > len) {
		ESP_LOGW(TAG, "Buffer overflow at temp_count");
		return;
	}
	uint8_t temp_count = data[offset++];
	ESP_LOGD(TAG, "Temperature count: %d", temp_count);

	if (temp_count < 2) {
		ESP_LOGW(TAG, "Analog data has too few temperatures: %d", temp_count);
		return;
	}

	if (offset + 2 > len) {
		ESP_LOGW(TAG, "Buffer overflow at MOS temp");
		return;
	}
	uint16_t mos_temp_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float mos_temp = this->parse_temperature_(mos_temp_raw);

	if (this->mos_temperature_sensor_ != nullptr) {
		this->mos_temperature_sensor_->publish_state(mos_temp);
	}
	ESP_LOGD(TAG, "MOS Temperature: %.1f C (raw=%u)",
	         mos_temp, static_cast<unsigned>(mos_temp_raw));

	if (offset + 2 > len) {
		ESP_LOGW(TAG, "Buffer overflow at PCB temp");
		return;
	}
	uint16_t pcb_temp_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float pcb_temp = this->parse_temperature_(pcb_temp_raw);
	if (this->pcb_temperature_sensor_ != nullptr) {
		this->pcb_temperature_sensor_->publish_state(pcb_temp);
	}

	std::vector<float> cell_temps;
	if (temp_count > 2) {
		size_t cell_temps_count = temp_count - 2;
		size_t cell_temps_bytes = cell_temps_count * 2;
		if (offset + cell_temps_bytes > len) {
			ESP_LOGW(TAG, "Buffer overflow in cell temperatures");
			return;
		}
		cell_temps.reserve(cell_temps_count);
		for (size_t i = 0; i < cell_temps_count; i++) {
			uint16_t temp_raw = (data[offset] << 8) | data[offset + 1];
			offset += 2;
			cell_temps.push_back(this->parse_temperature_(temp_raw));
		}
	}
	for (size_t i = 0; i < cell_temps.size() && i < this->cell_temperature_sensors_.size(); i++) {
		if (this->cell_temperature_sensors_[i] != nullptr) {
			this->cell_temperature_sensors_[i]->publish_state(cell_temps[i]);
		}
	}

	if (offset + 2 > len) {
		ESP_LOGW(TAG, "Buffer overflow at current");
		return;
	}
	float current = this->parse_current_negative_(&data[offset]);
	offset += 2;

	if (this->battery_current_sensor_ != nullptr) {
		this->battery_current_sensor_->publish_state(current);
	}
	ESP_LOGD(TAG, "Current: %.2fA", current);

	if (offset + 2 > len) {
		ESP_LOGW(TAG, "Buffer overflow at voltage");
		return;
	}
	uint16_t voltage_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float voltage = voltage_raw / 100.0f;

	if (this->battery_voltage_sensor_ != nullptr) {
		this->battery_voltage_sensor_->publish_state(voltage);
	}
	ESP_LOGD(TAG, "Voltage: %.2fV (raw=%u)", voltage, static_cast<unsigned>(voltage_raw));

	if (offset + 10 > len) {
		ESP_LOGW(TAG, "Analog data missing capacity/cycle/SOC fields: offset=%zu len=%zu", offset, len);
		return;
	}

	uint16_t remaining_capacity_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float remaining_capacity = remaining_capacity_raw / 10.0f;
	if (this->remaining_capacity_sensor_ != nullptr) {
		this->remaining_capacity_sensor_->publish_state(remaining_capacity);
	}

	uint16_t total_capacity_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float total_capacity = total_capacity_raw / 10.0f;
	if (this->total_capacity_sensor_ != nullptr) {
		this->total_capacity_sensor_->publish_state(total_capacity);
	}

	uint16_t cycles = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	if (this->cycles_sensor_ != nullptr) {
		this->cycles_sensor_->publish_state(cycles);
	}

	uint16_t design_capacity_raw = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	float design_capacity = design_capacity_raw / 10.0f;
	if (this->design_capacity_sensor_ != nullptr) {
		this->design_capacity_sensor_->publish_state(design_capacity);
	}

	uint16_t soc = (data[offset] << 8) | data[offset + 1];
	offset += 2;
	if (this->soc_sensor_ != nullptr) {
		this->soc_sensor_->publish_state(soc);
	}

	float power = voltage * current;
	if (this->battery_power_sensor_ != nullptr) {
		this->battery_power_sensor_->publish_state(power);
	}
	ESP_LOGD(TAG, "Power: %.1fW", power);
	ESP_LOGD(TAG, "SOC: %u%%, remaining: %.1fAh, total: %.1fAh, design: %.1fAh, cycles: %u",
	         static_cast<unsigned>(soc), remaining_capacity, total_capacity, design_capacity,
	         static_cast<unsigned>(cycles));
	ESP_LOGD(TAG, "PCB Temperature: %.1f C (raw=%u), cell temps: %zu",
	         pcb_temp, static_cast<unsigned>(pcb_temp_raw), cell_temps.size());

	if (!cell_voltages.empty()) {
		float min_cell = cell_voltages[0];
		float max_cell = cell_voltages[0];
		for (float cell_voltage : cell_voltages) {
			if (cell_voltage < min_cell) {
				min_cell = cell_voltage;
			}
			if (cell_voltage > max_cell) {
				max_cell = cell_voltage;
			}
		}
		ESP_LOGD(TAG, "Cell voltage min/max/spread: %.3fV / %.3fV / %.0fmV",
		         min_cell, max_cell, (max_cell - min_cell) * 1000.0f);
		if (this->cell_spread_sensor_ != nullptr) {
			this->cell_spread_sensor_->publish_state((max_cell - min_cell) * 1000.0f);
		}
	}
}

void WattCycleBLE::parse_product_data_(const uint8_t *data, size_t len) {
	std::string model_or_fw;
	std::string manufacturer;
	std::string serial;

	if (len == 60) {
		model_or_fw = this->trim_ascii_(data, 20);
		manufacturer = this->trim_ascii_(data + 20, 20);
		serial = this->trim_ascii_(data + 40, 20);
	} else {
		model_or_fw = this->hex_encode_(data, len);
		manufacturer = "";
		char buf[40];
		snprintf(buf, sizeof(buf), "unexpected length %zu", len);
		serial = buf;
	}

	ESP_LOGI(TAG, "Product model/fw: %s", model_or_fw.c_str());
	ESP_LOGI(TAG, "Product manufacturer: %s", manufacturer.c_str());
	ESP_LOGI(TAG, "Product serial: %s", serial.c_str());

	if (this->model_or_fw_text_sensor_ != nullptr) {
		this->model_or_fw_text_sensor_->publish_state(model_or_fw);
	}
	if (this->manufacturer_text_sensor_ != nullptr) {
		this->manufacturer_text_sensor_->publish_state(manufacturer);
	}
	if (this->serial_text_sensor_ != nullptr) {
		this->serial_text_sensor_->publish_state(serial);
	}
}

uint16_t WattCycleBLE::calculate_crc16_(const uint8_t *data, size_t len) {
	uint8_t crc_hi = 0xFF;
	uint8_t crc_lo = 0xFF;

	for (size_t i = 0; i < len; i++) {
		uint8_t index = crc_hi ^ data[i];
		crc_hi = crc_lo ^ CRC_HI_TABLE[index];
		crc_lo = CRC_LO_TABLE[index];
	}

	// Return the high byte first
	return (crc_lo << 8) | crc_hi;
}

float WattCycleBLE::parse_temperature_(uint16_t raw) {
	return (raw - 2730) / 10.0f;
}

float WattCycleBLE::parse_current_negative_(const uint8_t *buffer) {
	// Keep the bytes unsigned to avoid implementation-defined signed bit ops.
	uint8_t byte0 = buffer[0];
	uint8_t byte1 = buffer[1];

	bool     is_negative = (byte0 & 0x80) != 0;
	bool     has_decimal = (byte0 & 0x40) != 0;
	uint16_t raw_value   = byte1 | ((byte0 & 0x3F) << 8);

	float current = has_decimal ? (raw_value / 10.0f) : (float) raw_value;
	return is_negative ? -current : current;
}

std::string WattCycleBLE::hex_encode_(const uint8_t *data, size_t len) {
	std::string out;
	out.reserve(len * 2);
	for (size_t i = 0; i < len; i++) {
		char buf[3];
		snprintf(buf, sizeof(buf), "%02x", data[i]);
		out += buf;
	}
	return out;
}

std::string WattCycleBLE::trim_ascii_(const uint8_t *data, size_t len) {
	std::string out(reinterpret_cast<const char *>(data), len);
	while (!out.empty() && (out.back() == '\0' || out.back() == ' ')) {
		out.pop_back();
	}
	return out;
}

}  // namespace wattcycle_ble
}  // namespace esphome
