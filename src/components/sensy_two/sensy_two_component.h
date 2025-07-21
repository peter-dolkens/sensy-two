#ifndef SENSY_TWO_COMPONENT_H
#define SENSY_TWO_COMPONENT_H

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/uart/uart_component_esp_idf.h"
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <array>

namespace esphome {
namespace sensytwo {

class SensyTwoComponent : public Component, public uart::UARTDevice {
 public:
  static const size_t MAX_TARGETS = 10;
  static const size_t FIELDS = 7;
  explicit SensyTwoComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  void set_detection_range_threshold(float range_cm) {
    detection_range_threshold_ = range_cm;
  }

  void set_publish_interval_ms(uint32_t interval_ms) {
    publish_interval_ms_ = interval_ms;
  }

  void setup() override {
    this->radar_debug(3);
    this->radar_restart();
    this->radar_start();
    this->radar_report_interval(200);
    this->radar_monitor_interval(1);
    this->radar_heartbeat_timeout(10);
    this->radar_sensitivity(2);
    this->radar_seeking();
    this->radar_capture();
    if (auto *idf = static_cast<uart::IDFUARTComponent *>(this->parent_)) {
      uart_num_ = static_cast<uart_port_t>(idf->get_hw_serial_number());
      uart_queue_ = idf->get_uart_event_queue();
      xTaskCreatePinnedToCore(uart_task, "sensy_uart", 4096, this, 1, &task_handle_, 1);
    }
  }

  void radar_debug(int level) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+DEBUG=%d\n", level);
    this->write_str(cmd);
    delay(100);
  }
  void radar_start() { this->write_str("AT+START\n"); delay(100); }
  void radar_report_interval(int value) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+TIME=%d\n", value);
    this->write_str(cmd);
    delay(100);
  }
  void radar_monitor_interval(int value) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+MONTIME=%d\n", value);
    this->write_str(cmd);
    delay(100);
  }
  void radar_heartbeat_timeout(int value) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+HEATIME=%d\n", value);
    this->write_str(cmd);
    delay(100);
  }
  void radar_sensitivity(int value) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+SENS=%d\n", value);
    this->write_str(cmd);
    delay(100);
  }
  void radar_seeking() { this->write_str("AT+SEEKING\n"); delay(100); }
  void radar_restart() { this->write_str("AT+RESET\n"); delay(100); }
  void radar_capture() { this->write_str("AT+SETTING\n"); delay(100); }

  void loop() override {
    parse_ring();
    yield();  // Allow other tasks to run
    delay(10);  // Allow some time for UART processing
  }

  std::vector<sensor::Sensor *> get_target_sensors() {
    std::vector<sensor::Sensor *> sensors;
    sensors.reserve(MAX_TARGETS * FIELDS);
    for (size_t i = 0; i < MAX_TARGETS; ++i) {
      sensors.push_back(x_sensors_[i]);
      sensors.push_back(y_sensors_[i]);
      sensors.push_back(z_sensors_[i]);
      sensors.push_back(angle_sensors_[i]);
      sensors.push_back(speed_sensors_[i]);
      sensors.push_back(distance_resolution_sensors_[i]);
      sensors.push_back(distance_sensors_[i]);
    }
    return sensors;
  }

  std::vector<sensor::Sensor *> get_all_sensors() { return get_target_sensors(); }

  std::vector<text_sensor::TextSensor *> get_text_sensors() {
    return {radar_firmware, radar_mac};
  }

  void read_firmware() { this->write_str("AT+FIRMWARE\n"); delay(100); }

  void read_mac_address() {
    radar_mac->publish_state(esphome::get_mac_address());
  }

  sensor::Sensor *t1_x = new sensor::Sensor();
  sensor::Sensor *t1_y = new sensor::Sensor();
  sensor::Sensor *t1_z = new sensor::Sensor();
  sensor::Sensor *t1_angle = new sensor::Sensor();
  sensor::Sensor *t1_speed = new sensor::Sensor();
  sensor::Sensor *t1_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t1_distance = new sensor::Sensor();

  sensor::Sensor *t2_x = new sensor::Sensor();
  sensor::Sensor *t2_y = new sensor::Sensor();
  sensor::Sensor *t2_z = new sensor::Sensor();
  sensor::Sensor *t2_angle = new sensor::Sensor();
  sensor::Sensor *t2_speed = new sensor::Sensor();
  sensor::Sensor *t2_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t2_distance = new sensor::Sensor();

  sensor::Sensor *t3_x = new sensor::Sensor();
  sensor::Sensor *t3_y = new sensor::Sensor();
  sensor::Sensor *t3_z = new sensor::Sensor();
  sensor::Sensor *t3_angle = new sensor::Sensor();
  sensor::Sensor *t3_speed = new sensor::Sensor();
  sensor::Sensor *t3_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t3_distance = new sensor::Sensor();

  sensor::Sensor *t4_x = new sensor::Sensor();
  sensor::Sensor *t4_y = new sensor::Sensor();
  sensor::Sensor *t4_z = new sensor::Sensor();
  sensor::Sensor *t4_angle = new sensor::Sensor();
  sensor::Sensor *t4_speed = new sensor::Sensor();
  sensor::Sensor *t4_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t4_distance = new sensor::Sensor();

  sensor::Sensor *t5_x = new sensor::Sensor();
  sensor::Sensor *t5_y = new sensor::Sensor();
  sensor::Sensor *t5_z = new sensor::Sensor();
  sensor::Sensor *t5_angle = new sensor::Sensor();
  sensor::Sensor *t5_speed = new sensor::Sensor();
  sensor::Sensor *t5_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t5_distance = new sensor::Sensor();

  sensor::Sensor *t6_x = new sensor::Sensor();
  sensor::Sensor *t6_y = new sensor::Sensor();
  sensor::Sensor *t6_z = new sensor::Sensor();
  sensor::Sensor *t6_angle = new sensor::Sensor();
  sensor::Sensor *t6_speed = new sensor::Sensor();
  sensor::Sensor *t6_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t6_distance = new sensor::Sensor();

  sensor::Sensor *t7_x = new sensor::Sensor();
  sensor::Sensor *t7_y = new sensor::Sensor();
  sensor::Sensor *t7_z = new sensor::Sensor();
  sensor::Sensor *t7_angle = new sensor::Sensor();
  sensor::Sensor *t7_speed = new sensor::Sensor();
  sensor::Sensor *t7_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t7_distance = new sensor::Sensor();

  sensor::Sensor *t8_x = new sensor::Sensor();
  sensor::Sensor *t8_y = new sensor::Sensor();
  sensor::Sensor *t8_z = new sensor::Sensor();
  sensor::Sensor *t8_angle = new sensor::Sensor();
  sensor::Sensor *t8_speed = new sensor::Sensor();
  sensor::Sensor *t8_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t8_distance = new sensor::Sensor();

  sensor::Sensor *t9_x = new sensor::Sensor();
  sensor::Sensor *t9_y = new sensor::Sensor();
  sensor::Sensor *t9_z = new sensor::Sensor();
  sensor::Sensor *t9_angle = new sensor::Sensor();
  sensor::Sensor *t9_speed = new sensor::Sensor();
  sensor::Sensor *t9_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t9_distance = new sensor::Sensor();

  sensor::Sensor *t10_x = new sensor::Sensor();
  sensor::Sensor *t10_y = new sensor::Sensor();
  sensor::Sensor *t10_z = new sensor::Sensor();
  sensor::Sensor *t10_angle = new sensor::Sensor();
  sensor::Sensor *t10_speed = new sensor::Sensor();
  sensor::Sensor *t10_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t10_distance = new sensor::Sensor();

  text_sensor::TextSensor *radar_firmware = new text_sensor::TextSensor();
  text_sensor::TextSensor *radar_mac = new text_sensor::TextSensor();

  std::array<sensor::Sensor *, MAX_TARGETS> x_sensors_{t1_x, t2_x, t3_x, t4_x, t5_x, t6_x, t7_x, t8_x, t9_x, t10_x};
  std::array<sensor::Sensor *, MAX_TARGETS> y_sensors_{t1_y, t2_y, t3_y, t4_y, t5_y, t6_y, t7_y, t8_y, t9_y, t10_y};
  std::array<sensor::Sensor *, MAX_TARGETS> z_sensors_{t1_z, t2_z, t3_z, t4_z, t5_z, t6_z, t7_z, t8_z, t9_z, t10_z};
  std::array<sensor::Sensor *, MAX_TARGETS> angle_sensors_{t1_angle, t2_angle, t3_angle, t4_angle, t5_angle, t6_angle, t7_angle, t8_angle, t9_angle, t10_angle};
  std::array<sensor::Sensor *, MAX_TARGETS> speed_sensors_{t1_speed, t2_speed, t3_speed, t4_speed, t5_speed, t6_speed, t7_speed, t8_speed, t9_speed, t10_speed};
  std::array<sensor::Sensor *, MAX_TARGETS> distance_resolution_sensors_{t1_distance_resolution, t2_distance_resolution, t3_distance_resolution, t4_distance_resolution, t5_distance_resolution, t6_distance_resolution, t7_distance_resolution, t8_distance_resolution, t9_distance_resolution, t10_distance_resolution};
  std::array<sensor::Sensor *, MAX_TARGETS> distance_sensors_{t1_distance, t2_distance, t3_distance, t4_distance, t5_distance, t6_distance, t7_distance, t8_distance, t9_distance, t10_distance};

 protected:
  static const uint8_t HEADER[8];
  static const size_t RING_BUFFER_SIZE = 10240;
  volatile uint8_t ring_[RING_BUFFER_SIZE];
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
  static const size_t UART_BUFFER_SIZE = 128;

  uart_port_t uart_num_{};
  QueueHandle_t *uart_queue_{nullptr};
  TaskHandle_t task_handle_{nullptr};

  enum ParseState {
    SEARCHING_HEADER,
    READING_LENGTH,
    READING_TLV_HEADER,
    READING_POINTS,
    READING_PERSONS
  } state_ = SEARCHING_HEADER;

  uint8_t length_buf_[8];
  uint8_t tlv_header_[8];
  uint32_t expected_len_ = 0;
  uint32_t frame_no_ = 0;
  uint32_t frame_remaining_ = 0;
  uint32_t tlv_type_ = 0;
  uint32_t tlv_len_ = 0;
  uint32_t bytes_read_ = 0;
  uint32_t item_index_ = 0;

  float detection_range_threshold_ = 600.0f;
  uint32_t publish_interval_ms_ = 1000;

  struct TargetState {
    std::array<float, FIELDS> values{};
  };

  std::array<TargetState, MAX_TARGETS> current_state_{};
  std::array<TargetState, MAX_TARGETS> last_published_{};
  std::array<uint32_t, MAX_TARGETS> last_published_time_{};

  struct Person {
    uint32_t id;
    uint32_t q;
    float x, y, z;
    float vx, vy, vz;
  } __attribute__((packed));

  static constexpr uint32_t INVALID_ID = 0;
  std::array<uint32_t, MAX_TARGETS> target_ids_{};
  std::array<uint32_t, MAX_TARGETS> target_last_seen_{};
  std::vector<Person> persons_buffer_;

  void read_uart() {
    uint8_t temp[128];
    while (this->available()) {
      size_t n = this->read_array(temp, sizeof(temp));
      write_ring(temp, n);
    }
  }

  static void uart_task(void *param) {
    auto *self = static_cast<SensyTwoComponent *>(param);
    self->uart_task_loop();
  }

  void uart_task_loop() {
    uart_event_t event;
    uint8_t temp[UART_BUFFER_SIZE];
    size_t buffered_size;
    
    while (true) {
      uart_get_buffered_data_len(uart_num_, &buffered_size);

      if (buffered_size >= UART_BUFFER_SIZE) {
        if (buffered_size > UART_BUFFER_SIZE) buffered_size = UART_BUFFER_SIZE;
        int len = uart_read_bytes(uart_num_, temp, buffered_size, portMAX_DELAY);
        if (len > 0) {
          write_ring_task(temp, len);
          // // Log the contents of temp in hex format
          // char hexbuf[UART_BUFFER_SIZE * 3 + 1];
          // size_t hexpos = 0;
          // for (int i = 0; i < len; ++i) {
          //   hexpos += snprintf(hexbuf + hexpos, sizeof(hexbuf) - hexpos, "%02X ", temp[i]);
          //   if (hexpos >= sizeof(hexbuf) - 4) break;
          // }
          // hexbuf[hexpos] = '\0';
          // ESP_LOGI("SensyTwo", "UART HEX: %s", hexbuf);
        }
      }

      yield();  // Allow other tasks to run
      delay(10);  // Allow some time for UART processing
    }
  }

  void write_ring_task(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      size_t next = (head_ + 1) % RING_BUFFER_SIZE;
      if (next != tail_) {
        ring_[head_] = data[i];
        head_ = next;
      } else {
        break;
      }
    }
  }

  void write_ring(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      size_t next = (head_ + 1) % RING_BUFFER_SIZE;
      if (next != tail_) {
        ring_[head_] = data[i];
        head_ = next;
      } else {
        break;  // overflow discard
      }
    }
  }

  size_t available_ring() const {
    return (head_ + RING_BUFFER_SIZE - tail_) % RING_BUFFER_SIZE;
  }

  bool read_ring(uint8_t *dest, size_t len) {
    if (available_ring() < len) return false;
    size_t first = std::min(len, RING_BUFFER_SIZE - tail_);
    memcpy(dest, (const void *)&ring_[tail_], first);
    if (len > first) memcpy(dest + first, (const void *)&ring_[0], len - first);
    tail_ = (tail_ + len) % RING_BUFFER_SIZE;
    return true;
  }

  bool peek_ring(uint8_t *dest, size_t len) {
    if (available_ring() < len) return false;
    size_t first = std::min(len, RING_BUFFER_SIZE - tail_);
    memcpy(dest, (const void *)&ring_[tail_], first);
    if (len > first) memcpy(dest + first, (const void *)&ring_[0], len - first);
    return true;
  }

  void maybe_publish(size_t index) {
    uint32_t now = millis();
    if (now - last_published_time_[index] < publish_interval_ms_) return;
    if (current_state_[index].values != last_published_[index].values) {
      last_published_time_[index] = now;
      last_published_[index] = current_state_[index];
      auto &vals = current_state_[index].values;
      x_sensors_[index]->publish_state(vals[0]);
      y_sensors_[index]->publish_state(vals[1]);
      z_sensors_[index]->publish_state(vals[2]);
      angle_sensors_[index]->publish_state(vals[3]);
      speed_sensors_[index]->publish_state(vals[4]);
      distance_resolution_sensors_[index]->publish_state(vals[5]);
      distance_sensors_[index]->publish_state(vals[6]);
    }
  }

  void parse_ring() {
    switch (state_) {
      case SEARCHING_HEADER: {
        uint8_t buf[sizeof(HEADER)];
        while (peek_ring(buf, sizeof(buf))) {
          if (memcmp(buf, HEADER, sizeof(buf)) == 0) {
            read_ring(buf, sizeof(buf));
            state_ = READING_LENGTH;
            break;
          } else {
            uint8_t discard;
            read_ring(&discard, 1);
          }
        }
        break;
      }
      case READING_LENGTH:
        if (read_ring(length_buf_, 8)) {
          expected_len_ = *((uint32_t *)length_buf_);
          frame_no_ = *((uint32_t *)length_buf_ + 1);
          frame_remaining_ = expected_len_ - 8;
          state_ = (expected_len_ < 12) ? SEARCHING_HEADER : READING_TLV_HEADER;
        }
        break;
      case READING_TLV_HEADER:
        if (read_ring(tlv_header_, 8)) {
          tlv_type_ = *((uint32_t *)tlv_header_);
          tlv_len_ = *((uint32_t *)(tlv_header_ + 4));
          frame_remaining_ -= 8;
          bytes_read_ = 0;
          item_index_ = 0;
          if (tlv_type_ == 0x01) {
            state_ = READING_POINTS;
          } else if (tlv_type_ == 0x02) {
            persons_buffer_.clear();
            state_ = READING_PERSONS;
          } else {
            state_ = SEARCHING_HEADER; // Skip other TLV data
          }
        }
        break;
      case READING_PERSONS:
        if (available_ring() >= sizeof(Person)) {
          Person p;
          read_ring((uint8_t *)&p, sizeof(Person));
          bytes_read_ += sizeof(Person);
          persons_buffer_.push_back(p);
        }
        if (bytes_read_ >= tlv_len_) {
          assign_persons(persons_buffer_);
          persons_buffer_.clear();
          frame_remaining_ -= tlv_len_;
          state_ = (frame_remaining_ >= 8) ? READING_TLV_HEADER : SEARCHING_HEADER;
        }
        break;
      case READING_POINTS:
        if (available_ring() > 0) {
          uint8_t dump[64];
          size_t todo = std::min<size_t>(tlv_len_ - bytes_read_, sizeof(dump));
          todo = std::min(todo, available_ring());
          if (read_ring(dump, todo)) {
            bytes_read_ += todo;
          }
        }
        if (bytes_read_ >= tlv_len_) {
          frame_remaining_ -= tlv_len_;
          state_ = (frame_remaining_ >= 8) ? READING_TLV_HEADER : SEARCHING_HEADER;
        }
        break;
      default:
        state_ = SEARCHING_HEADER;
        break;
    }
  }

  void publish_target(size_t index, const Person &p) {
    if (index >= MAX_TARGETS) return;
    float distance = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
    float angle = (distance > 0.0f) ? atan2f(p.x, p.y) * 180.0f / M_PI : 0.0f;
    float speed = sqrtf(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz);

    TargetState state;
    if (distance > detection_range_threshold_) {
      state.values = {0, 0, 0, 0, 0, 0, 0};
    } else {
      state.values = {p.x, p.y, p.z, angle, speed, 0, distance};
    }
    current_state_[index] = state;

    maybe_publish(index);
  }

  void clear_targets() {
    for (size_t i = 0; i < MAX_TARGETS; ++i) {
      current_state_[i].values = {0, 0, 0, 0, 0, 0, 0};
      maybe_publish(i);
    }
  }

  size_t find_index_for_id(uint32_t id) const {
    for (size_t i = 0; i < MAX_TARGETS; ++i) {
      if (target_ids_[i] == id) return i;
    }
    return MAX_TARGETS;
  }

  size_t allocate_index_for_id(uint32_t id) {
    for (size_t i = 0; i < MAX_TARGETS; ++i) {
      if (target_ids_[i] == INVALID_ID) {
        target_ids_[i] = id;
        return i;
      }
    }
    size_t oldest = 0;
    for (size_t i = 1; i < MAX_TARGETS; ++i) {
      if (target_last_seen_[i] < target_last_seen_[oldest]) {
        oldest = i;
      }
    }
    target_ids_[oldest] = id;
    return oldest;
  }

  void publish_empty(size_t index) {
    if (index >= MAX_TARGETS) return;
    current_state_[index].values = {0, 0, 0, 0, 0, 0, 0};
    maybe_publish(index);
  }

  void assign_persons(const std::vector<Person> &persons) {
    std::array<bool, MAX_TARGETS> seen{};
    seen.fill(false);
    for (const auto &p : persons) {
      size_t idx = find_index_for_id(p.id);
      if (idx == MAX_TARGETS) {
        idx = allocate_index_for_id(p.id);
      }
      publish_target(idx, p);
      target_last_seen_[idx] = frame_no_;
      seen[idx] = true;
    }
    for (size_t i = 0; i < MAX_TARGETS; ++i) {
      if (!seen[i] && target_last_seen_[i] != frame_no_) {
        publish_empty(i);
        target_ids_[i] = INVALID_ID;
      }
    }
  }
};

const uint8_t SensyTwoComponent::HEADER[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

}  // namespace sensytwo
}  // namespace esphome

#endif  // SENSY_TWO_COMPONENT_H
