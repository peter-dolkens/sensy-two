#ifndef SENSY_TWO_COMPONENT_H
#define SENSY_TWO_COMPONENT_H

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <cmath>
#include <vector>

namespace esphome {
namespace sensytwo {

class SensyTwoComponent : public Component, public uart::UARTDevice {
 public:
  explicit SensyTwoComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  void set_detection_range_threshold(float range_cm) {
    detection_range_threshold_ = range_cm;
  }

  void setup() override {
    this->radar_restart();
    this->radar_start();
    this->radar_report_interval(200);
    this->radar_monitor_interval(1);
    this->radar_heartbeat_timeout(10);
    this->radar_sensitivity(2);
    this->radar_seeking();
    this->radar_capture();
    xTaskCreatePinnedToCore(uart_task, "sensy_uart", 4096, this, 1, &task_handle_, 1);
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
    return {t1_x, t1_y, t1_angle, t1_speed, t1_distance_resolution, t1_distance,
            t2_x, t2_y, t2_angle, t2_speed, t2_distance_resolution, t2_distance,
            t3_x, t3_y, t3_angle, t3_speed, t3_distance_resolution, t3_distance};
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
  sensor::Sensor *t1_angle = new sensor::Sensor();
  sensor::Sensor *t1_speed = new sensor::Sensor();
  sensor::Sensor *t1_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t1_distance = new sensor::Sensor();

  sensor::Sensor *t2_x = new sensor::Sensor();
  sensor::Sensor *t2_y = new sensor::Sensor();
  sensor::Sensor *t2_angle = new sensor::Sensor();
  sensor::Sensor *t2_speed = new sensor::Sensor();
  sensor::Sensor *t2_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t2_distance = new sensor::Sensor();

  sensor::Sensor *t3_x = new sensor::Sensor();
  sensor::Sensor *t3_y = new sensor::Sensor();
  sensor::Sensor *t3_angle = new sensor::Sensor();
  sensor::Sensor *t3_speed = new sensor::Sensor();
  sensor::Sensor *t3_distance_resolution = new sensor::Sensor();
  sensor::Sensor *t3_distance = new sensor::Sensor();

  text_sensor::TextSensor *radar_firmware = new text_sensor::TextSensor();
  text_sensor::TextSensor *radar_mac = new text_sensor::TextSensor();

 protected:
  static const uint8_t HEADER[8];
  static const size_t RING_BUFFER_SIZE = 10240;
  volatile uint8_t ring_[RING_BUFFER_SIZE];
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
  static const size_t UART_BUFFER_SIZE = 128;

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

  struct Person {
    uint32_t id;
    uint32_t q;
    float x, y, z;
    float vx, vy, vz;
  } __attribute__((packed));

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
    uint8_t temp[UART_BUFFER_SIZE];
    
    while (true) {
      size_t available = this->available();
      if (available > 0) {
        if (available > UART_BUFFER_SIZE) {
          available = UART_BUFFER_SIZE;
        }
        size_t len = this->read_array(temp, available);
        if (len > 0) {
          write_ring_task(temp, len);
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
            state_ = READING_PERSONS;
          } else {
            state_ = SEARCHING_HEADER; // Skip other TLV data
          }
        }
        break;
      case READING_PERSONS:
        if (item_index_ == 0) {
          clear_targets();
        }
        if (available_ring() >= sizeof(Person)) {
          Person p;
          read_ring((uint8_t *)&p, sizeof(Person));
          bytes_read_ += sizeof(Person);
          publish_target(item_index_, p);
          item_index_++;
          if (bytes_read_ >= tlv_len_) {
            frame_remaining_ -= tlv_len_;
            state_ = (frame_remaining_ >= 8) ? READING_TLV_HEADER : SEARCHING_HEADER;
          }
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
    float distance = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
    float angle = (distance > 0.0f) ? atan2f(p.x, p.y) * 180.0f / M_PI : 0.0f;
    float speed = sqrtf(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz);

    if (distance > detection_range_threshold_) {
      if (index == 0) {
        t1_x->publish_state(0);
        t1_y->publish_state(0);
        t1_angle->publish_state(0);
        t1_speed->publish_state(0);
        t1_distance_resolution->publish_state(0);
        t1_distance->publish_state(0);
      } else if (index == 1) {
        t2_x->publish_state(0);
        t2_y->publish_state(0);
        t2_angle->publish_state(0);
        t2_speed->publish_state(0);
        t2_distance_resolution->publish_state(0);
        t2_distance->publish_state(0);
      } else if (index == 2) {
        t3_x->publish_state(0);
        t3_y->publish_state(0);
        t3_angle->publish_state(0);
        t3_speed->publish_state(0);
        t3_distance_resolution->publish_state(0);
        t3_distance->publish_state(0);
      }
      return;
    }

    if (index == 0) {
      t1_x->publish_state(p.x);
      t1_y->publish_state(p.y);
      t1_angle->publish_state(angle);
      t1_speed->publish_state(speed);
      t1_distance_resolution->publish_state(0);
      t1_distance->publish_state(distance);
    } else if (index == 1) {
      t2_x->publish_state(p.x);
      t2_y->publish_state(p.y);
      t2_angle->publish_state(angle);
      t2_speed->publish_state(speed);
      t2_distance_resolution->publish_state(0);
      t2_distance->publish_state(distance);
    } else if (index == 2) {
      t3_x->publish_state(p.x);
      t3_y->publish_state(p.y);
      t3_angle->publish_state(angle);
      t3_speed->publish_state(speed);
      t3_distance_resolution->publish_state(0);
      t3_distance->publish_state(distance);
    }
  }

  void clear_targets() {
    return;
    t1_x->publish_state(0);
    t1_y->publish_state(0);
    t1_angle->publish_state(0);
    t1_speed->publish_state(0);
    t1_distance_resolution->publish_state(0);
    t1_distance->publish_state(0);
    t2_x->publish_state(0);
    t2_y->publish_state(0);
    t2_angle->publish_state(0);
    t2_speed->publish_state(0);
    t2_distance_resolution->publish_state(0);
    t2_distance->publish_state(0);
    t3_x->publish_state(0);
    t3_y->publish_state(0);
    t3_angle->publish_state(0);
    t3_speed->publish_state(0);
    t3_distance_resolution->publish_state(0);
    t3_distance->publish_state(0);
  }
};

const uint8_t SensyTwoComponent::HEADER[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

}  // namespace sensytwo
}  // namespace esphome

#endif  // SENSY_TWO_COMPONENT_H
