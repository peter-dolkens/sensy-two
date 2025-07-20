#ifndef SENSY_TWO_COMPONENT_H
#define SENSY_TWO_COMPONENT_H

#include "esphome.h"
#include <cstring>
#include <cmath>
#include <vector>

namespace esphome {
namespace sensytwo {

class SensyTwoComponent : public Component, public UARTDevice {
 public:
  explicit SensyTwoComponent(UARTComponent *parent)
      : UARTDevice(parent) {}

  void set_detection_range_threshold(float range_cm) {
    detection_range_threshold_ = range_cm;
  }

  void setup() override {
    this->write_str("AT+RESET\n");
    delay(100);
    this->write_str("AT+START\n");
    delay(100);
    this->write_str("AT+TIME=200\n");
    delay(100);
    this->write_str("AT+MONTIME=1\n");
    delay(100);
    this->write_str("AT+HEATIME=10\n");
    delay(100);
    this->write_str("AT+SENS=2\n");
    delay(100);
    this->write_str("AT+SEEKING\n");
    delay(100);
    this->write_str("AT+SETTING\n");
    delay(100);
  }

  void loop() override {
    read_uart_();
    parse_ring_();
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

  void restart_module() { this->write_str("AT+RESET\n"); }

  void reset_points() { this->write_str("AT+SETTING\n"); }

  void read_firmware() { this->write_str("AT+FIRMWARE?\n"); }

  void read_mac_address() { this->write_str("AT+MAC?\n"); }

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
  uint8_t ring_[RING_BUFFER_SIZE];
  size_t head_ = 0;
  size_t tail_ = 0;

  char ascii_buffer_[64];
  size_t ascii_pos_ = 0;

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

  void read_uart_() {
    uint8_t temp[128];
    while (this->available()) {
      size_t n = this->read_array(temp, sizeof(temp));
      write_ring_(temp, n);
      for (size_t i = 0; i < n; i++) {
        char c = static_cast<char>(temp[i]);
        if (c == '\n' || ascii_pos_ >= sizeof(ascii_buffer_) - 1) {
          ascii_buffer_[ascii_pos_] = '\0';
          parse_ascii_(ascii_buffer_);
          ascii_pos_ = 0;
        } else if (c != '\r') {
          ascii_buffer_[ascii_pos_++] = c;
        }
      }
    }
  }

  void write_ring_(const uint8_t *data, size_t len) {
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

  size_t available_ring_() const {
    return (head_ + RING_BUFFER_SIZE - tail_) % RING_BUFFER_SIZE;
  }

  bool read_ring_(uint8_t *dest, size_t len) {
    if (available_ring_() < len) return false;
    size_t first = std::min(len, RING_BUFFER_SIZE - tail_);
    memcpy(dest, &ring_[tail_], first);
    if (len > first) memcpy(dest + first, &ring_[0], len - first);
    tail_ = (tail_ + len) % RING_BUFFER_SIZE;
    return true;
  }

  bool peek_ring_(uint8_t *dest, size_t len) {
    if (available_ring_() < len) return false;
    size_t first = std::min(len, RING_BUFFER_SIZE - tail_);
    memcpy(dest, &ring_[tail_], first);
    if (len > first) memcpy(dest + first, &ring_[0], len - first);
    return true;
  }

  void parse_ring_() {
    switch (state_) {
      case SEARCHING_HEADER: {
        uint8_t buf[sizeof(HEADER)];
        while (peek_ring_(buf, sizeof(buf))) {
          if (memcmp(buf, HEADER, sizeof(buf)) == 0) {
            read_ring_(buf, sizeof(buf));
            state_ = READING_LENGTH;
            break;
          } else {
            uint8_t discard;
            read_ring_(&discard, 1);
          }
        }
        break;
      }
      case READING_LENGTH:
        if (read_ring_(length_buf_, 8)) {
          expected_len_ = *((uint32_t *)length_buf_);
          frame_no_ = *((uint32_t *)length_buf_ + 1);
          frame_remaining_ = expected_len_ - 8;
          state_ = (expected_len_ < 12) ? SEARCHING_HEADER : READING_TLV_HEADER;
        }
        break;
      case READING_TLV_HEADER:
        if (read_ring_(tlv_header_, 8)) {
          tlv_type_ = *((uint32_t *)tlv_header_);
          tlv_len_ = *((uint32_t *)(tlv_header_ + 4));
          frame_remaining_ -= 8;
          bytes_read_ = 0;
          item_index_ = 0;
          if (tlv_type_ == 0x02) {
            state_ = READING_PERSONS;
          } else {
            state_ = SEARCHING_HEADER;
          }
        }
        break;
      case READING_PERSONS:
        if (available_ring_() >= sizeof(Person)) {
          Person p;
          read_ring_((uint8_t *)&p, sizeof(Person));
          bytes_read_ += sizeof(Person);
          publish_target_(item_index_, p);
          item_index_++;
          if (bytes_read_ >= tlv_len_) {
            frame_remaining_ -= tlv_len_;
            state_ = (frame_remaining_ >= 8) ? READING_TLV_HEADER : SEARCHING_HEADER;
          }
        }
        break;
      case READING_POINTS:
      default:
        state_ = SEARCHING_HEADER;
        break;
    }
  }

  void publish_target_(size_t index, const Person &p) {
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

  void parse_ascii_(const char *line) {
    if (strncmp(line, "FW:", 3) == 0) {
      radar_firmware->publish_state(line + 3);
    } else if (strncmp(line, "MAC:", 4) == 0) {
      radar_mac->publish_state(line + 4);
    }
  }
};

const uint8_t SensyTwoComponent::HEADER[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

}  // namespace sensytwo
}  // namespace esphome

#endif  // SENSY_TWO_COMPONENT_H
