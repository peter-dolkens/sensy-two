#ifndef SENSY_TWO_COMPONENT_H
#define SENSY_TWO_COMPONENT_H

#include "esphome.h"
#include <cstring>

namespace esphome {
namespace sensytwo {

class SensyTwoComponent : public Component, public UARTDevice {
 public:
  explicit SensyTwoComponent(UARTComponent *parent)
      : UARTDevice(parent) {}

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

  std::vector<sensor::Sensor *> get_person_sensors() {
    return {p1_x, p1_y, p1_z, p1_vx, p1_vy, p1_vz,
            p2_x, p2_y, p2_z, p2_vx, p2_vy, p2_vz,
            p3_x, p3_y, p3_z, p3_vx, p3_vy, p3_vz};
  }

  sensor::Sensor *p1_x = new sensor::Sensor();
  sensor::Sensor *p1_y = new sensor::Sensor();
  sensor::Sensor *p1_z = new sensor::Sensor();
  sensor::Sensor *p1_vx = new sensor::Sensor();
  sensor::Sensor *p1_vy = new sensor::Sensor();
  sensor::Sensor *p1_vz = new sensor::Sensor();

  sensor::Sensor *p2_x = new sensor::Sensor();
  sensor::Sensor *p2_y = new sensor::Sensor();
  sensor::Sensor *p2_z = new sensor::Sensor();
  sensor::Sensor *p2_vx = new sensor::Sensor();
  sensor::Sensor *p2_vy = new sensor::Sensor();
  sensor::Sensor *p2_vz = new sensor::Sensor();

  sensor::Sensor *p3_x = new sensor::Sensor();
  sensor::Sensor *p3_y = new sensor::Sensor();
  sensor::Sensor *p3_z = new sensor::Sensor();
  sensor::Sensor *p3_vx = new sensor::Sensor();
  sensor::Sensor *p3_vy = new sensor::Sensor();
  sensor::Sensor *p3_vz = new sensor::Sensor();

 protected:
  static const uint8_t HEADER[8];
  static const size_t RING_BUFFER_SIZE = 10240;
  uint8_t ring_[RING_BUFFER_SIZE];
  size_t head_ = 0;
  size_t tail_ = 0;

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
          publish_person_(item_index_, p);
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

  void publish_person_(size_t index, const Person &p) {
    if (index == 0) {
      p1_x->publish_state(p.x);
      p1_y->publish_state(p.y);
      p1_z->publish_state(p.z);
      p1_vx->publish_state(p.vx);
      p1_vy->publish_state(p.vy);
      p1_vz->publish_state(p.vz);
    } else if (index == 1) {
      p2_x->publish_state(p.x);
      p2_y->publish_state(p.y);
      p2_z->publish_state(p.z);
      p2_vx->publish_state(p.vx);
      p2_vy->publish_state(p.vy);
      p2_vz->publish_state(p.vz);
    } else if (index == 2) {
      p3_x->publish_state(p.x);
      p3_y->publish_state(p.y);
      p3_z->publish_state(p.z);
      p3_vx->publish_state(p.vx);
      p3_vy->publish_state(p.vy);
      p3_vz->publish_state(p.vz);
    }
  }
};

const uint8_t SensyTwoComponent::HEADER[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

}  // namespace sensytwo
}  // namespace esphome

#endif  // SENSY_TWO_COMPONENT_H
