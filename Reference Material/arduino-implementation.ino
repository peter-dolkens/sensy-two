#include <HardwareSerial.h>

#define RADAR_RX 16
#define RADAR_TX 17
#define RADAR_BAUD 115200
#define RADAR_BUFFER_SIZE 128
#define RING_BUFFER_SIZE 10240

uint32_t maxpoints = 1000;
uint32_t maxpersons = 10;

HardwareSerial RadarSerial(1);

const uint8_t HEADER[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

enum ParseState {
  SEARCHING_HEADER,
  READING_LENGTH,
  READING_TLV_LENGTH,
  READING_POINTS,
  READING_PERSONS
};

ParseState state = SEARCHING_HEADER;

uint8_t lengthBuf[8];
uint8_t tlvHeaderBuf[8];
uint32_t expectedLength = 0;
uint32_t frameNo = 0;
uint32_t frameBytesRemaining = 0;
uint32_t tlv1Type = 0;
uint32_t tlv1Len = 0;
uint32_t bytesRead = 0;
uint32_t itemIndex = 0;

bool showPoints = false;
bool showPersons = true;

struct Point {
  float x, y, z;
  int8_t v;
  float snr;
  float pow;
  float dpk;
} __attribute__((packed));

struct Person {
  uint32_t id;
  uint32_t q;
  float x, y, z;
  float vx, vy, vz;
  // uint8_t reserved[8];
} __attribute__((packed));

// ISR-safe ring buffer
volatile uint8_t ringBuffer[RING_BUFFER_SIZE];
volatile size_t ringHead = 0;
volatile size_t ringTail = 0;
volatile size_t ringOverflows = 0;

void IRAM_ATTR writeRingBulkISR(const uint8_t *src, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    size_t nextHead = (ringHead + 1) % RING_BUFFER_SIZE;
    if (nextHead != ringTail) {
      ringBuffer[ringHead] = src[i];
      ringHead = nextHead;
    } else {
      ringOverflows++;
      break;  // Avoid partial writes beyond capacity
    }
  }
}

void IRAM_ATTR writeRingISR(uint8_t b) {
  size_t nextHead = (ringHead + 1) % RING_BUFFER_SIZE;
  if (nextHead != ringTail) {
    ringBuffer[ringHead] = b;
    ringHead = nextHead;
  } else {
    ringOverflows++;
  }
}

bool readRing(uint8_t *buf, size_t len) {
  size_t available = (ringHead + RING_BUFFER_SIZE - ringTail) % RING_BUFFER_SIZE;
  if (available < len) return false;

  size_t firstChunk = min(len, RING_BUFFER_SIZE - ringTail);
  memcpy(buf, (const void *)&ringBuffer[ringTail], firstChunk);
  if (len > firstChunk) {
    memcpy(buf + firstChunk, (const void *)&ringBuffer[0], len - firstChunk);
  }

  ringTail = (ringTail + len) % RING_BUFFER_SIZE;
  return true;
}

bool peekRing(uint8_t *buf, size_t len) {
  size_t available = (ringHead + RING_BUFFER_SIZE - ringTail) % RING_BUFFER_SIZE;
  if (available < len) return false;

  size_t firstChunk = min(len, RING_BUFFER_SIZE - ringTail);
  memcpy(buf, (const void *)&ringBuffer[ringTail], firstChunk);
  if (len > firstChunk) {
    memcpy(buf + firstChunk, (const void *)&ringBuffer[0], len - firstChunk);
  }

  return true;
}

void IRAM_ATTR onRadarSerial() {
  static uint8_t tempBuf[RADAR_BUFFER_SIZE];
  int available = RadarSerial.available();
  if (available > 0) {
    size_t toRead = min((size_t)available, sizeof(tempBuf));
    size_t bytesRead = RadarSerial.readBytes(tempBuf, toRead);
    writeRingBulkISR(tempBuf, bytesRead);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);

  Serial.println("Connecting to mmWave...");

  RadarSerial.printf("AT+RESET\n"); delay(100);
  RadarSerial.printf("AT+START\n"); delay(100);
  RadarSerial.printf("AT+TIME=200\n"); delay(100);   // default 100
  RadarSerial.printf("AT+MONTIME=1\n"); delay(100);  // default 1
  RadarSerial.printf("AT+HEATIME=10\n"); delay(100); // default 60
  RadarSerial.printf("AT+SENS=2\n"); delay(100);    // default 2
  RadarSerial.printf("AT+SEEKING\n"); delay(100);
  RadarSerial.printf("AT+SETTING\n"); delay(100);

  RadarSerial.setRxFIFOFull(RADAR_BUFFER_SIZE); // Trigger interrupt when 1+ byte received
  RadarSerial.onReceive(onRadarSerial);         // Attach ISR

  Serial.println("mmWave sensor setup complete");
  Serial.print("\033[2J\033[H");  // Clear screen
}

void loop() {
  switch (state) {
    case SEARCHING_HEADER: {
      uint8_t peekBuf[sizeof(HEADER)];
      while (peekRing(peekBuf, sizeof(HEADER))) {
        if (memcmp(peekBuf, HEADER, sizeof(HEADER)) == 0) {
          uint8_t discard[sizeof(HEADER)];
          readRing(discard, sizeof(HEADER));
          state = READING_LENGTH;
          break;
        } else {
          uint8_t discard;
          readRing(&discard, 1);
        }
      }
      break;
    }

    case READING_LENGTH:
      if (readRing(lengthBuf, 8)) {
        expectedLength = *((uint32_t *)lengthBuf);
        frameNo = *((uint32_t *)lengthBuf + 1);
        frameBytesRemaining = expectedLength - 8; // subtract frame number bytes

        state = (expectedLength < 12) ? SEARCHING_HEADER : READING_TLV_LENGTH;
      }
      break;

    case READING_TLV_LENGTH:
      if (readRing(tlvHeaderBuf, 8)) {
        tlv1Type = *(uint32_t *)(tlvHeaderBuf);
        tlv1Len = *(uint32_t *)(tlvHeaderBuf + 4);
        frameBytesRemaining -= 8;
        bytesRead = 0;
        itemIndex = 0;

        if (tlv1Type == 0x01) {
          state = READING_POINTS;
        } else if (tlv1Type == 0x02) {
          state = READING_PERSONS;
        } else {
          state = SEARCHING_HEADER;
        }
      }
      break;

    case READING_POINTS:
      if ((ringHead + RING_BUFFER_SIZE - ringTail) % RING_BUFFER_SIZE >= sizeof(Point)) {
        if (showPoints) {
          Point pointBuf;
          readRing((uint8_t *)&pointBuf, sizeof(Point));
          bytesRead += sizeof(Point);

          if (itemIndex == 0) Serial.printf("> Points in cloud:\033[K\r\n");
          if (itemIndex < maxpoints) {
            Serial.printf("  [%u] (x=%.2f, y=%.2f, z=%.2f), v=%d, SNR=%.2f, POW=%.2f, DPK=%.2f\033[K\r\n",
              itemIndex, pointBuf.x, pointBuf.y, pointBuf.z, pointBuf.v,
              pointBuf.snr, pointBuf.pow, pointBuf.dpk);
          }
        } else {
          uint8_t discard[sizeof(Point)];
          readRing(discard, sizeof(Point));
          bytesRead += sizeof(Point);
        }

        itemIndex++;
        if (bytesRead >= tlv1Len) {
          frameBytesRemaining -= tlv1Len;
          state = (frameBytesRemaining >= 8) ? READING_TLV_LENGTH : SEARCHING_HEADER;
        }
      }
      break;

    case READING_PERSONS:
      if ((ringHead + RING_BUFFER_SIZE - ringTail) % RING_BUFFER_SIZE >= sizeof(Person)) {
        if (showPersons) {
          if (bytesRead == 0) Serial.printf("> Detected persons:\033[K\r\n");

          Person personBuf;
          readRing((uint8_t *)&personBuf, sizeof(Person));
          bytesRead += sizeof(Person);

          if (itemIndex < maxpersons) {
            Serial.printf("  [%u:%u] ID %u, Q %u, Pos (%.2fm, %.2fm, %.2fm), Velocity (%.2f, %.2f, %.2f)\033[K\r\n",
              frameNo, itemIndex, personBuf.id, personBuf.q, personBuf.x, personBuf.y, personBuf.z,
              personBuf.vx, personBuf.vy, personBuf.vz);
          }
        } else {
          uint8_t discard[sizeof(Person)];
          readRing(discard, sizeof(Person));
          bytesRead += sizeof(Person);
        }

        itemIndex++;
        if (bytesRead >= tlv1Len) {
          frameBytesRemaining -= tlv1Len;
          state = (frameBytesRemaining >= 8) ? READING_TLV_LENGTH : SEARCHING_HEADER;
        }
      }
      break;
  }

  // Handle serial input (command toggles)
  while (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);
    // RadarSerial.write(c);

    if (c == 'q') {
      showPersons = true;
      showPoints = false;
      Serial.println("\r\n[MODE] Persons only");
    } else if (c == 'w') {
      showPersons = false;
      showPoints = true;
      Serial.println("\r\n[MODE] Points only");
    } else if (c == 'e') {
      showPersons = true;
      showPoints = true;
      Serial.println("\r\n[MODE] Showing all");
    } else if (c == 'r') {
      Serial.printf("\r\n[STATE] Current parser state: %s", stateToString(state));
    }
  }

  // Optional overflow check
  if (ringOverflows > 0) {
    Serial.printf("[WARN] Ring buffer overflowed %u times\r\n", ringOverflows);
    ringOverflows = 0;
  }

  yield();
}

const char* stateToString(ParseState s) {
  switch (s) {
    case SEARCHING_HEADER:   return "SEARCHING_HEADER";
    case READING_LENGTH:     return "READING_LENGTH";
    case READING_TLV_LENGTH: return "READING_TLV_LENGTH";
    case READING_POINTS:     return "READING_POINTS";
    case READING_PERSONS:    return "READING_PERSONS";
    default:                 return "UNKNOWN_STATE";
  }
}
