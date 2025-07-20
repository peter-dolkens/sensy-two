# Sensy-Two ESPHome Component

This repository contains an example ESPHome component to interface with the
Sensy-Two mmWave radar sensor. The implementation is derived from an Arduino
example and parses the radar data stream to publish information about up to
three detected targets.

## Component

`src/components/sensy_two/sensy_two_component.h` implements the parser and publishes sensor values for the
position and velocity of the first three targets reported by the sensor. The
component sends the initial configuration commands used in the Arduino sketch
and then continuously reads the UART stream to decode TLV frames.

To use this component in an ESPHome configuration, copy the header into your
project and include it in your YAML file:

```yaml
esphome:
  includes:
    - src/components/sensy_two/sensy_two_component.h

external_components:
  - source:
      type: local
      path: components

uart:
  id: ms72sf1
  baud_rate: 115200
  tx_pin: 17
  rx_pin: 16

sensy_two:
  id: sensy_component
  uart_id: ms72sf1
  # Optional sensor entities
  radar_firmware:
    name: "RADAR | Firmware"
  radar_mac:
    name: "RADAR | MAC"
```

This will create sensors for each detected target including position (x and y),
angle, speed and distance. The example configuration also exposes number
entities to configure up to three detection zones and an optional exclusion
zone. Template sensors calculate zone presence, movement and target counts
matching the Sensy-One reference implementation.
