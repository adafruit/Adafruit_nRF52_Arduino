# RSSI Proximity Project

This project demonstrates how you can use a single Bluefruit nRF52 device
running in Central mode to listen for a specific signature in the advertising
packets of nearby Bluefruit nRF52 devices running in peripheral mode.

The Central mode device will sort those devices with the matching signature by
proximity based on their RSSI (relative signal strength) value.

Most of the Central mode advertising API is used in this demo, making it an
excellent place to start if you want to create your own custom Central based
project(s).

## Project Files

This project consists of two parts:

- `rssi_proximity_central`: This project should be run on one Bluefruit nRF52
  board and will configure the board as a Central device, actively scanning
  for any peripheral devices in listening range.
- `rssi_proximity_peripheral`: This project should be run on one or more
  Bluefruit nRF52 boards and will configure the boards as peripheral devices,
  sending out a specifically formatted advertising packet at a regular
  interval.
