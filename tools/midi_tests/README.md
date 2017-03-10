# MIDI Tests

This folder contains a small number of tests used to verify BLE MIDI features.

## Setup

First, you will need the current stable version of [Node.js][node] installed.

```console
$ node -v
v7.4.0
```

Next, cd to `tools/midi_tests` and run `npm install`.

```console
$ cd tools/midi_tests
$ npm install
```

## RX Test

This will test receiving MIDI messages on the nRF52 feather.

* Upload the `rx_arduino` example to your nRF52 feather.
* Open the serial monitor, and connect to the feather on OS X using Audio MIDI Setup.
* Once you are connected, navigate to the `tools/midi_tests` folder and run the `rx.js` test.

```console
$ cd tools/midi_tests
$ node rx.js
```

The test will ask how fast you would like to send messages. The default is 100ms. Enter a number and press return.

```
? Send interval (ms) 100
```

Next, the test will ask you to select a MIDI port. Select the Bluefruit52 from the list.

```
? Select port Bluefruit52 Bluetooth
```

The test will begin sending messages at the specified interval. It will send 128 note on and 128 note off messages.

```
note on  [===============================================================] 128/128
note off [===============================================================] 128/128
```

Check the Arduino Serial monitor and confirm that you have received all 128 note on and 128 note off messages.

```
note on count: 128
note off count: 128
```

If you have 128 of each, the test passed.

## TX Test

This will test sending MIDI messages from the nRF52 feather.

* Upload the `tx_arduino` example to your nRF52 feather.
* Open the serial monitor, and connect to the feather on OS X using Audio MIDI Setup.
* Once you are connected, navigate to the `tools/midi_tests` folder and run the `tx.js` test.

```console
$ cd tools/midi_tests
$ node tx.js
```

The test will ask you to select a MIDI port. Select the Bluefruit52 from the list.

```
? Select port Bluefruit52 Bluetooth
```

In the Arduino Serial Monitor, you will see a message like the one seen below. Press **space** and then **return** to start the test.

```
Start the node TX test and press a key in the serial monitor to start the test...
```

The nRF52 will now start sending MIDI messages to the test. Check the counters and make sure you receive all 128 note on and 128 note off messages.

```
$ node tx.js
? Select port Bluefruit52 Bluetooth
note on  [===============================================================] 128/128
note off [===============================================================] 128/128
```

If you have 128 of each, the test passed.

[node]: https://nodejs.org/
