Fan Control Project using PIC16F877A

This project controls a fan automatically or manually using a PIC16F877A microcontroller.

A DHT11 sensor is used to read temperature and humidity.

A 16x2 LCD shows the current temperature, humidity, and mode (AUTO or MANUAL).

A fan is turned ON or OFF depending on the mode:

In AUTO mode, the fan turns ON if the temperature is above 26°C.

In MANUAL mode, you can control the fan by sending commands through Bluetooth (HC-06).

You can send commands like:

a → AUTO mode

m → MANUAL mode

1 → Fan ON (only in MANUAL mode)

0 → Fan OFF (only in MANUAL mode)

RGB LEDs are used to show temperature status:

Green LED = Cool (≤ 26°C)

Red LED = Hot (> 26°C)

This project is useful for basic home automation and learning embedded systems.

