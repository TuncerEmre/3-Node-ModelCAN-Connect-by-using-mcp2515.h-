CAN Bus Communication Project


This project demonstrates CAN bus communication between multiple nodes using the MCP2515 module. The system is implemented with the Arduino IDE and C++.

Overview
The project consists of three main nodes:

Arduino Nano Remote Control Node

Receives data from an RF wireless control module.
Sends control data to the Atmega 1 Engine Control Node.
Atmega 1 Engine Control Node

Manages the motor driver (L298N).
Controls a display screen to show data from sensors.


Atmega 2 Sensors Node

Processes data from various sensors, including:


RPM Sensor (LM393)

Distance Sensor (HC-SR04)

Components Used

CAN Module: MCP2515

Libraries Used: MCP2515

Development Environment: Arduino IDE

Programming Language: C++

Installation


Clone the Repository:

git clone https://github.com/TuncerEmre/3-Node-ModelCAN-Connect-by-using-mcp2515.h-.git

Install Required Libraries:

Install the MCP2515 library from the Arduino Library Manager or by downloading it from GitHub.

Upload the Code:

-Open the Arduino IDE.

-Load the respective sketch for each node.

-Connect each microcontroller to your computer and upload the code.

Usage


Remote Control Node:

Use the RF wireless control module to send commands.

The node will transmit data to the Atmega 1 Engine Control Node via CAN bus.

Engine Control Node:

The node will drive the motor through the L298N driver.

It will also display sensor data on the screen.


Sensors Node:

Collects and processes data from the RPM sensor and distance sensor.

Sends processed data to the Engine Control Node.

Contributing

Feel free to contribute to this project by submitting pull requests or issues. For detailed contribution guidelines, please refer to CONTRIBUTING.md.

License
This project is licensed under the MIT License. See the LICENSE file for details.

Contact
For any questions or comments, please contact Emre Tuncer.

E-mail: emretuncer396@gmail.com
