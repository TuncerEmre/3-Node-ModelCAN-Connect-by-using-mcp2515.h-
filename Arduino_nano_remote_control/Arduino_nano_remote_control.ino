#include <SPI.h> // Include the SPI library for communication
#include <mcp2515.h> // Include the MCP2515 CAN controller library

// Define a CAN frame structure
struct can_frame canMsg;
// Create an MCP2515 object with CS pin 10
MCP2515 mcp2515(10);

// Function prototypes
int readAndReturnHighPin();
void sendCanMessage(uint32_t id, uint8_t length, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);

int previous_high_pin = -1; // To store the previous high pin state
unsigned long lastStateChangeTime = 0; // To store the last time the state changed
const unsigned long stateChangeInterval = 200; // Interval between state changes in milliseconds

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  SPI.begin(); // Initialize SPI communication
  mcp2515.reset(); // Reset the MCP2515 CAN controller
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Set the CAN bitrate to 500 kbps
  mcp2515.setNormalMode(); // Set the MCP2515 to normal mode
  Serial.println("Transmitter initialized"); // Print initialization message

  // Set pins 4, 5, 6, and 7 as input
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  int high_pin = readAndReturnHighPin(); // Read and return the high pin

  // Check if the state change interval has elapsed and the high pin is not 1
  if (currentTime - lastStateChangeTime >= stateChangeInterval && high_pin != 1) {
    // Send a CAN message with the high pin value
    sendCanMessage(0x043, 8, high_pin, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01);
    previous_high_pin = high_pin; // Update the previous high pin value
    lastStateChangeTime = currentTime; // Update the last state change time
  }
}

void sendCanMessage(uint32_t id, uint8_t length, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {
  canMsg.can_id  = id; // Set the CAN message ID
  canMsg.can_dlc = length; // Set the data length code
  canMsg.data[0] = data0; // Set the first data byte
  canMsg.data[1] = data1; // Set the second data byte
  canMsg.data[2] = data2; // Set the third data byte
  canMsg.data[3] = data3; // Set the fourth data byte
  canMsg.data[4] = data4; // Set the fifth data byte
  canMsg.data[5] = data5; // Set the sixth data byte
  canMsg.data[6] = data6; // Set the seventh data byte
  canMsg.data[7] = data7; // Set the eighth data byte

  // Print the data being sent
  Serial.print("Sending Data: ");
  for (int i = 0; i < canMsg.can_dlc; i++) {
    Serial.print(canMsg.data[i], HEX); // Print data in hexadecimal format
    Serial.print(" ");
  }
  Serial.println();

  mcp2515.sendMessage(&canMsg); // Send the CAN message
}

int readAndReturnHighPin() {
  // Loop through pins 4 to 7
  for (int pin = 4; pin <= 7; pin++) {
    if (digitalRead(pin) == HIGH) { // Check if the pin is high
      return pin; // Return the pin number if it is high
    }
  }
  return 1; // Return 1 if no pins are high
}
