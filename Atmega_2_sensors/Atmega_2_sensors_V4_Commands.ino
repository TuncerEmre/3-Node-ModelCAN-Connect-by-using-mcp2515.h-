#include <SPI.h> // Include the SPI library for SPI communication
#include <mcp2515.h> // Include the MCP2515 CAN controller library

struct can_frame canMsg; // Define a CAN frame structure
MCP2515 mcp2515(10); // Create an MCP2515 object with CS pin 10
#define echoPin1 6 // Define the echo pin for the first distance sensor
#define trigPin1 7 // Define the trigger pin for the first distance sensor
#define echoPin2 5 // Define the echo pin for the second distance sensor
#define trigPin2 4 // Define the trigger pin for the second distance sensor

int maximumRange = 50; // Maximum range for the distance sensors
int minimumRange = 0; // Minimum range for the distance sensors

unsigned long lastSendTime = 0; // Store the last time a CAN message was sent
const unsigned long sendInterval = 1000; // Interval between sending CAN messages in milliseconds

const byte LM393 = 3; // Define the pin for the speed sensor data
unsigned int rpm = 0; // Store the revolutions per minute
float speed = 0.0; // Store the speed in km/h
volatile byte counter = 0; // Counter for the speed sensor pulses
unsigned long timeOld = 0; // Store the old time for RPM calculation
unsigned int encoder = 20; // Number of holes on the encoder disk
const int wheelDiameter = 70; // Diameter of the wheel in mm
static volatile unsigned long debounce = 0; // Debounce for noise reduction

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  SPI.begin(); // Initialize SPI communication
  mcp2515.reset(); // Reset the MCP2515 CAN controller
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Set the CAN bitrate to 500 kbps
  mcp2515.setNormalMode(); // Set the MCP2515 to normal mode
  Serial.println("Transmitter initialized"); // Print initialization message

  pinMode(trigPin1, OUTPUT); // Set the trigger pin for the first distance sensor as output
  pinMode(echoPin1, INPUT); // Set the echo pin for the first distance sensor as input
  pinMode(trigPin2, OUTPUT); // Set the trigger pin for the second distance sensor as output
  pinMode(echoPin2, INPUT); // Set the echo pin for the second distance sensor as input

  pinMode(LM393, INPUT); // Set the speed sensor data pin as input
  attachInterrupt(digitalPinToInterrupt(LM393), countPulses, RISING); // Attach an interrupt to count pulses on rising edge
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  
  int distance1 = measureDistance(trigPin1, echoPin1, maximumRange, minimumRange); // Measure distance from the first sensor
  int distance2 = measureDistance(trigPin2, echoPin2, maximumRange, minimumRange); // Measure distance from the second sensor
  measureRPMandSpeed(); // Calculate RPM and speed

  if (currentTime - lastSendTime >= sendInterval) { // Check if it's time to send a CAN message
    sendCanMessage(0x041, 8, distance1, distance2, rpm, 0x01, 0x01, 0x01, 0x01, 0x01); // Send the CAN message with sensor data
    lastSendTime = currentTime; // Update the last send time
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

  Serial.print("Sending Data: "); // Print sending data message
  for (int i = 0; i < canMsg.can_dlc; i++) { // Loop through the data bytes
    Serial.print(canMsg.data[i]); // Print each data byte
    Serial.print(" "); // Print a space
  }
  Serial.println(); // Print a new line

  mcp2515.sendMessage(&canMsg); // Send the CAN message
}

int measureDistance(int trigPin, int echoPin, int maxRange, int minRange) {
  long duration, distance; // Variables for the duration and distance

  digitalWrite(trigPin, LOW); // Set the trigger pin low
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH); // Set the trigger pin high
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(trigPin, LOW); // Set the trigger pin low again

  duration = pulseIn(echoPin, HIGH); // Measure the duration of the echo
  distance = duration / 58.2; // Calculate the distance
  delay(50); // Wait for 50 milliseconds

  if (distance >= maxRange || distance <= minRange) // Check if the distance is out of range
    return 0; // Return 0 if out of range
  return distance; // Return the measured distance
}

void measureRPMandSpeed() {
  if (millis() - timeOld >= 1000) { // Check if 1 second has passed
    noInterrupts(); // Disable interrupts
    rpm = (60 * 1000 / encoder) / (millis() - timeOld) * counter; // Calculate RPM
    rpm = rpm / 2; // Adjust RPM value
    speed = rpm * 3.1416 * wheelDiameter * 60 / 1000000; // Calculate speed
    timeOld = millis(); // Update the old time
    counter = 0; // Reset the counter
    
    interrupts(); // Enable interrupts
  }
}

void countPulses() {
  if (digitalRead(LM393) && (micros() - debounce > 500) && digitalRead(LM393)) { // Check for a valid pulse and debounce
    debounce = micros(); // Update the debounce time
    counter++; // Increment the counter
  }
}
