#include <SPI.h> // Include the SPI library for SPI communication
#include <mcp2515.h> // Include the MCP2515 CAN controller library
#include <Wire.h> // Include the Wire library for I2C communication
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library for the LCD display

struct can_frame CanMsg; // Define a CAN frame structure
MCP2515 mcp2515(10); // Create an MCP2515 object with CS pin 10

int ReceivedMessages_sensors[8]; // Array to store sensor values
int ReceivedMessages_nano[8]; // Array to store controller values
char motor_status[10]; // String to store the motor status
int previous_ReceivedMessage = -1; // Variable to store the previous received message
int sameMessageCount = 0; // Counter for consecutive same messages
int counter=0; // General purpose counter
int speed_calculated = 255; // Calculated speed

// Engine control pins
const int Engine_Vcc_1 = 4; // Define the VCC pin for motor 1
const int Engine_Gnd_1 = 5; // Define the GND pin for motor 1
const int Engine_Vcc_2 = 6; // Define the VCC pin for motor 2
const int Engine_Gnd_2 = 7; // Define the GND pin for motor 2
const int speed_control_1 = 9; // Define the speed control pin for motor 1
const int speed_control_2 = 3; // Define the speed control pin for motor 2

// Function declarations for engine controls
void forward(int);
void stop(int);
void left(int);
void right(int);
void backward(int);
void printToLCD(int, int, int, float, int, int);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize the LCD with I2C address 0x27 and 16x2 characters

unsigned long lastReceiveTime = 0; // Variable to store the last receive time
const unsigned long receiveInterval = 100; // Interval between receiving CAN messages in milliseconds

unsigned long lastCleanTime = 0; // Variable to store the last clean time
const unsigned long cleanInterval = 1105; // Interval for cleaning data in milliseconds

unsigned long lastStateChangeTime = 0; // Variable to store the last state change time
const unsigned long stateChangeInterval = 500; // Interval for changing state in milliseconds

int previousReceivedMessage = -1; // Variable to store the previous received message

float speed; // Variable to store the speed
int rpm; // Variable to store the RPM
const int wheelDiameter = 70; // Diameter of the wheel in mm

// Function declaration for receiving messages
void ReceiveMessages(int CAN_ID, int, int *data_array_1, int *data_array_2, int speed_calculated);

void setup() {
  // Set motor control pins as outputs
  pinMode(Engine_Vcc_1, OUTPUT);
  pinMode(Engine_Gnd_1, OUTPUT);
  pinMode(Engine_Vcc_2, OUTPUT);
  pinMode(Engine_Gnd_2, OUTPUT);
  pinMode(speed_control_1, OUTPUT);
  pinMode(speed_control_2, OUTPUT);
  
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the LCD backlight

  Serial.begin(9600); // Initialize serial communication at 9600 baud
  SPI.begin(); // Initialize SPI communication
  mcp2515.reset(); // Reset the MCP2515 CAN controller
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Set the CAN bitrate to 500 kbps
  mcp2515.setNormalMode(); // Set the MCP2515 to normal mode
  Serial.println("Receiver initialized"); // Print initialization message
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  if (currentTime - lastCleanTime >= cleanInterval) { // Check if it's time to clean data
    lastCleanTime = currentTime; // Update the last clean time
    //cleanData(); // Call the clean data function
  }

  if (currentTime - lastReceiveTime >= receiveInterval) { // Check if it's time to receive messages
    lastReceiveTime = currentTime; // Update the last receive time
    ReceiveMessages(0x43, 0x41, ReceivedMessages_nano, ReceivedMessages_sensors, speed_calculated); // Call the receive messages function
  }
  
  rpm = ReceivedMessages_sensors[2] * 2; // Calculate RPM from received sensor data
  speed = rpm * 3.1416 * wheelDiameter * 60 / 1000000; // Calculate speed

  printToLCD(ReceivedMessages_sensors[0], ReceivedMessages_sensors[1], rpm, speed, 0, 0); // Print values to LCD

  delay(100); // Delay for a short period
}

void ReceiveMessages(int CAN_ID_1, int CAN_ID_2, int *data_array_1, int *data_array_2, int speed_calculated) {
  if (mcp2515.readMessage(&CanMsg) == MCP2515::ERROR_OK) { // Check if a CAN message is received successfully
    if (CanMsg.can_id == CAN_ID_1) { // Check if the CAN ID matches CAN_ID_1
      Serial.println("Message received successfully"); // Print success message
      
      int currentReceivedMessage = CanMsg.data[0]; // Get the first byte of the received message
      Serial.println(currentReceivedMessage); // Print the received message
      
      if (counter >= 1 && currentReceivedMessage == previousReceivedMessage) { // Check if the received message is the same as the previous one
        stop(); // Stop the motors
        strcpy(motor_status, "STOP"); // Update motor status
        Serial.println("Received same message, stopping"); // Print stopping message
        counter = 0; // Reset counter
        previousReceivedMessage = currentReceivedMessage; // Update previous received message
      } else if (counter == 0 || currentReceivedMessage != previousReceivedMessage) { // Check if the message is different
        switch (currentReceivedMessage) { // Handle the received message
          case 6:
            forward(speed_calculated - 20); // Move forward
            Serial.println("FORWARD"); // Print forward message
            strcpy(motor_status, "forward"); // Update motor status
            counter = counter + 1; // Increment counter
            previousReceivedMessage = currentReceivedMessage; // Update previous received message
            break;
          case 4:
            backward(speed_calculated / 2); // Move backward
            Serial.println("backward"); // Print backward message
            strcpy(motor_status, "backward"); // Update motor status
            counter = counter + 1; // Increment counter
            previousReceivedMessage = currentReceivedMessage; // Update previous received message
            break;
          case 7:
            left(speed_calculated / 3); // Turn left
            Serial.println("left"); // Print left message
            strcpy(motor_status, "left"); // Update motor status
            counter = counter + 1; // Increment counter
            previousReceivedMessage = currentReceivedMessage; // Update previous received message
            break;
          case 5:
            right(speed_calculated / 3); // Turn right
            Serial.println("right"); // Print right message
            strcpy(motor_status, "right"); // Update motor status
            counter = counter + 1; // Increment counter
            previousReceivedMessage = currentReceivedMessage; // Update previous received message
            break;
          default:
            stop(); // Stop the motors
            strcpy(motor_status, "STOP"); // Update motor status
            Serial.println("Unknown command received, stopping"); // Print stopping message
            break;
        }
      }
    } else if (CanMsg.can_id == CAN_ID_2) { // Check if the CAN ID matches CAN_ID_2
      Serial.println("Message received successfully"); // Print success message
      Serial.print("Data received: ");
      Serial.print(" ");
      Serial.print("CAN ID 2 = ");
      Serial.print(CAN_ID_2);
      Serial.print(" ");
      for (int i = 0; i < CanMsg.can_dlc; i++) { // Loop through the received data
        data_array_2[i] = CanMsg.data[i]; // Store the received data in the array
        Serial.print(data_array_2[i]); // Print the received data
        Serial.print(" ");
      }
      Serial.println(); // Print a new line
    } else {
      Serial.println("Error receiving message"); // Print error message
    }
    delay(100); // Short delay to synchronize with the sender
  }
}

void forward(int speed_calculated) {
  digitalWrite(Engine_Vcc_1, HIGH); // Set motor 1 to move forward
  digitalWrite(Engine_Gnd_1, LOW); // Set motor 1 ground
  digitalWrite(Engine_Vcc_2, HIGH); // Set motor 2 to move forward
  digitalWrite(Engine_Gnd_2, LOW); // Set motor 2 ground
  analogWrite(speed_control_1, (speed_calculated)); // Control speed of motor 1
  analogWrite(speed_control_2, (speed_calculated)); // Control speed of motor 2
}

void stop() {
  digitalWrite(Engine_Vcc_1, LOW); // Stop motor 1
  digitalWrite(Engine_Gnd_1, LOW); // Stop motor 1
  digitalWrite(Engine_Vcc_2, LOW); // Stop motor 2
  digitalWrite(Engine_Gnd_2, LOW); // Stop motor 2
}

void left(int speed_calculated) {
  digitalWrite(Engine_Vcc_1, LOW); // Set motor 1 to move backward
  digitalWrite(Engine_Gnd_1, HIGH); // Set motor 1 ground
  digitalWrite(Engine_Vcc_2, HIGH); // Set motor 2 to move forward
  digitalWrite(Engine_Gnd_2, LOW); // Set motor 2 ground
  analogWrite(speed_control_1, (speed_calculated)); // Control speed of motor 1
  analogWrite(speed_control_2, (speed_calculated)); // Control speed of motor 2
}

void right(int speed_calculated) {
  digitalWrite(Engine_Vcc_1, HIGH); // Set motor 1 to move forward
  digitalWrite(Engine_Gnd_1, LOW); // Set motor 1 ground
  digitalWrite(Engine_Vcc_2, LOW); // Set motor 2 to move backward
  digitalWrite(Engine_Gnd_2, HIGH); // Set motor 2 ground
  analogWrite(speed_control_1, (speed_calculated)); // Control speed of motor 1
  analogWrite(speed_control_2, (speed_calculated)); // Control speed of motor 2
}

void backward(int speed_calculated) {
  digitalWrite(Engine_Vcc_1, LOW); // Set motor 1 to move backward
  digitalWrite(Engine_Gnd_1, HIGH); // Set motor 1 ground
  digitalWrite(Engine_Vcc_2, LOW); // Set motor 2 to move backward
  digitalWrite(Engine_Gnd_2, HIGH); // Set motor 2 ground
  analogWrite(speed_control_1, (speed_calculated)); // Control speed of motor 1
  analogWrite(speed_control_2, (speed_calculated)); // Control speed of motor 2
}

void printToLCD(int distance1, int distance2, int rpm, float speed, int x, int y) {
  lcd.clear(); // Clear the LCD display
  lcd.setCursor(x, y); // Set cursor to the specified position
  lcd.print("DisF/B = "); // Print "DisF/B = " on the LCD
  lcd.print(distance1); // Print the first distance value
  lcd.print("/ ");
  lcd.print(distance2); // Print the second distance value

  lcd.setCursor(0, 1); // Move the cursor to the second line
  lcd.print("RPM "); // Print "RPM " on the LCD
  lcd.print(rpm); // Print the RPM value
  lcd.print(" ");
  lcd.print("S "); // Print "S " on the LCD
  lcd.print(speed); // Print the speed value
}

void cleanData() {
  // Clear the data arrays for sensors and controller values
  for (int i = 0; i < 8; i++) {
    ReceivedMessages_sensors[i] = 0; // Clear the sensor values array
    ReceivedMessages_nano[i] = 0; // Clear the controller values array
  }
}
