#include "LumenProtocol.h"
#include <Servo.h>

// External functions for communication with Lumen
extern "C" void lumen_write_bytes(uint8_t *data, uint32_t length) {
  Serial.write(data, length);
}

extern "C" uint16_t lumen_get_byte() {
  if (Serial.available()) {
    return Serial.read();
  }
  return DATA_NULL;
}

#define LCM_BAUDRATE 115200  // Data transmission rate for the display with Arduino

// Servo initialization
Servo ServoMotor;

// Configuration constants
const int SERVO_PIN = 9;    // Servo control pin
const int ANALOG_PIN = A0;  // Analog pin for potentiometer reading

// User variable addresses
const uint16_t CURRENT_ANGLE_ADDRESS = 147;
const uint16_t CONTROL_DEVICE_SELECTOR_ADDRESS = 148;

// User variable packets
lumen_packet_t currentAnglePacket = { CURRENT_ANGLE_ADDRESS, kS16 };
lumen_packet_t controlDeviceSelectorPacket = { CONTROL_DEVICE_SELECTOR_ADDRESS, kBool };
lumen_packet_t *currentPacket;

// Global variables
int positionSetpoint = 0;     // Desired position of the servo
int basePosition = 0;         // Last servo position
bool updatePosition = false;  // Indicator for position update
bool controlSelect = false;   // Control via display (false) or potentiometer (true)

// Function to send the servo position to the display
void sendServoPosition(int angle) {
  lumen_packet_t anglePacket = { CURRENT_ANGLE_ADDRESS, kS16 };
  anglePacket.data._s16 = static_cast<int16_t>(angle);
  lumen_write_packet(&anglePacket);
}

// Function to update the servo position based on the potentiometer
void updateServoFromPotentiometer() {
  long potValue = analogRead(ANALOG_PIN);              // Read potentiometer value
  int servoPosition = map(potValue, 0, 1023, 0, 180);  // Map the value to 0-180 degrees

  ServoMotor.write(servoPosition);   // Move the servo to the new position
  sendServoPosition(servoPosition);  // Send the new position to the display

  delay(75);  // Optional delay to smooth the servo response
}

// Function to process display packets
void processDisplayPackets() {
  if (lumen_available() > 0) {
    currentPacket = lumen_get_first_packet();

    if (currentPacket->address == CONTROL_DEVICE_SELECTOR_ADDRESS) {
      controlSelect = currentPacket->data._bool;  // Update the control mode (display or potentiometer)
    } else if (currentPacket->address == CURRENT_ANGLE_ADDRESS) {
      updatePosition = true;
      positionSetpoint = currentPacket->data._s16;
      ServoMotor.write(positionSetpoint);

      if (updatePosition && basePosition != positionSetpoint) {
        basePosition = positionSetpoint;
      } else {
        updatePosition = false;
      }
    }
  }
}

// Initial setup
void setup() {
  Serial.begin(LCM_BAUDRATE);
  ServoMotor.attach(SERVO_PIN);
  updatePosition = true;
}

// Main program loop
void loop() {
  processDisplayPackets();  // Process packets received from the display

  // Continuous control from potentiometer if controlSelect is true
  if (controlSelect) {
    updateServoFromPotentiometer();
  }
}
