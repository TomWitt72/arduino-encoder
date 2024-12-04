// Author: jgronemeyer 2024

#include <Encoder.h>

// Define the encoder pins
Encoder rotary(2, 3);

// Initialize variables
long previousPosition = 0;
long currentPosition;
long positionChange;
unsigned long lastSampleTime = 0;
const unsigned long sampleWindow = 100; // Sample window in milliseconds

void setup() {
  // Initialize serial communication at a baud rate of 57600
  Serial.begin(57600);
}

void loop() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if it's time to sample
  if (currentMillis - lastSampleTime >= sampleWindow) {
    // Read the current position of the encoder
    currentPosition = rotary.read();

    // Calculate the change in position
    positionChange = currentPosition - previousPosition;

    // Optionally, check for unusually large position changes
    const long MAX_POSITION_CHANGE = 1000; // Adjust based on expected max change
    if (abs(positionChange) > MAX_POSITION_CHANGE) {
      // Handle or ignore the unexpected large change
      Serial.println("Warning: Unusually large position change detected.");
      positionChange = 0; // Ignore the change or set to zero
    } else {
      // Print the position change to the serial monitor
      Serial.println(positionChange);
    }

    // Update the previous position
    previousPosition = currentPosition;

    // Update the last sample time
    lastSampleTime = currentMillis;
  }
}
