#include <Arduino.h> // Include standard Arduino header
#include <string.h>  // Required for memcpy


#define TERMINATOR_1 0xAA // 170 in dec
#define TERMINATOR_2 0x55 // 85 in dec
const size_t FLOAT_SIZE = sizeof(float);
const size_t PACKET_SIZE = FLOAT_SIZE + 2; // Size of float + 2 terminators

float readFromMatlab();
void writeDataToMatlab(float number);

// A buffer to hold incoming serial data.
// We need a buffer large enough to potentially hold more than one packet
// if they arrive quickly, but not excessively large to avoid memory issues.
// A size that can hold a few packets should be reasonable for most applications.
// Let's make it large enough for, say, 4 packets.
const size_t READ_BUFFER_SIZE = PACKET_SIZE * 4;
byte readBuffer[READ_BUFFER_SIZE];
size_t readBufferIndex = 0; // Current number of bytes in the buffer


void setup() {
  Serial.begin(115200);
}

void loop() {
  static float counter = 0.0; // Use static to retain value across loop calls

  float value = readFromMatlab();
  if (!isnan(value)) {
    Serial.print("Received float: ");
    Serial.println(value, 6); // Print with 6 decimal places for better float representation
    // You could add code here to *do* something with the received 'value'
  }

  // Increment the counter and send it
  writeDataToMatlab(counter);
  counter += 0.1;

  // Reset counter to avoid overflow and keep values within a reasonable range
  if (counter > 1000.0) {
    counter = 0.0;
  }

  delay(10); // Small delay to prevent flooding and allow serial buffer to fill
}

float readFromMatlab() {
  while (Serial.available() && readBufferIndex < READ_BUFFER_SIZE) {
    byte incomingByte = Serial.read();
    readBuffer[readBufferIndex++] = incomingByte;
  }

  // Now, scan the buffer for the terminator sequence
  for (size_t i = 0; i + PACKET_SIZE <= readBufferIndex; ++i) {
    // Check if the terminator sequence is found
    if (readBuffer[i + FLOAT_SIZE] == TERMINATOR_1 && readBuffer[i + FLOAT_SIZE + 1] == TERMINATOR_2) {

      // Found a packet! The float data is right before the terminators.
      float result;
      // ** Endianness Consideration: **
      // memcpy copies bytes directly. If the system sending the float (Matlab)
      // has a different endianness than the Arduino, the bytes need to be
      // reversed before copying into the float variable 'result'.
      // Most PCs (running Matlab) are little-endian, and many Arduinos
      // (like Uno, Mega based on AVR) are also little-endian. If you are using
      // a different Arduino (e.g., some ARM-based boards might be big-endian),
      // you might need to reverse the bytes here.
      // For common setups (PC <-> AVR Arduino), direct memcpy usually works.
      // If you encounter strange float values, endianness is the likely culprit.
      // Example of reversing bytes (if needed):
      // byte tempBytes[FLOAT_SIZE];
      // for(int j = 0; j < FLOAT_SIZE; j++) {
      //   tempBytes[j] = readBuffer[i + FLOAT_SIZE - 1 - j]; // Reverse the order
      // }
      // memcpy(&result, tempBytes, FLOAT_SIZE);

      memcpy(&result, readBuffer + i, FLOAT_SIZE); // Copy the float bytes

      // Now, remove the processed packet from the buffer by shifting the remaining bytes
      size_t remainingBytes = readBufferIndex - (i + PACKET_SIZE);
      if (remainingBytes > 0) {
        memmove(readBuffer, readBuffer + i + PACKET_SIZE, remainingBytes);
      }
      readBufferIndex = remainingBytes; // Update the index

      return result; // Return the successfully read float
    }
  }

  // If we scanned the whole buffer and didn't find a complete packet with terminators,
  // check if the buffer is full and might contain an incomplete packet at the end.
  // If the buffer is full, we might need to discard old data to make room for new,
  // assuming the start of a packet was lost. This is a simple way to handle potential
  // misalignment if the buffer fills up completely without a terminator.
  if (readBufferIndex == READ_BUFFER_SIZE) {
      // If the buffer is full and no complete packet was found,
      // discard some old data (e.g., the first PACKET_SIZE bytes)
      // to try and resynchronize on the next incoming data.
      size_t discard_amount = PACKET_SIZE;
      if (READ_BUFFER_SIZE >= discard_amount) {
           memmove(readBuffer, readBuffer + discard_amount, READ_BUFFER_SIZE - discard_amount);
           readBufferIndex -= discard_amount;
      } else {
           // Should not happen if READ_BUFFER_SIZE >= PACKET_SIZE, but as a fallback:
           readBufferIndex = 0; // Clear the whole buffer
      }
       // Note: A more advanced approach might try to find the *next* potential
       // terminator sequence and discard up to that point.
  }


  return NAN; // Return NaN if no complete valid data packet was found
}


// Function to write a float as 4 raw bytes followed by terminators to Serial
void writeDataToMatlab(float number) {
  // Use a temporary buffer to hold the bytes
  byte floatBytes[FLOAT_SIZE];
  memcpy(floatBytes, &number, FLOAT_SIZE); // Copy float bytes into the buffer

  Serial.write(floatBytes, FLOAT_SIZE); // Write the 4 bytes
  Serial.write(TERMINATOR_1);          // Custom terminator byte 1
  Serial.write(TERMINATOR_2);          // Custom terminator byte 2
}