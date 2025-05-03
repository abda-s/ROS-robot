#include <Arduino.h>
#include <string.h>
#include <Wire.h> // Required for I2C communication
#include <LiquidCrystal_I2C.h> // Required for I2C LCD

#define TERMINATOR_1 0xAA // 170 in dec
#define TERMINATOR_2 0x55 // 85 in dec

// Changed from float to double (8 bytes instead of 4)
const size_t DOUBLE_SIZE = sizeof(double);
const size_t PACKET_SIZE = DOUBLE_SIZE + 2; // Size of double + 2 terminators

// --- LCD Configuration ---
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
const int LCD_I2C_ADDRESS = 0x27;

// Initialize the LCD object
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

double readFromMatlab();
void writeDataToMatlab(double number);

// Increased buffer size to accommodate larger packets
const size_t READ_BUFFER_SIZE = PACKET_SIZE * 4;
byte readBuffer[READ_BUFFER_SIZE];
size_t readBufferIndex = 0; // Current number of bytes in the buffer

void setup() {
  Serial.begin(115200);
  
  // --- Initialize LCD ---
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for data...");
}

void loop() {
  static double counter = 0.0;
  
  double value = readFromMatlab();
  if (!isnan(value)) {
    // Data received successfully, print to Serial Monitor and LCD
    Serial.print("Received double: ");
    Serial.println(value, 6);
    
    // --- Display on LCD ---
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Received:");
    lcd.setCursor(0, 1);
    
    // Format the number for display
    // Use dtostrf for better formatting
    char buffer[16];
    dtostrf(value, 8, 2, buffer); // value, min_width, decimal_places, char_buffer
    lcd.print(buffer);
  }
  
  // Increment the counter and send it
  writeDataToMatlab(counter);
  counter += 0.1;
  if (counter > 1000.0) {
    counter = 0.0;
  }
  
  delay(10);
}

double readFromMatlab() {
  // Read available bytes into the buffer
  while (Serial.available() && readBufferIndex < READ_BUFFER_SIZE) {
    byte incomingByte = Serial.read();
    readBuffer[readBufferIndex++] = incomingByte;
  }
  
  // Search for a complete packet in the buffer
  for (size_t i = 0; i + PACKET_SIZE <= readBufferIndex; ++i) {
    // Check if we have a valid packet with terminators in the right position
    if (readBuffer[i + DOUBLE_SIZE] == TERMINATOR_1 && 
        readBuffer[i + DOUBLE_SIZE + 1] == TERMINATOR_2) {
      
      // Found a valid packet, extract the double value
      double result;
      memcpy(&result, &readBuffer[i], DOUBLE_SIZE);
      
      // Move any remaining bytes to the beginning of the buffer
      size_t remainingBytes = readBufferIndex - (i + PACKET_SIZE);
      if (remainingBytes > 0) {
        memmove(readBuffer, &readBuffer[i + PACKET_SIZE], remainingBytes);
      }
      readBufferIndex = remainingBytes;
      
      return result;
    }
  }
  
  // If buffer is full but no valid packet found, discard oldest data
  if (readBufferIndex >= READ_BUFFER_SIZE - 10) {
    const size_t discard_amount = READ_BUFFER_SIZE / 2;
    memmove(readBuffer, &readBuffer[discard_amount], readBufferIndex - discard_amount);
    readBufferIndex -= discard_amount;
  }
  
  return NAN;  // No complete packet found
}

void writeDataToMatlab(double number) {
  byte doubleBytes[DOUBLE_SIZE];
  memcpy(doubleBytes, &number, DOUBLE_SIZE);
  Serial.write(doubleBytes, DOUBLE_SIZE);
  Serial.write(TERMINATOR_1);
  Serial.write(TERMINATOR_2);
}