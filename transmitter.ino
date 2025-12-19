#include <SPI.h>
#include <RF24.h>

// --- Configuration for ADXL335 ---
#define PIN_X   A0
#define PIN_Y   A1

  int speed_bits = 0;   // Bit [3:2]
  int steer_bits = 0;   // Bit [1:0]
  bool y_armed = true; 

RF24 radio(8, 10);              // CE=8, CSN=10
const byte address[6] = "1Node";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println("radio.begin() FAILED");
    while (1) {}
  }

  // Khớp FPGA
  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_LOW);

  // Test mode: tắt ACK để khỏi phụ thuộc receiver trả ACK
  radio.setAutoAck(false);

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("TX ready");
}

void loop() {
  // --- Get data from hand gestures ---
  int xVal = analogRead(PIN_X);
  int yVal = analogRead(PIN_Y);

  // --- Instruction Code (Command) ---

  if (yVal >= 300 && yVal <= 420) {
  y_armed = true;
  }
  // ---------------------------------------------
  // 2. SPEED PROCESSING (Based on the Y-axis - Forward/Backward Tilt)
  // Logic: 00=Stop, 01=Normal, 10=Fast, 11=Reverse
  // ---------------------------------------------
  
  if (yVal > 420) { 
    // Tilt Forward -> GO FORWARD
    // (Max Tilt for Turbo Mode)
    if (speed_bits < 2) {
       speed_bits ++; // 10: Turbo (Fast)
    } else if (speed_bits == 3) speed_bits = 0;
    y_armed = false;
  } 
  else if (yVal < 300) {
    // Tilt Backward -> GO BACKWARD
    if (speed_bits == 0) {
      speed_bits = 3;
    } else if (speed_bits != 3) speed_bits--;
    y_armed = false;
  } 

  // ---------------------------------------------
  // 3. STEERING PROCESSING (Based on the X-axis - Left/Right Tilt)
  // Logic: 00=Forward, 10=Left, 01=Right
  // ---------------------------------------------
  
  if (xVal < 300) {
    // Tilt to the left
    steer_bits = 2; // 10: Left
  } 
  else if (xVal > 400) {
    // Tilt to the right
    steer_bits = 1; // 01: Right
  } 
  else {
    // None -> GO FORWARD
    steer_bits = 0; // 00: Straight
  }

  // ---------------------------------------------
  // 4. Pack & Send (Payload[0])
  // ---------------------------------------------
  uint8_t payload[32] = {0};

  // Bitwise Matching: Speed [2 bits] - Steer [2 bits]
  uint8_t command = (speed_bits << 2) | steer_bits;

  payload[0] = command;

  radio.write(payload, 32);

  // --- DEBUG SECTION ---
  Serial.print("Tilt X: ");
  Serial.print(xVal);
  Serial.print(" | Tilt Y: ");
  Serial.print(yVal);
  Serial.print(" -> CMD: "); 
  if (command < 8) Serial.print('0');   // đảm bảo đủ 4 bit (0..15)
  if (command < 4) Serial.print('0');
  if (command < 2) Serial.print('0');
  Serial.println(command, BIN);

  delay(100);
}
