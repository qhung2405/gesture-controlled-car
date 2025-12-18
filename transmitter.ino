#include <SPI.h>
#include <RF24.h>

// --- Configuration for ADXL335 ---
#define PIN_X   A0
#define PIN_Y   A1

RF24 radio(8, 10);              // CE=8, CSN=10
const byte address[6] = "1Node";

// --- Threshold Configuration ---
const int X_REST = 340; // Giá trị khi để tay phẳng (nghỉ)
const int Y_REST = 340; 

// Khoảng "vùng chết" (Deadzone) để tay không bị rung
const int DEADZONE = 30; 

// Ngưỡng bắt đầu nhận lệnh
const int X_MIN = X_REST - DEADZONE; // Nghiêng phải  (Right)
const int X_MAX = X_REST + DEADZONE; // Nghiêng trái  (Left)
const int Y_MIN = Y_REST - DEADZONE; // Nghiêng sau   (Backward)
const int Y_MAX = Y_REST + DEADZONE; // Nghiêng trước (Forward)

// Giới hạn nghiêng tối đa (Max Tilt) để đạt tốc độ tối đa
const int TILT_LIMIT = 40;

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
  int speed_bits = 0;   // Bit [3:2]
  int steer_bits = 0;   // Bit [1:0]
  
  // ---------------------------------------------
  // 2. SPEED PROCESSING (Based on the Y-axis - Forward/Backward Tilt)
  // Logic: 00=Stop, 01=Normal, 10=Fast, 11=Reverse
  // ---------------------------------------------
  
  if (yVal > Y_MAX) { 
    // Tilt Forward -> GO FORWARD
    // (Max Tilt for Turbo Mode)
    if (yVal > Y_MAX + 40) {
       speed_bits = 2; // 10: Turbo (Fast)
    } else {
       speed_bits = 1; // 01: Normal
    }
  } 
  else if (yVal < Y_MIN) {
    // Tilt Backward -> GO BACKWARD
    speed_bits = 3; // 11: Reverse
  } 
  else {
    // None -> STOP
    speed_bits = 0; // 00: Stop
  }

  // ---------------------------------------------
  // 3. STEERING PROCESSING (Based on the X-axis - Left/Right Tilt)
  // Logic: 00=Forward, 10=Left, 01=Right
  // ---------------------------------------------
  
  if (xVal < X_MAX) {
    // Tilt to the left
    steer_bits = 2; // 10: Left
  } 
  else if (xVal > X_MIN) {
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
  Serial.println(command, BIN);

  delay(1000);
}
