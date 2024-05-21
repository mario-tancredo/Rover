#include <CRC32.h>

#define M0 2
#define M1 3

#define THROTTLE_PIN 0
#define DIRECTION_PIN 1
#define CAMERA_PIN 3

// Define the struct to hold the parameters
struct RoverParameters {
  int Throttle;
  int Direction;
  int Camera;
};

RoverParameters parameters;

CRC32 crc; // Initialize CRC32 object

void setup() {
  // Pins for transmitting mode
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Transmitting mode
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  // Send a number over Serial every second
  
  // Read the parameters
  parameters.Throttle = analogRead(THROTTLE_PIN);
  parameters.Direction = analogRead(DIRECTION_PIN);
  parameters.Camera = analogRead(CAMERA_PIN);

  // Send the parameters over UART
  Serial.write(0xAA); // Header byte indicating start of packet
  Serial.write((uint8_t*)&parameters, sizeof(parameters)); //Payload
  uint32_t checksum = crc.calculate((uint8_t*)&parameters, sizeof(parameters)); // Calculate CRC32 checksum
  Serial.write((uint8_t*)&checksum, sizeof(checksum)); // Checksum

  //Serial.println("");

  // Wait for 1 second before sending the next number
  delay(100);
}
