#define M0 24
#define M1 31

// Define the struct to hold the parameters

int Throttle;
int Direction;
int Pump;


void setup() {

  // Pins for transmitting mode
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  // Initialize Serial communication
  Serial.begin(9600);
  Serial3.begin(9600);

  //RoverParameters receivedParameters;
}

void loop() {
  // Receiving mode
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  // Check if data is available to read


  if (Serial.available() > 0) {
    // Read the incoming line
    String receivedString = Serial.readStringUntil('\n');

    Throttle = receivedString.substring(0, receivedString.indexOf(',')).toInt();
    receivedString.remove(0, receivedString.indexOf(',') + 1);
    Direction = receivedString.substring(0, receivedString.indexOf(',')).toInt();
    receivedString.remove(0, receivedString.indexOf(',') + 1);
    Pump = receivedString.toInt();

    // Print the received parameters
    Serial.print("Throttle: ");
    Serial.print(Throttle);
    Serial.print(", Direction: ");
    Serial.print(Direction);
    Serial.print(", Pump: ");
    Serial.println(Pump);
  }


}
  
