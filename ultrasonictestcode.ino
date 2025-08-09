// Define speed of sound in cm/us (half, because we measure round trip)
#define SOUND_SPEED 0.034 / 2

// Right Ultrasonic Sensor pins
#define TRIG_RIGHT 42
#define ECHO_RIGHT 41

// Left Ultrasonic Sensor pins
#define TRIG_LEFT 21
#define ECHO_LEFT 20

// Front Ultrasonic Sensor pins
#define TRIG_FRONT 2
#define ECHO_FRONT 1

// Function to read distance from ultrasonic sensor
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout after 30ms (max ~500cm)
  if (duration == 0) return -1;  // No echo received

  float distance = duration * SOUND_SPEED;
  return distance;
}

void setup() {
  Serial.begin(115200);

  // Initialize all ultrasonic pins
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  Serial.println("Ultrasonic sensor test started");
}

void loop() {
  float right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  float left = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);

  Serial.print("Right: ");
  if (right == -1) Serial.print("No Echo");
  else Serial.print(right, 2);
  Serial.print(" cm | Left: ");
  if (left == -1) Serial.print("No Echo");
  else Serial.print(left, 2);
  Serial.print(" cm | Front: ");
  if (front == -1) Serial.print("No Echo");
  else Serial.print(front, 2);
  Serial.println(" cm");

  delay(500);
}
