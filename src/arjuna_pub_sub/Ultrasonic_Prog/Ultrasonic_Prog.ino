// Define the pins for the two ultrasonic sensors
#define TRIG_PIN_1 2
#define ECHO_PIN_1 3
#define TRIG_PIN_2 4
#define ECHO_PIN_2 5

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
}

float get_distance(int trig_pin, int echo_pin) {
  // Clear the trig pin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Set the trig pin HIGH for 10 microseconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echo_pin, HIGH);
  
  // Calculate the distance in cm
  // Speed of sound is 343 m/s or 0.0343 cm/µs
  float distance = duration * 0.0343 / 2;
  
  // Ensure the distance is within a reasonable range
  if (distance > 400 || distance <= 0) {
    return -1.0; // Return -1 for invalid readings
  } else {
    return distance;
  }
}

void loop() {
  // Get distances from both sensors
  float distance1 = get_distance(TRIG_PIN_1, ECHO_PIN_1);
  float distance2 = get_distance(TRIG_PIN_2, ECHO_PIN_2);
  
  // Print the distances to the serial monitor
  // The format is "distance1,distance2"
  Serial.print(distance1);
  Serial.print(",");
  Serial.println(distance2);
  
  delay(100); // Wait for 100ms before the next reading
}