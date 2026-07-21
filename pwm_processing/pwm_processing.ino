const int throttlePin = 23;
const int steeringPin = 22;

float applyDeadband(float value, float deadband = 0.04f) {
  if (abs(value) < deadband) {
    return 0.0f;
  }
  float magnitude = (abs(value) - deadband) / (1.0f - deadband);
  return value < 0 ? -magnitude : magnitude;
}

void setup() {
  Serial.begin(115200);
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);
}

// command format: type,throttle,steer
// example command: D,0.3,0.5 (Drive, 30% throttle, 50% steer aka right 50%)

void loop() {
  unsigned long throttleWidth = pulseIn(throttlePin, HIGH, 30000);
  unsigned long steeringWidth = pulseIn(steeringPin, HIGH, 30000);

  if (steeringWidth == 0 || throttleWidth == 0) {
    Serial.println("No PWM signal");
  } else {
    // tuned to the neutral PWM signal outputted by the receiver, yours may be different.
    float throttleCommand = constrain((throttleWidth - 1490.0) / 500.0, -1.0, 1.0);
    float steeringCommand = constrain((steeringWidth - 1460.0) / 500.0, -1.0, 1.0);
    throttleCommand = applyDeadband(throttleCommand);
    steeringCommand = applyDeadband(steeringCommand);
    Serial.print("D,");
    Serial.print(throttleCommand, 3);
    Serial.print(",");
    Serial.println(steeringCommand, 3);
  }
}