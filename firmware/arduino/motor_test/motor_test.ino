const int IN1 = 8;
const int IN2 = 9;
const int ENA = 5;

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  Serial.println("Setup complete. Starting motor test loop.");
}

void loop() {
  // Forward
  Serial.println("FORWARD - IN1=HIGH, IN2=LOW, ENA=150");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);
  delay(2000);

  // Stop
  Serial.println("STOP - ENA=0");
  analogWrite(ENA, 0);
  delay(1000);

  // Reverse
  Serial.println("REVERSE - IN1=LOW, IN2=HIGH, ENA=150");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150);
  delay(2000);

  // Stop
  Serial.println("STOP - ENA=0");
  analogWrite(ENA, 0);
  delay(1000);

  Serial.println("--- loop restarting ---");
}
