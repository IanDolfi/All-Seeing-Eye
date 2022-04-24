void setup() {
Serial.begin(9600);
}

void loop() {
  Serial.print("0: "); Serial.println(analogRead(A0));
  Serial.print("1: "); Serial.println(analogRead(A1));
  Serial.print("2: "); Serial.println(analogRead(A2));
  Serial.print("3: "); Serial.println(analogRead(A3));
  delay(50);
}
