// Serial1 - LoRa радио в конструкторе стратоспутник

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  Serial1.begin(9600);
}

void loop() {
  Serial1.println("test " + millis());
  delay(1000);
}
