#define LaserPin A6

float farDist = 350;
float nearDist = 50;

void setup() {
  pinMode(LaserPin, INPUT);
  // analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
}

void loop() {
  float time = (float(micros()) / 1000000.0);
  int val = analogRead(LaserPin);
  float distance = (val / 1023.0) * (farDist - nearDist);

  // Start of packet marker
  uint8_t syncByte = 0xAA;
  Serial.write(syncByte);

  // // Send floats
  Serial.write((uint8_t*)&time, sizeof(float));
  Serial.write((uint8_t*)&distance, sizeof(float));
  // Serial.println(distance);
  delayMicroseconds(10);
}