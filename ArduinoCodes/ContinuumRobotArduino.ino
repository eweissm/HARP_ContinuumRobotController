/*HARP Continuum Robot Controller
By Eric Weissman
4/18/2025
  Description:
  This program runs on a microcontroller to control and monitor the pneumatic pressure 
  in a soft continuum robot called HARP. It interfaces with up to 9 pressure regulators 
  (muscles), sending target pressures and reading back actual values through analog 
  I/O pins. Communication is done over serial using a custom binary protocol.

  Functionality:
  - Waits for a sync byte (0xAA) to begin receiving a command packet.
  - Receives a packet of 9 float values representing desired pressures.
  - Each pressure is clamped to a safety limit (maxP = 35 psi) and converted 
    to an 8-bit PWM signal sent to corresponding output pins.
  - Reads the analog voltage from pressure sensors to compute actual pressure values.
  - Sends back the actual pressures as float values in response, prefixed by the sync byte.

  Serial Communication Protocol:
  - Incoming:
      [0xAA][float x9] => Sync byte, 9 floats for desired pressures.
  - Outgoing:
      [0xAA][float x9]=> Sync byte, 9 floats for actual pressures.

  Notes:
  - Pressure is linearly mapped to an 8-bit PWM signal to control the analog voltage regulators.
  - Actual pressure is calculated from the sensor's analog reading using a calibrated linear equation.
  - All communication is done at 250000 baud to support fast, low-latency updates.
*/



#define nPins 3  // number of regulators
const int maxP = 35;  // saftey limit for pressure to be sent to the muscles
const uint8_t syncByte = 0xAA;

//store regulator pins
byte SetPointPins[nPins] = { 2, 3, 4};
byte ActualPointPins[nPins] = { A7, A11, A3};


int setPressure[nPins];
float ActualP[nPins];
bool readyToSend = false;
size_t expectedBytes = nPins * sizeof(float);

void setup() {

  //start serial coms
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial to initialize
  //set pinmode for all of the pin, initialize setPressure
  for (unsigned i = 0; i < nPins; i++) {
    pinMode(SetPointPins[i], OUTPUT);
    // pinMode(ActualPointPins[i], INPUT);
    setPressure[i] = 0.0;
  }

}

void loop() {

  if (Serial.available() >= 1) { // if message in serial buffer
    uint8_t incoming = Serial.read(); // read next byte

    if (incoming == syncByte) { // if the byte is sync byte, we can read the rest of the message

      while(Serial.available() < expectedBytes ){} //wait to recieve the entire message

      for (size_t i = 0; i < nPins; ++i) {//store next 4*n bytes and decode them as floats

        byte floatBytes[4];
        for (int j = 0; j < 4; ++j) {
          floatBytes[j] = Serial.read();
        }
        float value;
        memcpy(&value, floatBytes, sizeof(float));
        setPressure[i] = min(maxP, int(value)); //convert to constrained int
      }


    }
  }

// set pressures and read actual pressures
  for (int i = 0; i < nPins; i++) {
    //set commanded Pressures
    int SetVoltage = map(setPressure[i], 0, 145, 0, 255);
    analogWrite(SetPointPins[i], SetVoltage);
  }


}