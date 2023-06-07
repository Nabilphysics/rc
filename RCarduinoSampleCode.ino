/*
RC Receiver with Arduino
Syed Razwanul Haque(Nabil)
*/

// Serial output on/off based on debugFlag
#define debugFlag true //If true show output in Serial

#if debugFlag == true
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x) 
#define debugln(x)
#endif

// ******** End RC Input Pin ***********
uint8_t rcCh1_input = 2; //
uint8_t rcCh2_input = 3;
uint8_t rcCh6_input = 4;
// ******** End RC Input Pin ***********

unsigned long ch1PulseDuration;
unsigned long ch2PulseDuration;
unsigned long ch6PulseDuration;

void setup() {
  Serial.begin(9600);
  pinMode(rcCh1_input, INPUT);
  pinMode(rcCh2_input, INPUT);
  pinMode(rcCh6_input, INPUT);
}

void loop() {
  ch1PulseDuration = pulseIn(rcCh1_input, HIGH);
  ch2PulseDuration = pulseIn(rcCh2_input, HIGH);
  ch6PulseDuration = pulseIn(rcCh6_input, HIGH);
  
  debug("RC Input:-- "); debug("CH1:"); debug(ch1PulseDuration); debug("  CH2:");debug(ch2PulseDuration); debug("  CH6:"); debug(ch6PulseDuration); debugln();
}
