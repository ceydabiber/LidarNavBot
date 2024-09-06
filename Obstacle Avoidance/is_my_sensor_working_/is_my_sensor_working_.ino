/*
 * Arduinoya verilen kod yüklendikten sonra serial monitor açılıp sensörün önüne obje konulup çekilerek sensörün çalışıp çalışmadığı gözlemlenebilir.
 */


#define Trigger 11
#define Echo 12

void setup(){
 
  Serial.begin(9600);
 
  // Define each pin as an input or output.
  pinMode(Echo, INPUT);
  pinMode(Trigger, OUTPUT);
}
 
void loop(){
 
  // Make the Trigger LOW (0 volts)
  // for 2 microseconds
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
 
  // Emit high frequency 40kHz sound pulse
  // by making Trigger HIGH (5 volts)
  // for 10 microseconds
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
 
  // Detect a pulse on the Echo pin.
  // pulseIn() measures the time in
  // microseconds until the sound pulse
  // returns back to the sensor.
  long duration = pulseIn(Echo, HIGH);
 
  // Speed of sound is:
  // 34300 centimeters per second
  // 34300 / 10^6 centimeters per microsecond
  // 0.0343 centimeters per microsecond
  // Below, we convert microseconds to centimeters by
  // multiplying by 0.0343 and then dividing by 2
  // to account for the roundtrip time.
  float distance = (duration * 0.0343) / 2;
 
  // Print the distance in centimeters
  Serial.println(distance);
 
  // Pause for 100 milliseconds
  delay(100);
}
