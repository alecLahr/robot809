// ENPM809T
// BNO055 sensor

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup() {
  Serial.begin(115200);
  Serial.println("BNO055 Transmitting...");

  // Prepare the nano's buildin LED
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

long flash_start_time = millis();

void loop() {
  if (Serial.available() > 0) {  // wait to receive request signal from RPi
    int inByte = Serial.read();    
    flash_start_time = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    

    // Get a new sensor event 
    sensors_event_t event; 
    bno.getEvent(&event);

    Serial.println(event.orientation.x, 4);  // transmit angle back to RPi
  } 
  else if (millis() - flash_start_time > 50) {  // blink the led when receiving a signal
    digitalWrite(LED_BUILTIN, LOW);
  }
}
