
#include <arduino.h>
#include "Discovery.h"
#include "Flight.h"


sensors_vec_t centralMagnetometer;
sensors_vec_t centralAccelerometer;

uint16_t battery = 0;

bool isDown = true;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause until serial console opens
  }

  Serial.println("Starting BLE...");
  Navi::Discovery::setup();
  Serial.println("Starting Flight...");
  Navi::Flight::setup();
}

void loop() {
  bool streaming = Navi::Discovery::loop(&centralAccelerometer, &centralMagnetometer);
  if( streaming ){
    if( isDown ){
      isDown = false;
    }
    Navi::Flight::loop(&centralAccelerometer, &centralMagnetometer);
  } else if( !isDown ){
    centralAccelerometer.x = 0.0;
    centralAccelerometer.y = 0.0;
    centralAccelerometer.z = 0.0;
    centralMagnetometer.x = 0.0;
    centralMagnetometer.y = 0.0;
    centralMagnetometer.z = 0.0;
    Navi::Flight::down();
    isDown = true;
  } else {
    delay(100);
    // saves battery
  }
}
