
#include <arduino.h>
#include "Discovery.h"
#include "Flight.h"


sensors_vec_t centralMagnetometer;
sensors_vec_t centralAccelerometer;

uint16_t battery = 0;

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
  bool connected = Navi::Discovery::loop(&centralAccelerometer, &centralMagnetometer);
  if( connected ){
    Navi::Flight::loop(&centralAccelerometer, &centralMagnetometer);
  } else {
    centralAccelerometer.x = 0.0;
    centralAccelerometer.y = 0.0;
    centralAccelerometer.z = 0.0;
    centralMagnetometer.x = 0.0;
    centralMagnetometer.y = 0.0;
    centralMagnetometer.z = 0.0;
    Navi::Flight::down();
  }
}
