
#ifndef NAVI_FLIGHT_H
#define NAVI_FLIGHT_H

#include <Adafruit_Sensor.h>
#include "MagNormalizer.h"

namespace Navi{
  namespace Flight{

    typedef struct {
      int front_left;
      int front_right;
      int aft_left;
      int aft_right;
    } flight_pwm_t;

    void setup();
    void loop(const sensors_vec_t *centralAccelerometer, const sensors_vec_t *centralMagnetometer);
    void down();
  }
}

#endif
