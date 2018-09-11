
#ifndef NAVI_DISCOVERY_H
#define NAVI_DISCOVERY_H

#include <Adafruit_Sensor.h>
#include "MagNormalizer.h"

namespace Navi{
  namespace Discovery{
    void setup();
    bool loop(sensors_vec_t *centralAccelerometer, sensors_vec_t *centralMagnetometer);
  }
}

#endif
