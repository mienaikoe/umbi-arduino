#include <Adafruit_Sensor.h>

#ifndef MAG_NORMALIZER_H
#define MAG_NORMALIZER_H

namespace Navi{

  class MagNormalizer{

    private:
      sensors_vec_t midpoints;
      sensors_vec_t deltas;
      uint8_t runs;
      bool tracking;

    public:
      MagNormalizer();
      MagNormalizer(float x, float y, float z, float dx, float dy, float dz);
      void reset();
      void normalizeMagnetometer(sensors_vec_t *point);

  };

}


#endif
