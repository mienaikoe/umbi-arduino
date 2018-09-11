#include "MagNormalizer.h"



namespace Navi {

  MagNormalizer::MagNormalizer(){
    this->tracking = true;
    this->reset();
  }

  MagNormalizer::MagNormalizer(float x, float y, float z, float dx, float dy, float dz){
    this->midpoints.x = x;
    this->midpoints.y = y;
    this->midpoints.z = z;
    this->deltas.x = dx;
    this->deltas.y = dy;
    this->deltas.z = dz;
    this->tracking = false;
  }


  void MagNormalizer::reset(){
    if( this->tracking ){
      midpoints.x = 0.0;
      midpoints.y = 0.0;
      midpoints.z = 0.0;
      deltas.x = 0.0;
      deltas.y = 0.0;
      deltas.z = 0.0;
    }
  }


  void recalcMax(float *midpoint, float *delta, float newMax){
    *delta = (newMax - (*midpoint - *delta)) / 2;
    *midpoint = newMax - *delta;
  }

  void recalcMin(float *midpoint, float *delta, float newMin){
    *delta = ((*midpoint + *delta) - newMin) / 2;
    *midpoint = newMin + *delta;
  }

  void MagNormalizer::normalizeMagnetometer(sensors_vec_t *ret){
    if( this->tracking ){
      if( ret->x > midpoints.x + deltas.x ){
        //deltas.x = (ret->x - (midpoints.x - deltas.x)) / 2;
        //midpoints.x = ret->x - deltas.x;

        recalcMax(&midpoints.x, &deltas.x, ret->x);
      } else if( ret->x < midpoints.x - deltas.x ){
        //deltas.x = ((midpoints.x + deltas.x) - ret->x) / 2;
        //midpoints.x = ret->x + deltas.x;
        recalcMin(&midpoints.x, &deltas.x, ret->x);
      }

      if( ret->y > midpoints.y + deltas.y ){
        recalcMax(&midpoints.y, &deltas.y, ret->y);
      } else if( ret->y < midpoints.y - deltas.y ){
        recalcMin(&midpoints.y, &deltas.y, ret->y);
      }

      if( ret->z > midpoints.z + deltas.z ){
        recalcMax(&midpoints.z, &deltas.z, ret->z);
      } else if( ret->z < midpoints.z - deltas.z ){
        recalcMin(&midpoints.z, &deltas.z, ret->z);
      }

      if( runs % 100 == 0 ){
        Serial.printf("%f %f %f\n", midpoints.x, midpoints.y, midpoints.z);
      }
    }

    ret->x = (ret->x - midpoints.x) * (1.0 / deltas.x);
    ret->y = (ret->y - midpoints.y) * (1.0 / deltas.y);
    ret->z = (ret->z - midpoints.z) * (1.0 / deltas.z);
  }

}
