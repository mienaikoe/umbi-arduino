
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>
#include "Flight.h"


#define BASELINE_PWM    200

#define DEBUGGING true

#define PIN_FRONT_LEFT  2
#define PIN_FRONT_RIGHT 3
#define PIN_AFT_LEFT    4
#define PIN_AFT_RIGHT   5
#define PIN_BATTERY     31


// TODO: Move this to a special trig approximation class
const float atancoarse[] = { // ratio increments of 1 from -40 to 40
  -89,-89,-88,-88,-88,-88,-88,-88,-88,-88,-88,-88,-88,-88, // 0 to 12
  -88,-88,-88,-88,-87,-87,-87,-87,-87,-87,-86,-86,-86,-86, // 13 to 26
  -85,-85,-84,-84,-83,-82,-81,-79,-76,-72,-63,-45,0,45,63, // 27 to 41
  72,76,79,81,82,83,84,84,85,85,86,86,86,86,87,87,87,87,87, // 42 to 61
  87,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89 // 62 to 81
};

const float atanfine[] = { // ratio increments of 0.01 from -0.99 to 0.99
  -45,-45,-44,-44,-44,-44,-43,-43,-43,-42,-42,-42,-41,-41,-41,-40,-40,-40,
  -39,-39,-39,-38,-38,-38,-37,-37,-37,-36,-36,-35,-35,-35,-34,-34,-33,-33,
  -33,-32,-32,-31,-31,-31,-30,-30,-29,-29,-28,-28,-27,-27,-27,-26,-26,-25,
  -25,-24,-24,-23,-23,-22,-22,-21,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,
  -16,-15,-15,-14,-13,-13,-12,-12,-11,-11,-10,-10,-9,-9,-8,-7,-7,-6,-6,-5,
  -5,-4,-3,-3,-2,-2,-1,-1,0,1,1,2,2,3,3,4,5,5,6,6,7,7,8,9,9,10,10,11,11,12,
  12,13,13,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,24,24,
  25,25,26,26,27,27,27,28,28,29,29,30,30,31,31,31,32,32,33,33,33,34,34,35,
  35,35,36,36,37,37,37,38,38,38,39,39,39,40,40,40,41,41,41,42,42,42,43,43,
  43,44,44,44,44,45
};


namespace Navi{
  namespace Flight{

    // i2c
    Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

    flight_pwm_t control;

    sensors_event_t localAccelerometer, localMagnetometer, localGyro;

    MagNormalizer magNorm;
      //0.2724, 0.1845, 0.2558, 0.4890,	0.5069,	0.4903); // values are deduced from calibration readings



    void setupGuidance(){
      // Try to initialise and warn if we couldn't detect the chip
      while (!lsm.begin()){
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        delay(1000);
      }
      Serial.println("Found LSM9DS1 9DOF");

      // 1.) Set the accelerometer range
      lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
      //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
      //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
      //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

      // 2.) Set the magnetometer sensitivity
      lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
      //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
      //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
      //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

      // 3.) Setup the gyroscope
      lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
      //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
      //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
    }

    void setupControl(){
      pinMode(    PIN_FRONT_LEFT, OUTPUT);
      analogWrite(PIN_FRONT_LEFT, 0);
      pinMode(    PIN_FRONT_RIGHT, OUTPUT);
      analogWrite(PIN_FRONT_RIGHT, 0);
      pinMode(    PIN_AFT_LEFT, OUTPUT);
      analogWrite(PIN_AFT_LEFT, 0);
      pinMode(    PIN_AFT_RIGHT, OUTPUT);
      analogWrite(PIN_AFT_RIGHT, 0);
    }

    void setup(){
      setupGuidance();
      setupControl();
    }




    int capPWM( int pwm ){
      if( pwm > 255 ){
        return 255;
      } else if( pwm < 0 ){
        return 0;
      } else {
        return pwm;
      }
    }

    float magnitudeSquared(const sensors_vec_t *vec ){
      return (vec->z * vec->z) +
        (vec->x * vec->x) +
        (vec->y * vec->y);
      // square root of 1 is approx 1
    }

    void matchAcceleration(const sensors_vec_t *aCentral, const sensors_vec_t *aLocal ){
      if( magnitudeSquared(aCentral) < 100 ){ // 9.8 * 9.8
        // effectively not moving
        // TODO: Determine if the central is moving by comparing with mag readings
        // (once we have normalized mag readings from central)
        control.front_left  = BASELINE_PWM;
        control.front_right = BASELINE_PWM;
        control.aft_left    = BASELINE_PWM;
        control.aft_right   = BASELINE_PWM;
        return;
      }

      // a positive number means the central is moving down more relative to local
      int zErr = (int)( (aCentral->z - aLocal->z) * 5 );
      // a positive number means the central is moving left more relative to local
      int xErr = (int)( (aCentral->x - aLocal->x) * 5 );
      // a positive number means the central is moving back more relative to local
      int yErr = (int)( (aCentral->y - aLocal->y) * 5 );

      control.front_left  = capPWM( BASELINE_PWM - zErr + xErr - yErr );
      control.front_right = capPWM( BASELINE_PWM - zErr - xErr - yErr );
      control.aft_left    = capPWM( BASELINE_PWM - zErr - xErr - yErr );
      control.aft_right   = capPWM( BASELINE_PWM - zErr + xErr + yErr );
    }

    int arctanApprox(const sensors_vec_t *vec){
      if( vec->x == 0.0 ){
        return vec->y > 0 ? 90 : 270;
      }

      float ratio = vec->y / vec->x;
      int adder = vec->x < 0 ? 180 : (vec->y < 0 ? 360 : 0);
      // in q1, leave as is
      // in q2, 180 + atan
      // in q3, 180 + atan
      // in q4, 360 + atan

      if( ratio > 39 || ratio < -39 ){
        // |y| is much larger than |x|, close enough to 90 to assume 90
        return adder + 90;
      } else if( ratio > 0.99 || ratio < -0.99)  {
        // approximation for |y| > |x|
        return adder + atancoarse[(int)(ratio + (ratio > 0 ? 40.5 : 39.5))];
      } else {
        // fine approximation
        return adder + atanfine[(int)((ratio*50) + (ratio > 0 ? 50.5 : 49.5))];
      }
    }

    void matchOrientation(const sensors_vec_t *mCentral, const sensors_vec_t *mLocal ){
      // mLocal       // y is forward, x is right, z is down
      // mCentral     // y is forward, x is left, z is up

      int centralAngle = arctanApprox(mCentral);
      int localAngle = arctanApprox(mLocal);
      int angleDiff = centralAngle - localAngle;
      if( angleDiff > 180 ){
        // centralAngle is in q3 or q4 while local is in q1 or q2
        // counterclockwise is the quickest anglediff
        angleDiff = -360 + angleDiff;
      } else if( angleDiff < -180 ){
        // centralAngle is in q1 or q2 while local is in q3 or q4
        // clockwise is the quickest anglediff
        angleDiff = 360 + angleDiff;
      }

      if( DEBUGGING ){
        /*
        Serial.printf("Local x: %f, Local y: %f, local atan: %d, central atan: %d, diff: %d\n",
        mLocal->x, mLocal->y, localAngle, centralAngle, angleDiff);
        */
      }

      int turn = 0;
      if( angleDiff < 10 && angleDiff > -10 ) {
        // within a small amount of angle difference, the turn shouldn't be very much
        turn = (int)(angleDiff / 2);
      } else {
        // otherwise, the turn should be a bit more pronounced
        turn = (angleDiff > 0 ? 6 : -6);
      }

      // clockwise
      control.front_left   = capPWM( control.front_left + turn );
      control.aft_right    = capPWM( control.aft_right + turn );
      // counterclockwise
      control.front_right  = capPWM( control.front_right - turn );
      control.aft_left     = capPWM( control.aft_left - turn );
    }


    void loop(const sensors_vec_t *centralAccelerometer, const sensors_vec_t *centralMagnetometer){
      // 1. Sense
      lsm.read();
      lsm.getEvent(&localAccelerometer, &localMagnetometer, &localGyro, NULL);

      magNorm.normalizeMagnetometer(&localMagnetometer.magnetic);

      if( DEBUGGING ){

        Serial.printf(
          "Local Acc  \t= x: %f, y: %f, z: %f\n",
          localAccelerometer.acceleration.x,
          localAccelerometer.acceleration.y,
          localAccelerometer.acceleration.z
        );


        Serial.printf(
          "Local Mag  \t= x: %f, y: %f, z: %f\n",
          localMagnetometer.magnetic.x,
          localMagnetometer.magnetic.y,
          localMagnetometer.magnetic.z
        );
      }


      // 2. Control
      // Match Acceleration - this also handles acceleration discrepancies
      matchAcceleration( centralAccelerometer, &localAccelerometer.acceleration );

      // Match Orientation - orients the drone in the same direction as the central
      matchOrientation( centralMagnetometer, &localMagnetometer.magnetic );

      // 2c. execute
      analogWrite( PIN_FRONT_LEFT,  control.front_left );
      analogWrite( PIN_FRONT_RIGHT, control.front_right );
      analogWrite( PIN_AFT_LEFT,    control.aft_left );
      analogWrite( PIN_AFT_RIGHT,   control.aft_right );

      if( DEBUGGING ){
        Serial.printf(
          "Control = fl: %d, fr: %d, al: %d, ar: %d\n",
          control.front_left, control.front_right,
          control.aft_left, control.aft_right
        );
      }
    }

    void down(){
      Serial.println("Heading Down");
      for( int ix=BASELINE_PWM; ix>0; ix -= 10 ){
        // power down the craft slowly
        analogWrite( PIN_FRONT_LEFT,  ix );
        analogWrite( PIN_FRONT_RIGHT, ix );
        analogWrite( PIN_AFT_LEFT,    ix );
        analogWrite( PIN_AFT_RIGHT,   ix );
        delay(100);
      }
      Serial.println("Touched Down");
      magNorm.reset();
    }


  }
}
