
#include <bluefruit.h>

#include "packetParser.h"
#include "Discovery.h"


#define ACCEL_THRESHOLD 0.5


namespace Navi{
  namespace Discovery{

    extern uint8_t packetbuffer[];

    BLEUart bleuart;
    bool connected = false;
    char central_name[32] = { 0 };

    MagNormalizer magNorm;


    void connect_callback(uint16_t conn_handle){
      connected = true;
      Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

      Serial.print("Connected to ");
      Serial.println(central_name);
    }

    void disconnect_callback(uint16_t conn_handle, uint8_t reason){
      connected = false;
      (void) conn_handle;
      (void) reason;

      magNorm.reset();

      Serial.println();
      Serial.print("Disconnected from");
      Serial.println(central_name);
    }






    void startAdv(void)
    {
      // Advertising packet
      Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
      Bluefruit.Advertising.addTxPower();

      // Include the BLE UART (AKA 'NUS') 128-bit UUID
      Bluefruit.Advertising.addService(bleuart);

      // Secondary Scan Response packet (optional)
      // Since there is no room for 'Name' in Advertising packet
      Bluefruit.ScanResponse.addName();

      /* Start Advertising
       * - Enable auto advertising if disconnected
       * - Interval:  fast mode = 20 ms, slow mode = ~1sec
       * - Timeout for fast mode is 30 seconds
       * - Start(timeout) with timeout = 0 will advertise forever (until connected)
       *
       * For recommended advertising interval
       * https://developer.apple.com/library/content/qa/qa1931/_index.html
       */
      Bluefruit.Advertising.restartOnDisconnect(true);
      Bluefruit.Advertising.setInterval(32, 1636);    // in unit of 0.625 ms
      Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
      Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
    }

    void setup(){
      Bluefruit.begin();
      // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
      Bluefruit.setTxPower(4);
      Bluefruit.setName("Navi");

      // Configure and start the BLE Uart service
      bleuart.begin();

      Bluefruit.setConnectCallback(connect_callback);
      Bluefruit.setDisconnectCallback(disconnect_callback);

      // Set up and start advertising
      startAdv();
      Serial.println("Advertising");
    }





    /*
     * This method assumes that the phone is held
     *
     */

    void levelAccelerometer( sensors_vec_t *ret, sensors_vec_t *mag ){
      // if the field is oriented off of the x-y plane,


      /*
      float zAbs = z > 0 ? z : -z;
      float xAbs = x > 0 ? x : -x;
      float yAbs = y > 0 ? y : -y;

      if( zAbs >= xAbs && zAbs >= yAbs ){ // phone facing up or down
        if( z < 0 ){ // phone on its back
          ret->z = 9.81 + z;
          ret->x = x;
        } else { // phone face down
          ret->z = 9.81 - z;
          ret->x = -x;
        }
        ret->y = y; //assumes phone top is facing out in all cases
      } else if( xAbs >= zAbs && xAbs >= yAbs ){ // phone is on its side, held landscape mode
        if( x < 0 ){ // phone on its left side
          ret->z = 9.81 + x;
          ret->x = y;
        } else { // phone on its right side
          ret->z = 9.81 - x;
          ret->x = -y;
        }
        ret->y = -z; // assumes phone is being held with the camera facing out and the screen facing in
      } else { // phone is on its top or bottom, held portrait mode
        if( y < 0 ){ // phone on its bottom
          ret->z = 9.81 + y;
          ret->y = -z;
        } else { // phone on its top
          ret->z = 9.81 - y;
          ret->y = z;
        }
        ret->x = x; // assumes phone is being held with the camera facing out and the screen facing in
      }
      */
    }





    bool loop(sensors_vec_t *centralAccelerometer, sensors_vec_t *centralMagnetometer){
      if( connected ){
        // Wait for new data to arrive
        uint8_t len = readPacket(&bleuart, 500);
        if (len == 0) {
          return true;
        }

        // Magnetometer
        if (packetbuffer[1] == 'M') {
          // TODO: Normalize phone-side not bot-side
          centralMagnetometer->x = parsefloat(packetbuffer+2);
          centralMagnetometer->y = parsefloat(packetbuffer+6);
          centralMagnetometer->z = parsefloat(packetbuffer+10);
          magNorm.normalizeMagnetometer( centralMagnetometer );

          Serial.print("Mag\t");
          Serial.print(centralMagnetometer->x); Serial.print('\t');
          Serial.print(centralMagnetometer->y); Serial.print('\t');
          Serial.print(centralMagnetometer->z); Serial.println();
        } else if( packetbuffer[1] == 'A'){
          // TODO: Normalize phone-side not bot-side
          centralAccelerometer->x = 9.81 * parsefloat(packetbuffer+2);
          centralAccelerometer->y = 9.81 * parsefloat(packetbuffer+6);
          centralAccelerometer->z = 9.81 * parsefloat(packetbuffer+10);
          levelAccelerometer( centralAccelerometer, centralMagnetometer );

          Serial.print("Acc\t");
          Serial.print(centralAccelerometer->x); Serial.print('\t');
          Serial.print(centralAccelerometer->y); Serial.print('\t');
          Serial.print(centralAccelerometer->z); Serial.println();
        }
      }

      return connected;
    }
  }
}
