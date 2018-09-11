
#include <bluefruit.h>


namespace Navi{
  namespace Discovery{
    // Function prototypes for packetparser.cpp
    uint8_t readPacket  (BLEUart *ble_uart, uint16_t timeout);
    float   parsefloat  (uint8_t *buffer);
    void    printHex    (const uint8_t * data, const uint32_t numBytes);
  }
}
