#ifndef wireBuffer
#define wireBuffer

#include <SPI.h>
#include <Wire.h>

class WireBuffer {
    public:
        enum ArduinoType {NANO, MKR, UNO};
        enum ComType {I2C, SERIAL, SPI, WIEGAND};
        WireBuffer(ArduinoType a_type, ComType c_type);
        unsigned char[] readBufferRaw(void);
        unsigned char[] readUntilDelim(char delim);

    private:
        unsigned char buffer[1024];
        ArduinoType at;
        ComType ct;
};

#endif