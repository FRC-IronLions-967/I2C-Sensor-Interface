#include "wireBuffer.h"

WireBuffer::WireBuffer(ArduinoType a_type, ComType c_type) {
    this.at = a_type;
    this.ct = c_type;
}

unsigned char[] WireBuffer::readBufferRaw() {
    return WireBuffer::buffer;
}

unsigned char[] WireBuffer::readUntilDelim(char delim) {
    for(int i = 0; i < sizeof(buffer)/sizeof(char)) {
        
    }
}