#include "serialSensor.h"

#define CHAR2NIBBLE(a) ((a)-(((a)>57)?(((a)<71)?55:87):48))
#define BYTE2CHAR(a) (((a)<10)?('0'+(a)):('A'+(a)-10))
	
#define BITCHECK(a,b) ((a&(1<<b))==(1<<b))

/*

	Instruction set for communicating with the angle sensor board.  These
	commands may be sent via I2C, or via the serial port.  If they
	are sent via I2C, the byte values shown below are used.  If they are
	sent via the serial port, then the command should be preceded by the
	character ":".  A terminating CR, or CRLF, may be used, but is not
	necessary.  The bytes shown below should be represented as hexadecimal
	pairs.  For example, to turn on LEDs 0 and 3, the command 0x19 would
	be sent as ":19".  A multibyte command such as set PWM on SENSOR2 to
	a value of 0x32 would be represented as ":C432".  Any response bytes
	are returned similarly encoded as hexadecimal and preceded by ":".  
	The ":" character allows the two sides of the communication to easily 
	parse commands and responses, and to stay in sync.

	get angle data					0x00
	
		Gets current value of angle sensor. Returns a value (0-4095)
		in two bytes, msbyte first.
		
	get ultrasonic reading          0x01

		Gets raw data from ultrasonic sensor (i.e. raw analog data 0-1023).

	get ultrasonic reading in mm   0x02

		Gets data in mm from ultrasonic sensor.

	turn on LEDs					0x10 - 0x1F
	
		Turns on the LEDs corresponding to the bit pattern in the lower_bound
		4 bits of the command.  LED0 = command bit 0, etc.  Setting the
		corresponding command bit to 1 turns that LED on.  Command bits of 0
		do not affect the LED state.
		
	turn off LEDs					0x20 - 0x2F
	
		Turns off the LEDs corresponding to the bit pattern in the lower_bound
		4 bits of the command.  LED0 = command bit 0, etc.  Setting the
		corresponding command bit to 1 turns that LED off.  Command bits of 0
		do not affect the LED state.
		
	set LEDs						0x30 - 0x3F
	
		Sets the state of the LEDs corresponding to the bit pattern in 
		the lower 4 bits of the command.  LED0 = command bit 0, etc.  
		Setting the corresponding command bit to 1 turns that LED on,
		while setting it to 0 turns that LED off.

	get LEDs						0x08
	
		Returns one byte with the state of the LEDs indicated in the 
		lower 4 bits.  A bit value of 1 indicates that the corresponding
		LED is on, a 0 indicates that it is off.  
		
	set SENSORs input				0x40 - 0x4F
	
		Sets the SENSOR pins to be inputs according to the bit 
		pattern in the lower 4 bits of the command.  SENSOR0 = bit 0,
		etc.  If a given bit is 1, then the corresponding SENSOR pin
		is set to be an input.
		
	set SENSORs input with pullup	0x50 - 0x5F
	
		Sets the SENSOR pins to be inputs with an internal pullup 
		according to the bit pattern in the lower 4 bits of the command.
		SENSOR0 = bit 0, etc.  If a given bit is 1, then the 
		corresponding SENSOR pin is set to be an input with pullup.
		
	set SENSORs analog input		0x60 - 0x6F
	
		Sets the SENSOR pins to be analog inputs according
		to the bit pattern in the lower 4 bits of the command.  If a 
		given bit is 1, then the corresponding SENSOR pin is configured
		to be an analog input, and the internal pullup is disabled.

	set SENSORs output				0x70 - 0x7F
	
		Sets the SENSOR pins to be outputs according to the bit pattern 
		in the lower 4 bits of the command.  If a given bit is 1, 
		then the corresponding SENSOR pin is configured to be an output. 
	
	set SENSORs pwm output			0x80 - 0x8F
	
		Sets the SENSOR pins to be pwm outputs according
		to the bit pattern in the lower 4 bits of the command.  If a 
		given bit is 1, then the corresponding SENSOR pin is configured
		to be a pwm output.

	set SENSORs output high			0x90 - 0x9F
	
		Sets the SENSOR pins to a high state according to the bit pattern
		in the lower 4 bits of the command.  If a given bit is 1, then
		the corresponding SENSOR pin is set to a high state.  If a given
		bit is 0, the corresponding sensor pin remains unchanged.  Only
		valid if the corresponding pin(s) are configured as outputs, 
		otherwise ignored.
		
	set SENSORS output low			0xA0 - 0xAF
	
		Sets the SENSOR pins to a low state according to the bit pattern
		in the lower 4 bits of the command.  If a given bit is 1, then
		the corresponding SENSOR pin is set to a low state.  If a given
		bit is 0, the corresponding sensor pin remains unchanged. Only
		valid if the corresponding pin(s) are configured as outputs, 
		otherwise ignored.
	
	set SENSORs output values		0xB0 - 0xBF
	
		Sets the SENSOR pins to the state according to the bit pattern
		in the lower 4 bits of the command.  If a given bit is 1, then
		the corresponding SENSOR pin is set to a high state.  If a given
		bit is 0, the corresponding sensor pin is set to a low state.  
		Only valid if the corresponding pin(s) are configured as outputs, 
		otherwise ignored.
	
	set SENSORs pwm values			0xC0 - 0xCF, 0xNN 
	
		Sets the SENSOR pins to a pwm value according to the bit pattern
		in the lower 4 bits of the command.  If a given bit is 1, then
		the corresponding SENSOR pin is set to the PWM value in the second
		byte of the command.  If a given bit is 0, the corresponding sensor 
		pin remains unchanged.  Only valid if the corresponding pin(s) 
		are configured as pwm outputs, otherwise ignored. Note that all
		SENSOR pin pwm outputs which are called out in the lower 4 bits of 
		the command will be set to the same pwm value.
		
	get SENSORs values				0x09
	
		Gets the values of the SENSOR pins, and returns a single byte with
		with the values indicated in the lower 4 bits.  Results for a given
		pin are only valid if that pin is configured as a digital input,
		or a digital input with internal pullup.
		
	get SENSORs analog values		0xD0 - 0xDF
	
		Gets the analog values of the SENSOR pins and returns them, based
		on the lower 4 bits of the command.  If a given bit is 1, then a 
		two byte value (msbyte first) will be returned for the corresponding 
		SENSOR pin (up to a total of 8 bytes if all four SENSORs are
		selected).  The returned values are only valid if the corresponding
		SENSOR pin is configured as an analog input.
		
	get USERDATAn value				0xE0 - 0xEF
	
		Gets the value of USERDATA variable n, where n is given by the lower
		4 bits of the command.  This is useful for custom sketches which 
		return values not normally available.  The requester must know how
		many bytes to expect in response to the get request (which may vary by
		USERDATA variable.

*/

int USERDATA[16] ;

//
// I2C communications and command parser
//

int i2c_slaveaddress = I2C_SLAVEADDRESS ;

byte i2c_inbuf[8], i2c_outbuf[16] ;
int n_i2c_inbuf = 0 ;
int n_i2c_outbuf = 0 ;

void i2c_onReceive(int n) {
	int i, v ;
	byte cmd, param ;
	
	for(i = 0 ; i < n ; i++) i2c_inbuf[i] = Wire.read() ;
	cmd = i2c_inbuf[0] >> 4 ;
	param = i2c_inbuf[0] & 0x0F ;
	
	n_i2c_outbuf = 0 ;
	
	switch(cmd) {
		case 0x00 :
			if(param == 0) { 		// get angle data
				v = ANGLE.readRaw() ;
				i2c_outbuf[n_i2c_outbuf++] = v >> 8 ;
				i2c_outbuf[n_i2c_outbuf++] = v & 0x0FF ;
			} else if(param == 8) { // get LEDs
				i2c_outbuf[n_i2c_outbuf++] =
					((LED[3].state()) ? 8 : 0) + ((LED[2].state()) ? 4 : 0) + ((LED[1].state()) ? 2 : 0) + ((LED[0].state()) ? 1 : 0) ;
			} else if(param == 9) { // get SENSORs values
				i2c_outbuf[n_i2c_outbuf++] = 
					((SENSOR[3].get()) ? 8 : 0) + ((SENSOR[2].get()) ? 4 : 0) +	((SENSOR[1].get()) ? 2 : 0) +((SENSOR[0].get()) ? 1 : 0) ;
			}
			break ;
		case 0x01 :	// turn on LEDs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) LED[i].on() ;
			break ;
		case 0x02 :	// turn off LEDs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) LED[i].off() ;
			break ;
		case 0x03 :	// set LEDs
			for(i = 3 ; i >= 0 ; i--) LED[i].set(BITCHECK(param, i)) ;
			break ;
		case 0x04 :	// configure SENSORs as inputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUT) ;
			break ;
		case 0x05 :	// configure SENSORs as inputs with internal pullup
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUTPULLUP) ;
			break ;
		case 0x06 :	// configure SENSORs as analog inputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUTANALOG) ;
			break ;
		case 0x07 :	// configure SENSORs as outputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_OUTPUT) ;
			break ;
		case 0x08 :	// configure SENSORs as pwm outputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_OUTPUTPWM) ;
			break ;
		case 0x09 :	// set SENSORs outputs high
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].on() ;
			break ;
		case 0x0A :	// set SENSORs outputs low
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].off() ;
			break ;
		case 0x0B :	// set SENSORs output values
			for(i = 3 ; i >= 0 ; i--) SENSOR[i].set(BITCHECK(param, i)) ;
			break ;
		case 0x0C :	// set SENSORs pwm values
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) 
					SENSOR[i].setPWM(i2c_inbuf[1]) ;
			break ;
		case 0x0D :	// get SENSORs analog values
			for(i = 3 ; i >= 0 ; i--) {
				if(BITCHECK(param, i)) {
					v = SENSOR[i].read() ;
					i2c_outbuf[n_i2c_outbuf++] = (byte)(v >> 8) ;
					i2c_outbuf[n_i2c_outbuf++] = (byte)(v & 0x0FF) ;
				}
			}
			break ;
		case 0x0E :	// get USERDATAn value
			for(i = 3 ; i >= 0 ; i--) i2c_outbuf[n_i2c_outbuf++] = (byte)((USERDATA[param] >> (8 * i)) & 0x0FF) ;
			break ;
	}
}

void i2c_onRequest() {
	Wire.write(i2c_outbuf, n_i2c_outbuf) ;
	n_i2c_outbuf = 0 ;
}

//
// serial communications and command parser
//

byte ser_inbuf[16], ser_outbuf[32] ;
int n_ser_inbuf = 0 ;
int n_ser_outbuf = 0 ;

void serialEvent() {
	int i, v ;
	byte cmd, param, b ;
	
	while(Serial.available() > 0) ser_inbuf[n_ser_inbuf++] = Serial.read() ;
	
	if(n_ser_inbuf < 3) return ;
	
	while((n_ser_inbuf > 0) && (ser_inbuf[0] != ':')) {
		for(i = 1 ; i < n_ser_inbuf ; i++) ser_inbuf[i - 1] = ser_inbuf[i] ;
		--n_ser_inbuf ;
	}

	if(n_ser_inbuf < 3) return ; 	// at least 3 characters required for a valid command

	cmd = CHAR2NIBBLE(ser_inbuf[1]) ;
	param = CHAR2NIBBLE(ser_inbuf[2]) ;
	
	if((cmd == 0x0c) && (n_ser_inbuf < 5)) return ;	// if command is 0xCn, 5 characters are required
	
	n_ser_outbuf = 0 ;

	switch(cmd) {
		case 0x00 : // miscellaneous commands
			if(param == 0) { 		// get angle data
				v = ANGLE.readRaw() ;
				ser_outbuf[n_ser_outbuf++] = v >> 8 ;
				ser_outbuf[n_ser_outbuf++] = v & 0x0FF ;
			} else if(param == 1) {
				//TODOs	
			} else if(param == 8) { // get LEDs
				ser_outbuf[n_ser_outbuf++] =
					((LED[3].state()) ? 8 : 0) + ((LED[2].state()) ? 4 : 0) + ((LED[1].state()) ? 2 : 0) + ((LED[0].state()) ? 1 : 0) ;
			} else if(param == 9) { // get SENSORs values
				ser_outbuf[n_ser_outbuf++] = 
					((SENSOR[3].get()) ? 8 : 0) + ((SENSOR[2].get()) ? 4 : 0) +	((SENSOR[1].get()) ? 2 : 0) +((SENSOR[0].get()) ? 1 : 0) ;
			}
			break ;
		case 0x01 :	// turn on LEDs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) LED[i].on() ;
			break ;
		case 0x02 :	// turn off LEDs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) LED[i].off() ;
			break ;
		case 0x03 :	// set LEDs
			for(i = 3 ; i >= 0 ; i--) LED[i].set(BITCHECK(param, i)) ;
			break ;
		case 0x04 :	// configure SENSORs as inputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUT) ;
			break ;
		case 0x05 :	// configure SENSORs as inputs with internal pullup
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUTPULLUP) ;
			break ;
		case 0x06 :	// configure SENSORs as analog inputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_INPUTANALOG) ;
			break ;
		case 0x07 :	// configure SENSORs as outputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_OUTPUT) ;
			break ;
		case 0x08 :	// configure SENSORs as pwm outputs
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].mode(SMODE_OUTPUTPWM) ;
			break ;
		case 0x09 :	// set SENSORs outputs high
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].on() ;
			break ;
		case 0x0A :	// set SENSORs outputs low
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) SENSOR[i].off() ;
			break ;
		case 0x0B :	// set SENSORs output values
			for(i = 3 ; i >= 0 ; i--) SENSOR[i].set(BITCHECK(param, i)) ;
			break ;
		case 0x0C :	// set SENSORs pwm values
			for(i = 3 ; i >= 0 ; i--) if(BITCHECK(param, i)) 
					SENSOR[i].setPWM((CHAR2NIBBLE(ser_inbuf[3]) << 4) + CHAR2NIBBLE(ser_inbuf[4])) ;
			break ;
		case 0x0D :	// get SENSORs analog values
			for(i = 3 ; i >= 0 ; i--) {
				if(BITCHECK(param, i)) {
					v = SENSOR[i].read() ;
					for(i = 1 ; i >= 0 ; i--) ser_outbuf[n_ser_outbuf++] = (byte)((v >> (8 * i)) & 0x0FF) ;
				}
			}
			break ;
		case 0x0E :	// get USERDATAn value
			for(i = 3 ; i >= 0 ; i--) ser_outbuf[n_ser_outbuf++] = (byte)((USERDATA[param] >> (8 * i)) & 0x0FF) ;
			break ;
	}
	
	// correct serial input buffer by removing either 3 or 5 bytes
	
	v = (cmd == 0x0c) ? 5 : 3 ;
	for(i = v ; i < n_ser_inbuf ; i++) ser_inbuf[i - v] = ser_inbuf[i] ;
	n_ser_inbuf -= v ;

	// send response if necessary
	
	if(n_ser_outbuf == 0) return ;

	Serial.write(':') ;
	for(i = 0 ; i < n_ser_outbuf ; i++) {
		Serial.write(BYTE2CHAR(ser_outbuf[i] >> 4)) ;
		Serial.write(BYTE2CHAR(ser_outbuf[i] & 0x0F)) ;
	}
	Serial.println("") ;
}

//
// Initialization
//

void angleSensorInitialize(int a) {
	i2c_slaveaddress = a ;
	angleSensorInitialize() ;
}
void angleSensorInitialize(void) {
	// initialize leds
	LED[0].pin(LED0_PIN) ;
	LED[1].pin(LED1_PIN) ;
	LED[2].pin(LED2_PIN) ;
	LED[3].pin(LED3_PIN) ;
	
	// initialize sensor pins
	SENSOR[0].pin(SENSOR0_PIN) ;
	SENSOR[1].pin(SENSOR1_PIN) ;
	SENSOR[2].pin(SENSOR2_PIN) ;
	SENSOR[3].pin(SENSOR3_PIN) ;
	
	// initialize i2c interface
	Wire.begin(i2c_slaveaddress) ;
	Wire.onReceive(i2c_onReceive) ;
	Wire.onRequest(i2c_onRequest) ;
	
	// initialize serial interface
	Serial.begin(9600) ;
}

// 
// LEDclass: allows control of indicator LEDs on sensor board
//

LEDclass::LEDclass(void) {
}

void LEDclass::pin(int p) {
	_pin = p ;
	digitalWrite(_pin, HIGH) ;
	pinMode(_pin, OUTPUT) ;
	_led = false ;
}

void LEDclass::on(void) {
	digitalWrite(_pin, LOW) ;
	_led = true ;
}

void LEDclass::off(void) {
	digitalWrite(_pin, HIGH) ;
	_led = false ;
}

void LEDclass::toggle(void) {
	digitalWrite(_pin, (_led) ? HIGH : LOW) ;
	_led = !_led ;
}

void LEDclass::set(bool s) {
	digitalWrite(_pin, (s) ? LOW : HIGH) ;
	_led = s ;
}

bool LEDclass::state(void) {
	return(_led) ;
}

LEDclass LED[4] ;

// 
// SENSORclass: allows configuration and control of sense / control pins
//				on sensor board
//

SENSORclass::SENSORclass(void) {
}

void SENSORclass::pin(int p) {
	_pin = p ;
	_mode = SMODE_INPUTPULLUP ;
	pinMode(_pin, INPUT_PULLUP) ;
}

void SENSORclass::mode(int m) {
	switch(m) {
		case SMODE_INPUT :
			_mode = m ;
			pinMode(_pin, INPUT) ;
			break ;
		case SMODE_INPUTPULLUP :
			_mode = m ;
			pinMode(_pin, INPUT_PULLUP) ;
			break ;
		case SMODE_INPUTANALOG :
			_mode = m ;
			pinMode(_pin, INPUT) ;
			break ;		
		case SMODE_OUTPUT :
			_mode = m ;
			digitalWrite(_pin, HIGH) ;
			pinMode(_pin, OUTPUT) ;
			_state = true ;
			break ;
		default :
			_mode = SMODE_INPUTPULLUP ;
			pinMode(_pin, INPUT_PULLUP) ;
	}
}

void SENSORclass::on(void) {
	if(_mode == SMODE_OUTPUT) digitalWrite(_pin, HIGH) ;
	_state = true ;
}

void SENSORclass::off(void) {
	if(_mode == SMODE_OUTPUT) digitalWrite(_pin, LOW) ;
	_state = false ;
}

void SENSORclass::toggle(void) {
	if(_mode == SMODE_OUTPUT) digitalWrite(_pin, (_state) ? LOW : HIGH) ;
	_state = !_state ;
}

void SENSORclass::set(bool s) {
	if(_mode == SMODE_OUTPUT) digitalWrite(_pin, (s) ? HIGH : LOW) ;
	_state = s ;
}

void SENSORclass::setPWM(byte v) {
	if(_mode == SMODE_OUTPUTPWM) {
		// TODO:  add pwm functionality
	}
}

bool SENSORclass::get(void) {
	if(_mode == SMODE_INPUT) return((digitalRead(_pin) == HIGH) ? true : false) ;
	if(_mode == SMODE_INPUTPULLUP) return((digitalRead(_pin) == HIGH) ? true : false) ;
	if(_mode == SMODE_OUTPUT) return(_state) ;
	return(false) ;
}

int SENSORclass::read(void) {
	if(_mode == SMODE_INPUTANALOG) return(analogRead(_pin)) ;
	return(0) ;
}

SENSORclass SENSOR[4] ;

// 
// MAGclass: provides access to magnetic angle sensor
//

#define CSF 10

ANGLEclass::ANGLEclass(void) {
  // initialize SPI interface
  digitalWrite(CSF, HIGH) ; pinMode(CSF, OUTPUT) ;
  SPI.begin() ;
  SPI.setDataMode(SPI_MODE3) ;
  SPI.setBitOrder(MSBFIRST) ;
}

float ANGLEclass::read(void) {
	return(0.08789063 * (float)(readRegister(0x20) & 0x0FFF)) ;
}

int ANGLEclass::readRaw(void) {
	return(readRegister(0x20) & 0x0FFF) ;
}

unsigned int ANGLEclass::readRegister(byte reg) {
  unsigned int lsByte, msByte ;

  digitalWrite(CSF, LOW) ;
  SPI.transfer(reg) ; SPI.transfer(0x00) ;
  digitalWrite(CSF, HIGH) ;
  
  delay(2) ;
  
  digitalWrite(CSF, LOW) ;
  msByte = SPI.transfer(0x00) ; lsByte = SPI.transfer(0x00) ;
  digitalWrite(CSF, HIGH) ;

  return((msByte << 8) | lsByte) ;
}

void ANGLEclass::writeRegister(byte reg, byte data) {
  digitalWrite(CSF, LOW);
  SPI.transfer(reg | 0x40) ;
  SPI.transfer(data) ;
  digitalWrite(CSF, HIGH);
}

ANGLEclass ANGLE ;

	
	
	
	
	
	


			
			