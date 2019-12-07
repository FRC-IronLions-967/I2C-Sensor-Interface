#ifndef angleSensor_h
#define angleSensor_h

#include <SPI.h>
#include <Wire.h>

#define LED0_PIN 3
#define LED1_PIN 2
#define LED2_PIN A7
#define LED3_PIN A6

#define SENSOR0_PIN A0
#define SENSOR1_PIN A1
#define SENSOR2_PIN A2
#define SENSOR3_PIN A3

#define SMODE_INPUT 1
#define SMODE_INPUTPULLUP 2
#define SMODE_INPUTANALOG 3
#define SMODE_OUTPUT 4
#define SMODE_OUTPUTPWM 5

#define I2C_SLAVEADDRESS 0x24

void angleSensorInitialize(void) ;

class LEDclass {
	public:
		LEDclass(void) ;
		void pin(int p) ;
		void on(void) ;
		void off(void) ;
		void toggle(void) ;
		void set(bool b) ;
		bool state(void) ;
	private:
		int _pin ;
		bool _led ;
} ;

extern LEDclass LED[4] ;

class SENSORclass {
	public:
		SENSORclass(void) ;
		void pin(int p) ;
		void mode(int m) ;
		void on(void) ;
		void off(void) ;
		void toggle(void) ;
		void set(bool b) ;
		void setPWM(byte v) ;
		bool get(void) ;
		int read(void) ;
	private:
		int _pin ;
		int _mode ;
		bool _state ;
} ;

extern SENSORclass SENSOR[4] ;

class ANGLEclass {
	public:
		ANGLEclass(void) ;
		float read(void) ;
		int readRaw(void) ;
	private:
		unsigned int readRegister(byte r) ;
		void writeRegister(byte r, byte d) ;
} ;

extern ANGLEclass ANGLE ;

#endif
