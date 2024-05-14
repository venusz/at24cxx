/**

AT24CXX.h
Library for using the EEPROM AT24C32/64

Copyright (c) 2024 HOSO

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */
#ifndef AT24CXX_h
#define AT24CXX_h

// includes
#include <Arduino.h>
#include <Wire.h>

// AT24CXX I2C adress
// 80
// 0x50
#define EEPROM_BASE_ADDRESS 0x50

// general class definition
class AT24CXX {
public:
	//AT24CXX(uint8_t i2c_address, uint8_t pageSize, TwoWire &tw);
	bool error;
	void begin(uint8_t i2c_address = EEPROM_BASE_ADDRESS, TwoWire &tw = Wire);
	bool write(unsigned int address, uint8_t data);
	bool write(unsigned int address, uint8_t *data, int n);
  template <typename T> void write(const unsigned int, const T);
	 
	uint8_t read();
	uint8_t read(unsigned int address);
	uint8_t* read(unsigned int address, uint8_t *data, int n);
  template <typename T> T* read(const unsigned int, T *);
	
protected:
	AT24CXX(uint8_t pageSize = 32);
private:
	bool pollACK();
	int _id;
	uint8_t _b[8];
	uint8_t _pageSize;
    TwoWire * _tw;
};

// AT24C32 class definiton
class AT24C32 : public AT24CXX {
};

// AT24C64 class definiton
class AT24C64 : public AT24CXX {
};

// AT24C128 class definiton
class AT24C128 : public AT24CXX {
public:
	AT24C128() : AT24CXX(64){};
};

// AT24C256 class definiton
class AT24C256 : public AT24CXX {
public:
	AT24C256() : AT24CXX(64){};
};

// AT24C512 class definiton
class AT24C512 : public AT24CXX {
public:
	AT24C512() : AT24CXX(128){};
};

template <typename T> void AT24CXX::write(const unsigned int address, const T data) {
	write(address, (uint8_t *) &data, sizeof(T));
}

template <typename T> T* AT24CXX::read(const unsigned int address, T *data) {
	return (T*)read(address, (uint8_t *) data, sizeof(T));
}

#endif