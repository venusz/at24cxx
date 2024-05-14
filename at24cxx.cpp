/**

AT24CXX.cpp
Library for using the EEPROM AT24C32/64

Copyright (c) 2014 Christian Paul

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
#include "AT24CXX.h"

/**
 * Constructor with AT24CXX EEPROM at i2c_address 0
 */

/**
 * Constructor with AT24CXX EEPROM at given i2c_address and size of page
 */
AT24CXX::AT24CXX(uint8_t pageSize) {
	_pageSize = pageSize;
}

void AT24CXX::begin(uint8_t i2c_address, TwoWire &tw){
	_id  = i2c_address;
    _tw = &tw;
}

bool AT24CXX::pollACK() {
	error = false;
	_tw->beginTransmission(_id);
	//digitalWrite(12, HIGH);
	uint32_t pollingStart = millis();
	while(millis()-pollingStart < 15)
		if(!_tw->endTransmission())return true;
	error = true;
	return false;
}

/**
 * Writing a byte
 */
bool AT24CXX::write(unsigned int address, uint8_t data) {
  if(pollACK()){  
		_tw->beginTransmission(_id);
		_tw->write(address >> 8);
		_tw->write(address & 0xFF);
			_tw->write(data);
		_tw->endTransmission();
		return true;
	}
	return false;
}

/**
 * Writing n byte
 */
bool AT24CXX::write(unsigned int address, uint8_t *data, int n) {
	int bytes_left = n;						// bytes left to write
	int offD = 0;					// current offset in data pointer
	int offP;						// current offset in page
	int bytes_to_write = 0;						// next n bytes to write

	// write all bytes in multiple steps
	while (bytes_left > 0) {
		// calc offset in page
		offP = address % _pageSize;
		// read max 30 -> 32 - 2 address bytes
		bytes_to_write = min(min(bytes_left, 30), _pageSize - offP);
		if (pollACK()) {
     	_tw->beginTransmission(_id);
    	_tw->write(address >> 8);
    	_tw->write(address & 0xFF);
    	uint8_t *adr = data+offD;
    	_tw->write(adr, bytes_to_write);
    	_tw->endTransmission();
    } else return false;
		bytes_left -= bytes_to_write;
		offD += bytes_to_write;
		address += bytes_to_write;
	}
	return true;
}

/**
 * Reading from current address
 */
uint8_t AT24CXX::read(){
	if(pollACK()){
		_tw->requestFrom(_id, 1);
		return _tw->read();
	}
	else return 0;
}

/**
 * Reading a byte
 */
uint8_t AT24CXX::read(unsigned int address) {
	uint8_t b = 0;
	int r = 0;
	if(pollACK()) {
		_tw->beginTransmission(_id);
		_tw->write(address >> 8);
		_tw->write(address & 0xFF);
		if (_tw->endTransmission()==0) {
			_tw->requestFrom(_id, 1);
			while (_tw->available() > 0 && r<1) {
				b = (uint8_t)_tw->read();
				r++;
			}
		}
	}
	return b;
}

/**
 * Reading sequence of n bytes
 */
uint8_t* AT24CXX::read(unsigned int address, uint8_t *data, int n) {
	int c = n;
	int offD = 0;
	// read until n bytes are read
	while (c > 0) {
		int nc = c;
		if (nc > 32)
			nc = 32;
		if(pollACK()){
			_tw->beginTransmission(_id);
			_tw->write(address >> 8);
			_tw->write(address & 0xFF);
			if (_tw->endTransmission()==0) {
				int r = 0;
				_tw->requestFrom(_id, nc);
				while (_tw->available() > 0 && r<nc) {
					data[offD+r] = (uint8_t)_tw->read();
					r++;
				}
			}
		}
		address+=nc;
		offD+=nc;
		c-=nc;
	}
  return data;
}