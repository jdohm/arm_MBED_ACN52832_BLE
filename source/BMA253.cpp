/* BMA253 I2C library - offers a class to handle the accelerometer
 * created by j.dohm
 * date: 2020-07-29
 */

#include <mbed.h>
#include <BMA253.h>
#include "SEGGER_RTT.h"
//----------- constructor -----------//
BMA253::BMA253(uint8_t address){
	_address = address;
}
//----------- functiones ------------//
uint8_t BMA253::begin(I2C &i2c){
	#ifdef _debug
		SEGGER_RTT_printf(0,"#entered BMA253::begin\n");
	#endif
  _i2c = &i2c;
	if (writeByte(REG_RANGE,RANGE_2G)) return 1;
	if (writeByte(REG_ACC_FILTER_BW,ACC_FILTER_BW_250HZ)) return 1;
	if (writeByte(REG_ACC_FILTER,ACC_FILTER_ON)) return 1;
	#ifdef _debug
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_PIN_1  read: 0x%02X\n",readByte(REG_INTERRUPT_PIN_1));
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_PIN_12 read: 0x%02X\n",readByte(REG_INTERRUPT_PIN_12));
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_PIN_2  read: 0x%02X\n",readByte(REG_INTERRUPT_PIN_2));
	#endif
	return 0;
}
//Interrupts
uint8_t BMA253::setElIntBehaviour(uint8_t pin, uint8_t beh, uint8_t lvl){
	char _readData;
	switch (pin){
		case INTERRUPT_PIN_INT1:
			_readData = readByte(REG_INTERRUPT_EL_BEHAVIOUR);
			#ifdef _debug
				SEGGER_RTT_printf(0,"#REG_INTERRUPT_EL_BEHAVIOUR read: 0x%X\n",_readData);
			#endif
			//set only the behaviour bit
			if(beh == INTERRUPT_EL_BEHAVIOUR_OPENDRIVE)	_readData = _readData|beh;
			else _readData = _readData&!beh;
			//set only the level bit
			if(lvl == INTERRUPT_EL_BEHAVIOUR_LVL_NORMAL)	_readData = _readData|lvl;
			else _readData = _readData&!lvl;
			#ifdef _debug
				SEGGER_RTT_printf(0,"#REG_INTERRUPT_EL_BEHAVIOUR write: 0x%X\n",_readData);
			#endif
			//write the changed data back
			return writeByte(REG_INTERRUPT_EL_BEHAVIOUR,_readData);
		break;
		case INTERRUPT_PIN_INT2:
			_readData = readByte(REG_INTERRUPT_EL_BEHAVIOUR);
			#ifdef _debug
				SEGGER_RTT_printf(0,"#REG_INTERRUPT_EL_BEHAVIOUR read: 0x%X\n",_readData);
			#endif
			//set only the behaviour bit
			if(beh == INTERRUPT_EL_BEHAVIOUR_OPENDRIVE)	_readData = _readData|(beh<<2);
			else _readData = _readData&!(beh<<2);
			//set only the level bit
			if(beh == INTERRUPT_EL_BEHAVIOUR_LVL_NORMAL)	_readData = _readData|(lvl<<2);
			else _readData = _readData&!(lvl<<2);
			#ifdef _debug
				SEGGER_RTT_printf(0,"#REG_INTERRUPT_EL_BEHAVIOUR write: 0x%X\n",_readData);
			#endif
			//write the changed data back
			return writeByte(REG_INTERRUPT_EL_BEHAVIOUR,_readData);
		break;
		default:
			return 1;//error unknown pin
		break;
	}
}
// chose Int Pin (INTERRUPT_PIN_INT*), and interrupt (INTERRUPT_PIN_*) to be set.
// optional set the interrupt mode (INTERRUPT_MODE_*)
uint8_t BMA253::activateInt(uint8_t pin, uint16_t interrupt){
	char _readData;
	#ifdef _debug
		SEGGER_RTT_printf(0,"#entered activateInt with pin:0x%X and interrupt:0x%X\n",pin,interrupt);
	#endif
	switch (pin){
		case INTERRUPT_PIN_INT1:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_1);
				return writeByte(REG_INTERRUPT_PIN_1,_readData|interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				return writeByte(REG_INTERRUPT_PIN_12,_readData|uint8_t(interrupt>>8));
			}
		break;
		case INTERRUPT_PIN_INT2:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_2);
				return writeByte(REG_INTERRUPT_PIN_2,_readData|interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				//shift the bit to the bits for pin Int2
				uint8_t _writeData;
				_writeData = (uint8_t(interrupt>>1)|0x80);
				_writeData = _writeData|(uint8_t(interrupt>>3)|0x40);
				_writeData = _writeData|(uint8_t(interrupt>>5)|0x20);
				return writeByte(REG_INTERRUPT_PIN_12,_readData|_writeData);
			}
		break;
		default:
			return 1;//error unknown pin
		break;
	}
}

uint8_t BMA253::activateInt(uint8_t pin, uint16_t interrupt, uint8_t mode){
	char _readData;
	writeByte(REG_INTERRUPT_MODE,mode);
  ThisThread::sleep_for(12ms);
	switch (pin){
		case INTERRUPT_PIN_INT1:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_1);
				return writeByte(REG_INTERRUPT_PIN_1,_readData|interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				return writeByte(REG_INTERRUPT_PIN_12,_readData|uint8_t(interrupt>>8));
			}
		break;
		case INTERRUPT_PIN_INT2:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_2);
				return writeByte(REG_INTERRUPT_PIN_2,_readData|interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				//shift the bit to the bits for pin Int2
				uint8_t _writeData;
				_writeData = (uint8_t(interrupt>>1)|0x80);
				_writeData = _writeData|(uint8_t(interrupt>>3)|0x40);
				_writeData = _writeData|(uint8_t(interrupt>>5)|0x20);
				return writeByte(REG_INTERRUPT_PIN_12,_readData|_writeData);
			}
		break;
		default:
			return 1;
		break;
	}
}

// chose Int Pin (INTERRUPT_PIN_INT*), and interrupt (INTERRUPT_PIN_*) to be shutoff.
uint8_t BMA253::deactivateInt(uint8_t pin, uint16_t interrupt){
	char _readData;
	switch (pin){
		case INTERRUPT_PIN_INT1:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_1);
				return writeByte(REG_INTERRUPT_PIN_1,_readData&!interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				return writeByte(REG_INTERRUPT_PIN_12,_readData&!uint8_t(interrupt>>8));
			}
		break;
		case INTERRUPT_PIN_INT2:
			if(	interrupt <= 0xFF){
				_readData = readByte(REG_INTERRUPT_PIN_2);
				return writeByte(REG_INTERRUPT_PIN_2,_readData&!interrupt);
			}
			else{
				_readData = readByte(REG_INTERRUPT_PIN_12);
				//shift the bit to the bits for pin Int2
				uint8_t _writeData;
				_writeData = (uint8_t(interrupt>>1)|0x80);
				_writeData = _writeData|(uint8_t(interrupt>>3)|0x40);
				_writeData = _writeData|(uint8_t(interrupt>>5)|0x20);
				return writeByte(REG_INTERRUPT_PIN_12,_readData&!_writeData);
			}
		break;
		default:
			return 0;
		break;
	}
}

//KnockOn | sets the double tap interrupt
uint8_t BMA253::knockOnInt(bool set, uint8_t pin){
	char _readData;
	//activate doubla tab without changing other register values
	_readData = readByte(REG_INTERRUPT_SLOPE_AXIS);
	_readData = _readData|INTERRUPT_D_TAP_EN;
	writeByte(REG_INTERRUPT_SLOPE_AXIS,_readData);
	if(set) return activateInt(pin,INTERRUPT_PIN_D_TAP);
	else return deactivateInt(pin,INTERRUPT_PIN_D_TAP);
}
//Movementdetection
uint8_t BMA253::moveInt(bool set, uint8_t pin){
	char _readData;
	//activate all axis without changing other register values
	_readData = readByte(REG_INTERRUPT_SLOPE_AXIS);
	_readData = _readData|INTERRUPT_SLOPE_AXIS_X|INTERRUPT_SLOPE_AXIS_Y|INTERRUPT_SLOPE_AXIS_Z;
	writeByte(REG_INTERRUPT_SLOPE_AXIS,_readData);
	if(set) return activateInt(pin,INTERRUPT_PIN_SLOPE,INTERRUPT_MODE_TMP_250ms);
	else return deactivateInt(pin,INTERRUPT_PIN_SLOPE);
}

// In terms of scaling 1 LSB of (thr) corresponds to 3.91 mg in 2g-range 
// (7.81 mg in 4g-range, 15.6 mg in 8g-range and 31.3 mg in 16g-range). 
// Therefore the maximum value is 996 mg in 2g-range 
// (1.99g in 4g-range, 3.98g in 8g-range and 7.97g in 16g-range).
uint8_t BMA253::moveIntSetThreashold(uint8_t thr){
	uint8_t _activeInts;
	//check if Pin Int1 was connected to slope interrupt before
	// if so deactivate and safe the information
	if(readByte(REG_INTERRUPT_PIN_1)&INTERRUPT_PIN_SLOPE){
		deactivateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE);
		_activeInts = 0x01;
	}
	//check if Pin Int2 was connected to slope interrupt before
	// if so deactivate and safe the information
	if(readByte(REG_INTERRUPT_PIN_2)&INTERRUPT_PIN_SLOPE){
		deactivateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		_activeInts = _activeInts|0x02;
	}
	#ifdef _debug
		SEGGER_RTT_printf(0,"#slope was active on: 0x%X\n",_activeInts);
	#endif
	// return 1 (error) if writeByte returned error, otherwise continue
	// leaves interrupts deaktivated even if they where active before
 	if(writeByte(REG_INTERRUPT_SLOPE_THREASHOLD,thr)) return 1;
  ThisThread::sleep_for(12ms);
	switch (_activeInts){
		case 0x01:
			return activateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE);
		break;
		case 0x02:
			return activateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		break;
		case 0x03:
			//only return 0 (SUCCESS) if both activations returned success
			return activateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE)|activateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		break;
		default:
			//return 0 (SUCCESS) if writeByte was a success and no reg needs activation
			return 0;
		break;
	}
}
// The time difference between the successive acceleration signals 
// depends on the selected bandwidth and equates to 1/(2*bandwidth) (t=1/(2*bw)). 
// In order to suppress false triggers, the interrupt is only generated (cleared) 
// if a certain number N of consecutive slope data points is larger (smaller) 
// than the slope threshold given by thr. 
// This number is set by the slope_dur bits. 
// It is N = slope_dur + 1.
// Example: slope_dur = 00b, ..., 11b = 1decimal, ..., 4decimal.
uint8_t BMA253::moveIntSetThreashold(uint8_t thr, uint8_t slope_dur){
	uint8_t _activeInts;
	#ifdef _debug
		SEGGER_RTT_printf(0,"#entered moveIntSetThreashold with thr:0x%X and slope_dur:0x%X\n",thr,slope_dur);
	#endif
	if(slope_dur >= 0x04) return 1;// only 2 bits for slope_dur!
	//check if Pin Int1 was connected to slope interrupt before
	// if so deactivate and safe the information
	if(readByte(REG_INTERRUPT_PIN_1)&INTERRUPT_PIN_SLOPE){
		deactivateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE);
		_activeInts = 0x01;
	}
	//check if Pin Int2 was connected to slope interrupt before
	// if so deactivate and safe the information
	if(readByte(REG_INTERRUPT_PIN_2)&INTERRUPT_PIN_SLOPE){
		deactivateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		_activeInts = _activeInts|0x02;
	}
	#ifdef _debug
		SEGGER_RTT_printf(0,"#slope was active on: 0x%X\n",_activeInts);
	#endif
	//---- slope duration ----//
	//read old register data
	uint8_t _readData = readByte(REG_INTERRUPT_SLOPE_DUR);
	#ifdef _debug
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_SLOPE_DUR read: 0x%X\n",_readData);
	#endif
	//set only slope_dur and keep old values for rest of register
	// return 1 (error) if writeByte returned error, otherwise continue
	// leaves interrupts deaktivated even if they where active before
	#ifdef _debug
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_SLOPE_DUR write: 0x%X\n",(_readData&0xFC)|slope_dur);
	#endif
 	if(writeByte(REG_INTERRUPT_SLOPE_DUR,(_readData&0xFC)|slope_dur)) return 1;
	//---- slope dur end -----//
	// return 1 (error) if writeByte returned error, otherwise continue
	// leaves interrupts deaktivated even if they where active before
	#ifdef _debug
		SEGGER_RTT_printf(0,"#REG_INTERRUPT_SLOPE_THREASHOLD write: 0x%X\n",thr);
	#endif
 	if(writeByte(REG_INTERRUPT_SLOPE_THREASHOLD,thr)) return 1;
  ThisThread::sleep_for(12ms);
	switch (_activeInts){
		case 0x01:
			return activateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE);
		break;
		case 0x02:
			return activateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		break;
		case 0x03:
			//only return 0 (SUCCESS) if both activations returned success
			return activateInt(INTERRUPT_PIN_INT1,INTERRUPT_PIN_SLOPE)|activateInt(INTERRUPT_PIN_INT2,INTERRUPT_PIN_SLOPE);
		break;
		default:
			//return 0 (SUCCESS) if writeByte was a success and no reg needs activation
			return 0;
		break;
	}
}

//Accelerations
bmaData BMA253::read(){
	char data[6];
	bmaData bdat;
	data[0] = REG_ACC_X;
	_i2c->write(_address,data,1,true);
	_i2c->read(_address,data,6,false);
	bdat.x = int16_t((data[0]&0xF0) | int16_t(data[1]<<8))>>4;
	bdat.y = int16_t((data[2]&0xF0) | int16_t(data[3]<<8))>>4;
	bdat.z = int16_t((data[4]&0xF0) | int16_t(data[5]<<8))>>4;
	/*
	switch (_range){
		case RANGE_2G:
			bdat.xf = (bdat.x>>5)*0.98;
			bdat.yf = (bdat.y>>5)*0.98;
			bdat.zf = (bdat.z>>5)*0.98;
		break;
		case RANGE_4G:
			bdat.xf = (bdat.x>>5)*1.95;
			bdat.yf = (bdat.y>>5)*1.95;
			bdat.zf = (bdat.z>>5)*1.95;
		break;
		case RANGE_8G:
			bdat.xf = (bdat.x>>5)*3.91;
			bdat.yf = (bdat.y>>5)*3.91;
			bdat.zf = (bdat.z>>5)*3.91;
		break;
		case RANGE_16G:
			bdat.xf = (bdat.x>>5)*7.81;
			bdat.yf = (bdat.y>>5)*7.81;
			bdat.zf = (bdat.z>>5)*7.81;
		break;
		default:
			bdat.xf = 0.0;
			bdat.yf = 0.0;
			bdat.zf = 0.0;
		break;
	}
	*/
	return bdat;
}

uint8_t BMA253::setFilter(uint8_t set){
	return writeByte(REG_ACC_FILTER,set);
}

uint8_t BMA253::setFilterBW(uint8_t bw){
	return writeByte(REG_ACC_FILTER_BW,bw);
}

uint8_t BMA253::setRange(uint8_t range){
	return writeByte(REG_RANGE,range);
}

//Chip temp
bmaData BMA253::readTemp(){
	char data[1];
	bmaData bdat;
	data[0] = REG_TEMP;
	_i2c->write(_address,data,1,true);
	_i2c->read(_address,data,1,false);
	bdat.temp = (data[0]);
	return bdat;
}

//private
uint8_t BMA253::writeByte(char reg, char dat){
	char data[2];
	data[0] = reg;
	data[1] = dat;
 // write data
	_i2c->write(_address,data,2,false);
  ThisThread::sleep_for(2ms);
 // readByte
	if(dat == readByte(reg)) return 0;
	return 1;
}

uint8_t BMA253::readByte(char reg){
	char data;
	_i2c->write(_address,&reg,1,true);
	_i2c->read(_address,&data,1,false);
	return data;
}
