/* BMA253 I2C library - offers a class to handle the accelerometer
 * created by j.dohm
 * date: 2020-07-29
 */
#ifndef _BMA253_Library
#define _BMA253_Library

//----------- includes --------------//
#include <mbed.h>
#include "SEGGER_RTT.h"
//----------- defines ---------------//
#define _debug
// 7-Bit address 0x18, 8-bit address 0x30
#define ADDRESS_ONE 	0x30
// 7-Bit address 0x19, 8-bit address 0x32
#define ADDRESS_TWO 	0x32
// register addresses
#define REG_ACC_X 	0x02 //read including 0x03 (12 bit data) | 0x02 LSB, 0x03 MSB
#define REG_ACC_Y 	0x04
#define REG_ACC_Z 	0x06
#define REG_TEMP 		0x08
#define REG_INTERRUPT_ACTIVE 0x09
#define REG_RANGE		0x0F
// 0011	| 2g	| 0.98mg/LSB
// 0101 | 4g	| 1.95mg/LSB
// 1000 | 8g	| 3.91mg/LSB
// 1100 | 16g | 7.81mg/LSB
#define RANGE_2G	0b0011
#define RANGE_4G	0b0101
#define RANGE_8G	0b1000
#define RANGE_16G	0b1100

#define REG_ACC_FILTER_BW	0x10 //Bandwidth 
// bits		|	f				| Update Time
// 01000 	| 7.81Hz	| 64ms
// 01001	| 15.63Hz	| 32ms
// 01010 	| 31.25Hz | 16ms
// 01011 	| 62.5Hz	| 8ms
// 01100 	| 125Hz		| 4ms
// 01101 	| 250Hz		| 2ms
// 01110 	| 500Hz		| 1ms
// 01111 	| 1000Hz	| 0.5ms
#define ACC_FILTER_BW_8HZ 		0b01000
#define ACC_FILTER_BW_16HZ 		0b01001
#define ACC_FILTER_BW_31HZ 		0b01010
#define ACC_FILTER_BW_63HZ 		0b01011
#define ACC_FILTER_BW_125HZ 	0b01100
#define ACC_FILTER_BW_250HZ 	0b01101
#define ACC_FILTER_BW_500HZ 	0b01110
#define ACC_FILTER_BW_1000HZ 	0b01111

#define REG_ACC_FILTER		0x13 //activate/deactivate Filter
#define ACC_FILTER_ON 		0x80 
#define ACC_FILTER_OFF 		0x00

#define REG_INTERRUPT_SLOPE_AXIS 	0x16
#define INTERRUPT_SLOPE_AXIS_X 		0x01
#define INTERRUPT_SLOPE_AXIS_Y 		0x02
#define INTERRUPT_SLOPE_AXIS_Z 		0x04
#define INTERRUPT_D_TAP_EN		 		0x10

// A safe way to change parameters of an enabled interrupt
// is to keep the following sequence: 
// disable the desired interrupt, change parameters, wait for at
// least 10ms, and then re-enable the desired interrupt.
#define REG_INTERRUPT_FILTER 0x1E
//  ́0 ́ ( ́1 ́) selects filtered (unfiltered)
// use read data | define to set unfiltered
// use read data &! define to set filtered
#define INTERRUPT_FILTER_DATA 				0b00100000
#define INTERRUPT_FILTER_TAP 					0b00010000
#define INTERRUPT_FILTER_SLO_NO_MOT		0b00001000
#define INTERRUPT_FILTER_SLOPE				0b00000100
#define INTERRUPT_FILTER_HIGH					0b00000010
#define INTERRUPT_FILTER_LOW 					0b00000001
 
#define REG_INTERRUPT_PIN_1 	0x19
#define REG_INTERRUPT_PIN_12 	0x1A
#define REG_INTERRUPT_PIN_2		0x1B
#define INTERRUPT_PIN_LOW			0b00000001
#define INTERRUPT_PIN_HIGH		0b00000010
#define INTERRUPT_PIN_SLOPE		0b00000100
#define INTERRUPT_PIN_SLO_NO_MOT 0b00001000
#define INTERRUPT_PIN_D_TAP		0b00010000
#define INTERRUPT_PIN_S_TAP		0b00100000
#define INTERRUPT_PIN_ORIENT	0b01000000
#define INTERRUPT_PIN_FLAT		0b10000000
#define INTERRUPT_PIN_DATA		0x0100
#define INTERRUPT_PIN_FWM			0x0200
#define INTERRUPT_PIN_FFULL		0x0400
#define INTERRUPT_PIN_INT1		0x01
#define INTERRUPT_PIN_INT2		0x02

#define REG_INTERRUPT_EL_BEHAVIOUR 0x20
#define INTERRUPT_EL_BEHAVIOUR_PUSHPULL 	0b00
#define INTERRUPT_EL_BEHAVIOUR_OPENDRIVE 	0b10
#define INTERRUPT_EL_BEHAVIOUR_LVL_NORMAL 0b1
#define INTERRUPT_EL_BEHAVIOUR_LVL_INV 		0b0

#define REG_INTERRUPT_MODE 0x21
#define INTERRUPT_MODE_NON_LATCHED 0b0000
#define INTERRUPT_MODE_TMP_250ms	 0b0001
#define INTERRUPT_MODE_TMP_500ms	 0b0010
#define INTERRUPT_MODE_TMP_1s			 0b0011
#define INTERRUPT_MODE_TMP_2s			 0b0100
#define INTERRUPT_MODE_TMP_4s			 0b0101
#define INTERRUPT_MODE_TMP_8s			 0b0110
//#define INTERRUPT_MODE_LATCHED 		 0b0111
//#define INTERRUPT_MODE_NON_LATCHED 0b1000
#define INTERRUPT_MODE_TMP_250us	 0b1001
#define INTERRUPT_MODE_TMP_500us	 0b1010
#define INTERRUPT_MODE_TMP_1ms	 	 0b1011
#define INTERRUPT_MODE_TMP_12ms	 	 0b1100 //12.5
#define INTERRUPT_MODE_TMP_25ms	 	 0b1101
#define INTERRUPT_MODE_TMP_50ms	 	 0b1110
#define INTERRUPT_MODE_LATCHED 		 0b1111
#define INTERRUPT_RESET 		 	 0b10001111 //resets the interrupt in latched mode (and stays in latched mode)

#define REG_INTERRUPT_SLOPE_DUR 			 0x27
#define REG_INTERRUPT_SLOPE_THREASHOLD 0x28

struct bmaData {
	int16_t x;   //12bit starting at MSB
	int16_t y;
	int16_t z;
	int8_t temp; //23 grad C +0.5K/LSB
	bool int1;
	bool int2;
};

//----------- class -----------------//
class BMA253
{
	public:
  //----------- variables -------------//
	//----------- constructor -----------//
	BMA253(uint8_t address = ADDRESS_ONE);
	//----------- functiones ------------//
	uint8_t begin(I2C &i2c);
	//Accelerations
	bmaData read();
	uint8_t setFilter(uint8_t set = ACC_FILTER_ON);
	uint8_t setFilterBW(uint8_t bw = ACC_FILTER_BW_250HZ);
	uint8_t setRange(uint8_t range = RANGE_2G);
	//Chip temp
	bmaData readTemp();
	//Interrupts
	// chose Int Pin (INTERRUPT_PIN_INT*), and behaviour (push pull or open drain) (INTERRUPT_EL_BEHAVIOUR_*) to be set.
	// optional set the lvl (INTERRUPT_EL_BEHAVIOUR_LVL_*) (inverted or normal)
	uint8_t setElIntBehaviour(uint8_t pin, uint8_t beh, uint8_t lvl = INTERRUPT_EL_BEHAVIOUR_LVL_NORMAL);
	// chose Int Pin (INTERRUPT_PIN_INT*), and interrupt (INTERRUPT_PIN_*) to be set.
	// optional set the interrupt mode (INTERRUPT_MODE_*)
	uint8_t activateInt(uint8_t pin, uint16_t interrupt);
	uint8_t activateInt(uint8_t pin, uint16_t interrupt, uint8_t mode);
	// chose Int Pin (INTERRUPT_PIN_INT*), and interrupt (INTERRUPT_PIN_*) to be shutoff.
	uint8_t deactivateInt(uint8_t pin, uint16_t interrupt);
	//KnockOn
	uint8_t knockOnInt(bool set, uint8_t pin = INTERRUPT_PIN_INT2);
	//Movementdetection
	uint8_t moveInt(bool set, uint8_t pin = INTERRUPT_PIN_INT1);
	uint8_t moveIntSetThreashold(uint8_t thr);
	uint8_t moveIntSetThreashold(uint8_t thr, uint8_t slope_dur);
	//basic functions
	uint8_t writeByte(char reg, char dat);
	uint8_t readByte(char reg);
	private:
  	//----------- variables -------------//
    uint8_t _address;
    I2C *_i2c;
		uint8_t _range = RANGE_2G;
		//----------- functiones ------------//
};
#endif
