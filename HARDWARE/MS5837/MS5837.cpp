#include "MS5837.h"
#include "math.h"
#include "myiic.h"
#include "delay.h"
#include "usart.h"

#define MS5837_ADDR               0xEC	//0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;


static uint16_t readFromMs5837(uint8_t add)
{
	uint8_t dataArr[2] = {0,0};
	
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	delay_us(10);
	IIC_Send_Byte(add);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_us(20);
	
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR|0x01);
	IIC_Wait_Ack();	
	delay_us(10);
	dataArr[0] = IIC_Read_Byte(1);
	delay_us(10);
	dataArr[1] = IIC_Read_Byte(0);
	IIC_Stop();
	delay_us(30);
	
	return (uint16_t)(dataArr[0]<<8)|dataArr[1];
}

MS5837::MS5837() {
	fluidDensity = 1029;
}

bool MS5837::init() {
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	IIC_Send_Byte(MS5837_RESET);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(10);

	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		C[i] = readFromMs5837(MS5837_PROM_READ+i*2);
	}

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);
	printf("crcRead:%X\tcrcCalculated:%X\n", crcRead, crcCalculated);
	delay_ms(10);

	if ( crcCalculated == crcRead ) {
		return true; // Initialization success
	}

	return false; // CRC fail
}

void MS5837::setModel(uint8_t model) {
	_model = model;
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::read() {
	// Request D1 conversion
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	delay_us(10);
	IIC_Send_Byte(MS5837_CONVERT_D1_8192);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(20);
	
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	delay_us(10);
	IIC_Send_Byte(MS5837_ADC_READ);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_us(20);

	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR|0x01);
	IIC_Wait_Ack();	
	delay_us(10);
	D1 = 0;
	D1 = IIC_Read_Byte(1);
	delay_us(10);
	D1 = (D1 << 8) | IIC_Read_Byte(1);
	delay_us(10);
	D1 = (D1 << 8) | IIC_Read_Byte(0);
	IIC_Stop();
	delay_us(45);
	
	// Request D2 conversion
	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	delay_us(10);
	IIC_Send_Byte(MS5837_CONVERT_D2_8192);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(20);

	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR);
	IIC_Wait_Ack();
	delay_us(10);
	IIC_Send_Byte(MS5837_ADC_READ);
	IIC_Wait_Ack();
	IIC_Stop();
	delay_us(20);

	IIC_Start();
	IIC_Send_Byte(MS5837_ADDR|0x01);
	IIC_Wait_Ack();	
	D2 = 0;
	D2 = IIC_Read_Byte(1);
	delay_us(10);
	D2 = (D2 << 8) | IIC_Read_Byte(1);
	delay_us(10);
	D2 = (D2 << 8) | IIC_Read_Byte(0);
	IIC_Stop();
	delay_us(45);

	calculate();
}

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;  
	int32_t Ti = 0;    
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}
	
	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	
	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}
	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	TEMP = (TEMP-Ti);
	
	if ( _model == MS5837_02BA ) {
		P = (((D1*SENS2)/2097152l-OFF2)/32768l); 
	} else {
		P = (((D1*SENS2)/2097152l-OFF2)/8192l);
	}
}

float MS5837::pressure(float conversion) {
    if ( _model == MS5837_02BA ) {
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

float MS5837::depth() {
	return (pressure(MS5837::Pa)-2022000)/(fluidDensity*9.80665*20.05)*100;
}

float MS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
