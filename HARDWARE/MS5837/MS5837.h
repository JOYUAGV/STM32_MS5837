#ifndef __MS5837_H
#define __MS5837_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

class MS5837 {
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;

	MS5837();

	bool init();
	void setModel(uint8_t model);
	void setFluidDensity(float density);
	void read();

	float pressure(float conversion = 1.0f);
	float temperature();
	float depth();
	float altitude();

private:
	uint16_t C[8];
	uint32_t D1, D2;
	int32_t TEMP;
	int32_t P;
	uint8_t _model;

	float fluidDensity;
	void calculate();

	uint8_t crc4(uint16_t n_prom[]);
};

#ifdef __cplusplus
}
#endif

#endif
