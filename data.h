#ifndef __DATA_H__
#define __DATA_H__

#include <HX711.h>

// Hardware Version
// HW_V_10	V1.0 PCB from fab
#define HW_PCB_V10

#ifdef HW_PCB_V10
	#define ACCELERATOR_PIN A3
	#define HX711DAT_PIN 2
	#define HX711SCK_PIN 3
	#define CLUTCH_PIN A2
	#define CALIBRATE_PIN 4
	#define CAL_LED_PIN 17
#endif // HW_PCB_V10

#define AXIS_MIN -32768
#define AXIS_MAX 32767
enum AxisByName : uint8_t
{
	AXIS_NULL,
	AXIS_THROTTLE,
	AXIS_BRAKE,
	AXIS_CLUTCH
};

typedef struct AxisData
{
	bool invert;
	HX711* hx711 = 0;
	int16_t value = 0;
	int16_t min = AXIS_MIN;
	int16_t max = AXIS_MAX;
};

typedef struct PedalData
{
	bool eepromDataFlag = false;
	bool eepromChangeFlag = false;
	bool calibrateFlag = false;
	uint16_t flashLEDInterval;
	AxisData accelerator;
	AxisData brake;
	AxisData clutch;
};

#endif	// __DATA_H__
