#include <EEPROM.h>
// https://github.com/MHeironimus/ArduinoJoystickLibrary
#include <Joystick.h>
// https://github.com/RobTillaart/HX711
#include <HX711.h>
// https://github.com/marcinbor85/SmartButton
#include <SmartButton.h>

#include "data.h"

#define ENABLE_ACCELERATOR true
#define ENABLE_BRAKE true
#define ENABLE_CLUTCH true
#define ENABLE_GHOST_CLUTCH false
#define ENABLE_GHOST_BUTTONS 1
#define ENABLE_GHOST_HATS 1

#define INVERT_ACCELERATOR true
#define INVERT_BRAKE false
#define INVERT_CLUTCH false

#define DEADZONE_PERCENTAGE 0.03	// Adds deadzones to top and bottom of range (throttle/clutch), based on percentage of total range
#define DEADZONE_PERCENTAGE_BRK 0.1	// Brake-specific lower deadzone
#define BRAKE_CALIBRATION_POINT 0.8	// Mapping point for maximum experienced point during calibration
									// 0.8 means that 100% output brake will be 20% more force than your calibrated 'max'
									// e.g. push your brake to your 80% 'standard' target force during calibration cycle

#define HX711SCALAR 511
	
// Accelerator on X, Brake on Y, Clutch on Z
//  X, Y, Z
// rX,rY,rZ
// rudder, throttle
// accelerator, brake, steering
Joystick_ joyInput(
	JOYSTICK_DEFAULT_REPORT_ID,
	JOYSTICK_TYPE_JOYSTICK,
	ENABLE_GHOST_BUTTONS, ENABLE_GHOST_HATS,
	ENABLE_ACCELERATOR, ENABLE_BRAKE, (ENABLE_CLUTCH ? true : ENABLE_GHOST_CLUTCH),
	false, false, true,
	false, false,
	false, false, false);

struct PedalData pedals;
HX711 brakeSensor;

using namespace smartbutton;
SmartButton buttonCal(CALIBRATE_PIN, SmartButton::InputType::NORMAL_HIGH);

const bool debugMode = true;
const bool debugSpam = false;

long debugTimer = 0;
const uint8_t debugInterval = 250; //ms

long usbTimer = 0;
const uint8_t usbInterval = 2; //ms

long calibrateTimer = 0;
const uint16_t calibrateInterval = 20000; // ms

#define LED_BLINK_COUNT 5
bool ledFlag = false;
uint8_t ledBlinkCounter = 0;
long ledTimer = 0;
uint16_t ledInterval = 2000; //ms


void setup()
{
	// Serial
	Serial.begin(115200);
	// Joystick
	joyInput.begin(false);
	// Button
	pinMode(CALIBRATE_PIN, INPUT_PULLUP);
	buttonCal.begin(buttonPress);
	// LED
	pinMode(CAL_LED_PIN, OUTPUT);
	// Check/get eeprom
	getEEPROM();
	// Brake HX711 setup
	if (ENABLE_BRAKE){
		pedals.brake.hx711 = &brakeSensor;
		pedals.brake.hx711->begin(HX711DAT_PIN, HX711SCK_PIN);
		pedals.brake.hx711->tare();
	}
	pinMode(ACCELERATOR_PIN, INPUT);
	pinMode(CLUTCH_PIN, INPUT);
	// Inverts from defines
	pedals.accelerator.invert = INVERT_ACCELERATOR;
	pedals.brake.invert = INVERT_BRAKE;
	pedals.clutch.invert = INVERT_CLUTCH;
	// Set up the joystick ranges after eeprom load
	joyRangeSettings();
}

void loop()
{
	ledAction();
	SmartButton::service();
	getAxes();
	if (debugSpam){
		if ((debugTimer + debugInterval) < millis()) {
			debugReport();
		}
	}
	if (pedals.calibrateFlag) {
		processCalibrate();
	}
	if (pedals.eepromChangeFlag) {
		putEEPROM();
	}
	if ((usbTimer + usbInterval) < millis()) {
		joyInput.sendState();
	}
}

void joyRangeSettings()
{
	joyInput.setXAxisRange(AXIS_MIN, AXIS_MAX);
	joyInput.setYAxisRange(AXIS_MIN, AXIS_MAX);
	joyInput.setZAxisRange(AXIS_MIN, AXIS_MAX);
}

void ledAction()
{
	if (ledFlag) {
		if ((ledTimer + ledInterval) < millis()) {
			if (ledBlinkCounter < LED_BLINK_COUNT){
				bool onoff = (ledBlinkCounter % 2);
				digitalWrite(CAL_LED_PIN, !onoff);
				ledBlinkCounter++;
				ledTimer = millis();
			} else {
				ledFlag = false;
				ledBlinkCounter = 0;
				digitalWrite(CAL_LED_PIN, false);
			}
		}
	}
}

void getAxes()
{
	if (!pedals.calibrateFlag) {
		int16_t outval = 0;
		if (ENABLE_ACCELERATOR) {
			pedals.accelerator.value = analogRead(ACCELERATOR_PIN);
			pedals.accelerator.value = clampu(pedals.accelerator.value, pedals.accelerator.min, pedals.accelerator.max);
			if (pedals.accelerator.invert) {
				outval = map(pedals.accelerator.value, pedals.accelerator.max, pedals.accelerator.min, AXIS_MIN, AXIS_MAX);
			} else {
				outval = map(pedals.accelerator.value, pedals.accelerator.min, pedals.accelerator.max, AXIS_MIN, AXIS_MAX);
			}
			joyInput.setXAxis(outval);
		}
		if (ENABLE_BRAKE) {
			pedals.brake.value = readHX711pedal(pedals.brake.hx711);
			pedals.brake.value = clampu(pedals.brake.value, pedals.brake.min, pedals.brake.max);
			if (pedals.brake.invert) {
				outval = map(pedals.brake.value, pedals.brake.max, pedals.brake.min, AXIS_MIN, AXIS_MAX);
			} else {
				outval = map(pedals.brake.value, pedals.brake.min, pedals.brake.max, AXIS_MIN, AXIS_MAX);
			}
			joyInput.setYAxis(outval);
		}
		if (ENABLE_CLUTCH) {
			pedals.clutch.value = analogRead(CLUTCH_PIN);
			pedals.clutch.value = clampu(pedals.clutch.value, pedals.clutch.min, pedals.clutch.max);
			if (pedals.clutch.invert) {
				outval = map(pedals.clutch.value, pedals.clutch.max, pedals.clutch.min, AXIS_MIN, AXIS_MAX);
			} else {
				outval = map(pedals.clutch.value, pedals.clutch.min, pedals.clutch.max, AXIS_MIN, AXIS_MAX);
			}
			joyInput.setZAxis(outval);
		}
	} else {
		pedals.accelerator.value = analogRead(ACCELERATOR_PIN);
		pedals.brake.value = readHX711pedal(pedals.brake.hx711);
		pedals.clutch.value = analogRead(CLUTCH_PIN);
	}
}

int16_t clampu(int16_t val, int16_t min, int16_t max)
{
	int16_t out = 0;
	if (val > max) {out = max;}
	else if (val < min) {out = min;}
	else {out = val;}
	return out;
}

int16_t readHX711pedal(HX711* sensor_in)
{
	float loadcellValue = sensor_in->read() / (float)HX711SCALAR;
	return static_cast<int16_t>(loadcellValue);
}

void buttonPress(SmartButton *button, SmartButton::Event event, int clickCounter)
{
	if (event == SmartButton::Event::CLICK) {
		// Calibration Mode
		startCalibrate();
	} else if (event == SmartButton::Event::LONG_HOLD) {
		// Reset EEPROM
		resetEEPROM();
	}
}

void startCalibrate()
{
	ledInterval = 1000;//ms
	ledFlag = true;
	pedals.calibrateFlag = true;
	pedals.accelerator.min = AXIS_MAX;
	pedals.accelerator.max = AXIS_MIN;
	pedals.brake.min = AXIS_MAX;
	pedals.brake.max = AXIS_MIN;
	pedals.clutch.min = AXIS_MAX;
	pedals.clutch.max = AXIS_MIN;
	if (debugMode) {
		Serial.println(F("Start calibration"));
	}
	// Timestamp for start of calibration period
	calibrateTimer = millis();
}

void processCalibrate()
{
	if (pedals.accelerator.value < pedals.accelerator.min) {pedals.accelerator.min = pedals.accelerator.value;}
	if (pedals.accelerator.value > pedals.accelerator.max) {pedals.accelerator.max = pedals.accelerator.value;}
	if (pedals.brake.value < pedals.brake.min) {pedals.brake.min = pedals.brake.value;}
	if (pedals.brake.value > pedals.brake.max) {pedals.brake.max = pedals.brake.value;}
	if (pedals.clutch.value < pedals.clutch.min) {pedals.clutch.min = pedals.clutch.value;}
	if (pedals.clutch.value > pedals.clutch.max) {pedals.clutch.max = pedals.clutch.value;}
	// Watch for end of calibration period
	if (calibrateTimer + calibrateInterval < millis()) {
		finishCalibration();
		if (debugMode) {
			Serial.println(F("End calibration"));
		}
	}
}

void finishCalibration()
{
	pedals.calibrateFlag = false;
	pedals.eepromChangeFlag = true;
	Serial.println("Raw Calibration Values:");
	debugReport();
	if (ENABLE_BRAKE) {
		calcBrakeMax();
		Serial.println("Brake Calibration Values:");
		debugReport();
	}
	addDeadzones();
	Serial.println("DZ Calibration Values:");
	debugReport();
	//joyRangeSettings();
}

void calcBrakeMax()
{
	pedals.brake.max = (int)(pedals.brake.max / BRAKE_CALIBRATION_POINT);
}

void invertAxes()
{
	int16_t endA;
	int16_t endB;
	if (INVERT_ACCELERATOR) {
		endA = pedals.accelerator.min;
		endB = pedals.accelerator.max;
		pedals.accelerator.min = endB;
		pedals.accelerator.max = endA;
	}
	if (INVERT_BRAKE) {
		endA = pedals.brake.min;
		endB = pedals.brake.max;
		pedals.brake.min = endB;
		pedals.brake.max = endA;
	}
	if (INVERT_CLUTCH) {
		endA = pedals.clutch.min;
		endB = pedals.clutch.max;
		pedals.clutch.min = endB;
		pedals.clutch.max = endA;
	}
}

void addDeadzones()
{
	int16_t dz;
	int16_t range;
	if (ENABLE_ACCELERATOR) {
		range = pedals.accelerator.max - pedals.accelerator.min;
		dz = range * DEADZONE_PERCENTAGE;
		pedals.accelerator.min += dz;
		pedals.accelerator.max -= dz;
	}
	if (ENABLE_BRAKE) {
		// brake calculates dz on the bottom only
		// Uses brake-specific deadzone %
		range = pedals.brake.max - pedals.brake.min;
		dz = range * DEADZONE_PERCENTAGE_BRK;
		pedals.brake.min += dz;
	}
	if (ENABLE_CLUTCH) {
		range = pedals.clutch.max - pedals.clutch.min;
		dz = range * DEADZONE_PERCENTAGE;
		pedals.clutch.min += dz;
		pedals.clutch.max -= dz;
	}
}

void debugReport()
{
	Serial.print("accelerator: ");
	Serial.print(pedals.accelerator.value);
	Serial.print(" (min/max ");
	Serial.print(pedals.accelerator.min);
	Serial.print("/");
	Serial.print(pedals.accelerator.max);
	Serial.print(") ");
	Serial.print("brake: ");
	Serial.print(pedals.brake.value);
	Serial.print(" / ");
	Serial.print(pedals.brake.hx711->read());
	Serial.print(" (min/max ");
	Serial.print(pedals.brake.min);
	Serial.print("/");
	Serial.print(pedals.brake.max);
	Serial.print(") ");
	Serial.print("clutch: ");
	Serial.print(pedals.clutch.value);
	Serial.print(" (min/max ");
	Serial.print(pedals.clutch.min);
	Serial.print("/");
	Serial.print(pedals.clutch.max);
	Serial.println(") ");
	//Reset timer
	debugTimer = millis();
}

void getEEPROM()
{
	PedalData eepromData;
	EEPROM.get(0, eepromData);
	if (eepromData.eepromDataFlag) {
		if (debugMode) {
			Serial.println(F("Getting EEPROM"));
		}
		pedals = eepromData;
	} else {
		if (debugMode) {
			Serial.println(F("EEPROM flagged bad"));
		}
	}
	pedals.eepromChangeFlag = false;
}

void resetEEPROM()
{
	if (debugMode) {
		Serial.println(F("Resetting EEPROM"));
	}
	ledInterval = 100;//ms
	ledFlag = true;
	pedals.eepromDataFlag = false;
	EEPROM.put(0, 0);
}

void putEEPROM()
{
	if (debugMode) {
		Serial.println(F("Saving EEPROM"));
	}
	pedals.eepromDataFlag = true;
	pedals.eepromChangeFlag = false;
	EEPROM.put(0, pedals);
}
