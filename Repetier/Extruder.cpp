/*
	This file is part of Repetier-Firmware.

	Repetier-Firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Repetier-Firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

	This firmware is a nearly complete rewrite of the sprinter firmware
	by kliment (https://github.com/kliment/Sprinter)
	which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"
#include "pins_arduino.h"
#include "ui.h"
#if EEPROM_MODE!=0
#include "Eeprom.h"
#include "Extruder.h"
#endif


uint8_t manageMonitor = 0; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed, Changed from 255 to 0
unsigned int counterPeriodical = 0;
volatile uint8_t executePeriodical = 0;
uint8_t counter250ms = 25;
#if FEATURE_DITTO_PRINTING
uint8_t Extruder::dittoMode = 0;
#endif

#ifdef SUPPORT_MAX6675
extern int16_t read_max6675(uint8_t ss_pin);
#endif
#ifdef SUPPORT_MAX31855
extern int16_t read_max31855(uint8_t ss_pin);
#endif

#if ANALOG_INPUTS>0
const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
uint8 osAnalogInputCounter[ANALOG_INPUTS];
uint osAnalogInputBuildup[ANALOG_INPUTS];
uint8 osAnalogInputPos = 0; // Current sampling position
volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif

#ifdef USE_GENERIC_THERMISTORTABLE_1
short temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
short temptable_generic2[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
short temptable_generic3[GENERIC_THERM_NUM_ENTRIES][2];
#endif
/** Makes updates to temperatures and heater state every call.

Is called every 100ms.
*/
static uint8_t extruderTempErrors = 0;
//void Extruder::manageTemperatures()
//{
//#if FEATURE_WATCHDOG
//	HAL::pingWatchdog();
//#endif // FEATURE_WATCHDOG
//	uint8_t errorDetected = 0;
//	for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
//	{
//		if (controller == autotuneIndex) continue;
//		TemperatureController *act = tempController[controller];
//		// Get Temperature
//		//int oldTemp = act->currentTemperatureC;
//		//act->updateCurrentTemperature();
//		if (controller < NUM_EXTRUDER)
//		{
//#if NUM_EXTRUDER>=2 && EXT0_EXTRUDER_COOLER_PIN==EXT1_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN>=0
//			if (controller == 1 && autotuneIndex != 0 && autotuneIndex != 1)
//				if (tempController[0]->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && tempController[0]->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP &&
//					tempController[1]->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && tempController[1]->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
//					extruder[0].coolerPWM = 0;
//				else
//					extruder[0].coolerPWM = extruder[0].coolerSpeed;
//			if (controller > 1)
//#endif // NUM_EXTRUDER
//				if (act->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
//					extruder[controller].coolerPWM = 0;
//				else
//					extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
//		}
//		if (!Printer::isAnyTempsensorDefect() && (act->currentTemperatureC < MIN_DEFECT_TEMPERATURE || act->currentTemperatureC > MAX_DEFECT_TEMPERATURE))   // no temp sensor or short in sensor, disable heater
//		{
//			extruderTempErrors++;
//			errorDetected = 1;
//			if (extruderTempErrors > 10)   // Ignore short temporary failures
//			{
//				Printer::flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
//				//reportTempsensorError();
//			}
//		}
//		if (Printer::isAnyTempsensorDefect()) continue;
//		uint8_t on = act->currentTemperature >= act->targetTemperature ? LOW : HIGH;
//		if (!on && act->isAlarm()) {
//			beep(50 * (controller + 1), 3);
//		// 	act->setAlarm(false);  //reset alarm
//		}
//#ifdef TEMP_PID
//		act->tempArray[act->tempPointer++] = act->currentTemperatureC;
//		act->tempPointer &= 3;
//		if (act->heatManager == 1)
//		{
//			uint8_t output;
//			float error = act->targetTemperatureC - act->currentTemperatureC;
//			if (act->targetTemperatureC < 20.0f) output = 0; // off is off, even if damping term wants a heat peak!
//			else if (error > PID_CONTROL_RANGE)
//				output = act->pidMax;
//			else if (error < -PID_CONTROL_RANGE)
//				output = 0;
//			else
//			{
//				float pidTerm = act->pidPGain * error;
//				act->tempIState = constrain(act->tempIState + error, act->tempIStateLimitMin, act->tempIStateLimitMax);
//				pidTerm += act->pidIGain * act->tempIState*0.1;
//				long dgain = act->pidDGain * (act->tempArray[act->tempPointer] - act->currentTemperatureC)*3.333f;
//				pidTerm += dgain;
//#if SCALE_PID_TO_MAX==1
//				pidTerm = (pidTerm*act->pidMax)*0.0039062;
//#endif
//				output = constrain((int)pidTerm, 0, act->pidMax);
//			}
//			pwm_pos[act->pwmIndex] = output;
//		}
//		else if (act->heatManager == 3)     // deat-time control
//		{
//			uint8_t output;
//			float error = act->targetTemperatureC - act->currentTemperatureC;
//			if (act->targetTemperatureC < 20.0f)
//				output = 0; // off is off, even if damping term wants a heat peak!
//			else if (error > PID_CONTROL_RANGE)
//				output = act->pidMax;
//			else if (error < -PID_CONTROL_RANGE)
//				output = 0;
//			else
//			{
//				float raising = 3.333 * (act->currentTemperatureC - act->tempArray[act->tempPointer]); // raising dT/dt, 3.33 = reciproke of time interval (300 ms)
//				act->tempIState = 0.25 * (3.0 * act->tempIState + raising); // damp raising
//				output = (act->currentTemperatureC + act->tempIState * act->pidPGain > act->targetTemperatureC ? 0 : output = act->pidDriveMax);
//			}
//			pwm_pos[act->pwmIndex] = output;
//		}
//		else
//#endif
//			if (act->heatManager == 2)    // Bang-bang with reduced change frequency to save relais life
//			{
//				unsigned long time = HAL::timeInMilliseconds();
//				if (time - act->lastTemperatureUpdate > HEATED_BED_SET_INTERVAL)
//				{
//					pwm_pos[act->pwmIndex] = (on ? 255 : 0);
//					act->lastTemperatureUpdate = time;
//				}
//			}
//			else     // Fast Bang-Bang fallback
//			{
//				pwm_pos[act->pwmIndex] = (on ? 255 : 0);
//			}
//#ifdef MAXTEMP
//		if (act->currentTemperatureC > MAXTEMP) // Force heater off if MAXTEMP is exceeded
//			pwm_pos[act->pwmIndex] = 0;
//#endif
//#if LED_PIN>-1
//		if (act == &Extruder::current->tempControl)
//			WRITE(LED_PIN, on);
//#endif
//	}
//	if (errorDetected == 0 && extruderTempErrors > 0)
//		extruderTempErrors--;
//	if (Printer::isAnyTempsensorDefect())
//	{
//		for (uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
//		{
//			pwm_pos[tempController[i]->pwmIndex] = 0;
//		}
//		Printer::debugLevel |= 8; // Go into dry mode
//	}
//
//}




/** \brief Initalizes all extruder.

Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder()
{
	uint8_t i = 0;
	Extruder::current = &extruder[i];
#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN>-1
	SET_OUTPUT(EXT0_DIR_PIN);
	SET_OUTPUT(EXT0_STEP_PIN);
#endif

	for (i = 0; i < NUM_EXTRUDER; ++i)
	{
		Extruder *act = &extruder[i];
		if (act->enablePin > -1)
		{
			HAL::pinMode(act->enablePin, OUTPUT);
			if (!act->enableOn) HAL::digitalWrite(act->enablePin, HIGH);
		}
		HAL::analogStart();

	}
	std::Extruder *Extruder::current;

	//bool reportTempsensorError()
	//{
	//    if(!Printer::isAnyTempsensorDefect()) return false;
	//    for(uint8_t i=0; i<NUM_TEMPERATURE_LOOPS; i++)
	//    {
	//        int temp = tempController[i]->currentTemperatureC;
	//        if(i==NUM_EXTRUDER) Com::printF(Com::tHeatedBed);
	//        else Com::printF(Com::tExtruderSpace,i);
	//        if(temp<MIN_DEFECT_TEMPERATURE || temp>MAX_DEFECT_TEMPERATURE)
	//        {
	//            Com::printFLN(Com::tTempSensorDefect);
	//        }
	//        else Com::printFLN(Com::tTempSensorWorking);
	//    }
	//    Com::printErrorFLN(Com::tDryModeUntilRestart);
	//    return true;
	//}

//#ifdef SUPPORT_MAX6675
//	int16_t read_max6675(uint8_t ss_pin)
//	{
//		int16_t max6675_temp = 0;
//		HAL::spiInit(1);
//		HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX6675
//		HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine
//		max6675_temp = HAL::spiReceive(0);
//		max6675_temp <<= 8;
//		max6675_temp |= HAL::spiReceive(0);
//		HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX6675
//		return max6675_temp & 4 ? 2000 : max6675_temp >> 3; // thermocouple open?
//	}
//#endif
//#ifdef SUPPORT_MAX31855
//	int16_t read_max31855(uint8_t ss_pin)
//	{
//		uint32_t data = 0;
//		int16_t temperature;
//		HAL::spiInit(1);
//		HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX31855
//		HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine
//
//		for (unsigned short byte = 0; byte < 4; byte++)
//		{
//			data <<= 8;
//			data |= HAL::spiReceive();
//		}
//
//		HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX31855
//
//		//Process temp
//		if (data & 0x00010000)
//			return 20000; //Some form of error.
//		else
//		{
//			data = data >> 18;
//			temperature = data & 0x00001FFF;
//
//			if (data & 0x00002000)
//			{
//				data = ~data;
//				temperature = -1 * ((data & 0x00001FFF) + 1);
//			}
//		}
//		return temperature;
//	}
//#endif


#if NUM_EXTRUDER>0
	const char ext0_select_cmd[] PROGMEM = EXT0_SELECT_COMMANDS;
	const char ext0_deselect_cmd[] PROGMEM = EXT0_DESELECT_COMMANDS;
#endif
	#if NUM_EXTRUDER>1
	const char ext1_select_cmd[] PROGMEM = EXT1_SELECT_COMMANDS;
	const char ext1_deselect_cmd[] PROGMEM = EXT1_DESELECT_COMMANDS;
	#endif
	//#if NUM_EXTRUDER>2
	//const char ext2_select_cmd[] PROGMEM = EXT2_SELECT_COMMANDS;
	//const char ext2_deselect_cmd[] PROGMEM = EXT2_DESELECT_COMMANDS;
	//#endif
	//#if NUM_EXTRUDER>3
	//const char ext3_select_cmd[] PROGMEM = EXT3_SELECT_COMMANDS;
	//const char ext3_deselect_cmd[] PROGMEM = EXT3_DESELECT_COMMANDS;
	//#endif
	//#if NUM_EXTRUDER>4
	//const char ext4_select_cmd[] PROGMEM = EXT4_SELECT_COMMANDS;
	//const char ext4_deselect_cmd[] PROGMEM = EXT4_DESELECT_COMMANDS;
	//#endif
	//#if NUM_EXTRUDER>5
	//const char ext5_select_cmd[] PROGMEM = EXT5_SELECT_COMMANDS;
	//const char ext5_deselect_cmd[] PROGMEM = EXT5_DESELECT_COMMANDS;
	//#endif

	Extruder extruder[NUM_EXTRUDER] =
	{
	#if NUM_EXTRUDER>=0
	{
			0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_STEPS_PER_MM,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
			EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,EXT0_WATCHPERIOD
			,EXT0_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_UNITS
	#endif
	#ifdef USE_ADVANCE
	#endif
	#ifdef ENABLE_QUADRATIC_ADVANCE
			,EXT0_ADVANCE_K	,EXT0_ADVANCE_L
	#endif
	}
		//#if NUM_EXTRUDER>1
		//    {
		//        1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_STEPS_PER_MM,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
		//        EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,EXT1_WATCHPERIOD
		//        ,EXT1_WAIT_RETRACT_TEMP,EXT1_WAIT_RETRACT_UNITS
		//#ifdef USE_ADVANCE
		//#ifdef ENABLE_QUADRATIC_ADVANCE
		//        ,EXT1_ADVANCE_K
		//#endif
		//        ,EXT1_ADVANCE_L,EXT1_ADVANCE_BACKLASH_STEPS
		//#endif
		//        ,{
		//            1,EXT1_TEMPSENSOR_TYPE,EXT1_SENSOR_INDEX,0,0,0,0,0,EXT1_HEAT_MANAGER
		//#ifdef TEMP_PID
		//            ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_P,EXT1_PID_I,EXT1_PID_D,EXT1_PID_MAX,0,0,0,{0,0,0,0}
		//#endif
		//        ,0}
		//        ,ext1_select_cmd,ext1_deselect_cmd,EXT1_EXTRUDER_COOLER_SPEED,0
		//    }
		//#endif
		//};

	#if HAVE_HEATED_BED
	#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
		TemperatureController heatedBedController = { NUM_EXTRUDER,HEATED_BED_SENSOR_TYPE,BED_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
		#ifdef TEMP_PID
				,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0,{0,0,0,0}
		#endif
													,0 };
	#else
	#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
	#endif
	}