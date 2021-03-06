/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

eepromConfig_t eepromConfig;

uint8_t        execUpCount = 0;

sensors_t      sensors;

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
	uint32_t currentTime;

    systemInit();

    systemReady = true;

    while (1)
    {
    	///////////////////////////////

        if (frame_50Hz)
        {
        	frame_50Hz = false;

        	currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processFlightCommands();

			executionTime50Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
        	frame_10Hz = false;

        	currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			sensors.mag10Hz[XAXIS] = -((float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS]);
			sensors.mag10Hz[YAXIS] =   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
			sensors.mag10Hz[ZAXIS] = -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

			newMagData = false;
			magDataUpdate = true;

        	pressureAverage = pressureSum / 10;
        	pressureSum = 0;
        	calculateTemperature();
        	calculatePressureAltitude();
        	sensors.pressureAlt10Hz = pressureAlt;

        	cliCom();

        	executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_200Hz)
        {
        	frame_200Hz = false;

       	    currentTime       = micros();
       	    deltaTime200Hz    = currentTime - previous200HzTime;
       	    previous200HzTime = currentTime;

       	    dt200Hz = (float)deltaTime200Hz * 0.000001f;  // For integrations in 200 Hz loop

            sensors.accel200Hz[XAXIS] = -((float)accelSummedSamples200Hz[XAXIS] / 5.0f - accelRTBias[XAXIS] - eepromConfig.accelBias[XAXIS]) * eepromConfig.accelScaleFactor[XAXIS];
			sensors.accel200Hz[YAXIS] = -((float)accelSummedSamples200Hz[YAXIS] / 5.0f - accelRTBias[YAXIS] - eepromConfig.accelBias[YAXIS]) * eepromConfig.accelScaleFactor[YAXIS];
			sensors.accel200Hz[ZAXIS] = -((float)accelSummedSamples200Hz[ZAXIS] / 5.0f - accelRTBias[ZAXIS] - eepromConfig.accelBias[ZAXIS]) * eepromConfig.accelScaleFactor[ZAXIS];

            sensors.accel200Hz[XAXIS] = computeFourthOrder200Hz(sensors.accel200Hz[XAXIS], &fourthOrder200Hz[AX_FILTER]);
            sensors.accel200Hz[YAXIS] = computeFourthOrder200Hz(sensors.accel200Hz[YAXIS], &fourthOrder200Hz[AY_FILTER]);
            sensors.accel200Hz[ZAXIS] = computeFourthOrder200Hz(sensors.accel200Hz[ZAXIS], &fourthOrder200Hz[AZ_FILTER]);

            computeGyroTCBias();
            sensors.gyro200Hz[ROLL ] =  ((float)gyroSummedSamples200Hz[ROLL]  / 5.0f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
			sensors.gyro200Hz[PITCH] = -((float)gyroSummedSamples200Hz[PITCH] / 5.0f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            sensors.gyro200Hz[YAW  ] = -((float)gyroSummedSamples200Hz[YAW]   / 5.0f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;

            MargAHRSupdate( sensors.gyro200Hz[ROLL],   sensors.gyro200Hz[PITCH],  sensors.gyro200Hz[YAW],
                            sensors.accel200Hz[XAXIS], sensors.accel200Hz[YAXIS], sensors.accel200Hz[ZAXIS],
                            sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                            eepromConfig.accelCutoff,
                            magDataUpdate,
                            dt200Hz );

            magDataUpdate = false;

            computeAxisCommands(dt200Hz);
            mixTable();
            writeServos();
            writeMotors();

            executionTime200Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_100Hz)
        {
        	frame_100Hz = false;

        	currentTime       = micros();
			deltaTime100Hz    = currentTime - previous100HzTime;
			previous100HzTime = currentTime;

			dt100Hz = (float)deltaTime100Hz * 0.000001f;  // For integrations in 100 Hz loop

            sensors.accel100Hz[XAXIS] = -((float)accelSummedSamples100Hz[XAXIS] / 10.0f - accelRTBias[XAXIS] - eepromConfig.accelBias[XAXIS]) * eepromConfig.accelScaleFactor[XAXIS];
			sensors.accel100Hz[YAXIS] = -((float)accelSummedSamples100Hz[YAXIS] / 10.0f - accelRTBias[YAXIS] - eepromConfig.accelBias[YAXIS]) * eepromConfig.accelScaleFactor[YAXIS];
			sensors.accel100Hz[ZAXIS] = -((float)accelSummedSamples100Hz[ZAXIS] / 10.0f - accelRTBias[ZAXIS] - eepromConfig.accelBias[ZAXIS]) * eepromConfig.accelScaleFactor[ZAXIS];

        	sensors.accel100Hz[XAXIS] = computeFourthOrder100Hz(sensors.accel100Hz[XAXIS], &fourthOrder100Hz[AX_FILTER]);
            sensors.accel100Hz[YAXIS] = computeFourthOrder100Hz(sensors.accel100Hz[YAXIS], &fourthOrder100Hz[AY_FILTER]);
            sensors.accel100Hz[ZAXIS] = computeFourthOrder100Hz(sensors.accel100Hz[ZAXIS], &fourthOrder100Hz[AZ_FILTER]);

            //computeGyroTCBias();
            //sensors.gyro100Hz[ROLL ] =  ((float)gyroSummedSamples100Hz[ROLL]  / 10.0f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
			//sensors.gyro100Hz[PITCH] = -((float)gyroSummedSamples100Hz[PITCH] / 10.0f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            //sensors.gyro100Hz[YAW  ] = -((float)gyroSummedSamples100Hz[YAW]   / 10.0f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;

            createRotationMatrix();
            bodyAccelToEarthAccel();
            vertCompFilter(dt100Hz);

            //computeAxisCommands(dt100Hz);
            //mixTable();
            //writeServos();
            //writeMotors();

            if ( rfTelem1Enabled == true )
            {
            	// 200 Hz Accels
            	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel200Hz[XAXIS],
            	        			               sensors.accel200Hz[YAXIS],
            	        			               sensors.accel200Hz[ZAXIS]);
            }

            if ( rfTelem2Enabled == true )
            {
            	// 200 Hz Gyros
            	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro200Hz[ROLL ],
            	        			               sensors.gyro200Hz[PITCH],
            	        					       sensors.gyro200Hz[YAW  ]);
            }

            if ( rfTelem3Enabled == true )
            {
            	// Roll Rate, Roll Rate Command
            	cliPrintF("%9.4f, %9.4f\n", sensors.gyro200Hz[ROLL],
            			                    rxCommand[ROLL]);
            }

            if ( rfTelem4Enabled == true )
            {
            	// Pitch Rate, Pitch Rate Command
            	cliPrintF("%9.4f, %9.4f\n", sensors.gyro200Hz[PITCH],
            	            			    rxCommand[PITCH]);
            }

            if ( rfTelem5Enabled == true )
            {
            	// Yaw Rate, Yaw Rate Command
            	cliPrintF("%9.4f, %9.4f\n", sensors.gyro200Hz[YAW],
            	            	            rxCommand[YAW]);
            }

            if ( rfTelem6Enabled == true )
            {
            	// 200 Hz Attitudes
            	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude200Hz[ROLL ],
            	        			               sensors.attitude200Hz[PITCH],
            	        			               sensors.attitude200Hz[YAW  ]);
            }

            executionTime100Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
        	frame_5Hz = false;

        	currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

        	executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
        	frame_1Hz = false;

        	currentTime     = micros();
			deltaTime1Hz    = currentTime - previous1HzTime;
			previous1HzTime = currentTime;

			if (execUp == false)
			    execUpCount++;

			if ((execUpCount == 5) && (execUp == false))
			{
				execUp = true;
				LED0_OFF;
				LED1_OFF;
			}

			executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
