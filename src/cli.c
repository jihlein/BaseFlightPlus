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

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

uint8_t cliBusy = false;

uint8_t rfTelem1Enabled = false;
uint8_t rfTelem2Enabled = false;
uint8_t rfTelem3Enabled = false;
uint8_t rfTelem4Enabled = false;
uint8_t rfTelem5Enabled = false;
uint8_t rfTelem6Enabled = false;
uint8_t rfTelem7Enabled = false;
uint8_t rfTelem8Enabled = false;
uint8_t rfTelem9Enabled = false;

static volatile uint8_t cliQuery;
static volatile uint8_t validCliCommand = false;

///////////////////////////////////////////////////////////////////////////////
// RF Telemetry Disable
///////////////////////////////////////////////////////////////////////////////

void rfTelemDisable(void)
{
	rfTelem1Enabled = false;
	rfTelem2Enabled = false;
	rfTelem3Enabled = false;
	rfTelem4Enabled = false;
	rfTelem5Enabled = false;
	rfTelem6Enabled = false;
	rfTelem7Enabled = false;
	rfTelem8Enabled = false;
	rfTelem9Enabled = false;
}

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if (cliAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    do
    {
        if (cliAvailable() == false)
        {
            delay(10);
            timeout++;
        }
        else
        {
            data[index] = cliRead();\
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

void readCliPID(unsigned char PIDid)
{
  struct PIDdata* pid = &eepromConfig.PID[PIDid];

  pid->B              = readFloatCLI();
  pid->P              = readFloatCLI();
  pid->I              = readFloatCLI();
  pid->D              = readFloatCLI();
  pid->windupGuard    = readFloatCLI();
  pid->iTerm          = 0.0f;
  pid->lastDcalcValue = 0.0f;
  pid->lastDterm      = 0.0f;
  pid->lastLastDterm  = 0.0f;
  pid->dErrorCalc     =(uint8_t)readFloatCLI();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	uint8_t  index;

	if ((cliAvailable() && !validCliCommand))
    	cliQuery = cliRead();

    switch (cliQuery)
    {
        ///////////////////////////////

        case 'a': // Rate PIDs
            cliPrintF("\nRoll Rate PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_RATE_PID].B,
                            		                                               eepromConfig.PID[ROLL_RATE_PID].P,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].I,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].D,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[ROLL_RATE_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Pitch Rate PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_RATE_PID].B,
                            		                                               eepromConfig.PID[PITCH_RATE_PID].P,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].I,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].D,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[PITCH_RATE_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Yaw Rate PID:   %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[YAW_RATE_PID].B,
                             		                                               eepromConfig.PID[YAW_RATE_PID].P,
                		                                                           eepromConfig.PID[YAW_RATE_PID].I,
                		                                                           eepromConfig.PID[YAW_RATE_PID].D,
                		                                                           eepromConfig.PID[YAW_RATE_PID].windupGuard,
                		                                                           eepromConfig.PID[YAW_RATE_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'b': // Attitude PIDs
            cliPrintF("\nRoll Attitude PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[ROLL_ATT_PID].B,
              		                                                                   eepromConfig.PID[ROLL_ATT_PID].P,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].I,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].D,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].windupGuard,
               		                                                                   eepromConfig.PID[ROLL_ATT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Pitch Attitude PID: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[PITCH_ATT_PID].B,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].P,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].I,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].D,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].windupGuard,
               		                                                                   eepromConfig.PID[PITCH_ATT_PID].dErrorCalc ? "Error" : "State");

            cliPrintF("Heading PID:        %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n",   eepromConfig.PID[HEADING_PID].B,
               		                                                                   eepromConfig.PID[HEADING_PID].P,
               		                                                                   eepromConfig.PID[HEADING_PID].I,
               		                                                                   eepromConfig.PID[HEADING_PID].D,
               		                                                                   eepromConfig.PID[HEADING_PID].windupGuard,
               		                                                                   eepromConfig.PID[HEADING_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'c': // Velocity PIDs
            cliPrintF("\nhDot PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[HDOT_PID].B,
               		                                                          eepromConfig.PID[HDOT_PID].P,
               		                                                          eepromConfig.PID[HDOT_PID].I,
               		                                                          eepromConfig.PID[HDOT_PID].D,
               		                                                          eepromConfig.PID[HDOT_PID].windupGuard,
               		                                                          eepromConfig.PID[HDOT_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'd': // Position PIDs
            cliPrintF("\nh PID:  %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %s\n", eepromConfig.PID[H_PID].B,
               		                                                       eepromConfig.PID[H_PID].P,
               		                                                       eepromConfig.PID[H_PID].I,
               		                                                       eepromConfig.PID[H_PID].D,
               		                                                       eepromConfig.PID[H_PID].windupGuard,
               		                                                       eepromConfig.PID[H_PID].dErrorCalc ? "Error" : "State");
            cliQuery = 'x';
            validCliCommand = false;
          	break;

        ///////////////////////////////

        case 'e': // Loop Delta Times
           	cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", deltaTime1000Hz,
               		                                                deltaTime200Hz,
               		                                                deltaTime100Hz,
               		                                                deltaTime50Hz,
               		                                                deltaTime10Hz,
               		                                                deltaTime5Hz,
               		                                                deltaTime1Hz);
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'f': // Loop Execution Times
           	cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", executionTime1000Hz,
           	        			                                    executionTime200Hz,
           	        			                                    executionTime100Hz,
           	        			                                    executionTime50Hz,
           	        			                                    executionTime10Hz,
           	        			                                    executionTime5Hz,
           	        			                                    executionTime1Hz);
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'g': // 200 Hz Accels
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel200Hz[XAXIS],
        			                           sensors.accel200Hz[YAXIS],
        			                           sensors.accel200Hz[ZAXIS]);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'h': // 100 hz Earth Axis Accels
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", earthAxisAccels[XAXIS],
        			                           earthAxisAccels[YAXIS],
        			                           earthAxisAccels[ZAXIS]);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'i': // 200 Hz Gyros
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro200Hz[ROLL ] * R2D,
        			                                  sensors.gyro200Hz[PITCH] * R2D,
        					                          sensors.gyro200Hz[YAW  ] * R2D,
        					                          mpu6050Temperature);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'j': // 10 Hz Mag Data
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
        			                           sensors.mag10Hz[YAXIS],
        			                           sensors.mag10Hz[ZAXIS]);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'k': // Vertical Axis Variables
        	cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", earthAxisAccels[ZAXIS],
        			                                  sensors.pressureAlt10Hz,
        					                          hDotEstimate,
        					                          hEstimate);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'l': // Attitudes
        	cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude200Hz[ROLL ] * R2D,
        			                           sensors.attitude200Hz[PITCH] * R2D,
        			                           sensors.attitude200Hz[YAW  ] * R2D);
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'm': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'n': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'o': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'p': // Not Used
            cliQuery = 'x';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'q': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'r':
        	if (flightMode == RATE)
        		cliPrint("Flight Mode = RATE      ");
        	else if (flightMode == ATTITUDE)
        		cliPrint("Flight Mode = ATTITUDE  ");

        	if (headingHoldEngaged == true)
        	    cliPrint("Heading Hold = ENGAGED     ");
        	else
        	    cliPrint("Heading Hold = DISENGAGED  ");

        	if (altitudeHoldState == DISENGAGED)
        		cliPrint("Altitude Hold = DISENAGED\n");
            else if (altitudeHoldState == ENGAGED)
            	cliPrint("Altitude Hold = ENGAGED\n");
            else if (altitudeHoldState == PANIC)
            	cliPrint("Altitude Hold = PANIC\n");

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 's': // Raw Receiver Commands
            if (eepromConfig.receiverType == SPEKTRUM)
            {
				for (index = 0; index < eepromConfig.spektrumChannels - 1; index++)
                     cliPrintF("%4ld, ", spektrumChannelData[index]);

                cliPrintF("%4ld\n", spektrumChannelData[eepromConfig.spektrumChannels - 1]);
            }
		    else
		    {
				for (index = 0; index < 7; index++)
                    cliPrintF("%4i, ", rxRead(index));

                cliPrintF("%4i\n", rxRead(7));
            }

        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 't': // Processed Receiver Commands
            for (index = 0; index < 7; index++)
                cliPrintF("%8.2f, ", rxCommand[index]);

            cliPrintF("%8.2f\n", rxCommand[7]);

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'u': // Command in Detent Discretes
        	cliPrintF("%s, ", commandInDetent[ROLL ] ? " true" : "false");
        	cliPrintF("%s, ", commandInDetent[PITCH] ? " true" : "false");
        	cliPrintF("%s\n", commandInDetent[YAW  ] ? " true" : "false");

            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'v': // ESC PWM Outputs
        	cliPrintF("%4ld, ", TIM1->CCR1);
        	cliPrintF("%4ld, ", TIM1->CCR4);
        	cliPrintF("%4ld, ", TIM4->CCR1);
        	cliPrintF("%4ld, ", TIM4->CCR2);
        	cliPrintF("%4ld, ", TIM4->CCR3);
        	cliPrintF("%4ld\n", TIM4->CCR4);

        	validCliCommand = false;
            break;

        ///////////////////////////////

        case 'w': // Not Used
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'x':
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'y': // ESC Calibration
        	escCalibration();

        	cliQuery = 'x';
        	break;

        ///////////////////////////////

        case 'z': // Not Used
            validCliCommand = false;
            break;

        ///////////////////////////////

        case '1': // Turn RF telemetry 1 on
        	rfTelemDisable();
          	rfTelem1Enabled = true;

        	cliQuery = 'x';
            break;

        ///////////////////////////////

        case '2': // Turn RF telemetry 2 on
           	rfTelemDisable();
           	rfTelem2Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '3': // Turn RF telemetry 3 on
           	rfTelemDisable();
           	rfTelem3Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '4': // Turn RF telemetry 4 on
           	rfTelemDisable();
           	rfTelem4Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '5': // Turn RF telemetry 5 on
           	rfTelemDisable();
           	rfTelem5Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '6': // Turn RF telemetry 6 on
           	rfTelemDisable();
           	rfTelem6Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '7': // Turn RF telemetry 7 on
           	rfTelemDisable();
           	rfTelem7Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '8': // Turn RF telemetry 8 on
           	rfTelemDisable();
           	rfTelem8Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '9': // Turn RF telemetry 9 on
           	rfTelemDisable();
           	rfTelem9Enabled = true;

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        case '0': // Disable RF telemetry
           	rfTelemDisable();

            cliQuery = 'x';
           	break;

        ///////////////////////////////

        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        ///////////////////////////////

        case 'A': // Read Roll Rate PID Values
            readCliPID(ROLL_RATE_PID);
            cliPrint( "\nRoll Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'B': // Read Pitch Rate PID Values
            readCliPID(PITCH_RATE_PID);
            cliPrint( "\nPitch Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'C': // Read Yaw Rate PID Values
            readCliPID(YAW_RATE_PID);
            cliPrint( "\nYaw Rate PID Received....\n" );

        	cliQuery = 'a';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'D': // Read Roll Attitude PID Values
            readCliPID(ROLL_ATT_PID);
            cliPrint( "\nRoll Attitude PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'E': // Read Pitch Attitude PID Values
            readCliPID(PITCH_ATT_PID);
            cliPrint( "\nPitch Attitude PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'F': // Read Heading Hold PID Values
            readCliPID(HEADING_PID);
            cliPrint( "\nHeading PID Received....\n" );

        	cliQuery = 'b';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'G': // Not Used
            validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'H': // Not Used
        	validCliCommand = false;
          	break;

        ///////////////////////////////

        case 'I': // Read hDot PID Values
            readCliPID(HDOT_PID);
            cliPrint( "\nhDot PID Received....\n" );

          	cliQuery = 'c';
          	validCliCommand = false;
          	break;

       	///////////////////////////////

        case 'J': // Not Used
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'K': // Not Used
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'L': // Read h PID Values
            readCliPID(H_PID);
            cliPrint( "\nh PID Received....\n" );

            cliQuery = 'd';
        	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'M': // Not Used
           	validCliCommand = false;
        	break;

        ///////////////////////////////

        case 'N': // Mixer CLI
            mixerCLI();

            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'O': // Receiver CLI
            receiverCLI();

            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'P': // Sensor CLI
           	sensorCLI();

           	cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'Q': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'R': // Reset to Bootloader
        	cliPrint("Entering Bootloader....\n\n");
        	delay(100);
        	systemReset(true);
        	break;

        ///////////////////////////////

        case 'S': // Reset System
        	cliPrint("\nSystem Reseting....\n\n");
        	delay(100);
        	systemReset(false);
        	break;

        ///////////////////////////////

        case 'T': // Not Used
            cliQuery = 'x';
           	validCliCommand = false;
           	break;

        ///////////////////////////////

        case 'U': // Not Used
            cliQuery = 'x';
         	validCliCommand = false;
         	break;

        ///////////////////////////////

        case 'V': // Reset EEPROM Parameters
            cliPrint( "\nEEPROM Parameters Reset....\n" );
            checkFirstTime(true);
            cliPrint("\nSystem Resetting....\n\n");
            delay(100);
            systemReset(false);
            break;

        ///////////////////////////////

        case 'W': // Write EEPROM Parameters
            cliPrint("\nWriting EEPROM Parameters....\n");
            writeEEPROM();

            cliQuery = 'x';
         	validCliCommand = false;
         	break;

        ///////////////////////////////

        case 'X': // Not Used
            cliQuery = 'x';
            validCliCommand = false;
            break;

        ///////////////////////////////

        case 'Y': // Not Used
            cliQuery = 'x';
            break;

        ///////////////////////////////

        case 'Z': // Not Used
            cliQuery = 'x';
            break;

        ///////////////////////////////

        case '?': // Command Summary
        	cliBusy = true;

        	cliPrint("\n");
   		    cliPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'g' 500 Hz Accels                          'G' Not Used\n");
   		    cliPrint("'h' 100 Hz Earth Axis Accels               'H' Not Used\n");
   		    cliPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("'j' 10 hz Mag Data                         'J' Not Used\n");
   		    cliPrint("'k' Vertical Axis Variable                 'K' Not Used\n");
   		    cliPrint("'l' Attitudes                              'L' Set h PID Data           LB;P;I;D;windupGuard;dErrorCalc\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		        validCliCommand = true;
   		        cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'m' Not Used                               'M' Not Used\n");
   		    cliPrint("'n' Not Used                               'N' Mixer CLI\n");
   		    cliPrint("'o' Not Used                               'O' Receiver CLI\n");
   		    cliPrint("'p' Not Used                               'P' Sensor CLI\n");
   		    cliPrint("'q' Not Used                               'Q' Not Used\n");
   		    cliPrint("'r' Mode States                            'R' Reset and Enter Bootloader\n");
   		    cliPrint("'s' Raw Receiver Commands                  'S' Reset\n");
   		    cliPrint("'t' Processed Receiver Commands            'T' Not Used\n");
   		    cliPrint("'u' Command In Detent Discretes            'U' Not Used\n");
   		    cliPrint("'v' PWM Outputs                            'V' Reset EEPROM Parameters\n");
   		    cliPrint("'w' Not Used                               'W' Write EEPROM Parameters\n");
   		    cliPrint("'x' Terminate Serial Communication         'X' Not Used\n");
   		    cliPrint("\n");

   		    cliPrint("Press space bar for more, or enter a command....\n");
   		    while (cliAvailable() == false);
   		    cliQuery = cliRead();
   		    if (cliQuery != ' ')
   		    {
   		    	validCliCommand = true;
   		    	cliBusy = false;
   		    	return;
   		    }

   		    cliPrint("\n");
   		    cliPrint("'y' ESC Calibration                        'Y' Not Used\n");
   		    cliPrint("'z' Not Used                               'Z' Not Used\n");
   		    cliPrint("'1' RF Telemetry 1 Enable\n");
   		    cliPrint("'2' RF Telemetry 2 Enable\n");
   		    cliPrint("'3' RF Telemetry 3 Enable\n");
   		    cliPrint("'4' RF Telemetry 4 Enable\n");
   		    cliPrint("'5' RF Telemetry 5 Enable\n");
   		    cliPrint("'6' RF Telemetry 6 Enable\n");
   		    cliPrint("'7' RF Telemetry 7 Enable\n");
   		    cliPrint("'8' RF Telemetry 8 Enable\n");
   		    cliPrint("'9' RF Telemetry 9 Enable\n");
   		    cliPrint("'0' RF Telemetry Disable                   '?' Command Summary\n");
   		    cliPrint("\n");

  		    cliQuery = 'x';
  		    cliBusy = false;
   		    break;

            ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
