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
// Gyro Temperature Calibration
///////////////////////////////////////////////////////////////////////////////

void gyroTempCalibration(void)
{
    uint16_t gyroSampleRate = 1000;
    uint16_t numberOfGyroSamples = 2000;

    float gyroBias1[3] = { 0.0f, 0.0f, 0.0f };
    float gyroTemperature1 = 0.0f;

    float gyroBias2[3] = { 0.0f, 00.f, 0.0f };
    float gyroTemperature2 = 0.0f;

    uint16_t index;

    gyroCalibrating = true;

    cliPrint("\nGyro Temperature Calibration:\n");

    ///////////////////////////////////
    // Get samples at temperature1
    ///////////////////////////////////
    cliPrint("\nBegin 1st Gyro Measurements...\n");
    for (index = 0; index < numberOfGyroSamples; index++) {
        readGyro();
        gyroBias1[ROLL ] += rawGyro[ROLL].value;
        gyroBias1[PITCH] += rawGyro[PITCH].value;
        gyroBias1[YAW  ] += rawGyro[YAW].value;
        gyroTemperature1 += ((float) rawGyroTemperature.value + 13200.0f) / 280.0f + 35.0f;
        delayMicroseconds(gyroSampleRate);
    }

    gyroBias1[ROLL]  /= (float) numberOfGyroSamples;
    gyroBias1[PITCH] /= (float) numberOfGyroSamples;
    gyroBias1[YAW]   /= (float) numberOfGyroSamples;
    gyroTemperature1 /= (float) numberOfGyroSamples;

    cliPrintF("\nGyro Temperature Reading: %6.2f", gyroTemperature1);

    cliPrint("\n\nEnd 1st Gyro Measurements\n");

    ///////////////////////////////////
    // Time delay for temperature
    // Stabilizaiton
    ///////////////////////////////////

    cliPrint("\nWaiting for 10 minutes for gyro temp to rise...\n");
    delay(600000);              // Number of mSec in 10 minutes

    ///////////////////////////////////
    // Get samples at temperature2
    ///////////////////////////////////
    cliPrint("\nBegin 2nd Gyro Measurements...\n");
    for (index = 0; index < numberOfGyroSamples; index++) {
        readGyro();
        gyroBias2[ROLL]  += rawGyro[ROLL].value;
        gyroBias2[PITCH] += rawGyro[PITCH].value;
        gyroBias2[YAW]   += rawGyro[YAW].value;
        gyroTemperature2 += ((float) rawGyroTemperature.value + 13200.0f) / 280.0f + 35.0f;
        delayMicroseconds(gyroSampleRate);
    }

    gyroBias2[ROLL ] /= (float) numberOfGyroSamples;
    gyroBias2[PITCH] /= (float) numberOfGyroSamples;
    gyroBias2[YAW  ] /= (float) numberOfGyroSamples;
    gyroTemperature2 /= (float) numberOfGyroSamples;

    cliPrintF("\nGyro Temperature Reading: %6.2f", gyroTemperature2);

    cliPrint("\n\nEnd 2nd Gyro Measurements\n");

    eepromConfig.gyroTCBiasSlope[ROLL ] = (gyroBias2[ROLL ] - gyroBias1[ROLL ]) / (gyroTemperature2 - gyroTemperature1);
    eepromConfig.gyroTCBiasSlope[PITCH] = (gyroBias2[PITCH] - gyroBias1[PITCH]) / (gyroTemperature2 - gyroTemperature1);
    eepromConfig.gyroTCBiasSlope[YAW  ] = (gyroBias2[YAW  ] - gyroBias1[YAW  ]) / (gyroTemperature2 - gyroTemperature1);

    eepromConfig.gyroTCBiasIntercept[ROLL ] = gyroBias2[ROLL ] - eepromConfig.gyroTCBiasSlope[ROLL ] * gyroTemperature2;
    eepromConfig.gyroTCBiasIntercept[PITCH] = gyroBias2[PITCH] - eepromConfig.gyroTCBiasSlope[PITCH] * gyroTemperature2;
    eepromConfig.gyroTCBiasIntercept[YAW  ] = gyroBias2[YAW  ] - eepromConfig.gyroTCBiasSlope[YAW  ] * gyroTemperature2;

    gyroCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
