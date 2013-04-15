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

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define     PI 3.14159265f
#define TWO_PI 6.28318531f

#define D2R PI / 180.0f

#define R2D 180.0f / PI

#define SQR(x)  x * x

///////////////////////////////////////////////////////////////////////////////

#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define AUX1     4
#define AUX2     5
#define AUX3     6
#define AUX4     7

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////

typedef union {
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union {
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union {
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

///////////////////////////////////////////////////////////////////////////////
// Sensor Variables
///////////////////////////////////////////////////////////////////////////////

typedef struct sensors_t {
    float accel200Hz[3];
    float attitude200Hz[3];
    float gyro200Hz[3];
} sensors_t;

extern sensors_t sensors;

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_PIDS 6

#define ROLL_RATE_PID  0
#define PITCH_RATE_PID 1
#define YAW_RATE_PID   2

#define ROLL_ATT_PID   3
#define PITCH_ATT_PID  4
#define HEADING_PID    5

///////////////////////////////////////////////////////////////////////////////
// Mixer Configurations
///////////////////////////////////////////////////////////////////////////////

enum { NA_MIXER,                  //  0

       MIXERTYPE_GIMBAL,          //  1

       MIXERTYPE_FLYING_WING,     //  2

       MIXERTYPE_BI,              //  3

       MIXERTYPE_TRI,             //  4

       MIXERTYPE_QUADP,           //  5

       MIXERTYPE_QUADX,           //  6

       MIXERTYPE_VTAIL4_NO_COMP,  //  7
       MIXERTYPE_VTAIL4_Y_COMP,   //  8
       MIXERTYPE_VTAIL4_RY_COMP,  //  9
       MIXERTYPE_VTAIL4_PY_COMP,  // 10
       MIXERTYPE_VTAIL4_RP_COMP,  // 11
       MIXERTYPE_VTAIL4_RPY_COMP, // 12

       MIXERTYPE_Y4,              // 13

       MIXERTYPE_HEX6P,           // 14
       MIXERTYPE_HEX6X,           // 15
       MIXERTYPE_Y6,              // 16

       MIXERTYPE_FREEMIX          // 17
     };

///////////////////////////////////////////////////////////////////////////////
// Flight Modes
///////////////////////////////////////////////////////////////////////////////

enum { RATE, ATTITUDE };

///////////////////////////////////////////////////////////////////////////////
// Receiver Configurations
///////////////////////////////////////////////////////////////////////////////

enum { NA_RECEIVER, PARALLEL_PWM, SERIAL_PWM, SPEKTRUM };

///////////////////////////////////////////////////////////////////////////////
// MPU6050 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

///////////////////////////////////////////////////////////////////////////////
// EEPROM
///////////////////////////////////////////////////////////////////////////////

typedef struct eepromConfig_t {
    uint8_t version;

    float accelTCBiasSlope[3];
    float accelTCBiasIntercept[3];

    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    float accelCutoff;

    float KpAcc;

    float KiAcc;

    float KpMag;

    float KiMag;

    uint8_t dlpfSetting;

    ///////////////////////////////////

    float rateScaling;

    float attitudeScaling;

    ///////////////////////////////////

    uint8_t receiverType;
    uint8_t spektrumChannels;
    uint8_t spektrumHires;

    uint8_t rcMap[8];

    uint16_t escPwmRate;
    uint16_t servoPwmRate;

    uint8_t mixerConfiguration;
    float yawDirection;

    float midCommand;
    float minCheck;
    float maxCheck;
    float minThrottle;
    float maxThrottle;

    PIDdata_t PID[NUMBER_OF_PIDS];

    float biLeftServoMin;
    float biLeftServoMid;
    float biLeftServoMax;

    float biRightServoMin;
    float biRightServoMid;
    float biRightServoMax;

    float triYawServoMin;
    float triYawServoMid;
    float triYawServoMax;

    float gimbalRollServoMin;
    float gimbalRollServoMid;
    float gimbalRollServoMax;
    float gimbalRollServoGain;

    float gimbalPitchServoMin;
    float gimbalPitchServoMid;
    float gimbalPitchServoMax;
    float gimbalPitchServoGain;

    float vTailAngle;

    uint8_t freeMixMotors;

    float freeMix[6][3];

    float rollDirectionLeft;
    float rollDirectionRight;
    float pitchDirectionLeft;
    float pitchDirectionRight;
    float wingLeftMinimum;
    float wingLeftMaximum;
    float wingRightMinimum;
    float wingRightMaximum;

    uint8_t armCount;
    uint8_t disarmCount;

} eepromConfig_t;

extern eepromConfig_t eepromConfig;

///////////////////////////////////////////////////////////////////////////////
