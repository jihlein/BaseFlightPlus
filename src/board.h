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

#pragma once

///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////

#include "stm32f10x_conf.h"
#include "core_cm3.h"

///////////////////////////////////////

#include "pid.h"

#include "baseFlightPlus.h"

#include "drv_cli.h"
#include "drv_i2c.h"
#include "drv_led.h"
#include "drv_rx.h"
#include "drv_pwmOutput.h"
#include "drv_rx.h"
#include "drv_system.h"

#include "adxl345.h"
#include "bmp085.h"
#include "hmc5883.h"
#include "mpu3050.h"

#include "accelCalibration.h"
#include "cli.h"
#include "cliSupport.h"
#include "computeAxisCommands.h"
#include "config.h"
#include "coordinateTransforms.h"
#include "escCalibration.h"
#include "flightCommand.h"
#include "gyroTempCalibration.h"
#include "lowPassFilter.h"
#include "magCalibration.h"
#include "MargAHRS.h"
#include "mixer.h"
#include "utilities.h"
#include "vertCompFilter.h"

///////////////////////////////////////////////////////////////////////////////
