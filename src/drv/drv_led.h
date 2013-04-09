/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

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
// LED Defines
////////////////////////////////////////////////////////////////////////////////

#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4

///////////////////////////////////////

#define LED0_OFF         GPIO_SetBits(LED0_GPIO,         LED0_PIN)
#define LED0_ON          GPIO_ResetBits(LED0_GPIO,       LED0_PIN)
#define LED0_TOGGLE      GPIO_ToggleBits(LED0_GPIO,      LED0_PIN)

#define LED1_OFF         GPIO_SetBits(LED1_GPIO,         LED1_PIN)
#define LED1_ON          GPIO_ResetBits(LED1_GPIO,       LED1_PIN)
#define LED1_TOGGLE      GPIO_ToggleBits(LED1_GPIO,      LED1_PIN)

///////////////////////////////////////////////////////////////////////////////
// LED Initialization
///////////////////////////////////////////////////////////////////////////////

void ledInit();

///////////////////////////////////////////////////////////////////////////////
