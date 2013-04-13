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

#define RX_PULSE_1p5MS 3000  // 1.5 ms pulse width

uint8_t rcActive = false;

///////////////////////////////////////////////////////////////////////////////

static struct TIM_Channel { TIM_TypeDef *tim;
                            uint16_t channel;
                            uint16_t cc;
                          } Channels[] = { { TIM2, TIM_Channel_1, TIM_IT_CC1 },
                                           { TIM2, TIM_Channel_2, TIM_IT_CC2 },
                                           { TIM2, TIM_Channel_3, TIM_IT_CC3 },
                                           { TIM2, TIM_Channel_4, TIM_IT_CC4 },
                                           { TIM3, TIM_Channel_1, TIM_IT_CC1 },
                                           { TIM3, TIM_Channel_2, TIM_IT_CC2 },
                                           { TIM3, TIM_Channel_3, TIM_IT_CC3 },
                                           { TIM3, TIM_Channel_4, TIM_IT_CC4 }, };

static struct PWM_State { uint8_t  state;          // 0 = looking for rising edge, 1 = looking for falling edge
                          uint16_t riseTime;       // Timer value at rising edge of pulse
                          uint16_t pulseWidth;     // Computed pulse width
                        } Inputs[8] = { { 0, } };

static TIM_ICInitTypeDef  TIM_ICInitStructure = { 0, };


///////////////////////////////////////////////////////////////////////////////

#define SPEKTRUM_UART_PIN  GPIO_Pin_3
#define SPEKTRUM_UART_GPIO GPIOA
#define SPEKTRUM_BIND_PIN  GPIO_Pin_0
#define SPEKTRUM_BIND_GPIO GPIOA

#define SPEKTRUM_FRAME_SIZE 16

uint8_t  i;
uint8_t  spektrumBindCount;

uint32_t spektrumChannelData[SPEKTRUM_MAX_CHANNEL];
uint8_t  spektrumChannelMask;
uint8_t  spektrumChannelShift;

uint8_t  spektrumFrame[SPEKTRUM_FRAME_SIZE];
bool     spektrumFrameComplete = false;
uint8_t  spektrumFramePosition;

uint32_t spektrumTimeInterval;
uint32_t spektrumTimeLast;

///////////////////////////////////////////////////////////////////////////////

static void serialPWM_IRQHandler(TIM_TypeDef *tim)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t  chan = 0;

    if (TIM_GetITStatus(tim, TIM_IT_CC1) == SET)
    {
        last = now;
        now = TIM_GetCapture1(tim);
        rcActive = true;
    }

    TIM_ClearITPendingBit(tim, TIM_IT_CC1);

    diff = now - last;

    if (diff > 2700 * 2)   // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
    {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
        chan = 0;          // you will be fine. I use 2.7ms just to be safe."
    }
    else
    {
        if (diff > 750 * 2 && diff < 2250 * 2 && chan < 8)    // 750 to 2250 ms is our 'valid' channel range
        {
            Inputs[chan].pulseWidth = diff;
        }
        chan++;
    }
}

///////////////////////////////////////////////////////////////////////////////

static void parallelPWM_IRQHandler(TIM_TypeDef *tim)
{
    uint8_t i;
    uint16_t inputCaptureValue = 0;

    for (i = 0; i < 8; i++)
    {
        struct TIM_Channel channel = Channels[i];
        struct PWM_State *state = &Inputs[i];

        if (channel.tim == tim && (TIM_GetITStatus(tim, channel.cc) == SET))
        {
            TIM_ClearITPendingBit(channel.tim, channel.cc);
            if ( i == 0 )
                rcActive = true;

            switch (channel.channel)
            {
                case TIM_Channel_1:
                    inputCaptureValue = TIM_GetCapture1(channel.tim);
                    break;
                case TIM_Channel_2:
                    inputCaptureValue = TIM_GetCapture2(channel.tim);
                    break;
                case TIM_Channel_3:
                    inputCaptureValue = TIM_GetCapture3(channel.tim);
                    break;
                case TIM_Channel_4:
                    inputCaptureValue = TIM_GetCapture4(channel.tim);
                    break;
            }

            if (state->state == 0)
            {
                state->riseTime = inputCaptureValue;

                // switch states
                state->state = 1;

                TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
                TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
            else
            {
                // inputCaptureValue has falling edge timer value

				// compute capture
				state->pulseWidth = inputCaptureValue - state->riseTime;

				// switch state
				state->state = 0;

				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
				TIM_ICInitStructure.TIM_Channel = channel.channel;
                TIM_ICInit(channel.tim, &TIM_ICInitStructure);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

void TIM2_IRQHandler(void)
{
    if (eepromConfig.receiverType == SERIAL_PWM)
        serialPWM_IRQHandler(TIM2);
    else
        parallelPWM_IRQHandler(TIM2);
}

///////////////////////////////////////////////////////////////////////////////

void TIM3_IRQHandler(void)
{
    parallelPWM_IRQHandler(TIM3);
}

///////////////////////////////////////////////////////////////////////////////

void USART2_IRQHandler(void)
{
    uint8_t  b;
    uint8_t  spektrumChannel;
    uint32_t spektrumTime;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        rcActive             = true;
        spektrumTime         = micros();
        spektrumTimeInterval = spektrumTime - spektrumTimeLast;
        spektrumTimeLast     = spektrumTime;

        if (spektrumTimeInterval > 5000)
            spektrumFramePosition = 0;

        spektrumFrame[spektrumFramePosition] = (uint8_t)USART_ReceiveData(USART2);

        if (spektrumFramePosition == SPEKTRUM_FRAME_SIZE - 1)
        {
            spektrumFrameComplete = true;
            //failsafeCnt = 0;
        }
        else
        {
            spektrumFramePosition++;
        }

        if (spektrumFrameComplete)
		{
		    for (b = 3; b < SPEKTRUM_FRAME_SIZE; b += 2)
		    {
		        spektrumChannel = 0x0F & (spektrumFrame[b - 1] >> spektrumChannelShift);
		        if (spektrumChannel < SPEKTRUM_MAX_CHANNEL)
		            spektrumChannelData[spektrumChannel] = ((uint32_t)(spektrumFrame[b - 1] & spektrumChannelMask) << 8) + spektrumFrame[b];
		    }

		    spektrumFrameComplete = false;
		}
    }
}

///////////////////////////////////////////////////////////////////////////////

void rxInit(void)
{
    uint8_t i;

    GPIO_InitTypeDef         GPIO_InitStructure    = { 0, };
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0, };
    NVIC_InitTypeDef         NVIC_InitStructure    = { 0, };
    USART_InitTypeDef        USART_InitStructure   = { 0, };

    ///////////////////////////////////

    if (eepromConfig.receiverType == SERIAL_PWM)
    {
        // Serial PPM input
        // TIM2_CH1 PA0

        //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOA, &GPIO_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        TIM_TimeBaseStructure.TIM_Prescaler   = (36 - 1);
        TIM_TimeBaseStructure.TIM_Period      = 0xffff;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

        TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter    = 0x0;
        TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;

        TIM_ICInit(TIM2, &TIM_ICInitStructure);

        TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
        TIM_Cmd(TIM2, ENABLE);
    }

    ///////////////////////////////////

    else if (eepromConfig.receiverType == PARALLEL_PWM)
    {
        // Parallel PWM Inputs
        // RX1  TIM2_CH1 PA0
        // RX2  TIM2_CH2 PA1
        // RX3  TIM2_CH3 PA2
        // RX4  TIM2_CH4 PA3
        // RX5  TIM3_CH1 PA6
        // RX6  TIM3_CH2 PA7
        // RX7  TIM3_CH3 PB0
        // RX8  TIM3_CH4 PB1

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;

        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // Input timers on TIM2 and TIM3 for PWM
        NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;

        NVIC_Init(&NVIC_InitStructure);

        // TIM2 and TIM3 timebase
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        TIM_TimeBaseStructure.TIM_Prescaler   = (36 - 1);
        TIM_TimeBaseStructure.TIM_Period      = 0xffff;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

        // PWM Input capture
        TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter    = 0x0;

        for (i = 0; i < 8; i++)
        {
            TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
            TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
        }

        TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
        TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
        TIM_Cmd(TIM2, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
    }

    ///////////////////////////////////

    else if (eepromConfig.receiverType == SPEKTRUM)
	    {
	        // Spektrum Satellite RX Input
	    	// USART2 RX PA3
	    	// Spektrum Satellite Bind Input
	    	// PA0

	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
	        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

			// Configure UART RX pin
	        GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_UART_PIN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;

			GPIO_Init(SPEKTRUM_UART_GPIO, &GPIO_InitStructure);


	        NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
	        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
	        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	        NVIC_Init(&NVIC_InitStructure);

	        USART_InitStructure.USART_BaudRate            = 115200;
	        USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	        USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	        USART_InitStructure.USART_Parity              = USART_Parity_No;
	        USART_InitStructure.USART_Mode                = USART_Mode_Rx;
	        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	        USART_Init(USART2, &USART_InitStructure);

	        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	        USART_Cmd(USART2, ENABLE);

	        ///////////////////////////////

	        if (eepromConfig.spektrumHires)
			{
			    // 11 bit frames
			    spektrumChannelShift = 3;
			    spektrumChannelMask  = 0x07;
			}
			else
			{
			    // 10 bit frames
			    spektrumChannelShift = 2;
			    spektrumChannelMask  = 0x03;
			}

        ///////////////////////////////
	}
}

///////////////////////////////////////////////////////////////////////////////

uint16_t rxRead(uint8_t channel)
{
    uint16_t data;

    if (eepromConfig.receiverType == SPEKTRUM)
    {
        if (channel >= eepromConfig.spektrumChannels)
    	{
    	    data = MINCOMMAND;
    	}
       	else
       	{
       	    if (eepromConfig.spektrumHires)
       	        data = 1000 + spektrumChannelData[channel];         // 2048 mode
       	    else
       	        data = (1000 + spektrumChannelData[channel]) << 1;  // 1024 mode
       	}
        return data;
    }
    else
    {
        return Inputs[channel].pulseWidth;
    }
}

///////////////////////////////////////////////////////////////////////////////

void checkSpektrumBind()
{
    // Spektrum Satellite RX Input
    // USART2 RX PA3
	// Spektrum Satellite Bind Input
	// PA0

	GPIO_InitTypeDef  GPIO_InitStructure;

	uint8_t i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    ///////////////////////////////

    // Configure bind pin as input
    GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_BIND_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPEKTRUM_BIND_GPIO, &GPIO_InitStructure);

    // Check bind pin state, if high (true), return without binding
    if (GPIO_ReadInputDataBit(SPEKTRUM_BIND_GPIO, SPEKTRUM_BIND_PIN) == true)
    	return;

    if (eepromConfig.spektrumChannels <= 7)
        spektrumBindCount = 3;  // Master receiver with 7 or less channels
    else
        spektrumBindCount = 5;  // Master receiver with 8 or more channels

    // Configure UART pin as output
    GPIO_InitStructure.GPIO_Pin   = SPEKTRUM_UART_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPEKTRUM_UART_GPIO, &GPIO_InitStructure);

    GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_SET);

    delay(60);

    for (i = 0; i < spektrumBindCount; i++)
    {
	    GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_RESET);
	    delayMicroseconds(120);
		GPIO_WriteBit(SPEKTRUM_UART_GPIO, SPEKTRUM_UART_PIN, Bit_SET  );
        delayMicroseconds(120);
	}
}

///////////////////////////////////////////////////////////////////////////////
