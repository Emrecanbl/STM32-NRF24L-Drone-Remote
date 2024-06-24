/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#include "stm32g0xx_hal.h"          // change to whatever MCU or Cortex-M core you use
#include "dwt_delay.h"

extern TIM_HandleTypeDef htim1;
/**
 * Initialization routine.
 * You might need to enable access to DWT registers on Cortex-M7
 *   DWT->LAR = 0xC5ACCE55
 */
void DWT_Init(void)
{	HAL_TIM_Base_Start(&htim1);

}

#if DWT_DELAY_NEWBIE
/**
 * If you are a newbie and see magic in DWT_Delay, consider this more
 * illustrative function, where you explicitly determine a counter
 * value when delay should stop while keeping things in bounds of uint32.
 *
 * @param uint32_t us  Number of microseconds to delay for
*/
void DWT_Delay(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim1) < us); // it is modifyt because of m1 core
}
#else
/**
 * Delay routine itself.
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void DWT_Delay(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim1) < us); // it is modifyt because of m1 core
}

#endif
