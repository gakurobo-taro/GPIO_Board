/*
 * board_id.hpp
 *
 *  Created on: Feb 25, 2024
 *      Author: yaa3k
 */

#ifndef BOARD_ID_HPP_
#define BOARD_ID_HPP_

#include <stdint.h>

namespace G24_STM32HAL::GPIOLib{
	enum class CommonReg:uint16_t{
		NOP,
		ID_REQEST,
		EMERGENCY_STOP = 0xE,
		RESET_EMERGENCY_STOP = 0xF,
	};
	enum class GPIOReg : uint16_t{
		NOP,
		PORT_MODE,
		PORT_READ,
		PORT_WRITE,
		PORT_INT_EN,

		PWM1_PERIOD = 0x10,
		PWM2_PERIOD,
		PWM3_PERIOD,
		PWM4_PERIOD,
		PWM5_PERIOD,
		PWM6_PERIOD,
		PWM7_PERIOD,
		PWM8_PERIOD,
		PWM9_PERIOD,

		PWM1_DUTY = 0x20,
		PWM2_DUTY,
		PWM3_DUTY,
		PWM4_DUTY,
		PWM5_DUTY,
		PWM6_DUTY,
		PWM7_DUTY,
		PWM8_DUTY,
		PWM9_DUTY,

		MONITOR_PERIOD = 0xF0,
		MONITOR_REG,
	};
}



#endif /* BOARD_ID_HPP_ */
