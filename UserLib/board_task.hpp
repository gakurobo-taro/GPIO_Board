/*
 * board_task.hpp
 *
 *  Created on: 2024/01/23
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_

#include "STM32HAL_CommonLib/can_comm.hpp"
#include "STM32HAL_CommonLib/pwm.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/data_convert.hpp"
#include "STM32HAL_CommonLib/serial_comm.hpp"

#include "main.h"
#include "can.h"
#include "gpio.h"
#include "array"

#include <array>
#include <bitset>

namespace G24_STM32HAL::GPIOBoard{
	inline auto *pwm_timer = &htim16;
	inline auto *monitor_timer = &htim17;

	inline auto IO = std::array<CommonLib::PWMLLSoft,9>{
			CommonLib::PWMLLSoft{IO1_GPIO_Port,IO1_Pin},
			CommonLib::PWMLLSoft{IO2_GPIO_Port,IO2_Pin},
			CommonLib::PWMLLSoft{IO3_GPIO_Port,IO3_Pin},
			CommonLib::PWMLLSoft{IO4_GPIO_Port,IO4_Pin},
			CommonLib::PWMLLSoft{IO5_GPIO_Port,IO5_Pin},
			CommonLib::PWMLLSoft{IO6_GPIO_Port,IO6_Pin},
			CommonLib::PWMLLSoft{IO7_GPIO_Port,IO7_Pin},
			CommonLib::PWMLLSoft{IO8_GPIO_Port,IO8_Pin},
			CommonLib::PWMLLSoft{IO9_GPIO_Port,IO9_Pin},
	};

	//LEDs
	inline auto LED_R = CommonLib::PWMHard{&htim1,TIM_CHANNEL_1};
	inline auto LED_G = CommonLib::PWMHard{&htim1,TIM_CHANNEL_2};
	inline auto LED_B = CommonLib::PWMHard{&htim1,TIM_CHANNEL_3};

	//can
	inline auto can = CommonLib::CanComm<4,4>(&hcan,CAN_RX_FIFO0,CAN_FILTER_FIFO0,CAN_IT_RX_FIFO0_MSG_PENDING);

	void init(void){
		LED_R.start();
		LED_G.start();
		LED_B.start();

		can.start();
	}
}




#endif /* BOARD_TASK_HPP_ */
