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

#include "id_control.hpp"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "array"

#include <array>
#include <bitset>

namespace G24_STM32HAL::GPIOBoard{
	inline auto *pwm_timer = &htim16;
	inline auto *monitor_timer = &htim17;

	struct GPIOParam{
		GPIO_TypeDef * port;
		uint16_t pin;
		GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
	};

	inline auto dip_sw = std::array<GPIOParam,4>{
		GPIOParam{ID0_GPIO_Port,ID0_Pin},
		GPIOParam{ID1_GPIO_Port,ID1_Pin},
		GPIOParam{ID2_GPIO_Port,ID2_Pin},
		GPIOParam{ID3_GPIO_Port,ID3_Pin},
	};

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

	auto port_write = [](uint16_t data){ for(size_t i = 0; i < IO.size(); i++) IO[i].set_output_state(data & (1u<<i)); };
	auto port_read = []()->uint16_t{
		uint16_t data = 0;
		for(size_t i = 0; i < IO.size(); i++) data |= IO[i].get_input_state() ? (1u << i) : 0;
		return data;
	};
	auto set_port_mode = [](uint16_t data){ for(size_t i = 0; i < IO.size(); i++) IO[i].set_input_mode(data & (1u<<i)); };

	inline auto id_map = GPIOLib::IDMapBuilder()
		.add(0x01, GPIOLib::DataManager::generate<uint16_t>(set_port_mode))
		.add(0x02, GPIOLib::DataManager::generate<uint16_t>(port_read))
		.add(0x03, GPIOLib::DataManager::generate<uint16_t>(port_write))
		.add(0x10, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[0].set_period(data);},[]()->uint16_t { return IO[0].get_period();}))
		.add(0x11, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[1].set_period(data);},[]()->uint16_t { return IO[1].get_period();}))
		.add(0x12, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[2].set_period(data);},[]()->uint16_t { return IO[2].get_period();}))
		.add(0x13, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[3].set_period(data);},[]()->uint16_t { return IO[3].get_period();}))
		.add(0x14, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[4].set_period(data);},[]()->uint16_t { return IO[4].get_period();}))
		.add(0x15, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[5].set_period(data);},[]()->uint16_t { return IO[5].get_period();}))
		.add(0x16, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[6].set_period(data);},[]()->uint16_t { return IO[6].get_period();}))
		.add(0x17, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[7].set_period(data);},[]()->uint16_t { return IO[7].get_period();}))
		.add(0x18, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[8].set_period(data);},[]()->uint16_t { return IO[8].get_period();}))
		.add(0x20, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[0].set_duty(data);},[]()->uint16_t { return IO[0].get_duty();}))
		.add(0x21, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[1].set_duty(data);},[]()->uint16_t { return IO[1].get_duty();}))
		.add(0x22, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[2].set_duty(data);},[]()->uint16_t { return IO[2].get_duty();}))
		.add(0x23, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[3].set_duty(data);},[]()->uint16_t { return IO[3].get_duty();}))
		.add(0x24, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[4].set_duty(data);},[]()->uint16_t { return IO[4].get_duty();}))
		.add(0x25, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[5].set_duty(data);},[]()->uint16_t { return IO[5].get_duty();}))
		.add(0x26, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[6].set_duty(data);},[]()->uint16_t { return IO[6].get_duty();}))
		.add(0x27, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[7].set_duty(data);},[]()->uint16_t { return IO[7].get_duty();}))
		.add(0x28, GPIOLib::DataManager::generate<uint16_t>([](uint16_t data){ IO[8].set_duty(data);},[]()->uint16_t { return IO[8].get_duty();}))
		.build();

	void init(void);

	void main_data_process(void);
}




#endif /* BOARD_TASK_HPP_ */
