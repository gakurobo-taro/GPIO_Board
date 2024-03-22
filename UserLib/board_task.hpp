/*
 * board_task.hpp
 *
 *  Created on: 2024/01/23
 *      Author: yaa3k
 */

#ifndef BOARD_TASK_HPP_
#define BOARD_TASK_HPP_

#include "board_id.hpp"
#include "LED_pattern.hpp"
#include "pwm_sequence_control.hpp"

#include "STM32HAL_CommonLib/can_comm.hpp"
#include "STM32HAL_CommonLib/pwm.hpp"
#include "STM32HAL_CommonLib/data_packet.hpp"
#include "STM32HAL_CommonLib/data_convert.hpp"
#include "STM32HAL_CommonLib/serial_comm.hpp"
#include "STM32HAL_CommonLib/id_map_control.hpp"
#include "STM32HAL_CommonLib/timer_control.hpp"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "array"

#include <array>
#include <bitset>

namespace G24_STM32HAL::GPIOBoard{

	inline uint8_t board_id = 0;
	inline auto led_timer = CommonLib::InterruptionTimerHard(&htim15);
	inline auto pwm_timer = CommonLib::InterruptionTimerHard(&htim16);
	inline auto monitor_timer = CommonLib::InterruptionTimerHard(&htim17);

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

	inline auto IO = std::array<GPIOLib::ProgramableSoftPWM,9>{
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO1_GPIO_Port,IO1_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO2_GPIO_Port,IO2_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO3_GPIO_Port,IO3_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO4_GPIO_Port,IO4_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO5_GPIO_Port,IO5_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO6_GPIO_Port,IO6_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO7_GPIO_Port,IO7_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO8_GPIO_Port,IO8_Pin}),
			GPIOLib::ProgramableSoftPWM(CommonLib::PWMLLSoft{IO9_GPIO_Port,IO9_Pin}),
	};

	//LEDs
	inline auto LED_R = CommonLib::LEDPwm{&htim1,TIM_CHANNEL_3};
	inline auto LED_G = CommonLib::LEDPwm{&htim1,TIM_CHANNEL_2};
	inline auto LED_B = CommonLib::LEDPwm{&htim1,TIM_CHANNEL_1};

	//can
	inline auto can = CommonLib::CanComm(&hcan,
			std::make_unique<CommonLib::RingBuffer<CommonLib::CanFrame,4>>(),
			std::make_unique<CommonLib::RingBuffer<CommonLib::CanFrame,4>>(),
			CAN_RX_FIFO0,
			CAN_FILTER_FIFO0,
			CAN_IT_RX_FIFO0_MSG_PENDING);

	//monitor
	inline auto monitor = std::bitset<0x29>{};

	//pin interrupt
	inline uint16_t pin_interrupt_mask = 0;
	inline uint16_t port_read_old_val = 0;

	inline uint16_t port_read(void){
		uint16_t data = 0;
		for(size_t i = 0; i < IO.size(); i++){ data = (IO[i].pwm.get_input_state() ? (1<<i) : 0) | data; }
		return data;
	};

	inline auto esc_mode = std::bitset<IO.size()>{};
	inline void set_esc_mode(uint16_t val){
		esc_mode = std::bitset<IO.size()>{val};
		for(size_t i = 0; i < esc_mode.size(); i++){
			if(esc_mode.test(i)){
				IO[i].pwm.set_input_mode(false);
				IO[i].pwm.set_period(1000);
				IO[i].pwm.set_output_state(true);
				IO[i].start_sequcence(GPIOLib::PWMSequence::servo_init);
			}
		}
	}

	inline auto id_map = CommonLib::IDMapBuilder()
		.add((uint16_t)GPIOLib::GPIOReg::PORT_MODE,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ for(size_t i = 0; i < IO.size(); i++) IO[i].pwm.set_input_mode(data & (1u<<i)); }))
		.add((uint16_t)GPIOLib::GPIOReg::PORT_READ,     CommonLib::DataAccessor::generate<uint16_t>(port_read))
		.add((uint16_t)GPIOLib::GPIOReg::PORT_WRITE,    CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ for(size_t i = 0; i < IO.size(); i++) IO[i].pwm.set_output_state(data & (1u<<i)); }))
		.add((uint16_t)GPIOLib::GPIOReg::PORT_INT_EN,   CommonLib::DataAccessor::generate<uint16_t>(&pin_interrupt_mask))
		.add((uint16_t)GPIOLib::GPIOReg::ESC_MODE_EN,   CommonLib::DataAccessor::generate<uint16_t>(set_esc_mode))
		.add((uint16_t)GPIOLib::GPIOReg::PWM1_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[0].pwm.set_period(data);},[]()->uint16_t { return IO[0].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM2_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[1].pwm.set_period(data);},[]()->uint16_t { return IO[1].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM3_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[2].pwm.set_period(data);},[]()->uint16_t { return IO[2].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM4_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[3].pwm.set_period(data);},[]()->uint16_t { return IO[3].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM5_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[4].pwm.set_period(data);},[]()->uint16_t { return IO[4].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM6_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[5].pwm.set_period(data);},[]()->uint16_t { return IO[5].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM7_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[6].pwm.set_period(data);},[]()->uint16_t { return IO[6].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM8_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[7].pwm.set_period(data);},[]()->uint16_t { return IO[7].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM9_PERIOD,   CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[8].pwm.set_period(data);},[]()->uint16_t { return IO[8].pwm.get_period();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM1_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[0].set_duty(data);},[]()->uint16_t { return IO[0].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM2_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[1].set_duty(data);},[]()->uint16_t { return IO[1].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM3_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[2].set_duty(data);},[]()->uint16_t { return IO[2].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM4_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[3].set_duty(data);},[]()->uint16_t { return IO[3].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM5_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[4].set_duty(data);},[]()->uint16_t { return IO[4].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM6_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[5].set_duty(data);},[]()->uint16_t { return IO[5].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM7_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[6].set_duty(data);},[]()->uint16_t { return IO[6].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM8_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[7].set_duty(data);},[]()->uint16_t { return IO[7].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::PWM9_DUTY,     CommonLib::DataAccessor::generate<uint16_t>([](uint16_t data){ IO[8].set_duty(data);},[]()->uint16_t { return IO[8].pwm.get_duty();}))
		.add((uint16_t)GPIOLib::GPIOReg::MONITOR_PERIOD,CommonLib::DataAccessor::generate<uint16_t>([](uint16_t period){monitor_timer.set_and_start(period);}, []()->uint16_t{return monitor_timer.get_state();}))
		.add((uint16_t)GPIOLib::GPIOReg::MONITOR_REG,   CommonLib::DataAccessor::generate<uint64_t>([](uint64_t val){ monitor = std::bitset<0x29>{val};}, []()->uint64_t{ return monitor.to_ullong();}))
		.build();

	void init(void);

	uint8_t read_board_id(void);

	void main_data_process(void);

	void execute_gpio_command(size_t board_id,const CommonLib::DataPacket &rx_data);
	void execute_common_command(size_t board_id,const CommonLib::DataPacket &rx_data);

	void emergency_stop_sequence(void);
	void emergency_stop_release_sequence(void);

	void monitor_task(void);

	void pin_interrupt_check(void);
}




#endif /* BOARD_TASK_HPP_ */
