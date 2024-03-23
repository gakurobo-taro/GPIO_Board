/*
 * pwm_sequence_control.hpp
 *
 *  Created on: Mar 22, 2024
 *      Author: gomas
 */

#ifndef PWM_SEQUENCE_CONTROL_HPP_
#define PWM_SEQUENCE_CONTROL_HPP_


#include "STM32HAL_CommonLib/pwm.hpp"

namespace G24_STM32HAL::GPIOLib{

	struct PWMPattern{
		uint16_t duty;
		uint32_t length;
	};

	class ProgramableSoftPWM{
		const PWMPattern *sequence_data = nullptr;
		uint32_t sequence_count = 0;
		uint32_t length_count = 0;
	public:
		CommonLib::PWMLLSoft pwm;
		ProgramableSoftPWM(CommonLib::PWMLLSoft &&_pwm):pwm(_pwm){}

		void set_duty(const uint16_t duty){
			if(sequence_data == nullptr){
				pwm.set_duty(duty);
			}
		}

		void start_sequcence(const PWMPattern *data){
			sequence_data = data;
			sequence_count = 0;
			length_count = 0;

			length_count = sequence_data[sequence_count].length;
			pwm.set_duty(sequence_data[sequence_count].duty);
		}

		void update(void){
			if(sequence_data != nullptr){
				length_count  --;
				if(length_count <= 0){
					sequence_count ++;

					if(sequence_data[sequence_count].length == 0){
						sequence_data = nullptr;
						return;
					}
					length_count = sequence_data[sequence_count].length;
					pwm.set_duty(sequence_data[sequence_count].duty);
				}
			}
		}

		bool is_doing(void)const{return sequence_data==nullptr?false:true;}

	};

	namespace PWMSequence{
		inline const PWMPattern esc_init[]{
				{0,500},
				{100,2000},
				{50,2000},
				{0,0},
		};
	}

}


#endif /* PWM_SEQUENCE_CONTROL_HPP_ */
