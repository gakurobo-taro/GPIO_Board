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

	inline constexpr PWMPattern end_sequence = {0,0};

	class ProgramablePWMLLSoft : public CommonLib::PWMLLSoft{
		const PWMPattern *sequence_data = nullptr;
		uint32_t sequence_count = 0;
		uint32_t length_count = 0;
	public:
		ProgramablePWMLLSoft(GPIO_TypeDef *_port,uint16_t _pin,float _min = 0,float _max = 1)
				:PWMLLSoft(_port,_pin,_min,_max){
		}

		void set_input_mode(bool mode){
			if(mode) LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);
			else LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
		}

		bool get_input_state(void){ return LL_GPIO_IsInputPinSet(port,pin); }

		void set_duty_weak(const uint16_t _duty){
			if(sequence_data == nullptr){
				PWMLLSoft::set_duty(_duty);
			}
		}

		void start_sequcence(const PWMPattern *data){
			sequence_data = data;
			sequence_count = 0;
			length_count = 0;

			length_count = sequence_data[sequence_count].length;
			set_duty(sequence_data[sequence_count].duty);
		}

		void sequence_update(void){
			if(sequence_data != nullptr){
				length_count  --;
				if(length_count <= 0){
					sequence_count ++;

					if(sequence_data[sequence_count].length == 0){
						sequence_data = nullptr;
						return;
					}
					length_count = sequence_data[sequence_count].length;
					set_duty(sequence_data[sequence_count].duty);
				}
			}
		}

		bool is_doing(void)const{return sequence_data==nullptr?false:true;}

	};

	namespace PWMSequence{
		inline const PWMPattern esc_init[]{
				{0,1000},
				{100,2000},
				{50,5000},
				{50,1000},
				end_sequence,
		};
	}

}


#endif /* PWM_SEQUENCE_CONTROL_HPP_ */
