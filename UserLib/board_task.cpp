/*
 * board_task.cpp
 *
 *  Created on: Feb 24, 2024
 *      Author: yaa3k
 */

#include "board_task.hpp"

namespace G24_STM32HAL::GPIOBoard{
	void init(void){
		LED_R.start();
		LED_G.start();
		LED_B.start();

		board_id = read_board_id();
		can.set_filter_mask(0, 0x00300000|(board_id<<16), 0x00FF0000, CommonLib::FilterMode::STD_AND_EXT, true);
		can.set_filter_mask(1, 0x00000000|(board_id<<16), 0x00FF0000, CommonLib::FilterMode::STD_AND_EXT, true);
		can.set_filter_mask(2, 0x00F00000, 0x00F00000, CommonLib::FilterMode::STD_AND_EXT, true);
		can.start();

		for(auto &io:GPIOBoard::IO){
			io.set_period(0);
			io.set_duty(0xFFFF);
		}

		monitor_timer.set_task(monitor_task);
		pwm_timer.set_task([](){
			for(auto &io:GPIOBoard::IO){io.update();}
			GPIOBoard::pin_interrupt_check();
		});
		led_timer.set_task([](){
			LED_R.update();
			LED_G.update();
			LED_B.update();
			for(auto &io:IO){ io.sequence_update(); }
		});

		pwm_timer.set_and_start(20-1);
		led_timer.set_and_start(1000-1);
	}

	uint8_t read_board_id(void){
		struct GPIOParam{
			GPIO_TypeDef * port;
			uint16_t pin;
			GPIOParam(GPIO_TypeDef * _port,uint16_t _pin):port(_port),pin(_pin){}
		};

		auto dip_sw = std::array<GPIOParam,4>{
			GPIOParam{ID0_GPIO_Port,ID0_Pin},
			GPIOParam{ID1_GPIO_Port,ID1_Pin},
			GPIOParam{ID2_GPIO_Port,ID2_Pin},
			GPIOParam{ID3_GPIO_Port,ID3_Pin},
		};

		uint8_t id = 0;
		for(int i = 0; i<4; i++){
			id |= !(uint8_t)HAL_GPIO_ReadPin(dip_sw.at(i).port,dip_sw.at(i).pin) << i;
		}
		return id;
	}

	void can_data_process(void){
		if(can.rx_available()){
			CommonLib::DataPacket rx_data;
			CommonLib::CanFrame rx_frame;
			can.rx(rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);

			if(rx_data.board_ID == board_id && rx_data.data_type == CommonLib::DataType::GPIOC_DATA){
				execute_gpio_command(board_id,rx_data,GPIOLib::CommPort::CAN_BUS);
			}else if((board_id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::COMMON_DATA)
					||(rx_data.data_type == CommonLib::DataType::COMMON_DATA_ENFORCE)){
				execute_common_command(board_id,rx_data,GPIOLib::CommPort::CAN_BUS);
			}
		}
	}

	void uart_data_process(void){
		if(uart.rx_available()){
			CommonLib::DataPacket rx_data;
			CommonLib::CanFrame rx_frame;
			CommonLib::SerialData rx_serial;

			uart.rx(rx_serial);

			if(CommonLib::DataConvert::slcan_to_can((char*)rx_serial.data, rx_frame)){
				CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);

				if(rx_data.board_ID == board_id && rx_data.data_type == CommonLib::DataType::GPIOC_DATA){
					execute_gpio_command(board_id,rx_data,GPIOLib::CommPort::UART);
				}else if((board_id == rx_data.board_ID && rx_data.data_type == CommonLib::DataType::COMMON_DATA)
						||(rx_data.data_type == CommonLib::DataType::COMMON_DATA_ENFORCE)){
					execute_common_command(board_id,rx_data,GPIOLib::CommPort::UART);
				}
			}
		}
	}

	void execute_gpio_command(size_t board_id,const CommonLib::DataPacket &rx_data,GPIOLib::CommPort port){
		if(rx_data.is_request){
			CommonLib::CanFrame tx_frame;
			CommonLib::DataPacket tx_data;
			CommonLib::SerialData tx_serial;
			auto writer = tx_data.writer();

			if(id_map.get(rx_data.register_ID, writer)){
				tx_data.board_ID = board_id;
				tx_data.data_type = CommonLib::DataType::GPIOC_DATA;
				tx_data.priority = rx_data.priority;
				tx_data.register_ID = rx_data.register_ID;

				CommonLib::DataConvert::encode_can_frame(tx_data, tx_frame);

				switch(port){
				case GPIOLib::CommPort::CAN_BUS:
					can.tx(tx_frame);
					break;
				case GPIOLib::CommPort::UART:
					tx_serial.size = CommonLib::DataConvert::can_to_slcan(tx_frame, (char*)tx_serial.data, tx_serial.max_size);
					uart.tx(tx_serial);
					break;
				}
			}
		}else{
			auto reader = rx_data.reader();
			id_map.set(rx_data.register_ID, reader);
		}
	}
	void execute_common_command(size_t board_id,const CommonLib::DataPacket &rx_data,GPIOLib::CommPort port){
		CommonLib::DataPacket tx_data;
		CommonLib::CanFrame tx_frame;
		CommonLib::SerialData tx_serial;

		switch((GPIOLib::CommonReg)rx_data.register_ID){
		case GPIOLib::CommonReg::NOP:
			break;

		case GPIOLib::CommonReg::ID_REQEST:
			if(rx_data.is_request){
				tx_data.board_ID = board_id;
				tx_data.data_type = CommonLib::DataType::COMMON_DATA;
				tx_data.register_ID = (uint16_t)GPIOLib::CommonReg::ID_REQEST;
				tx_data.writer().write<uint8_t>((uint8_t)CommonLib::DataType::GPIOC_DATA);
				tx_data.priority = rx_data.priority;

				CommonLib::DataConvert::encode_can_frame(tx_data,tx_frame);

				switch(port){
				case GPIOLib::CommPort::CAN_BUS:
					can.tx(tx_frame);
					break;
				case GPIOLib::CommPort::UART:
					tx_serial.size = CommonLib::DataConvert::can_to_slcan(tx_frame, (char*)tx_serial.data, tx_serial.max_size);
					uart.tx(tx_serial);
					break;
				}
			}
			break;

		case GPIOLib::CommonReg::EMERGENCY_STOP:
			emergency_stop_sequence();
			break;

		case GPIOLib::CommonReg::RESET_EMERGENCY_STOP:
			emergency_stop_release_sequence();
			break;

		default:
			break;
		}
	}
	void emergency_stop_sequence(void){
		GPIOBoard::LED_R.play(GPIOLib::error);
		for(auto &io: IO){
			io.set_output_state(false);
		}
	}
	void emergency_stop_release_sequence(void){
		for(size_t i = 0; i < esc_mode.size(); i++){
			if(esc_mode.test(i)){
				IO[i].set_input_mode(false);
				IO[i].set_period(1000);
				IO[i].set_output_state(true);
				IO[i].start_sequcence(GPIOLib::PWMSequence::esc_init);
			}
		}
	}

	void monitor_task(void){
		for(auto &map_element : id_map.accessors_map){
			if(map_element.first < monitor.size()){
				if(monitor.test(map_element.first)){
					CommonLib::DataPacket tx_packet;
					CommonLib::CanFrame tx_frame;
					tx_packet.register_ID = map_element.first;
					tx_packet.board_ID = board_id;
					tx_packet.data_type = CommonLib::DataType::GPIOC_DATA;

					auto writer = tx_packet.writer();
					if(map_element.second.get(writer)){
						CommonLib::DataConvert::encode_can_frame(tx_packet, tx_frame);
						can.tx(tx_frame);
					}
				}
			}
		}
	}

	static int count = 0;
	void pin_interrupt_check(void){
		uint16_t tmp = GPIOBoard::port_read();
		if((tmp&pin_interrupt_mask) != (port_read_old_val&pin_interrupt_mask)){
			if(count > 10){
				count = 0;
				CommonLib::DataPacket tx_packet;
				CommonLib::CanFrame tx_frame;

				tx_packet.data_type = CommonLib::DataType::GPIOC_DATA;
				tx_packet.register_ID = 0x02;
				tx_packet.board_ID = board_id;
				tx_packet.writer().write<uint16_t>(tmp);

				CommonLib::DataConvert::encode_can_frame(tx_packet, tx_frame);

				can.tx(tx_frame);

				port_read_old_val = tmp;
			}else{
				count ++;
			}

		}else{
			count = 0;
		}

	}
}
