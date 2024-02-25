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

		can.start();
		can.set_filter_free(0);

		for(auto &io:GPIOBoard::IO){
			io.set_period(0);
			io.set_duty(0xFFFF);
		}
	}
	uint8_t read_board_id(void){
		uint8_t id = 0;
		for(int i = 0; i<4; i++){
			id |= !(uint8_t)HAL_GPIO_ReadPin(dip_sw.at(i).port,dip_sw.at(i).pin) << i;
		}
		return id;
	}

	void main_data_process(void){
		int id = read_board_id();

		if(can.rx_available()){
			CommonLib::DataPacket rx_data;
			CommonLib::CanFrame rx_frame;
			can.rx(rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);

			if(rx_data.board_ID == id && rx_data.data_type == CommonLib::DataType::GPIOC_DATA){
				if(rx_data.is_request){
					CommonLib::CanFrame tx_frame;
					auto writer = tx_frame.writer();

					if(id_map.get(rx_data.register_ID, writer)){
						tx_frame.id = rx_frame.id;
						tx_frame.is_ext_id = true;
						tx_frame.is_remote = false;

						can.tx(tx_frame);
					}
				}else{
					auto reader = rx_data.reader();
					id_map.set(rx_data.register_ID, reader);
				}
			}
		}
	}

	void monitor_task(void){
		for(auto &map_element : id_map.managers_map){
			if(map_element.first < monitor.size()){
				if(monitor.test(map_element.first)){
					CommonLib::DataPacket tx_packet;
					CommonLib::CanFrame tx_frame;
					tx_packet.register_ID = map_element.first;
					tx_packet.board_ID = read_board_id();
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
	int count = 0;
	void pin_interrupt_check(void){
		uint16_t tmp = GPIOBoard::port_read();
		if((port_read()&pin_interrupt_mask) != (port_read_old_val&pin_interrupt_mask)){
			if(count > 10){
				count = 0;
				CommonLib::DataPacket tx_packet;
				CommonLib::CanFrame tx_frame;

				tx_packet.data_type = CommonLib::DataType::GPIOC_DATA;
				tx_packet.register_ID = 0x02;
				tx_packet.board_ID = read_board_id();
				auto writer = tx_packet.writer();

				writer.write<uint16_t>(tmp);

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
