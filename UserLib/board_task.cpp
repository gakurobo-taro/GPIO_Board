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

		CommonLib::DataPacket rx_data;
		if(can.rx_available()){
			CommonLib::CanFrame rx_frame;
			can.rx(rx_frame);
			CommonLib::DataConvert::decode_can_frame(rx_frame, rx_data);

			if(rx_data.board_ID == id){
				if(rx_data.is_request){
					CommonLib::CanFrame tx_frame;
					auto writer = tx_frame.writer();

					if(id_map.get(rx_data.register_ID, writer)){
						tx_frame.id = rx_frame.id;
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
}
