/*
 * Register_contorl.hpp
 *
 *  Created on: Feb 23, 2024
 *      Author: yaa3k
 */

#ifndef ID_CONTROL_HPP_
#define ID_CONTROL_HPP_

#include "STM32HAL_CommonLib/byte_reader_writer.hpp"
#include <functional>

namespace G24_STM32HAL::GPIOLib{

	struct DataManager{
		std::function<bool(CommonLib::ByteReader&)> f_set;
		std::function<bool(CommonLib::ByteWriter&)> f_get;
		DataManager(std::function<bool(CommonLib::ByteReader&)>&& _write,
				std::function<bool(CommonLib::ByteWriter&)>&&_read):
			f_set(_write),f_get(_read){
		}
		//byte->ref
		bool set(CommonLib::ByteReader& r){
			return f_set?f_set(r):false;
		}
		//ref->byte
		bool get(CommonLib::ByteWriter& w){
			return f_get?f_get(w):false;
		}

		template<class T> static DataManager generate(T& ref){
			auto readf = [&](CommonLib::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					ref = val.value();
					return true;
				}else{
					return false;
				}
			};
			auto writef = [&](CommonLib::ByteWriter& w){
				w.write<T>(ref);
				return true;
			};
			return DataManager(readf,writef);
		}

		template<class T> static DataManager generate(std::function<void(T)>&& setter,std::function<T(void)>&& getter){
			auto readf = [&](CommonLib::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					setter(val.value());
					return true;
				}else{
				   return false;
				}
			};
			auto writef = [&](CommonLib::ByteWriter& w){
				w.write(getter());
				return true;
			};
			return DataManager(readf,writef);
		}
		template<class T> static DataManager generate(std::function<void(T)>&& setter){
			auto readf = [&](CommonLib::ByteReader& r){
				std::optional<T> val = r.read<T>();
				if(val.has_value()){
					setter(val.value());
					return true;
				}else{
				   return false;
				}
			};
			return DataManager(readf,nullptr);
		}
		template<class T> static DataManager generate(std::function<T(void)>&& getter){
			auto writef = [&](CommonLib::ByteWriter& w){
				w.write(getter());
				return true;
			};
			return DataManager(nullptr,writef);
		}
	};


	class IDMap{
		std::unordered_map<int, DataManager> cells;
	public:
		IDMap(std::unordered_map<int, DataManager>&& _cells):cells(_cells){}

		bool set(int id,CommonLib::ByteReader& r){
			auto iter=cells.find(id);
			if (iter!=cells.end()){
				return iter->second.f_set(r);
			}
			return false;
		}
		bool get(int id,CommonLib::ByteWriter& w){
			auto iter=cells.find(id);
			if (iter!=cells.end()){
				return iter->second.f_get(w);
			}
			return false;
		}
	};

	class IDMapBuilder{
	public:
		std::unordered_map<int, DataManager> managers_map;
		IDMapBuilder& add(int id,const DataManager& c){
			managers_map.insert(std::pair(id, c));
			return *this;
		}
		IDMap build(){
		   return IDMap(std::move(managers_map));
		}
	};


}



#endif /* ID_CONTROL_HPP_ */
