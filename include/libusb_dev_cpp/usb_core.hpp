#pragma once

#include "common_util/Byte_util.hpp"

#include <array>

#include <cstdint>

class USB_setup_packet
{
public:

	typedef std::array<uint8_t, 8> USB_setup_packet_array;

	enum class DEVICE_REQUEST : uint8_t
	{
		GET_STATUS        = 0x00,
		CLEAR_FEATURE     = 0x01,
		SET_FEATURE       = 0x03,
		SET_ADDRESS       = 0x05,
		GET_DESCRIPTOR    = 0x06,
		SET_DESCRIPTOR    = 0x07,
		GET_CONFIGURATION = 0x08,
		SET_CONFIGURATION = 0x09,
	}

	bool serialize(USB_setup_packet_array* const out_array)
	{
		if(!request_type.serialize(&out_array[0]))
		{
			return false;
		}

		in_array[1] = request;

		in_array[2] = Byte_util::get_b0(value);
		in_array[3] = Byte_util::get_b1(value);

		in_array[4] = Byte_util::get_b0(index);
		in_array[5] = Byte_util::get_b1(index);

		in_array[6] = Byte_util::get_b0(count);
		in_array[7] = Byte_util::get_b1(count);
	}
	bool deserialize(const USB_setup_packet_array& in_array)
	{
		if(!request_type.deserialize(in_array[0]))
		{
			return false;
		}

		request = in_array[1];

		value = Byte_util::make_u16(in_array[3], in_array[2]);
		index = Byte_util::make_u16(in_array[5], in_array[4]);
		count = Byte_util::make_u16(in_array[7], in_array[6]);
	}

	class RequestType
	{
	public:

		bool serialize(uint8_t* const out_request_type)
		{
			uint8_t req_type_temp = 0;

			switch(data_dir)
			{
				case DATA_DIR::HOST_TO_DEV:
				{
					req_type_temp &= ~(0x80);
					break;
				}
				case DATA_DIR::DEV_TO_HOST:
				{
					req_type_temp |= 0x80;
					break;
				}
				default:
				{
					return false;
				}
			}

			switch(type)
			{
				case TYPE::STANDARD:
				{
					req_type_temp |= (0 << 5);
					break;
				}
				case TYPE::CLASS:
				{
					req_type_temp |= (1 << 5);
					break;
				}
				case TYPE::VENDOR:
				{
					req_type_temp |= (2 << 5);
					break;
				}
				case TYPE::RESEVED:
				{
					req_type_temp |= (3 << 5);
					break;
				}
				default:
				{
					return false;
				}
			}

			switch(recipient)
			{
				case DEVICE:
				{
					req_type_temp |= 0;
					break;
				}
				case INTERFACE:
				{
					req_type_temp |= 1;
					break;
				}
				case ENDPOINT:
				{
					req_type_temp |= 2;
					break;
				}
				case OTHER:
				{
					req_type_temp |= 3;
					break;
				}
				default:
				{
					return false;
				}
			}

			*out_request_type = req_type_temp;
			return true;
		}
		bool deserialize(const uint8_t request_type)
		{
			if(request_type & 0x80)
			{
				data_dir = DATA_DIR::HOST_TO_DEV;
			}
			else
			{
				data_dir = DATA_DIR::DEV_TO_HOST;
			}

			switch( (request_type & 0x60) >> 5 )
			{
				case 0U:
				{
					type = TYPE::STANDARD;
					break;
				}
				case 1U:
				{
					type = TYPE::CLASS;
					break;
				}
				case 2U:
				{
					type = TYPE::VENDOR;
					break;
				}
				case 3U:
				{
					type = TYPE::RESEVED;
					break;
				}
				default:
				{
					//this should never happen
					return false;	
				}
			}

			switch(request_type & 0x1F)
			{
				case 0U:
				{
					recipient = RECIPIENT::DEVICE;
					break;
				}
				case 1U:
				{
					recipient = RECIPIENT::INTERFACE;
					break;
				}
				case 2U:
				{
					recipient = RECIPIENT::ENDPOINT;
					break;
				}
				case 3U:
				{
					recipient = RECIPIENT::OTHER;
					break;
				}
				default:
				{
					return false;
				}
			}

			return true;
		}

		enum class DATA_DIR
		{
			HOST_TO_DEV = 0,
			DEV_TO_HOST = 1
		};

		enum class TYPE
		{
			STANDARD = 0,
			CLASS = 1,
			VENDOR = 2,
			RESEVED = 3
		};

		enum class RECIPIENT
		{
			DEVICE = 0,
			INTERFACE = 1,
			ENDPOINT = 2,
			OTHER = 3
		};

	protected:
		DATA_DIR data_dir;
		TYPE type;
		RECIPIENT recipient;
	};

protected:
	RequestType request_type;
	uint8_t request;
	uint16_t value;
	uint16_t index;
	uint16_t count;
};

