#include "libusb_dev_cpp/core/Request_type.hpp"

bool Request_type::serialize(uint8_t* const out_request_type)
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
		case RECIPIENT::DEVICE:
		{
			req_type_temp |= 0;
			break;
		}
		case RECIPIENT::INTERFACE:
		{
			req_type_temp |= 1;
			break;
		}
		case RECIPIENT::ENDPOINT:
		{
			req_type_temp |= 2;
			break;
		}
		case RECIPIENT::OTHER:
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
bool Request_type::deserialize(const uint8_t request_type)
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