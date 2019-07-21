#include "libusb_dev_cpp/class/cdc/cdc_mgmt_requests.hpp"

#include "common_util/Byte_util.hpp"

bool CDC::LINE_CODING::serialize(Line_coding_array* const out_array) const
{
	return false;
}
bool CDC::LINE_CODING::serialize(Buffer_adapter* const out_array) const
{
	return false;
}

bool CDC::LINE_CODING::deserialize(const Line_coding_array& array)
{
	dwDTERRate  = Byte_util::make_u32(array[0], array[1], array[2], array[3]);

	bool ret = true;

	switch(bCharFormat)
	{
		case CHAR_FORMAT::ONE_STOP:
		case CHAR_FORMAT::ONEPTFIVE_STOP:
		case CHAR_FORMAT::TWO_STOP:
		{
			bCharFormat = static_cast<CHAR_FORMAT>(array[4]);
			break;
		}
		default:
		{
			ret = false;
			break;
		}
	}
	switch(bParityType)
	{
		case PARITY_TYPE::NONE:
		case PARITY_TYPE::ODD:
		case PARITY_TYPE::EVEN:
		case PARITY_TYPE::MARK:
		case PARITY_TYPE::SPACE:
		{
			bParityType = static_cast<PARITY_TYPE>(array[5]);
			break;
		}
		default:
		{
			ret = false;
			break;	
		}
	}
	switch(bDataBits)
	{
		case DATA_BITS::FIVE:
		case DATA_BITS::SIX:
		case DATA_BITS::SEVEN:
		case DATA_BITS::EIGHT:
		case DATA_BITS::SIXTEEN:
		{
			bDataBits = static_cast<DATA_BITS>(array[6]);
			break;
		}
		default:
		{
			ret = false;
			break;	
		}
	}

	return ret;
}