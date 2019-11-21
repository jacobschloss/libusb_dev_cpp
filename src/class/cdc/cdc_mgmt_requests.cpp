#include "libusb_dev_cpp/class/cdc/cdc_mgmt_requests.hpp"

#include "common_util/Byte_util.hpp"

#include "freertos_cpp_util/logging/Global_logger.hpp"

using freertos_util::logging::Global_logger;
using freertos_util::logging::LOG_LEVEL;

bool CDC::LINE_CODING::serialize(Line_coding_array* const out_array) const
{
	return false;
}
bool CDC::LINE_CODING::serialize(Buffer_adapter_tx* const out_array) const
{
	return false;
}

bool CDC::LINE_CODING::deserialize(const Buffer_adapter_base* buf)
{
	freertos_util::logging::Logger* const logger = freertos_util::logging::Global_logger::get();

	if(buf->size() != std::tuple_size<Line_coding_array>::value)
	{
		logger->log(LOG_LEVEL::INFO, "CDC::LINE_CODING", "deserialize: buf wrong size");
		return false;
	}

	Line_coding_array array;

	std::copy_n(buf->data(), buf->size(), array.data());

	return deserialize(array);
}

bool CDC::LINE_CODING::deserialize(const Line_coding_array& array)
{
	freertos_util::logging::Logger* const logger = freertos_util::logging::Global_logger::get();

	dwDTERRate  = Byte_util::make_u32(array[0], array[1], array[2], array[3]);

	bool ret = true;

	switch(static_cast<CHAR_FORMAT>(array[4]))
	{
		case CHAR_FORMAT::ONE_STOP:
		case CHAR_FORMAT::ONEPTFIVE_STOP:
		case CHAR_FORMAT::TWO_STOP:
		{
			bCharFormat = array[4];
			break;
		}
		default:
		{
			logger->log(LOG_LEVEL::INFO, "CDC::LINE_CODING", "deserialize: CHAR_FORMAT unk");
			ret = false;
			break;
		}
	}
	switch(static_cast<PARITY_TYPE>(array[5]))
	{
		case PARITY_TYPE::NONE:
		case PARITY_TYPE::ODD:
		case PARITY_TYPE::EVEN:
		case PARITY_TYPE::MARK:
		case PARITY_TYPE::SPACE:
		{
			bParityType = array[5];
			break;
		}
		default:
		{
			logger->log(LOG_LEVEL::INFO, "CDC::LINE_CODING", "deserialize: PARITY_TYPE unk");
			ret = false;
			break;	
		}
	}
	switch(static_cast<DATA_BITS>(array[6]))
	{
		case DATA_BITS::FIVE:
		case DATA_BITS::SIX:
		case DATA_BITS::SEVEN:
		case DATA_BITS::EIGHT:
		case DATA_BITS::SIXTEEN:
		{
			bDataBits = array[6];
			break;
		}
		default:
		{
			logger->log(LOG_LEVEL::INFO, "CDC::LINE_CODING", "deserialize: DATA_BITS unk: %d", array[6]);
			ret = false;
			break;	
		}
	}

	return ret;
}