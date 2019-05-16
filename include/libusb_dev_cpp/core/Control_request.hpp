#pragma once

#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include <array>

#include <cstdint>

class Control_request
{
public:

	Control_request() : data_stage_buffer(nullptr), data_stage_buffer_max(0)
	{
		data_stage_buffer_len = 0;
	}

	Control_request(uint8_t* const buffer, const size_t len) : data_stage_buffer(buffer), data_stage_buffer_max(len)
	{
		data_stage_buffer_len = 0;
	}

	Setup_packet setup_packet;
	uint8_t* const data_stage_buffer;
	const size_t data_stage_buffer_max;
	
	size_t	 data_stage_buffer_len;
};