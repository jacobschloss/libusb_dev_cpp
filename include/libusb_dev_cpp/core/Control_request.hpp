#pragma once

#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include <array>

#include <cstdint>

class Control_request
{
public:

	Setup_packet setup_packet;
	uint8_t* data_stage_buffer;
	size_t	 data_stage_buffer_max;
	size_t	 data_stage_buffer_maxlen;
};