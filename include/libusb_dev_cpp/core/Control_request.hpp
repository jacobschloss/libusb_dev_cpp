/**
 * @brief Control_request
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include <array>

#include <cstdint>

class Control_request
{
public:

	Control_request()
	{
		data_stage_buffer = nullptr;
		data_stage_buffer_max = 0;
		data_stage_buffer_len = 0;
	}

	Control_request(uint8_t* const buffer, const size_t len)
	{
		data_stage_buffer = buffer;
		data_stage_buffer_max = len;
		data_stage_buffer_len = 0;
	}

	Setup_packet setup_packet;
	uint8_t* data_stage_buffer;
	size_t data_stage_buffer_max;
	size_t	 data_stage_buffer_len;

};