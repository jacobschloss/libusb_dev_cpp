/**
 * @brief Request_type
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <array>

#include <cstdint>

class Request_type
{
public:

	bool serialize(uint8_t* const out_request_type);
	bool deserialize(const uint8_t request_type);

	enum class DATA_DIR
	{
		HOST_TO_DEV = 0,
		DEV_TO_HOST = 1
	};

	enum class TYPE
	{
		STANDARD  = 0,
		CLASS     = 1,
		VENDOR    = 2,
		RESERVED  = 3
	};

	enum class RECIPIENT
	{
		DEVICE    = 0,
		INTERFACE = 1,
		ENDPOINT  = 2,
		OTHER     = 3
	};

	DATA_DIR data_dir;
	TYPE type;
	RECIPIENT recipient;
};
