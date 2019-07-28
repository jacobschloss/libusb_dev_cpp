/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

class EP_buffer_mgr_base
{
public:

	virtual ~EP_buffer_mgr_base()
	{
		
	}

	//both
	virtual size_t get_num_ep() const = 0;
	virtual bool set_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;
	virtual Buffer_adapter_base* get_buffer(const uint8_t ep) = 0;

	
	virtual Buffer_adapter_base* poll_allocate_buffer_isr(const uint8_t ep) = 0;
	virtual Buffer_adapter_base* poll_allocate_buffer(const uint8_t ep) = 0;
	virtual Buffer_adapter_base* wait_allocate_buffer(const uint8_t ep) = 0;
	
	virtual bool poll_enqueue_buffer_isr(const uint8_t ep, Buffer_adapter_base* const buf) = 0;
	virtual bool poll_enqueue_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;
	
	virtual Buffer_adapter_base* poll_dequeue_buffer_isr(const uint8_t ep) = 0;
	virtual Buffer_adapter_base* poll_dequeue_buffer(const uint8_t ep) = 0;
	virtual Buffer_adapter_base* wait_dequeue_buffer(const uint8_t ep) = 0;

	virtual void release_buffer_isr(const uint8_t ep, Buffer_adapter_base* const buf) = 0;
	virtual void release_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;

protected:
};
