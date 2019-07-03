/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include "freertos_cpp_util/object_pool/Object_pool.hpp"

template <size_t LEN, size_t ALLIGN>
class EP_buffer_array : public Buffer_adapter_base
{
public:

	EP_buffer_array() : Buffer_adapter_base(m_buf.data(), m_buf.size())
	{

	}

protected:
	alignas(ALLIGN) std::array<uint8_t, LEN> m_buf;
};

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

	//driver process
	virtual Buffer_adapter_base* poll_allocate_buffer(const uint8_t ep) = 0;
	virtual Buffer_adapter_base* wait_allocate_buffer(const uint8_t ep) = 0;
	virtual bool enqueue_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;

	//application process
	virtual Buffer_adapter_base* wait_buffer(const uint8_t ep) = 0;
	virtual void release_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;

protected:
};

template<size_t NUM_EP, size_t BUFFER_DEPTH, size_t BUFFER_LEN, size_t BUFFER_ALLIGN>
class EP_buffer_mgr_freertos : public EP_buffer_mgr_base
{
	public:
	
	EP_buffer_mgr_freertos()
	{
		m_active_buffer.fill(nullptr);
	}

	//both
	size_t get_num_ep() const override
	{
		return NUM_EP;
	}

	bool set_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override
	{
		if(ep > NUM_EP)
		{
			return false;
		}

		m_active_buffer[ep] = buf;
		return true;
	}
	Buffer_adapter_base* get_buffer(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		return m_active_buffer[ep];
	}

	Buffer_adapter_base* poll_allocate_buffer(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		return m_ep_buffer[ep].allocate();
	}
	Buffer_adapter_base* wait_allocate_buffer(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		return m_ep_buffer[ep].try_allocate_for_ticks(portMAX_DELAY);
	}
	Buffer_adapter_base* allocate_buffer(const uint8_t ep, const TickType_t xTicksToWait)
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		return m_ep_buffer[ep].try_allocate_for_ticks(xTicksToWait);
	}
	template< class Rep, class Period >
	Buffer_adapter_base* allocate_buffer(const uint8_t ep, const std::chrono::duration<Rep,Period>& duration)
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		return m_ep_buffer[ep].try_allocate_for(duration);
	}

	bool enqueue_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override
	{
		if(ep > NUM_EP)
		{
			return false;
		}

		// EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = dynamic_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = static_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		if(ptr == nullptr)
		{
			return false;	
		}

		return m_app_buffer[ep].push_back(ptr);
	}

	Buffer_adapter_base* wait_buffer(const uint8_t ep) override
	{
		return wait_buffer(ep, portMAX_DELAY);
	}
	Buffer_adapter_base* wait_buffer(const uint8_t ep, const TickType_t xTicksToWait)
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		Buffer_adapter_base* buf = nullptr;
		m_app_buffer[ep].pop_front(&buf, xTicksToWait);

		return buf;
	}
	template< class Rep, class Period >
	Buffer_adapter_base* wait_buffer(const uint8_t ep, const std::chrono::duration<Rep,Period>& duration)
	{
		const std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
		return wait_buffer(ep, pdMS_TO_TICKS(duration_ms.count()));
	}
	void release_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override
	{
		if(ep > NUM_EP)
		{
			//throw? log?
			return;
		}

		// EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = dynamic_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = static_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		if(ptr == nullptr)
		{
			return;	
		}

		m_ep_buffer[ep].deallocate(ptr);
	}

	protected:
		std::array<
			Object_pool<
				EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>,
				BUFFER_DEPTH
				>,
			NUM_EP> m_ep_buffer;

		std::array<
			Buffer_adapter_base*,
			NUM_EP> m_active_buffer;

		std::array<
			Queue_static_pod<
				Buffer_adapter_base*,
				BUFFER_DEPTH
				>,
			NUM_EP> m_app_buffer;
};
