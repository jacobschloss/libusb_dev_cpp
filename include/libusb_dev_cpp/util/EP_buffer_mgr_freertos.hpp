/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "freertos_cpp_util/object_pool/Object_pool.hpp"

#include "libusb_dev_cpp/util/EP_buffer_array.hpp"
#include "libusb_dev_cpp/util/EP_buffer_mgr_base.hpp"

#include <atomic>

template<size_t NUM_EP, size_t BUFFER_DEPTH, size_t BUFFER_LEN, size_t BUFFER_ALLIGN>
class EP_buffer_mgr_freertos : public EP_buffer_mgr_base
{
	public:
	
	EP_buffer_mgr_freertos()
	{
		for(size_t i = 0; i < m_active_buffer.size(); i++)
		{
			std::atomic_init<Buffer_adapter_base*>(&m_active_buffer[i], nullptr);
		}
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
	Buffer_adapter_base* poll_allocate_buffer_isr(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		Buffer_adapter_base* buf = m_ep_buffer[ep].try_allocate_isr(&xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

		return buf;
	}
	Buffer_adapter_base* poll_allocate_buffer(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		//verify if this is really an interrupt
		//in some cases eg the USB library will have a code path that is optionally polled or ISR
		if(xPortIsInsideInterrupt() == pdTRUE)
		{
			return poll_allocate_buffer_isr(ep);
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

	bool poll_enqueue_buffer_isr(const uint8_t ep, Buffer_adapter_base* const buf) override
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

		return m_app_buffer[ep].push_back_isr(ptr);
	}

	bool poll_enqueue_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override
	{
		if(ep > NUM_EP)
		{
			return false;
		}

		//verify if this is really an interrupt
		//in some cases eg the USB library will have a code path that is optionally polled or ISR
		if(xPortIsInsideInterrupt() == pdTRUE)
		{
			return poll_enqueue_buffer_isr(ep, buf);
		}

		// EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = dynamic_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* ptr = static_cast< EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>* >(buf);
		if(ptr == nullptr)
		{
			return false;	
		}

		return m_app_buffer[ep].push_back(ptr);
	}

	Buffer_adapter_base* poll_dequeue_buffer_isr(const uint8_t ep) override
	{
		if(ep > NUM_EP)
		{
			return nullptr;
		}

		Buffer_adapter_base* buf = nullptr;
		m_app_buffer[ep].pop_front_isr(&buf);

		return buf;
	}
	Buffer_adapter_base* poll_dequeue_buffer(const uint8_t ep) override
	{
		//verify if this is really an interrupt
		//in some cases eg the USB library will have a code path that is optionally polled or ISR
		if(xPortIsInsideInterrupt() == pdTRUE)
		{
			return poll_dequeue_buffer_isr(ep);
		}

		return wait_dequeue_buffer(ep, 0);
	}
	Buffer_adapter_base* wait_dequeue_buffer(const uint8_t ep) override
	{
		return wait_dequeue_buffer(ep, portMAX_DELAY);
	}
	Buffer_adapter_base* wait_dequeue_buffer(const uint8_t ep, const TickType_t xTicksToWait)
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
	Buffer_adapter_base* wait_dequeue_buffer(const uint8_t ep, const std::chrono::duration<Rep,Period>& duration)
	{
		const std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
		return wait_dequeue_buffer(ep, pdMS_TO_TICKS(duration_ms.count()));
	}
	void release_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override
	{
		if(ep > NUM_EP)
		{
			//throw? log?
			return;
		}

		//verify if this is really an interrupt
		//in some cases eg the USB library will have a code path that is optionally polled or ISR
		if(xPortIsInsideInterrupt() == pdTRUE)
		{
			release_buffer_isr(ep, buf);
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

	void release_buffer_isr(const uint8_t ep, Buffer_adapter_base* const buf) override
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

		m_ep_buffer[ep].deallocate_isr(ptr);
	}

	protected:
		std::array<
			Object_pool<
				EP_buffer_array<BUFFER_LEN, BUFFER_ALLIGN>,
				BUFFER_DEPTH
				>,
			NUM_EP> m_ep_buffer;

		std::array<
			std::atomic<Buffer_adapter_base*>,
			NUM_EP> m_active_buffer;

		std::array<
			Queue_static_pod<
				Buffer_adapter_base*,
				BUFFER_DEPTH
				>,
			NUM_EP> m_app_buffer;
};
