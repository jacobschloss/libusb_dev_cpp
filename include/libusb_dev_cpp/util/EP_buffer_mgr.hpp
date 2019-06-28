#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include "freertos_cpp_util/object_pool/Object_pool.hpp"

template <size_t LEN, size_t ALLIGN>
class EP_buffer_array : public Buffer_adapter_base
{

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

	//driver process
	virtual Buffer_adapter_base* get_empty_buffer(const uint8_t ep) = 0;
	virtual void enqueue_buffer(Buffer_adapter_base* const buf) = 0;

	//application process
	virtual Buffer_adapter_base* wait_buffer(const uint8_t ep) = 0;
	virtual void release_buffer(Buffer_adapter_base* const buf) = 0;

protected:
};

template<size_t NUM_EP>
class EP_buffer_mgr_freertos
{
	public:

	protected:
		std::array<Object_pool<EP_buffer_array<512, 32>, 4>, NUM_EP> m_tx_buffer;
		std::array<Object_pool<EP_buffer_array<512, 32>, 4>, NUM_EP> m_rx_buffer;
};