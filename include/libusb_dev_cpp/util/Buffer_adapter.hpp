/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <cstdint>
#include <cstddef>

#include <algorithm>

template <typename T>
class Buffer_adapter_base_T
{
public:

	Buffer_adapter_base_T()
	{
		clear();
	}

	Buffer_adapter_base_T(T* const buf_ptr, const size_t buf_max)
	{
		reset(buf_ptr, buf_max);
	}

	virtual ~Buffer_adapter_base_T()
	{

	}

	void reset()
	{
		m_buf_size  = 0;

		curr_ptr = m_buf_ptr;
		rem_len  = 0;
	}

	void reset(T* const buf, const size_t maxlen)
	{
		m_buf_ptr  = buf;
		m_buf_max  = maxlen;
		m_buf_size = 0;

		curr_ptr = m_buf_ptr;
		rem_len  = 0;
	}

	virtual size_t insert(const T& buf)
	{
		return insert(&buf, 1);
	}

	virtual size_t insert(const T* buf_ptr, const size_t len)
	{
		const size_t num_to_copy = std::min(len, capacity());
		std::copy_n(buf_ptr, num_to_copy, m_buf_ptr + m_buf_size);

		m_buf_size += num_to_copy;

		return num_to_copy;
	}

	T* data()
	{
		return m_buf_ptr;
	}
	const T* data() const
	{
		return m_buf_ptr;
	}

	bool full() const
	{
		return m_buf_size == m_buf_max;
	}
	bool empty() const
	{
		return m_buf_size == 0;
	}

	size_t size() const
	{
		return m_buf_size;
	}
	size_t capacity() const
	{
		return m_buf_max - m_buf_size;
	}
	size_t max_size() const
	{
		return m_buf_max;
	}

	void resize(const size_t len)
	{
		m_buf_size = len;
	}

	//remaining length
	size_t rem_len;

	//current position in buffer
	T* curr_ptr;

protected:
	
	void clear()
	{
		m_buf_ptr  = nullptr;
		m_buf_max  = 0;
		m_buf_size = 0;

		curr_ptr = nullptr;
		rem_len  = 0;
	}

	T* m_buf_ptr;
	size_t m_buf_max;
	size_t m_buf_size;
};

template <typename T>
class Buffer_adapter_tx_T : public Buffer_adapter_base_T<T>
{
public:
	Buffer_adapter_tx_T()
	{
		
	}

	size_t insert(const T& buf) override
	{
		return Buffer_adapter_tx_T::insert(&buf, 1);
	}

	size_t insert(const T* buf, const size_t len) override
	{
		const size_t num_inserted = Buffer_adapter_base_T<T>::insert(buf, len);
		
		Buffer_adapter_base_T<T>::rem_len += num_inserted;

		return num_inserted;
	}

protected:
};

template <typename T>
class Buffer_adapter_rx_T : public Buffer_adapter_base_T<T>
{
public:
	Buffer_adapter_rx_T()
	{
		
	}

	size_t insert(const T& buf) override
	{
		return Buffer_adapter_rx_T::insert(&buf, 1);
	}

	size_t insert(const T* buf, const size_t len) override
	{
		const size_t num_inserted = Buffer_adapter_base_T<T>::insert(buf, len);
		
		Buffer_adapter_base_T<T>::rem_len -= num_inserted;

		return num_inserted;
	}

protected:

};

typedef Buffer_adapter_base_T<uint8_t> Buffer_adapter_base;
typedef Buffer_adapter_rx_T<uint8_t> Buffer_adapter_rx;
typedef Buffer_adapter_tx_T<uint8_t> Buffer_adapter_tx;
