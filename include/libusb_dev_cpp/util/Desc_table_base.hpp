#pragma once

#include <map>
#include <memory>
#include <type_traits>

template <typename T>
class Desc_table_base
{
public:
	typedef std::shared_ptr<T> Desc_ptr;
	typedef std::shared_ptr<const T> Desc_const_ptr;

	template <typename U, typename = std::enable_if_t<std::is_copy_constructible<U>::value> >
	void set_config(const uint8_t idx, const U& desc)
	{
		Desc_ptr desc_ptr = std::make_shared<U>(desc);
		m_table.insert(std::make_pair(idx, desc_ptr));
	}

	void set_config(const uint8_t idx, const Desc_ptr& desc_ptr)
	{
		m_table.insert(std::make_pair(idx, desc_ptr));
	}

	Desc_ptr get_config(const uint8_t idx)
	{
		auto it = m_table.find(idx);

		if(it == m_table.end())
		{
			return Desc_ptr();
		}

		return it->second;
	}

	Desc_const_ptr get_config(const uint8_t idx) const
	{
		auto it = m_table.find(idx);

		if(it == m_table.end())
		{
			return Desc_const_ptr();
		}

		return it->second;
	}

private:

	std::map<uint8_t, Desc_ptr> m_table;
};