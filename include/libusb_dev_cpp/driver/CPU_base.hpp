#pragma once

class CPU_base
{
public:
	inline virtual void instruction_barrier() = 0;
	inline virtual void data_barrier() = 0;

	inline virtual void instruction_sync() = 0;
	inline virtual void data_sync() = 0;
	inline virtual void data_instruction_sync() = 0;
};
