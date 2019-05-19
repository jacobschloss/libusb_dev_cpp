#pragma once

class CPU_base
{
public:
	virtual void instruction_barrier() = 0;
	virtual void data_barrier() = 0;
	virtual void data_instruction_barrier() = 0;
};
