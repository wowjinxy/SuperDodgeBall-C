#ifndef SDB_HPP
#define SDB_HPP

#include "nesemu1.hh"
#include "Types.hpp"

/**
 * Abstraction for interfacing with memory addresses.
 */
class MemoryAccess
{
public:
	MemoryAccess(int address)
	{
		this->address = address;
	}

	bool operator == (NESbyte value) const
	{
		return ((NESbyte)(*this) == value);
	}

	bool operator != (NESbyte value) const
	{
		return !(*this == value);
	}

	MemoryAccess& operator = (NESbyte value)
	{
		writeMemory(address, value);
		return *this;
	}

	MemoryAccess operator [] (int index)
	{
		return MemoryAccess(address + index);
	}

	MemoryAccess& operator++()
	{
		writeMemory(address, readMemory(address) + 1);
		return *this;
	}

	MemoryAccess& operator++(int)
	{
		++(*this);
		return *this;
	}

	MemoryAccess& operator--()
	{
		writeMemory(address, readMemory(address) - 1);
		return *this;
	}

	MemoryAccess& operator--(int)
	{
		--(*this);
		return *this;
	}

	operator NESbyte() const
	{
		return readMemory(address);
	}

private:
	int address;
};

/// Type for SDB C++ translated functions
typedef void(*SDB_Function)(void);

/// Translated functions are called from this table on each instruction fetch
extern SDB_Function SDB_TranslationTable[65536];

/*
 * Public functions
 */

 /**
  * Initialize Super Mario Bros.
  */
void SDB_Init();

#endif // SDB_HPP