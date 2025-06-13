#ifndef NESEMU1_HH
#define NESEMU1_HH

#include <stdint.h>
#include <assert.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <SDL2/SDL.h>
#include <vector>

#include "Types.hpp"

// Integer types
typedef uint_least32_t u32;
typedef uint_least16_t u16;
typedef uint_least8_t   u8;
typedef  int_least8_t   s8;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Bitfield utilities
template<unsigned bitno, unsigned nbits = 1, typename T = u8>
struct RegBit
{
	T data;
	enum { mask = (1u << nbits) - 1u };
	template<typename T2>
	RegBit& operator=(T2 val)
	{
		data = (data & ~(mask << bitno)) | ((nbits > 1 ? val & mask : !!val) << bitno);
		return *this;
	}
	operator unsigned() const { return (data >> bitno) & mask; }
	RegBit& operator++ () { return *this = *this + 1; }
	unsigned operator++ (int) { unsigned r = *this; ++*this; return r; }
};

namespace IO
{
	extern SDL_Window* window;
	extern SDL_Renderer* renderer;
	extern SDL_Texture* texture;
	extern Uint32* frameBuffer;
	extern int joy_current[2], joy_next[2], joypos[2];

	void Init();
	void PutPixel(unsigned px, unsigned py, unsigned pixel, int offset);
	void FlushScanline(unsigned py);
	void JoyStrobe(unsigned v);
	u8 JoyRead(unsigned idx);
}

namespace GamePak
{
	extern std::vector<u8> ROM, VRAM;
	extern unsigned mappernum;
	extern const unsigned VROM_Granularity, VROM_Pages;
	extern const unsigned ROM_Granularity, ROM_Pages;
	extern unsigned char NRAM[0x1000], PRAM[0x2000];
	extern unsigned char* banks[8];  // ROM_Pages = 8
	extern unsigned char* Vbanks[8]; // VROM_Pages = 8
	extern unsigned char* Nta[4];

	template<unsigned npages, unsigned char* (&b)[npages], std::vector<u8>& r, unsigned granu>
	static void SetPages(unsigned size, unsigned baseaddr, unsigned index)
	{
		for (unsigned v = r.size() + index * size,
			p = baseaddr / granu;
			p < (baseaddr + size) / granu && p < npages;
			++p, v += granu)
			b[p] = &r[v % r.size()];
	}
	// SetROM and SetVROM are defined in the source file as aliases to template instantiations

	u8 Access(unsigned addr, u8 value, bool write);
	void Init();
}

namespace CPU /* CPU: Ricoh RP2A03 (based on MOS6502, almost the same as in Commodore 64) */
{
	extern u8 RAM[0x800];
	extern bool reset, nmi, nmi_edge_detected, intr;
	extern u16 PC;
	extern u8 A, X, Y, S;

	// Define StatusFlags type to match the union structure
	typedef union /* Status flags: */
	{
		u8 raw;
		RegBit<0> C; // carry
		RegBit<1> Z; // zero
		RegBit<2> I; // interrupt enable/disable
		RegBit<3> D; // decimal mode (unsupported on NES, but flag exists)
		// 4,5 (0x10,0x20) don't exist
		RegBit<6> V; // overflow
		RegBit<7> N; // negative
	} StatusFlags;

	extern StatusFlags P;

	template<bool write> u8 MemAccess(u16 addr, u8 v = 0);
	u8 RB(u16 addr);
	u8 WB(u16 addr, u8 v);
	void tick();
	u16 wrap(u16 oldaddr, u16 newaddr);
	void Misfire(u16 old, u16 addr);
	u8 Pop();
	void Push(u8 v);

	template<u16 op> void Ins();
	void Op();
}

namespace PPU /* Picture Processing Unit */
{
	typedef union regtype // PPU register file
	{
		u32 value;
		// Reg0 (write)             // Reg1 (write)             // Reg2 (read)
		RegBit<0, 8, u32> sysctrl;    RegBit< 8, 8, u32> dispctrl;  RegBit<16, 8, u32> status;
		RegBit<0, 2, u32> BaseNTA;    RegBit< 8, 1, u32> Grayscale; RegBit<21, 1, u32> SPoverflow;
		RegBit<2, 1, u32> Inc;        RegBit< 9, 1, u32> ShowBG8;   RegBit<22, 1, u32> SP0hit;
		RegBit<3, 1, u32> SPaddr;     RegBit<10, 1, u32> ShowSP8;   RegBit<23, 1, u32> InVBlank;
		RegBit<4, 1, u32> BGaddr;     RegBit<11, 1, u32> ShowBG;    // Reg3 (write)
		RegBit<5, 1, u32> SPsize;     RegBit<12, 1, u32> ShowSP;    RegBit<24, 8, u32> OAMaddr;
		RegBit<6, 1, u32> SlaveFlag;  RegBit<11, 2, u32> ShowBGSP;  RegBit<24, 2, u32> OAMdata;
		RegBit<7, 1, u32> NMIenabled; RegBit<13, 3, u32> EmpRGB;    RegBit<26, 6, u32> OAMindex;
	} regtype;

	extern regtype reg;

	// Raw memory data as read&written by the game
	extern u8 palette[32], OAM[256];

	// Sprite structure definition
	typedef struct {
		u8 sprindex, y, index, attr, x;
		u16 pattern;
	} sprite_info;

	// Decoded sprite information, used & changed during each scanline
	extern sprite_info OAM2[8], OAM3[8];

	typedef union scrolltype
	{
		RegBit<3, 16, u32> raw;       // raw VRAM address (16-bit)
		RegBit<0, 8, u32> xscroll;   // low 8 bits of first write to 2005
		RegBit<0, 3, u32> xfine;     // low 3 bits of first write to 2005
		RegBit<3, 5, u32> xcoarse;   // high 5 bits of first write to 2005
		RegBit<8, 5, u32> ycoarse;   // high 5 bits of second write to 2005
		RegBit<13, 2, u32> basenta;   // nametable index (copied from 2000)
		RegBit<13, 1, u32> basenta_h; // horizontal nametable index
		RegBit<14, 1, u32> basenta_v; // vertical   nametable index
		RegBit<15, 3, u32> yfine;     // low 3 bits of second write to 2005
		RegBit<11, 8, u32> vaddrhi;   // first write to 2006 (with high 2 bits set to zero)
		RegBit<3, 8, u32> vaddrlo;   // second write to 2006
	} scrolltype;

	extern scrolltype scroll, vaddr;

	extern unsigned pat_addr, sprinpos, sproutpos, sprrenpos, sprtmp;
	extern u16 tileattr, tilepat, ioaddr;
	extern u32 bg_shift_pat, bg_shift_attr;
	extern int scanline, x, scanline_end, VBlankState, cycle_counter;
	extern int read_buffer, open_bus, open_bus_decay_timer;
	extern bool even_odd_toggle, offset_toggle;

	u8& mmap(int i);
	u8 Access(u16 index, u8 v, bool write);
	void rendering_tick();
	void render_pixel();
	void tick();
}

namespace APU /* Audio Processing Unit */
{
	extern const u8 LengthCounters[32];
	extern const u16 NoisePeriods[16];
	extern const u16 DMCperiods[16];

	extern bool FiveCycleDivider, IRQdisable, ChannelsEnabled[5];
	extern bool PeriodicIRQ, DMC_IRQ;

	bool count(int& v, int reset);

	struct channel
	{
		int length_counter, linear_counter, address, envelope;
		int sweep_delay, env_delay, wave_counter, hold, phase, level;
		union // Per-channel register file
		{
			// 4000, 4004, 400C, 4012:            // 4001, 4005, 4013:            // 4002, 4006, 400A, 400E:
			RegBit<0, 8, u32> reg0;                 RegBit< 8, 8, u32> reg1;          RegBit<16, 8, u32> reg2;
			RegBit<6, 2, u32> DutyCycle;            RegBit< 8, 3, u32> SweepShift;    RegBit<16, 4, u32> NoiseFreq;
			RegBit<4, 1, u32> EnvDecayDisable;      RegBit<11, 1, u32> SweepDecrease; RegBit<23, 1, u32> NoiseType;
			RegBit<0, 4, u32> EnvDecayRate;         RegBit<12, 3, u32> SweepRate;     RegBit<16, 11, u32> WaveLength;
			RegBit<5, 1, u32> EnvDecayLoopEnable;   RegBit<15, 1, u32> SweepEnable;   // 4003, 4007, 400B, 400F, 4010:
			RegBit<0, 4, u32> FixedVolume;          RegBit< 8, 8, u32> PCMlength;     RegBit<24, 8, u32> reg3;
			RegBit<5, 1, u32> LengthCounterDisable;                                 RegBit<27, 5, u32> LengthCounterInit;
			RegBit<0, 7, u32> LinearCounterInit;                                    RegBit<30, 1, u32> LoopEnabled;
			RegBit<7, 1, u32> LinearCounterDisable;                                 RegBit<31, 1, u32> IRQenable;
		} reg;

		// Function for updating the wave generators and taking the sample for each channel.
		template<unsigned c>
		int tick()
		{
			channel& ch = *this;
			if (!ChannelsEnabled[c]) return c == 4 ? 64 : 8;
			int wl = (ch.reg.WaveLength + 1) * (c >= 2 ? 1 : 2);
			if (c == 3) wl = NoisePeriods[ch.reg.NoiseFreq];
			int volume = ch.length_counter ? ch.reg.EnvDecayDisable ? ch.reg.FixedVolume : ch.envelope : 0;
			// Sample may change at wavelen intervals.
			auto& S = ch.level;
			if (!count(ch.wave_counter, wl)) return S;
			switch (c)
			{
			default:// Square wave. With four different 8-step binary waveforms (32 bits of data total).
				if (wl < 8) return S = 8;
				return S = (0xF33C0C04u & (1u << (++ch.phase % 8 + ch.reg.DutyCycle * 8))) ? volume : 0;

			case 2: // Triangle wave
				if (ch.length_counter && ch.linear_counter && wl >= 3) ++ch.phase;
				return S = (ch.phase & 15) ^ ((ch.phase & 16) ? 15 : 0);

			case 3: // Noise: Linear feedback shift register
				if (!ch.hold) ch.hold = 1;
				ch.hold = (ch.hold >> 1)
					| (((ch.hold ^ (ch.hold >> (ch.reg.NoiseType ? 6 : 1))) & 1) << 14);
				return S = (ch.hold & 1) ? 0 : volume;

			case 4: // Delta modulation channel (DMC)
				// hold = 8 bit value, phase = number of bits buffered
				if (ch.phase == 0) // Nothing in sample buffer?
				{
					if (!ch.length_counter && ch.reg.LoopEnabled) // Loop?
					{
						ch.length_counter = ch.reg.PCMlength * 16 + 1;
						ch.address = (ch.reg.reg0 | 0x300) << 6;
					}
					if (ch.length_counter > 0) // Load next 8 bits if available
					{
						// Note: Re-entrant! But not recursive, because even
						// the shortest wave length is greater than the read time.
						// TODO: proper clock
						if (ch.reg.WaveLength > 20)
							for (unsigned t = 0; t < 3; ++t) CPU::RB(u16(ch.address) | 0x8000); // timing
						ch.hold = CPU::RB(u16(ch.address++) | 0x8000); // Fetch NESbyte
						ch.phase = 8;
						--ch.length_counter;
					}
					else // Otherwise, disable channel or issue IRQ
						ChannelsEnabled[4] = ch.reg.IRQenable && (CPU::intr = DMC_IRQ = true);
				}
				if (ch.phase != 0) // Update the signal if sample buffer nonempty
				{
					int v = ch.linear_counter;
					if (ch.hold & (0x80 >> --ch.phase)) v += 2; else v -= 2;
					if (v >= 0 && v <= 0x7F) ch.linear_counter = v;
				}
				return S = ch.linear_counter;
			}
		}
	};

	extern channel channels[5];

	struct CounterType { short lo, hi; };
	extern CounterType hz240counter;

	void Write(u8 index, u8 value);
	u8 Read();
	void tick();
}

// Global constants and variables
extern const char* inputfn;
extern const char* romFileName;

// Function prototypes for nesemu1.hh implementation
NESbyte readMemory(int address);
void writeMemory(int address, NESbyte value);
NESbyte readA();
NESbyte readX();
NESbyte readY();
void writeA(NESbyte value);
void writeX(NESbyte value);
void writeY(NESbyte value);

#endif // NESEMU1_HH