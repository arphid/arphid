#include <avr/io.h>
#include <util/delay.h>

// half of the low byte of the jump address to phaseA or phaseB
#define A 0x10
#define B 0x40
static unsigned char bigdata[] =  {
	// The first bit is always A, and is shown here (as a comment) for
	// convenience but is not acutally ouput from this array.
	// 30 bits: synchronization sequence.
	/*A,*/ A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A,
	A, A, A, A, A,
	// 22 bits: const
	B, A, A, A, A, A, B, B, B, A, A, B, B, A, A, A, A, B, A, B, B, A,

	// 33 bits: user key (the only part where cards differ)
	// BEGIN CODE
	A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A, A,
	A, A, A, A, A, A, A, A,
	// END CODE

	// 139 bits: const
	A, B, A, B, B, A, A, A, B, A, B, A, B, B, A, B, A, B, A, A, A, A, A, B, B,
	A, A, B, A, B, B, B, A, A, A, B, A, B, B, A, B, B, B, A, A, B, B, A, A, A,
	B, B, A, B, A, B, A, B, A, B, B, B, A, B, A, B, B, B, B, A, A, B, A, A, B,
	A, A, A, B, A, A, A, B, A, A, B, B, B, A, B, B, B, B, A, B, B, A, A, B, A,
	B, B, B, A, A, B, B, B, A, B, B, A, A, B, A, A, A, B, A, B, A, A, B, B, A,
	A, B, B, B, B, B, A, A, A, B, B, B, B, B,
	0}; // rewind to the beginning of sequence, output A (the one that is commented out)
#undef A
#undef B
 
int main(void) {
	unsigned char temp1=0, temp2=0x18;
	unsigned char* data = bigdata;
	asm volatile(""
		".balign 0x100,,\n\t"
		"rewind:\n\t"
			"; Y = {r29, r28} will be a pointer into the bigdata buffer\n\t"
			"movw r28, %0\n\t"             // 1 cycle
			"; Z = {r31, r30} will be used to jump to phaseA or phaseB based on bigdata[i]\n\t"
			"ldi r31, hi8(pm(phaseA))\n\t" // 1 cycle
			"; ldi r30, lo8(pm(phaseA)) ; will be done by phaseSel\n\t"
			"rjmp phaseA+8\n\t"            // 2 cycles
		"\n"
		".balign 0x20,,\n\t"
		"phaseA:\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n"
		"phaseAsel:\n\t"
			"ld  r30, Y+\n\t"
			"ijmp\n\t"
		"\n"
		".balign 0x80,,\n\t"
		"phaseB:\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n\t"
			"out	0x17, %2\n\t"
			"out	0x17, %1\n"
		"phaseBsel:\n\t"
			"ld  r30, Y+\n\t"
			"ijmp\n\t"
		: 
		: "e" (data), "r" (temp1), "r" (temp2)
		: "r28", "r29", "r30", "r31"
	);
	return 0;
}
