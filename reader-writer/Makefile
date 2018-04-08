BINARY		= main
OBJS		+= $(BINARY).o

OPENCM3_DIR := libopencm3
CPPFLAGS	+= -DSTM32F3
CPPFLAGS	+= -I$(OPENCM3_DIR)/include
LDSCRIPT	= stm32f303-nucleo32.ld
LDLIBS		+= -lopencm3_stm32f3
FP_FLAGS	?= -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCH_FLAGS	= -mthumb -mcpu=cortex-m4 $(FP_FLAGS)

CPPFLAGS	+= -MD

CFLAGS		?= -O3
CFLAGS		+= $(ARCH_FLAGS)
CFLAGS		+= -std=c11 -Wall -Wextra
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

CFLAGS		?= $(CFLAGS)
CXXFLAGS	+= $(ARCH_FLAGS)
CXXFLAGS	+= -Wall -Wextra -Wshadow -Wredundant-decls  -Weffc++
CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

LDFLAGS		+= $(ARCH_FLAGS)
LDFLAGS		+= -L$(OPENCM3_DIR)/lib
LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map -Wl,--cref
LDFLAGS		+= -Wl,--gc-sections

LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

.SUFFIXES: .elf .bin .map

all: $(BINARY).bin $(BINARY).elf

TOOLCHAIN	?= arm-none-eabi
%.o: %.c
	$(TOOLCHAIN)-gcc $(CFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).c

%.o: %.cpp
	$(TOOLCHAIN)-g++ $(CXXFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).cpp

%.elf %.map: $(OBJS) $(LDSCRIPT)
	$(TOOLCHAIN)-gcc $(LDFLAGS) $(OBJS) $(LDLIBS) -o $(*).elf

%.bin: %.elf
	$(TOOLCHAIN)-objcopy -Obinary $(*).elf $(*).bin

STFLASH		= $(shell which st-flash)
flash: $(BINARY).bin
	$(STFLASH) write $< 0x8000000

.PHONY: clean
clean:
	$(RM) *.elf *.bin *.map *.d ${OBJS} ${OBJS:%.o:%.d}

print-%:
	@echo $*=$($*)

-include $(OBJS:.o=.d)
