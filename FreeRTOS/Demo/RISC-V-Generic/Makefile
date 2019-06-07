RISCV_XLEN ?= 64
RISCV_LIB  ?= elf
CCPATH =

BSP ?= spike-rv32imac-ilp32
BSP_CONFIGS = $(word $2,$(subst -, ,$(BSP)))

PLATFORM ?=$(call BSP_CONFIGS,$*,1)
ARCH ?=$(call BSP_CONFIGS,$*,2)
ABI ?=$(call BSP_CONFIGS,$*,3)

TARGET=$(CCPATH)riscv${RISCV_XLEN}-unknown-${RISCV_LIB}

GCC		= $(TARGET)-gcc
OBJCOPY	= $(TARGET)-objcopy
OBJDUMP	= $(TARGET)-objdump
AR		= $(TARGET)-ar
RANLIB	= $(TARGET)-ranlib

# Use main_blinky as demo source and target file name if not specified
PROG 	?= main_blinky
PLATFORM ?= spike
CRT0	= bsp/boot.S

# For debugging
$(info $$PROG is [${PROG}])

FREERTOS_SOURCE_DIR	= ../../Source
FREERTOS_PLUS_SOURCE_DIR = ../../../FreeRTOS-Plus/Source
FREERTOS_TCP_SOURCE_DIR = $(FREERTOS_PLUS_SOURCE_DIR)/FreeRTOS-Plus-TCP

WARNINGS= -Wall -Wextra -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align -Wsign-compare \
		-Waggregate-return -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations -Wunused


# Root of RISC-V tools installation
RISCV ?= /opt/riscv

FREERTOS_SRC = \
	$(FREERTOS_SOURCE_DIR)/croutine.c \
	$(FREERTOS_SOURCE_DIR)/list.c \
	$(FREERTOS_SOURCE_DIR)/queue.c \
	$(FREERTOS_SOURCE_DIR)/tasks.c \
	$(FREERTOS_SOURCE_DIR)/timers.c \
	$(FREERTOS_SOURCE_DIR)/event_groups.c \
	$(FREERTOS_SOURCE_DIR)/stream_buffer.c \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang/heap_4.c # TODO TEST

APP_SOURCE_DIR	= ../Common/Minimal

PORT_SRC = $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/port.c
PORT_ASM = $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/portASM.S

INCLUDES = \
	-I. \
	-I./bsp \
	-I$(FREERTOS_SOURCE_DIR)/include \
	-I../Common/include \
	-I$(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V

ASFLAGS  += -g -march=$(ARCH) -mabi=$(ABI)  -Wa,-Ilegacy -I$(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions -DportasmHANDLE_INTERRUPT=external_interrupt_handler

CFLAGS = $(WARNINGS) $(INCLUDES)
CFLAGS += -O0 -g3 -march=$(ARCH) -mabi=$(ABI) -mcmodel=medany

DEMO_SRC = main.c \
	demo/$(PROG).c

APP_SRC = \
	bsp/bsp.c \
	bsp/plic_driver.c \
	bsp/syscalls.c \
  bsp/uart16550.c \
  bsp/htif.c

INCLUDES = \
	-I. \
	-I./bsp \
	-I$(FREERTOS_SOURCE_DIR)/include \
	-I../Common/include \
	-I$(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V

ifeq ($(PROG),main_blinky)
	CFLAGS += -DmainDEMO_TYPE=1
else
	$(info unknown demo: $(PROG))
endif

# PLATFORM Variants
ifeq ($(PLATFORM),spike)
	CFLAGS += -DPLATFORM_SPIKE=1
else
ifeq ($(PLATFORM),piccolo)
	CFLAGS += -DPLATFORM_PICCOLO=1
else
ifeq ($(PLATFORM),sail)
	CFLAGS += -DPLATFORM_SAIL=1
else
ifeq ($(PLATFORM),qemu_virt)
	CFLAGS += -DPLATFORM_QEMU_VIRT=1
else
	$(info unknown platform: $(PLATFORM))
endif
endif
endif
endif

ARFLAGS=crsv

#
# Define all object files.
#
RTOS_OBJ = $(FREERTOS_SRC:.c=.o)
APP_OBJ  = $(APP_SRC:.c=.o)
PORT_OBJ = $(PORT_SRC:.c=.o)
DEMO_OBJ = $(DEMO_SRC:.c=.o)
PORT_ASM_OBJ = $(PORT_ASM:.S=.o)
CRT0_OBJ = $(CRT0:.S=.o)
OBJS = $(CRT0_OBJ) $(PORT_ASM_OBJ) $(PORT_OBJ) $(RTOS_OBJ) $(DEMO_OBJ) $(APP_OBJ)

LDFLAGS	 = -T link.ld -nostartfiles -nostdlib -defsym=_STACK_SIZE=4K -march=$(ARCH) -mabi=$(ABI)
LIBS	 =  -lc -lgcc

$(info ASFLAGS=$(ASFLAGS))
$(info LDLIBS=$(LDLIBS))
$(info CFLAGS=$(CFLAGS))
$(info LDFLAGS=$(LDFLAGS))
$(info ARFLAGS=$(ARFLAGS))

%.o: %.c
	@echo "    CC $<"
	@$(GCC) -c $(CFLAGS) -o $@ $<

%.o: %.S
	@echo "    CC $<"
	@$(GCC) $(ASFLAGS) -c $(CFLAGS) -o $@ $<

all: $(PROG).elf

$(PROG).elf  : $(OBJS) Makefile
	@echo Building FreeRTOS/RISC-V for PLATFORM=$(PLATFORM) ARCH=$(ARCH) ABI=$(ABI)
	@echo Linking....
	@$(GCC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS)
	@$(OBJDUMP) -S $(PROG).elf > $(PROG).asm
	@echo Completed $@

clean :
	@rm -f $(OBJS)
	@rm -f $(PROG).elf
	@rm -f $(PROG).map
	@rm -f $(PROG).asm
