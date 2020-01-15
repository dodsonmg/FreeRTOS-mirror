# For debugging
$(info $$PROG is [${PROG}])

CCPATH 		?=
XARCH?=32
USE_CLANG?=yes

ifeq ($(XARCH),64)
	ARCH 		= -march=rv64imacxcheri
	ABI 		= -mabi=l64pc128
	CLANG_ARCH  = riscv64-unknown-elf
else
	ARCH 		= -march=rv32imxcheri
	ABI 		= -mabi=il32pc64
	CLANG_ARCH  = riscv32-unknown-elf
endif


# Decide which compiler to use
ifeq ($(USE_CLANG),yes)
	CC		= clang --target=$(CLANG_ARCH) --sysroot=/no/such/path
	CPP		= clang++ --target=$(CLANG_ARCH) --sysroot=/no/such/path
	LD		= clang --target=$(CLANG_ARCH) --sysroot=/no/such/path
	OBJCOPY	= llvm-objcopy
	OBJDUMP	= llvm-objdump
	AR		= llvm-ar
	RANLIB	= llvm-ranlib
	COMPILER_FLAGS = -mcmodel=medium
else
	TARGET=$(CCPATH)riscv${XARCH}-unknown-elf
	CC		= $(TARGET)-gcc
	CPP		= $(TARGET)-g++
	LD		= $(CC)
	OBJCOPY	= $(TARGET)-objcopy
	OBJDUMP	= $(TARGET)-objdump
	AR		= $(TARGET)-ar
	RANLIB	= $(TARGET)-ranlib
	COMPILER_FLAGS = -mcmodel=medany
endif


$(info CC=${CC})

# Use main_blinky as demo source and target file name if not specified
PROG 	?= main_blinky
CRT0	= bsp/boot.S

FREERTOS_SOURCE_DIR	= ../../Source
FREERTOS_PLUS_SOURCE_DIR = ../../../FreeRTOS-Plus/Source
FREERTOS_TCP_SOURCE_DIR = $(FREERTOS_PLUS_SOURCE_DIR)/FreeRTOS-Plus-TCP
FREERTOS_PROTOCOLS_DIR = ./protocols

#WARNINGS = -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-align -Wsign-compare \
#		-Waggregate-return -Wmissing-declarations -Wunused -Werror 
 
C_WARNINGS = -Wbad-function-cast -Wmissing-prototypes -Wstrict-prototypes

CPP_WARNINGS = 

CPP_SRC = 

FREERTOS_SRC = \
	$(FREERTOS_SOURCE_DIR)/croutine.c \
	$(FREERTOS_SOURCE_DIR)/list.c \
	$(FREERTOS_SOURCE_DIR)/queue.c \
	$(FREERTOS_SOURCE_DIR)/tasks.c \
	$(FREERTOS_SOURCE_DIR)/timers.c \
	$(FREERTOS_SOURCE_DIR)/event_groups.c \
	$(FREERTOS_SOURCE_DIR)/stream_buffer.c \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang/heap_4.c

APP_SOURCE_DIR	= ../Common/Minimal

PORT_SRC = $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/port.c
PORT_ASM = $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/portASM.S

DEMO_SRC = main.c \
	demo/$(PROG).c

APP_SRC = \
	bsp/bsp.c \
	bsp/plic_driver.c \
	bsp/syscalls.c \
	bsp/uart.c \
	bsp/iic.c \
	bsp/gpio.c \
	bsp/spi.c \
	bsp/xilinx/uartns550/xuartns550.c \
	bsp/xilinx/uartns550/xuartns550_g.c \
	bsp/xilinx/uartns550/xuartns550_sinit.c \
	bsp/xilinx/uartns550/xuartns550_selftest.c \
	bsp/xilinx/uartns550/xuartns550_stats.c \
	bsp/xilinx/uartns550/xuartns550_options.c \
	bsp/xilinx/uartns550/xuartns550_intr.c \
	bsp/xilinx/uartns550/xuartns550_l.c \
	bsp/xilinx/axidma/xaxidma_bd.c \
	bsp/xilinx/axidma/xaxidma_bdring.c \
	bsp/xilinx/axidma/xaxidma.c \
	bsp/xilinx/axidma/xaxidma_selftest.c \
	bsp/xilinx/axidma/xaxidma_g.c \
	bsp/xilinx/axidma/xaxidma_sinit.c \
	bsp/xilinx/axiethernet/xaxiethernet.c \
	bsp/xilinx/axiethernet/xaxiethernet_control.c \
	bsp/xilinx/axiethernet/xaxiethernet_g.c \
	bsp/xilinx/axiethernet/xaxiethernet_sinit.c \
	bsp/xilinx/iic/xiic.c \
	bsp/xilinx/iic/xiic_g.c \
	bsp/xilinx/iic/xiic_l.c \
	bsp/xilinx/iic/xiic_sinit.c \
	bsp/xilinx/iic/xiic_selftest.c \
	bsp/xilinx/iic/xiic_master.c \
	bsp/xilinx/iic/xiic_intr.c \
	bsp/xilinx/iic/xiic_stats.c \
	bsp/xilinx/spi/xspi.c \
	bsp/xilinx/spi/xspi_g.c \
	bsp/xilinx/spi/xspi_sinit.c \
	bsp/xilinx/spi/xspi_selftest.c \
	bsp/xilinx/spi/xspi_options.c \
	bsp/xilinx/gpio/xgpio.c \
	bsp/xilinx/gpio/xgpio_extra.c \
	bsp/xilinx/gpio/xgpio_g.c \
	bsp/xilinx/gpio/xgpio_intr.c \
	bsp/xilinx/gpio/xgpio_selftest.c \
	bsp/xilinx/gpio/xgpio_sinit.c \
	bsp/xilinx/common/xbasic_types.c \
	bsp/xilinx/common/xil_io.c \
	bsp/xilinx/common/xil_assert.c

INCLUDES = \
	-I. \
	-I./bsp \
	-I./bsp/xilinx \
	-I./bsp/xilinx/common \
	-I./bsp/xilinx/axidma \
	-I./bsp/xilinx/axiethernet \
	-I./bsp/xilinx/uartns550 \
	-I./bsp/xilinx/iic \
	-I./bsp/xilinx/spi \
	-I./bsp/xilinx/gpio \
	-I$(FREERTOS_SOURCE_DIR)/include \
	-I../Common/include \
	-I$(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V \
	-I./demo \
	-I$(NEWLIB_DIR)/newlib/libc/include

ASFLAGS  += -g $(ARCH) $(ABI) -Wa,-Ilegacy \
	-I$(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions \
	-DportasmHANDLE_INTERRUPT=external_interrupt_handler

CFLAGS = $(WARNINGS) $(INCLUDES) -DAXI_DEBUG_PRINT=1 \
	-g -O0 $(ARCH) $(ABI) -mno-relax


FREERTOS_IP_SRC = \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_IP.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_ARP.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_DHCP.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_DNS.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_Sockets.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_TCP_IP.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_UDP_IP.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_TCP_WIN.c \
	$(FREERTOS_TCP_SOURCE_DIR)/FreeRTOS_Stream_Buffer.c \
	$(FREERTOS_TCP_SOURCE_DIR)/portable/BufferManagement/BufferAllocation_2.c \
	$(FREERTOS_TCP_SOURCE_DIR)/portable/NetworkInterface/RISC-V/riscv_hal_eth.c \
	$(FREERTOS_TCP_SOURCE_DIR)/portable/NetworkInterface/RISC-V/NetworkInterface.c \
	bsp/rand.c

FREERTOS_IP_INCLUDE = \
	-I$(FREERTOS_TCP_SOURCE_DIR) \
	-I$(FREERTOS_TCP_SOURCE_DIR)/include \
	-I$(FREERTOS_TCP_SOURCE_DIR)/portable/Compiler/GCC

FREERTOS_IP_DEMO_SRC = \
	demo/SimpleUDPClientAndServer.c \
	demo/TCPEchoClient_SingleTasks.c \
	demo/SimpleTCPEchoServer.c

ifeq ($(PROG),main_blinky)
	CFLAGS += -DmainDEMO_TYPE=1
else 
ifeq ($(PROG),main_full)
	CFLAGS += -DmainDEMO_TYPE=2
	PORT_ASM += demo/RegTest.S
	APP_SRC +=  \
		$(APP_SOURCE_DIR)/AbortDelay.c \
		$(APP_SOURCE_DIR)/BlockQ.c \
		$(APP_SOURCE_DIR)/blocktim.c \
		$(APP_SOURCE_DIR)/countsem.c \
		$(APP_SOURCE_DIR)/death.c \
		$(APP_SOURCE_DIR)/dynamic.c \
		$(APP_SOURCE_DIR)/integer.c \
		$(APP_SOURCE_DIR)/MessageBufferDemo.c \
		$(APP_SOURCE_DIR)/PollQ.c \
		$(APP_SOURCE_DIR)/GenQTest.c \
		$(APP_SOURCE_DIR)/QPeek.c \
		$(APP_SOURCE_DIR)/recmutex.c \
		$(APP_SOURCE_DIR)/TimerDemo.c \
		$(APP_SOURCE_DIR)/EventGroupsDemo.c \
		$(APP_SOURCE_DIR)/TaskNotify.c \
		$(APP_SOURCE_DIR)/StreamBufferDemo.c \
		$(APP_SOURCE_DIR)/StreamBufferInterrupt.c \
		$(APP_SOURCE_DIR)/semtest.c
else
ifeq ($(PROG),main_iic)
	CFLAGS += -DmainDEMO_TYPE=3
	INCLUDES += -I./devices
else
ifeq ($(PROG),main_gpio)
	CFLAGS += -DmainDEMO_TYPE=4
	INCLUDES += -I./demo
else
ifeq ($(PROG),main_tcp)
	CFLAGS += -DmainDEMO_TYPE=5
	CFLAGS += -DmainCREATE_TCP_ECHO_SERVER_TASK=1
	INCLUDES += $(FREERTOS_IP_INCLUDE)
	FREERTOS_SRC += $(FREERTOS_IP_SRC)
	DEMO_SRC += $(FREERTOS_IP_DEMO_SRC)

else
ifeq ($(PROG),main_peekpoke)
	CFLAGS += -DmainDEMO_TYPE=9
	CFLAGS += -DmainCREATE_PEEKPOKE_SERVER_TASK=1
	CFLAGS += -DmainCREATE_HTTP_SERVER=1
	CFLAGS += -DipconfigUSE_HTTP=1
	CFLAGS += '-DconfigHTTP_ROOT="/notused"'
	CFLAGS += -DffconfigMAX_FILENAME=4096
	INCLUDES += \
		$(FREERTOS_IP_INCLUDE) \
		-I$(FREERTOS_PROTOCOLS_DIR)/include
	FREERTOS_SRC += \
		$(FREERTOS_IP_SRC) \
		$(FREERTOS_PROTOCOLS_DIR)/Common/FreeRTOS_TCP_server.c \
		$(FREERTOS_PROTOCOLS_DIR)/HTTP/FreeRTOS_HTTP_server.c \
		$(FREERTOS_PROTOCOLS_DIR)/HTTP/FreeRTOS_HTTP_commands.c \
		$(FREERTOS_PROTOCOLS_DIR)/HTTP/peekpoke.c
	DEMO_SRC += $(FREERTOS_IP_DEMO_SRC)
else
ifeq ($(PROG),main_udp)
	CFLAGS += -DmainDEMO_TYPE=5
	CFLAGS += -DmainCREATE_SIMPLE_UDP_CLIENT_SERVER_TASKS=1
	INCLUDES += $(FREERTOS_IP_INCLUDE)
	FREERTOS_SRC += $(FREERTOS_IP_SRC)
	DEMO_SRC += $(FREERTOS_IP_DEMO_SRC)
else
ifeq ($(PROG),main_sd)
	CFLAGS += -DmainDEMO_TYPE=6
	CPPLAGS += -DmainDEMO_TYPE=6
	CPP_SRC += SD/src/SD.cpp \
			   SD/src/File.cpp \
			   SD/src/utility/Sd2Card.cpp \
			   SD/src/utility/SdFile.cpp \
			   SD/src/utility/SdVolume.cpp \
			   SD/src/SDLib.cpp
SD_EXAMPLE ?= CardInfo
USE_RTC_CLOCK ?= 0
$(info SD_EXAMPLE=${SD_EXAMPLE})
ifeq ($(SD_EXAMPLE),ReadWrite)
	CPP_SRC += SD/examples/SdDemo_ReadWrite.cpp
else
ifeq ($(SD_EXAMPLE),DumpFile)
	CPP_SRC += SD/examples/SdDemo_DumpFile.cpp
else
ifeq ($(SD_EXAMPLE),CardInfo)
	CPP_SRC += SD/examples/SdDemo_CardInfo.cpp
else
ifeq ($(SD_EXAMPLE),StressTest)
	DEMO_SRC += SD/examples/SdDemo_StressTest.c
else
$(error unknown SD_EXAMPLE: $(SD_EXAMPLE))
endif
endif
endif
endif

	INCLUDES += -I./SD/src
ifeq ($(USE_RTC_CLOCK),1)
# Below includes for RTC clock (SD FAT time)
	CFLAGS += -DUSE_RTC_CLOCK=1
	CPPLAGS += -DUSE_RTC_CLOCK=1
	INCLUDES += -I./devices
	DEMO_SRC += devices/ds1338rtc.c
endif # USE_RTC_CLOCK
else
ifeq ($(PROG),main_uart)
	CFLAGS += -DmainDEMO_TYPE=7
	INCLUDES += -I./devices
else
ifeq ($(PROG),main_rtc)
RTC_YEAR = $(shell date +%-y)
RTC_MONTH = $(shell date +%-m)
RTC_DAY = $(shell date +%-d)
RTC_HOUR = $(shell date +%-H)
RTC_MINUTE = $(shell date +%-M)
RTC_SECONDS = $(shell date +%-S)
	CFLAGS += -DRTC_YEAR=$(RTC_YEAR)
	CFLAGS += -DRTC_MONTH=$(RTC_MONTH)
	CFLAGS += -DRTC_DAY=$(RTC_DAY)
	CFLAGS += -DRTC_HOUR=$(RTC_HOUR)
	CFLAGS += -DRTC_MINUTE=$(RTC_MINUTE)
	CFLAGS += -DRTC_SECONDS=$(RTC_SECONDS)
	CFLAGS += -DRTC_SET_TIME=1
	CFLAGS += -DmainDEMO_TYPE=10
	INCLUDES += -I./devices
	DEMO_SRC += devices/ds1338rtc.c
else
ifeq ($(PROG),main_uart_malware)
	CFLAGS += -DmainDEMO_TYPE=11
else
ifeq ($(PROG),main_testgen)
	include $(INC_TESTGEN)/testgenEnvironment.mk
else
$(error unknown demo: $(PROG))
endif # main_testgen
endif # main_uart_malware
endif # main_rtc
endif # main_uart
endif # main_sd
endif # main_udp
endif # main_peekpoke
endif # main_tcp
endif # main_gpio
endif # main_iic
endif # main_full
endif # main_blinky

ARFLAGS=crsv

CFLAGS += -O0 -g3 $(ARCH) $(ABI) $(COMPILER_FLAGS) $(INCLUDES)
# Disable warnings for C++ for now
CPPFLAGS += -O0 -g3 $(ARCH) $(ABI) $(COMPILER_FLAGS) $(INCLUDES)

#
# Define all object files.
#
RTOS_OBJ = $(FREERTOS_SRC:.c=.o)
APP_OBJ  = $(APP_SRC:.c=.o)
PORT_OBJ = $(PORT_SRC:.c=.o)
DEMO_OBJ = $(DEMO_SRC:.c=.o)
CPP_OBJ = $(CPP_SRC:.cpp=.o)
PORT_ASM_OBJ = $(PORT_ASM:.S=.o)
CRT0_OBJ = $(CRT0:.S=.o)
OBJS = $(CRT0_OBJ) $(PORT_ASM_OBJ) $(PORT_OBJ) $(RTOS_OBJ) $(DEMO_OBJ) $(APP_OBJ) $(CPP_OBJ)

LDFLAGS = -fuse-ld=lld -mno-relax
LDFLAGS += -T link.ld -nostartfiles -nostdlib -L$(NEWLIB_DIR)/newlib/libc/build -L.
LIBS	 =  -lc -lcompiler_rt

$(info ASFLAGS=$(ASFLAGS))
$(info LDLIBS=$(LDLIBS))
$(info CFLAGS=$(CFLAGS))
$(info LDFLAGS=$(LDFLAGS))
$(info ARFLAGS=$(ARFLAGS))

%.o: %.c
	@echo "    CC $<"
	@$(CC) -c $(CFLAGS) -o $@ $<

%.o: %.cpp
	@echo "    C++ $<"
	@$(CPP) -c $(CPPFLAGS) -o $@ $<

%.o: %.S
	@echo "    CC $<"
	@$(CC) $(ASFLAGS) -c $(CFLAGS) -o $@ $<

all: $(PROG).elf

$(PROG).elf  : $(OBJS) libcompiler_rt.a Makefile
	@echo Linking....
	@$(CC) -v -o $@ $(LDFLAGS) $(OBJS) $(LIBS)
	@$(OBJDUMP) -disassemble -h -t -p -r -source $(PROG).elf > $(PROG).dump
	@echo Completed $@

clean :
	@rm -f $(OBJS)
	@rm -f libcompiler_rt.a
	@rm -f $(PROG).elf
	@rm -f $(PROG).map
	@rm -f $(PROG).dump

docs :
	@doxygen

# Compiler-RT building

COMPILER_RT_SRC = \
	compiler-rt/lib/builtins/absvdi2.c \
	compiler-rt/lib/builtins/absvsi2.c \
	compiler-rt/lib/builtins/absvti2.c \
	compiler-rt/lib/builtins/adddf3.c \
	compiler-rt/lib/builtins/addsf3.c \
	compiler-rt/lib/builtins/addtf3.c \
	compiler-rt/lib/builtins/addvdi3.c \
	compiler-rt/lib/builtins/addvsi3.c \
	compiler-rt/lib/builtins/addvti3.c \
	compiler-rt/lib/builtins/apple_versioning.c \
	compiler-rt/lib/builtins/ashldi3.c \
	compiler-rt/lib/builtins/ashlti3.c \
	compiler-rt/lib/builtins/ashrdi3.c \
	compiler-rt/lib/builtins/ashrti3.c \
	compiler-rt/lib/builtins/bswapdi2.c \
	compiler-rt/lib/builtins/bswapsi2.c \
	compiler-rt/lib/builtins/clzdi2.c \
	compiler-rt/lib/builtins/clzsi2.c \
	compiler-rt/lib/builtins/clzti2.c \
	compiler-rt/lib/builtins/cmpdi2.c \
	compiler-rt/lib/builtins/cmpti2.c \
	compiler-rt/lib/builtins/comparedf2.c \
	compiler-rt/lib/builtins/comparesf2.c \
	compiler-rt/lib/builtins/ctzdi2.c \
	compiler-rt/lib/builtins/ctzsi2.c \
	compiler-rt/lib/builtins/ctzti2.c \
	compiler-rt/lib/builtins/divdc3.c \
	compiler-rt/lib/builtins/divdf3.c \
	compiler-rt/lib/builtins/divdi3.c \
	compiler-rt/lib/builtins/divmoddi4.c \
	compiler-rt/lib/builtins/divmodsi4.c \
	compiler-rt/lib/builtins/divsc3.c \
	compiler-rt/lib/builtins/divsf3.c \
	compiler-rt/lib/builtins/divsi3.c \
	compiler-rt/lib/builtins/divtc3.c \
	compiler-rt/lib/builtins/divti3.c \
	compiler-rt/lib/builtins/divtf3.c \
	compiler-rt/lib/builtins/extendsfdf2.c \
	compiler-rt/lib/builtins/extendhfsf2.c \
	compiler-rt/lib/builtins/ffsdi2.c \
	compiler-rt/lib/builtins/ffssi2.c \
	compiler-rt/lib/builtins/ffsti2.c \
	compiler-rt/lib/builtins/fixdfdi.c \
	compiler-rt/lib/builtins/fixdfsi.c \
	compiler-rt/lib/builtins/fixdfti.c \
	compiler-rt/lib/builtins/fixsfdi.c \
	compiler-rt/lib/builtins/fixsfsi.c \
	compiler-rt/lib/builtins/fixsfti.c \
	compiler-rt/lib/builtins/fixunsdfdi.c \
	compiler-rt/lib/builtins/fixunsdfsi.c \
	compiler-rt/lib/builtins/fixunsdfti.c \
	compiler-rt/lib/builtins/fixunssfdi.c \
	compiler-rt/lib/builtins/fixunssfsi.c \
	compiler-rt/lib/builtins/fixunssfti.c \
	compiler-rt/lib/builtins/floatdidf.c \
	compiler-rt/lib/builtins/floatdisf.c \
	compiler-rt/lib/builtins/floatsidf.c \
	compiler-rt/lib/builtins/floatsisf.c \
	compiler-rt/lib/builtins/floattidf.c \
	compiler-rt/lib/builtins/floattisf.c \
	compiler-rt/lib/builtins/floatundidf.c \
	compiler-rt/lib/builtins/floatundisf.c \
	compiler-rt/lib/builtins/floatunsidf.c \
	compiler-rt/lib/builtins/floatunsisf.c \
	compiler-rt/lib/builtins/floatuntidf.c \
	compiler-rt/lib/builtins/floatuntisf.c \
	compiler-rt/lib/builtins/int_util.c \
	compiler-rt/lib/builtins/lshrdi3.c \
	compiler-rt/lib/builtins/lshrti3.c \
	compiler-rt/lib/builtins/moddi3.c \
	compiler-rt/lib/builtins/modsi3.c \
	compiler-rt/lib/builtins/modti3.c \
	compiler-rt/lib/builtins/muldc3.c \
	compiler-rt/lib/builtins/muldf3.c \
	compiler-rt/lib/builtins/muldi3.c \
	compiler-rt/lib/builtins/mulodi4.c \
	compiler-rt/lib/builtins/mulosi4.c \
	compiler-rt/lib/builtins/muloti4.c \
	compiler-rt/lib/builtins/mulsc3.c \
	compiler-rt/lib/builtins/mulsf3.c \
	compiler-rt/lib/builtins/multi3.c \
	compiler-rt/lib/builtins/multf3.c \
	compiler-rt/lib/builtins/mulvdi3.c \
	compiler-rt/lib/builtins/mulvsi3.c \
	compiler-rt/lib/builtins/mulvti3.c \
	compiler-rt/lib/builtins/negdf2.c \
	compiler-rt/lib/builtins/negdi2.c \
	compiler-rt/lib/builtins/negsf2.c \
	compiler-rt/lib/builtins/negti2.c \
	compiler-rt/lib/builtins/negvdi2.c \
	compiler-rt/lib/builtins/negvsi2.c \
	compiler-rt/lib/builtins/negvti2.c \
	compiler-rt/lib/builtins/os_version_check.c \
	compiler-rt/lib/builtins/paritydi2.c \
	compiler-rt/lib/builtins/paritysi2.c \
	compiler-rt/lib/builtins/parityti2.c \
	compiler-rt/lib/builtins/popcountdi2.c \
	compiler-rt/lib/builtins/popcountsi2.c \
	compiler-rt/lib/builtins/popcountti2.c \
	compiler-rt/lib/builtins/powidf2.c \
	compiler-rt/lib/builtins/powisf2.c \
	compiler-rt/lib/builtins/powitf2.c \
	compiler-rt/lib/builtins/subdf3.c \
	compiler-rt/lib/builtins/subsf3.c \
	compiler-rt/lib/builtins/subvdi3.c \
	compiler-rt/lib/builtins/subvsi3.c \
	compiler-rt/lib/builtins/subvti3.c \
	compiler-rt/lib/builtins/subtf3.c \
	compiler-rt/lib/builtins/trampoline_setup.c \
	compiler-rt/lib/builtins/truncdfhf2.c \
	compiler-rt/lib/builtins/truncdfsf2.c \
	compiler-rt/lib/builtins/truncsfhf2.c \
	compiler-rt/lib/builtins/ucmpdi2.c \
	compiler-rt/lib/builtins/ucmpti2.c \
	compiler-rt/lib/builtins/udivdi3.c \
	compiler-rt/lib/builtins/udivmoddi4.c \
	compiler-rt/lib/builtins/udivmodsi4.c \
	compiler-rt/lib/builtins/udivmodti4.c \
	compiler-rt/lib/builtins/udivsi3.c \
	compiler-rt/lib/builtins/udivti3.c \
	compiler-rt/lib/builtins/umoddi3.c \
	compiler-rt/lib/builtins/umodsi3.c \
	compiler-rt/lib/builtins/umodti3.c \
	compiler-rt/lib/builtins/comparetf2.c \
	compiler-rt/lib/builtins/extenddftf2.c \
	compiler-rt/lib/builtins/extendsftf2.c \
	compiler-rt/lib/builtins/fixtfdi.c \
	compiler-rt/lib/builtins/fixtfsi.c \
	compiler-rt/lib/builtins/fixtfti.c \
	compiler-rt/lib/builtins/fixunstfdi.c \
	compiler-rt/lib/builtins/fixunstfsi.c \
	compiler-rt/lib/builtins/fixunstfti.c \
	compiler-rt/lib/builtins/floatditf.c \
	compiler-rt/lib/builtins/floatsitf.c \
	compiler-rt/lib/builtins/floattitf.c \
	compiler-rt/lib/builtins/floatunditf.c \
	compiler-rt/lib/builtins/floatunsitf.c \
	compiler-rt/lib/builtins/floatuntitf.c \
	compiler-rt/lib/builtins/multc3.c \
	compiler-rt/lib/builtins/trunctfdf2.c \
	compiler-rt/lib/builtins/trunctfsf2.c

COMPILER_RT_ASM = compiler-rt/lib/builtins/riscv/mulsi3.S
COMPILER_RT_OBJ = $(COMPILER_RT_SRC:.c=.o) $(COMPILER_RT_ASM:.S=.o)

libcompiler_rt.a: $(COMPILER_RT_OBJ)
	@echo Archiving....
	@$(AR) $(ARFLAGS) $@ $(COMPILER_RT_OBJ)
	@echo Completed $@
