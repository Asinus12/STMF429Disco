# Name of the binaries.
PROJ_NAME=STM32F429disco

######################################################################
#                         SETUP SOURCES                              #
######################################################################


# base folder
STM_DIR=../STM32F429I-Discovery_FW_V1.0.1


# can.c, i2c.c, gpio.c, rtc.c ..
STM_SRC = $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src
STM_SRC += $(STM_DIR)/Utilities/STM32F429I-Discovery

# if cannot find a source in current folder use 
vpath %.c $(STM_SRC)

# My source file
SRCS   = main.c

# Initialize clock, apb, pll, vector table on reg leve, 600 lines
SRCS  += system_stm32f4xx.c

# make finds them by searching the vpath defined above.
SRCS  += stm32f4xx_rcc.c 
SRCS  += stm32f4xx_gpio.c
SRCS  += mojInclude.c
SRCS  += stm32f4xx_usart.c
SRCS  += stm32f4xx_exti.c
SRCS  += misc.c
SRCS  += stm32f4xx_syscfg.c
SRCS  += stm32f4xx_it.c
SRCS  += stm32f429i_discovery_l3gd20.c
SRCS  += stm32f4xx_spi.c

# Assembler startup file by ST
SRCS += $(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f429_439xx.s


# .h and .c files for specific development board functions (lgd320.c,lcd.h)
INC_DIRS  = $(STM_DIR)/Utilities/STM32F429I-Discovery
# core_cm3.h, arm_common_tables.h, arm_math.h..
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Include
# stm32f4xx.h: HSE VALUE, interrupt table,.. 
# system_stm32f4xx.h: declaration of SystemInit, SystemCoreClockUpdate functions
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include/
# # can.h, i2c.h, gpio.h, rtc.h ..
INC_DIRS += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc/
INC_DIRS += .

# in case we have to many sources and don't want 
# to compile all sources every time
# OBJS = $(SRCS:.c=.o)

######################################################################
#                         SETUP TOOLS                                #
######################################################################


# This is the path to the toolchain /gcc-arm-none-eabi
# We don't put our toolchain on $PATH if we want to keep the system clean
TOOLS_DIR = /usr/bin

CC      = $(TOOLS_DIR)/arm-none-eabi-gcc
OBJCOPY = $(TOOLS_DIR)/arm-none-eabi-objcopy
GDB     = $(TOOLS_DIR)/arm-none-eabi-gdb

## Preprocessor options

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

# #defines needed when working with the STM library
DEFS    = -DUSE_STDPERIPH_DRIVER
# if you use the following option, you must implement the function 
#    assert_failed(uint8_t* file, uint32_t line)
# because it is conditionally used in the library
# DEFS   += -DUSE_FULL_ASSERT

## Compiler options
CFLAGS  = -ggdb
# please do not optimize anything because we are debugging
CFLAGS += -O0 
CFLAGS += -Wall -Wextra -Warray-bounds
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

## Linker options
# tell ld which linker file to use
# (this file is in the current directory)
LFLAGS  = -TSTM32F429ZI_FLASH.ld --specs=nosys.specs


######################################################################
#                         SETUP TARGETS                              #
######################################################################

.PHONY: $(PROJ_NAME)
$(PROJ_NAME): $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(LFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf   $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F4
flash: 
	st-flash write $(PROJ_NAME).bin 0x8000000

.PHONY: debug
debug:
# before you start gdb, you must start st-util
	$(GDB) $(PROJ_NAME).elf
	

.PHONY: install
install: all


