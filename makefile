TCPREFIX := arm-none-eabi-
CC       := $(TCPREFIX)gcc
LD       := $(TCPREFIX)g++
CP       := $(TCPREFIX)objcopy
OD       := $(TCPREFIX)objdump
GD  	 := $(TCPREFIX)gdb

CFLAGS 	:= -c -Wall -fno-common -O0 -g -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nosys.specs
LFLAGS  = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -T$(LDFILE) --specs=nosys.specs
CPFLAGS = -Obinary
ODFLAGS = -S

OBJDIR 	:= obj

BLACK 	:= "\033[30;1m"
RED  	:= "\033[31;1m"
GREEN 	:= "\033[32;1m"
YELLOW 	:= "\033[33;1m"
BLUE  	:= "\033[34;1m"
PURPLE 	:= "\033[35;1m"
CYAN  	:= "\033[36;1m"
RST		:= "\033[0m"

NOW := $(shell date +"%Y-%m-%d %H:%M:%S")

export

LDFILE = ./User/Startup/STM32F429ZI_FLASH.ld

OCDCFG 		= ./openocd.cfg
TAIL		= multitail
LOGFILE		= ./build.log

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./User/src/*.c)) \
$(patsubst %.c, %.o, $(wildcard ./System/*/src/*.c)) \
$(patsubst %.c, %.o, $(wildcard ./CoOS/src/*.c)) \
$(patsubst %.c, %.o, $(wildcard ./NET/src/*/*.c)) \
$(patsubst %.c, %.o, $(wildcard ./NET/src/*/*/*.c)) \
./User/Startup/startup.o

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

all: main.bin
	@echo $(YELLOW)"Flash $< into board..."$(RST)
	openocd -f $(OCDCFG)  				\
			-c "init"                   \
            -c "reset init"             \
            -c "poll"             \
            -c "reset halt"             \
            -c "stm32f4x unlock 0"      \
            -c "flash probe 0"          \
            -c "flash info 0"           \
            -c "stm32f4x mass_erase 0"           \
            -c "flash write_image erase $< 0x8000000" \
            -c "reset run" -c shutdown
	@echo $(GREEN)"Finish flash $< into board."$(RST)
	@echo ""

obj:
	$(MAKE) all -C System
	@echo $(YELLOW)"Build System files..."$(RST)
	$(MAKE) all -C User
	@echo $(GREEN)"Finish building user files."$(RST)
	$(MAKE) all -C CoOS
	@echo $(GREEN)"Finish building CoOS files."$(RST)
	$(MAKE) all -C NET
	@echo $(GREEN)"Finish building NET files."$(RST)
	@echo ""

main.bin: obj main.elf
	@echo $(YELLOW)"Copy file main.elf..."$(RST)
	$(CP) $(CPFLAGS) main.elf $@ \
		&& echo $(NOW) INFO Copying main.elf success. >> build.log \
        || echo $(NOW) ERROR Copying main.elf failed, stop building. >> build.log | exit 1
	$(OD) $(ODFLAGS) main.elf > main.lst
	@echo $(GREEN)"Finish copy file main.elf."$(RST)
	@echo ""

main.elf: $(OBJS) $(LDFILE)
	@echo "link file: $@"
	$(LD) $(LFLAGS) -o $@ $(OBJS) \
        && echo $(NOW) INFO Linking $@ success. >> build.log \
        || echo $(NOW) ERROR Linking $@ failed, stop building. >> build.log | exit 1

log:
	$(TAIL) -cS build_log $(LOGFILE)

debug:
	$(GD) -ex "target remote :3333" \
	-ex "c" main.elf
	

clean:
	$(MAKE) clean -e -C System
	$(MAKE) clean -e -C User
	$(MAKE) clean -e -C CoOS
	$(MAKE) clean -e -C NET
	-rm -rf main.bin main.elf main.lst
	@echo $(NOW) INFO Clean everythings up. >> build.log
ifdef clean_dir
	-rm -rf */obj */*/obj
	@echo $(NOW) INFO Clean every dir:$(OBJDIR) up. >> build.log
endif
ifdef clean_log
	@cat /dev/null > build.log
	@echo $(NOW) INFO Clean up building log. >> build.log
endif

.PHONY: all clean debug log obj
