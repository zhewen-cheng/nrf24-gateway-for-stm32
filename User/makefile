INCFLAG =\
-I. \
-Iinc \
-I../System/STM32/inc \
-I../System/CMSIS/inc \
-I../System/NRF24/inc \
-I../System/ETH/inc \
-I../CoOS/inc \
-I../NET/inc/ipv4/lwip \
-I../NET/inc/lwip \
-I../NET/inc/netif \
-I../NET/inc/arch	\
-I../NET/inc/ipv4 \
-I../NET/inc/ipv6 \
-I../NET/inc/posix/sys \
-I../NET/inc/posix \
-I../NET/inc \
-I../NET/inc/port	\
-I../NET/inc/app



CFLAGS  += $(INCFLAG)

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./src/*.c)) \
Startup/startup.o

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

all: $(OBJS)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $<\
        && echo $(NOW) INFO Objecting User/$< success. >> ../build.log \
        || echo $(NOW) ERROR Objecting User/$< failed, stop building. >> ../build.log | exit 1
	
Startup/startup.o: ./Startup/startup.s | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $< \
        && echo $(NOW) INFO Objecting User/$< success. >> ../build.log \
        || echo $(NOW) ERROR Objecting User/$< failed, stop building. >> ../build.log | exit 1

$(OBJDIR):
	@echo $(NOW) INFO Make new folder User/$(OBJDIR). >> ../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o 
	-rm -rf $(OBJDIR)/*.lst
	-rm -rf $(OBJDIR)/*.d
	-rm -rf Startup/*.o
