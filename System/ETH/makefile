INCFLAG =\
-I. \
-Iinc \
-I../STM32/inc \
-I../CMSIS/inc \
-I../../CoOS/inc \
-I../../NET/inc/arch \
-I../../NET/inc/ipv4 \
-I../../NET/inc/ipv6 \
-I../../NET/inc/lwip \
-I../../NET/inc/netif \
-I../../NET/inc/port \
-I../../NET/inc \
-I../../NET/inc/appmak


CFLAGS  += $(INCFLAG)

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./src/*.c)) 

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

all: $(OBJS)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $<\
        && echo $(NOW) INFO Objecting System/ETH/$< success. >> ../../build.log \
        || echo $(NOW) ERROR Objecting System/ETH/$< failed, stop building. >> ../../build.log | exit 1

$(OBJDIR):
	@echo $(NOW) INFO Make new folder System/ETH/$(OBJDIR). >> ../../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o 
	-rm -rf $(OBJDIR)/*.d
