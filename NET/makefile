INCFLAG =\
-I. \
-Iinc \
-Iinc/ipv4 \
-Iinc/ipv6 \
-Iinc/ipv4/lwip \
-Iinc/ipv6/lwip \
-Iinc/lwip \
-Iinc/netif \
-Iinc/arch \
-Iinc/posix/sys \
-Iinc/posix \
-I../CoOS/inc \
-I../System/CMSIS/inc \
-Iinc/port \
-I../System/ETH/inc \
-I../System/STM32/inc \
-Iinc/app \


CFLAGS += $(INCFLAG)

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./src/*/*.c)) \
$(patsubst %.c, %.o, $(wildcard ./src/*/*/*.c)) 

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

SUBOBJDIR=\
$(OBJDIR)/api \
$(OBJDIR)/core \
$(OBJDIR)/core/ipv4 \
$(OBJDIR)/core/ipv6 \
$(OBJDIR)/core/snmp \
$(OBJDIR)/netif \
$(OBJDIR)/netif/ppp \
$(OBJDIR)/arch \
$(OBJDIR)/port \
$(OBJDIR)/app \


all: $(SUBOBJDIR) $(OBJS)	 

.SECONDEXPANSION:
$(OBJS): %: $$(firstword $$(subst .o,.c, $$(subst obj/, src/, %))) | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $< \
	    && echo $(NOW) INFO Objecting NET/$< success. >> ../build.log \
	    || echo $(NOW) ERROR Objecting NET/$< failed, stop building. >> ../build.log | exit 1
 
$(SUBOBJDIR): $(OBJDIR)
	@echo $(NOW) INFO Make new sub folders NET/$(OBJDIR). >> ../build.log
	mkdir -p $(SUBOBJDIR)

$(OBJDIR):
	@echo $(NOW) INFO Make new folder NET/$(OBJDIR). >> ../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*/*.o
	-rm -rf $(OBJDIR)/*/*.d
	-rm -rf $(OBJDIR)/*/*/*.o
	-rm -rf $(OBJDIR)/*/*/*.d 