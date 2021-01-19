INCFLAG =\
-I. \
-Iinc \
-I../STM32/inc \
-I../ETH/inc

CFLAGS += $(INCFLAG)

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./src/*.c)) 

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

all: $(OBJS)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $< \
        && echo $(NOW) INFO Objecting System/CMSIS/$< success. >> ../../build.log \
        || echo $(NOW) ERROR Objecting System/CMSIS/$< failed, stop building. >> ../../build.log | exit 1

$(OBJDIR):
	echo $(NOW) INFO Make new folder System/CMSIS/$(OBJDIR). >> ../../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o
	-rm -rf $(OBJDIR)/*.d
	