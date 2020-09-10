INCFLAG =\
-I. \
-Iinc \
-I../STM32/inc \
-I../CMSIS/inc 

CFLAGS  += $(INCFLAG)

OBJS =\
$(OBJDIR)/nrf24l01.o \

all: $(OBJS)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $<\
        && echo $(NOW) INFO Objecting User/$< success. >> ../build.log \
        || echo $(NOW) ERROR Objecting User/$< failed, stop building. >> ../build.log | exit 

$(OBJDIR):
	@echo $(NOW) INFO Make new folder User/$(OBJDIR). >> ../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o 
	-rm -rf $(OBJDIR)/*.d
