INCFLAG =\
-I. \
-Iinc

CFLAGS += $(INCFLAG)

SRCOBJS =\
$(patsubst %.c, %.o, $(wildcard ./src/*.c)) 

OBJS = $(subst /src/,/obj/, $(SRCOBJS))

all: $(OBJS)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	@echo "bulid file: $<"
	$(CC) $(CFLAGS) -MMD -MF$(@:%.o=%.d) -o $@ $< \
        && echo $(NOW) INFO Objecting CoOS/$< success. >> ../build.log \
        || echo $(NOW) ERROR Objecting CoOS/$< failed, stop building. >> ../build.log | exit 1

$(OBJDIR):
	@echo $(NOW) INFO Make new folder CoOS/$(OBJDIR). >> ../build.log
	mkdir -p $(OBJDIR)

clean:
	-rm -rf $(OBJDIR)/*.o
	-rm -rf $(OBJDIR)/*.d
