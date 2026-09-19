IOP_BIN = iidx_diag.irx
IOP_OBJS = iidx_diag.o imports.o

IOP_INCS += -I../include -I$(PS2SDK)/iop/include -I$(PS2SDK)/common/include

all: $(IOP_BIN)

clean:
	rm -f -r $(IOP_BIN) $(IOP_OBJS)

rebuild: clean all

include $(PS2SDK)/Defs.make
include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.iopglobal
