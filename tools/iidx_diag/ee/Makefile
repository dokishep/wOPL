EE_BIN = iidx_diag.elf
EE_OBJS = iidx_diag.o usbd_mini_irx.o iidx_diag_irx.o

EE_INCS += -I../include -I$(PS2SDK)/ee/include -I$(PS2SDK)/common/include
EE_LIBS = -ldebug -lpad -lc -lkernel -lpatches
EE_CFLAGS = -Wall -O2
EE_LDFLAGS = -s

BIN2C = $(PS2SDK)/bin/bin2c

all: $(EE_BIN)

clean:
	rm -f -r $(EE_OBJS) $(EE_BIN) usbd_mini_irx.c iidx_diag_irx.c

rebuild: clean all

usbd_mini_irx.c: ../../../modules/usb-vendor/usbd_mini.irx
	$(BIN2C) $< $@ usbd_mini_irx

iidx_diag_irx.c: ../iop/iidx_diag.irx
	$(BIN2C) $< $@ iidx_diag_irx

include $(PS2SDK)/Defs.make
include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal
