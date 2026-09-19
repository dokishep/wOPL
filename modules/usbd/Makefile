IOP_BIN = usbd.irx
IOP_OBJS = usbd.o hcd.o hub.o usbio.o driver.o mem.o interface.o imports.o exports.o

IOP_INCS += -I$(PS2SDK)/iop/include -I$(PS2SDK)/common/include

include $(PS2SDK)/Defs.make
include ../Rules.bin.make
include $(PS2SDK)/samples/Makefile.iopglobal

