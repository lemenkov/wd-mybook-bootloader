# Initial Makefile to create the second stage loader from sources 
# stored in SRC directory
# J J Larkworthy 20 March 2006
#********************************************************************
# JJL: 21/07/06 Add debugging to compile options.

# this is a stand alone system so needs a custom linking and building script.
# Cross compiled for the ARM system as well.
#

BUILD_TIME := "$(shell date)"

# architecture set up options.
ARCH		?= arm
CROSS_COMPILE ?= arm-linux-gnu-
PLL400 ?= 400000000
OVERCLOCK_PLL ?= 0

# now for the commands needed.
AS		= $(CROSS_COMPILE)as
LD		= $(CROSS_COMPILE)ld
CC		= $(CROSS_COMPILE)gcc
CPP		= $(CC) -E
AR		= $(CROSS_COMPILE)ar
NM		= $(CROSS_COMPILE)nm
STRIP		= $(CROSS_COMPILE)strip
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump
AWK		= awk
GENKSYMS	= scripts/genksyms/genksyms
DEPMOD		= /sbin/depmod
KALLSYMS	= scripts/kallsyms
PERL		= perl
CHECK		= sparse
CHECKFLAGS     := -D__linux__ -Dlinux -D__STDC__ -Dunix -D__unix__
MODFLAGS	= -DMODULE
#CFLAGS_MODULE   = $(MODFLAGS)
#AFLAGS_MODULE   = $(MODFLAGS)
LDFLAGS_MODULE  = -r
ASFLAGS         = -mapcs-32 -g
CFLAGS_KERNEL	= 
AFLAGS_KERNEL	=
INCLUDE         = -Iinclude
LDOPTS          = -M -nostdlib --verbose --gc-sections
CCOPTS          = -DNOMINAL_PLL400_FREQ=$(PLL400) -DOVERCLOCK_PLL=$(OVERCLOCK_PLL) $(INCLUDE) -O2 -c -x c -ffunction-sections -fdata-sections -Wall -Werror -ggdb -DBUILD_DATE='$(BUILD_TIME)'
OBJCOPYFLAGS    = -O binary -R .note -R .comment -S

OBJDIR = ./obj
SRCDIR = ./src

OBJECTS = $(AOBJECTS) $(COBJECTS) 

COBJECTS =  $(OBJDIR)/stage1.o $(OBJDIR)/sata.o $(OBJDIR)/ns16550.o $(OBJDIR)/debug.o $(OBJDIR)/crc32.o

STAGE1_OBJECTS = $(OBJDIR)/start.o $(OBJDIR)/stage1.o $(OBJDIR)/sata.o $(OBJDIR)/dma.o \
                $(OBJDIR)/ns16550.o $(OBJDIR)/debug.o $(OBJDIR)/crc32.o $(OBJDIR)/build.o
	
AOBJECTS = $(OBJDIR)/start.o

all : stage1.bin $(OBJDIR)/.


$(OBJDIR)/%.o : $(SRCDIR)/%.c Makefile
	mkdir -p $(OBJDIR)
	$(CC) $(CCOPTS)  $< -o $@

$(OBJDIR)/%.o : $(SRCDIR)/%.S Makefile
	mkdir -p $(OBJDIR)
	$(AS) $(ASFLAGS) $< -o $@

%.d : %.c
	$(CC) -M $(CCOPTS) $< > $@.$$$$; \
	sed 's,\(.*\)\.o[ :]*,$(OBJDIR)/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

stage1.bin: stage1.elf ./tools/update_header
	$(OBJCOPY) $(OBJCOPYFLAGS) stage1.elf stage1.bin
	./tools/update_header stage1.bin 

./tools/update_header : ./tools/update_header.c tools/Makefile
	$(MAKE)  -C ./tools

stage1.elf: $(STAGE1_OBJECTS) linkfile 
	$(LD) $(LDOPTS) -T linkfile $(STAGE1_OBJECTS) -o $@ > $@.map

.PHONY : .lastmake

$(OBJDIR)/build.o : .lastmake

$(OBJDIR)/. :
	mkdir $(OBJDIR)
	

sources = $(COBJECTS:$(OBJDIR)%.o=$(SRCDIR)%.c)
include $(sources:.c=.d)

clean :
	@rm -f $(SRCDIR)/*.d
	@rm -rf $(OBJDIR)
	@rm -f stage1.bin
	@rm -f stage1.elf
	@rm -f stage1.elf.map
	$(MAKE) -C ./tools clean
