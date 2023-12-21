
################ Top configuration.
PRJ_TOP = ..
TARGET = ch554
OBJDIR = obj

MCU = mcs51

BUILD_STRICT = TRUE
BUILD_PRINT = TRUE

VER_FILE =

XRAM_SIZE = 0x0300
XRAM_LOC = 0x0100 # Start of XRAM.

ifndef CODE_SIZE
CODE_SIZE = 0x2800
endif

################ source files ##############
# Source file들은 project TOP 에서의 위치를 나타낸다.
CSRC = \
		app/main.c \
		app/time.c \
		util/debug.c

CPPSRC =

ASRC =

DATA =

LINK_SCRIPT =

################ Object ##############
OBJ =	$(CSRC:%.c=$(OBJDIR)/%.c.rel) \
		$(CPPSRC:%.cpp=$(OBJDIR)/%.p.rel) \
		$(ASRC:%.S=$(OBJDIR)/%.s.rel) \
		$(DATA:%.bin=$(OBJDIR)/%.b.rel)

LST =	$(OBJ:%.rel=%.lst)
DEP =	$(OBJ:%.rel=%.d)

################  Define
DEFINE = -DFREQ_SYS=24000000

################  Library directory.
PRJ_LIB_DIR =

EXT_LIB_DIR = 

LIB_FILE =

LIB_OPT =	$(patsubst %,-L$(PRJ_TOP)/%,$(PRJ_LIB_DIR)) \
			$(patsubst %,-L%,$(EXT_LIB_DIR)) \
			$(patsubst %,-l%,$(LIB_FILE))

################  Include.
# Add relative path from $(PRJ_TOP)
PRJ_INC =	util \
			app

# Add absolue path. (ex. c:/lib/inc)
EXT_INC =

INCLUDE =	$(patsubst %,-I$(PRJ_TOP)/%,$(PRJ_INC)) \
			$(patsubst %,-I%,$(EXT_INC))

################  Common Options for C, C++, ASM
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
FLAGS  = -m$(MCU)
FLAGS += --no-xinit-opt
#FLAGS += --opt-code-speed
FLAGS += --model-large
FLAGS += --xram-size $(XRAM_SIZE)
FLAGS += --xram-loc $(XRAM_LOC)
FLAGS += --code-size $(CODE_SIZE)

ifeq ($(BUILD_STRICT), TRUE)
FLAGS +=
endif

################  C Options
CFLAGS  = $(FLAGS)
CFLAGS += $(DEFINE)
CFLAGS += $(INCLUDE)

################  C++ Options
CPPFLAGS  = $(FLAGS)
CPPFLAGS += $(DEFINE)
CPPFLAGS += $(INCLUDE)

################  Assembler Options
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex 
#       dump that will be displayed for a given single line of source input.
ASFLAGS  = $(FLAGS)
#ASFLAGS += -Wa,-gstabs,--listing-cont-lines=100
ASFLAGS += $(DEFINE)
ASFLAGS += $(INCLUDE)

################  Linker Options
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDS_OPT = $(patsubst %, -T $(PRJ_TOP)/%, $(LINK_SCRIPT))

LDFLAGS = $(FLAGS)
LDFLAGS += $(LIB_OPT)

################### Action #################
# add @ for silent.
CC 		= sdcc
OBJ_CPY = sdobjcopy
LD		= sdld

SHELL = sh
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp

FLASH = ../tools/vnproch55x.exe

ifeq ($(BUILD_PRINT), TRUE)
# some trick.
C_MAKE	= $(CC)
CPP_MAKE= $(CC)
ASM_MAKE= $(CC)
ELF_MAKE= $(CC)
HEX_MAKE= $(OBJ_CPY)
else
# some trick.
C_MAKE	= @$(CC)
CPP_MAKE= @$(CC)
ASM_MAKE= @$(CC)
ELF_MAKE= @$(CC)
HEX_MAKE= @$(OBJ_CPY)
endif

INFO 	= @echo Making: $@
DIR_CHK = @mkdir -p $(@D)

################ Object file
$(OBJDIR)/%.c.rel : $(PRJ_TOP)/%.c
	$(INFO)
	$(DIR_CHK)
	@$(C_MAKE) -c -V $(CFLAGS)$< -o $@ 

################### File Creation #################
all: $(TARGET).bin $(TARGET).hex

$(PRJ_TOP)/$(VER_FILE):
	$(INFO)
	@echo "char* gpVer = \"$$(date +%Y%m%d_%H%M%S)\";" > $@

$(TARGET).ihx: $(OBJ)
	$(INFO)
	$(DIR_CHK)
	$(ELF_MAKE) --dump-ast $(LDFLAGS) $^ --out-fmt-ihx -o $@ 

$(TARGET).bin: $(TARGET).ihx
	$(INFO)
	$(DIR_CHK)
	$(HEX_MAKE) -I ihex -O binary $< $@

$(TARGET).hex: $(TARGET).ihx
	$(INFO)
	$(DIR_CHK)
	$(HEX_MAKE) -I ihex -O ihex $< $@

################ Actions.
TARGET_ALL=	$(TARGET).hex $(TARGET).lk $(TARGET).map $(TARGET).mem $(TARGET).ihx $(TARGET).bin

version:
	@echo "char* gpVer = \"$$(date +%Y%m%d_%H%M%S)\";" > $(PRJ_TOP)/$(VER_FILE)

obj: $(OBJ)
hex: $(TARGET).hex

size: $(TARGET).elf
	@$(SIZE) --format=gnu $<
	@$(SIZE) --format=sysv --radix=16 --common $<

clean:
	$(REMOVE) $(TARGET_ALL)
	$(REMOVEDIR) $(OBJDIR)

prog: $(TARGET).hex
	$(FLASH) -r 2 -t CH552 -c 3 $(TARGET).hex


# Include the dependency files.
-include $(patsubst %.c,$(OBJDIR)/%.c.d,$(CSRC))
-include $(patsubst %.cpp,$(OBJDIR)/%.p.d,$(CPPSRC))
-include $(patsubst %.S,$(OBJDIR)/%.S.d,$(ASRC))

# Listing of phony targets.
.PHONY : all obj elf bin hex lst size gccversion clean version
