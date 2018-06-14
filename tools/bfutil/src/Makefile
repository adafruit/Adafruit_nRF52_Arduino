CC     = gcc
LD		 = gcc
CFLAGS += -O0 -g3 -Wall -std=gnu99
LIBS   += -lusb-1.0
OBJDIR = _build



ifeq ($(OS),Windows_NT)
	LIBS   += -Llibusb-1.0
	BINFILE = bfutil.exe
	BINDIR := win32
	COPYDLL = cp libusb-1.0/libusb-1.0.dll $(BINDIR)/libusb-1.0.dll
else
	BINFILE = bfutil
	COPYDLL =
	
	OS := $(shell uname -s)
	ifeq ($(OS),Linux)
	  ifeq ($(MBIT),32)
	    BINDIR :=linux32
	    CFLAGS +=-m32
	  else
	    LBITS := $(shell getconf LONG_BIT)
      ifeq ($(LBITS),64)
        BINDIR := linux64
      else
        BINDIR := linux32
      endif
    endif
	endif
	ifeq ($(OS),Darwin)
		BINDIR := macos
  endif
endif

BINDIR := ../binaries/$(BINDIR)
$(info $(BINDIR))

all: clean $(OBJDIR) bfutil

.PHONY: $(OBJDIR)
$(OBJDIR):
	@mkdir -p $(OBJDIR)
	@mkdir -p $(BINDIR)
	
$(OBJDIR)/%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

bfutil: $(OBJDIR)/bfutil.o
	$(LD) -o $(BINDIR)/$(BINFILE) $^ $(CFLAGS) $(LIBS)
	$(COPYDLL) 
  
clean: 
	@rm -rf $(OBJDIR)
	@rm -f $(BINDIR)/$(BINFILE)
