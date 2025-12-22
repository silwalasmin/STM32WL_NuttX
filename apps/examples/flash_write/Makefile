include $(APPDIR)/Make.defs

PROGNAME  = flash_write
PRIORITY  = 100
STACKSIZE = 2048
MODULE    = $(CONFIG_EXAMPLES_FLASH_WRITE)

CFLAGS   += -I$(APPDIR)/examples/uart_read

MAINSRC   = flash_write_main.c

include $(APPDIR)/Application.mk