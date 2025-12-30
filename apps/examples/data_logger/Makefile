include $(APPDIR)/Make.defs

PROGNAME  = data_logger
PRIORITY  = 100
STACKSIZE = 4096
MODULE    = $(CONFIG_EXAMPLES_DATA_LOGGER)

CFLAGS   += -I$(APPDIR)/examples/uart_read

MAINSRC   = data_logger_main.c
CSRCS	  = mcu_mag.c
include $(APPDIR)/Application.mk