# gnss_crpa - GNSS anti-jam firmware (crpa13: hardware NLMS) for the ANTSDR E310 on Linux.
# Built by buildroot (package/gnss-crpa). CC / CFLAGS / LDFLAGS may come from
# the command line (buildroot's TARGET_CONFIGURE_OPTS); the project's own flags
# are kept apart so they are never lost. Host test without libiio:
#   make NO_LIBIIO=1 CC=gcc
CC      ?= gcc
CFLAGS  ?= -O2
GC_CFLAGS := -std=gnu99 -D_GNU_SOURCE -Wall -Wno-unused-function -Wno-pointer-sign \
             -Wno-unused-but-set-variable -DGNSS_LINUX -DGNSS_LOG_SD \
             -Isrc -Ilinux -include linux/lx_force.h
GC_LIBS := -lm
ifeq ($(NO_LIBIIO),1)
GC_CFLAGS += -DGNSS_CRPA_NO_LIBIIO
else
GC_LIBS += -liio
endif

SRCS := $(sort $(wildcard src/*.c)) linux/lx_io.c linux/lx_ad9361.c linux/lx_dmac.c \
        linux/lx_ff.c linux/lx_main.c
HDRS := $(wildcard src/*.h linux/*.h)

all: gnss_crpa

gnss_crpa: $(SRCS) $(HDRS)
	$(CC) $(GC_CFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS) $(GC_LIBS) $(LDLIBS)

install: gnss_crpa
	install -D -m 0755 gnss_crpa $(DESTDIR)/usr/bin/gnss_crpa
	ln -sf gnss_crpa $(DESTDIR)/usr/bin/gnss

clean:
	rm -f gnss_crpa

.PHONY: all install clean
