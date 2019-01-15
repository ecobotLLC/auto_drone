SUBDIRS = util io
bin_PROGRAMS = AUTODRONE

#INCLUDES = -I../../../../imx8PRJ/imx-yocto-bsp/build-xwayland/tmp/work-shared/imx8mqevk/kernel-source/include/
#INCLUDES += -I../../../../imx8PRJ/imx-yocto-bsp/build-xwayland/tmp/work-shared/imx8mqevk/kernel-source/arch/arm64/include/
#INCLUDES += -I../../../../imx8PRJ/imx-yocto-bsp/build-xwayland/tmp/work-shared/imx8mqevk/kernel-source/arch/x86/include/asm

AUTODRONE_LDFLAGS = -L./util
AUTODRONE_LDFLAGS = -L./io

AUTODRONE_LDADD = -lutil
AUTODRONE_LDADD = -ldrone

AUTODRONE_CFLAGS =  -I ./util
AUTODRONE_CFLAGS +=  -I ./io



AUTODRONE_DEPENDENCIES = ./util/libutil.a
AUTODRONE_DEPENDENCIES = ./io/libdrone.a
AUTODRONE_SOURCES = main.c
