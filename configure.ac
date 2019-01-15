#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([AUTO_DRONE], [1.0.0], [contact@ecobot.biz])
AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE([foreign])
# Checks for programs.
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL

AC_PROG_INSTALL
AM_PROG_CC_C_O
AC_PROG_RANLIB


# Checks for libraries.
# FIXME: Replace `main' with a function in `-lutil':
AC_CHECK_LIB([util], [main])
# FIXME: Replace `main' with a function in `-ldrone':
AC_CHECK_LIB([drone], [main])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdlib.h sys/ioctl.h termios.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL

# Checks for library functions.

AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/io/Makefile
                 src/util/Makefile])
AC_OUTPUT