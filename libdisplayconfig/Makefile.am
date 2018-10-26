HEADER_PATH := ${WORKSPACE}/display/display-hal/libdisplayconfig

displayconfig_h_sources = $(HEADER_PATH)/*.h
displayconfig_includedir = $(includedir)
displayconfig_include_HEADERS = $(displayconfig_h_sources)

c_sources = displayconfig.c

lib_LTLIBRARIES = libdisplayconfig.la
libdisplayconfig_la_CC = @CC@
libdisplayconfig_la_SOURCES = $(c_sources)
libdisplayconfig_la_CFLAGS = $(COMMON_CFLAGS)
