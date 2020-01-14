HEADER_PATH := $(top_srcdir)/include


#gr_adreno_info.h
#gr_allocator.h
#gralloc_priv.h
#gr_buf_descriptor.h
#gr_buf_mgr.h
#gr_device_impl.h
#gr_ion_alloc.h
#gr_priv_handle.h
#gr_utils.h



#h_sources = alloc_controller.h \
#            memalloc.h
#
cpp_sources = gr_utils.cpp gr_adreno_info.cpp gr_camera_info.cpp
h_sources = gr_device_impl.h gralloc_priv.h gr_priv_handle.h
#
memalloc_includedir = $(pkgincludedir)/
memalloc_include_HEADERS = $(h_sources)
#
lib_LTLIBRARIES = libmemalloc.la
libmemalloc_la_CC = @CC@
libmemalloc_la_SOURCES = $(cpp_sources)
libmemalloc_la_CFLAGS = $(COMMON_CFLAGS) -DLOG_TAG=\"qdmemalloc\" -I../libqdutils/
libmemalloc_la_CPPFLAGS = $(AM_CPPFLAGS)
libmemalloc_la_LIBADD = ../libqdutils/libqdMetaData.la
libmemalloc_la_LIBADD += -lhardware -lcutils -llog -lutils -ldl  -lsync
libmemalloc_la_LDFLAGS = -shared -avoid-version

#
#header_sources = gralloc_priv.h \
#                 gr.h \
#                 adreno_utils.h \
#                 $(HEADER_PATH)/color_metadata.h




header_sources := gr_device_impl.h gralloc_priv.h gr_priv_handle.h


c_sources := gr_ion_alloc.cpp \
             gr_allocator.cpp \
             gr_buf_mgr.cpp \
             gr_device_impl.cpp

library_includedir = $(includedir)
library_include_HEADERS = $(header_sources)

lib_LTLIBRARIES += libgralloc.la
libgralloc_la_CC = @CC@
libgralloc_la_SOURCES = $(c_sources)
libgralloc_la_CFLAGS = $(COMMON_CFLAGS) -DLOG_TAG=\"qdgralloc\"  -I../libqdutils/
libgralloc_la_CPPFLAGS = $(AM_CPPFLAGS) 
libgralloc_la_LIBADD = ../libqdutils/libqdMetaData.la
libgralloc_la_LIBADD += -lhardware -lcutils -llog -lutils -lbinder  -lsync
libgralloc_la_LIBADD += libmemalloc.la
libgralloc_la_LDFLAGS = -shared -avoid-version
