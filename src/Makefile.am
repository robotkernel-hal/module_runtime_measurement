ACLOCAL_AMFLAGS = -I m4 

lib_LTLIBRARIES = libmodule_runtime_measurement.la

BUILT_SOURCES = $(top_builddir)/src/gen_definitions.h
CLEANFILES = $(top_builddir)/src/gen_definitions.h

libmodule_runtime_measurement_la_SOURCES  = runtime_measurement.cpp
libmodule_runtime_measurement_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
											@ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_runtime_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

VPATH = $(srcdir):$(top_srcdir)/share

GEN_DEFINITIONS = msg_definitions/module_runtime_measurement/pds

$(top_builddir)/src/gen_definitions.h: Makefile $(GEN_DEFINITIONS)
	@echo creating $@
	rk_generate --indir $(top_srcdir)/share --out $@ $(GEN_DEFINITIONS)
	touch $@
