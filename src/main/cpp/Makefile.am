ACLOCAL_AMFLAGS = -I m4 

lib_LTLIBRARIES = libmodule_runtime_measurement.la

libmodule_runtime_measurement_la_SOURCES  = runtime_measurement.cpp
libmodule_runtime_measurement_la_CXXFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                           @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_PROCESS_DATA_INSPECTION_CFLAGS@
libmodule_runtime_measurement_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

install-exec-hook:
	mkdir -p $(prefix)/share/ln
	-cp -rf $(top_builddir)/share/ln/* $(prefix)/share/ln/
