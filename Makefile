# Project Name
TARGET = sniffer

# Sources
CPP_SOURCES = sniffer.cpp

# Library Locations (override if your checkout lives elsewhere, e.g.
#   make LIBDAISY_DIR=path/to/libDaisy DAISYSP_DIR=path/to/DaisySP)
LIBDAISY_DIR ?= ../../libDaisy
DAISYSP_DIR ?= ../../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
