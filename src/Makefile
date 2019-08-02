#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := M5Reflow

EXTRA_CFLAGS += --save-temps

include $(IDF_PATH)/make/project.mk

release:
	cp build/M5Reflow.bin ../bin
	cp build/M5Reflow_partitions.bin ../bin
	cp build/bootloader/bootloader.bin ../bin