#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.

EXTRA_CFLAGS += -Ilge/include

ifeq ($(HW_REV),rev_a)

EXTRA_CFLAGS += -Idrivers/misc/k3gtr

EXTRA_CFLAGS += -DLGE_SENSOR=1
EXTRA_CFLAGS += -DLGE_SENSOR_FUSION=1
EXTRA_CFLAGS += -DLGE_SENSOR_ACCELEROMETER=1
EXTRA_CFLAGS += -DLGE_SENSOR_GYROSCOPE=1
EXTRA_CFLAGS += -DLGE_SENSOR_DCOMPASS=1

obj-y	+= k3dh.o
obj-y	+= k3g.o
obj-y	+= ami306.o

endif

