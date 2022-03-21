# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

LIBRARY_IOC = Automation1

SRCS += Automation1MotorAxis.cpp
SRCS += Automation1MotorController.cpp

ifdef MOTOR_AUTOMATION1
automation1compiler_DIR += $(MOTOR_AUTOMATION1)/bin/$(EPICS_HOST_ARCH)
automation1c_DIR += $(MOTOR_AUTOMATION1)/bin/$(EPICS_HOST_ARCH)
else
automation1compiler_DIR += $(MOTOR)/bin/$(EPICS_HOST_ARCH)
automation1c_DIR += $(MOTOR)/bin/$(EPICS_HOST_ARCH)
endif

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
	Automation1_LIBS += automation1c
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
	Automation1_LIBS += automation1c64
else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
	Automation1_LIBS += automation1compiler
	Automation1_LIBS += automation1c
endif



DBD += devAutomation1Motor.dbd

Automation1_LIBS += motor asyn
Automation1_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

