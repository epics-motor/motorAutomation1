# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

LIBRARY_IOC = Automation1

SRCS += Automation1MotorAxis.cpp
SRCS += Automation1MotorController.cpp

automation1compiler_DIR += $(MOTOR_AUTOMATION1)/automation1Sup/Lib
Automation1_LIBS += automation1compiler
automation1c_DIR += $(MOTOR_AUTOMATION1)/automation1Sup/Lib
Automation1_LIBS += automation1c

DBD += devAutomation1Motor.dbd

Automation1_LIBS += motor asyn
Automation1_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

