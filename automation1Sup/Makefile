TOP=../

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += $(patsubst ../%, %, $(wildcard ../Include/*.h))

BIN_INSTALLS += ../Lib/libautomation1c.so
BIN_INSTALLS += ../Lib/libautomation1compiler.so

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

