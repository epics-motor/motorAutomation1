TOP=../

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += $(patsubst ../%, %, $(wildcard ../Include/*.h))

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
	BIN_INSTALLS += ../Library/Windows/x86/Automation1C.dll
	LIB_INSTALLS += ../Library/Windows/x86/Automation1C.lib
	BIN_INSTALLS += ../Library/Windows/x86/Automation1Compiler.dll
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
	BIN_INSTALLS += ../Library/Windows/x64/Automation1C64.dll
	LIB_INSTALLS += ../Library/Windows/x64/Automation1C64.lib
	BIN_INSTALLS += ../Library/Windows/x64/Automation1Compiler64.dll
# Use OS_CLASS check for linux to avoid breaking custom Linux arches
else ifeq ($(OS_CLASS),Linux)
ifeq (x86_64, $(findstring x86_64, $(T_A)))
	LIB_INSTALLS += ../Library/Linux/x64/libautomation1c.so
	LIB_INSTALLS += ../Library/Linux/x64/libautomation1compiler.so
endif
# End OS_CLASS=Linux
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

