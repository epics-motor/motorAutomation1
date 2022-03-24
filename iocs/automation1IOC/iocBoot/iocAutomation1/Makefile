TOP = ../..
include $(TOP)/configure/CONFIG
ARCH = $(EPICS_HOST_ARCH)
# Linux only needs envPaths
#TARGETS = envPaths
# Windows needs envPaths and dllPath.bat
TARGETS = envPaths dllPath.bat
include $(TOP)/configure/RULES.ioc
