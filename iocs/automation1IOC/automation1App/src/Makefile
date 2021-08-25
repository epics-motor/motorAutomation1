TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = automation1
# automation1.dbd will be created and installed
DBD += automation1.dbd

# automation1.dbd will be made up from these files:
automation1_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
automation1_DBD += asyn.dbd
automation1_DBD += drvAsynSerialPort.dbd
automation1_DBD += drvAsynIPPort.dbd
#endif
#ifdef BUSY
automation1_DBD += busySupport.dbd
#endif
automation1_DBD += motorSupport.dbd
automation1_DBD += devAutomation1Motor.dbd

# automation1_LDFLAGS_Linux += -Wl,-rpath,/home/garrett/EPICS/synApps_6_1/support/motor-R7-1/bin/linux-x86_64/libautomation1compiler.so
automation1compiler_DIR += /home/garrett/EPICS/synApps_6_1/support/motor-R7-1/bin/linux-x86_64
automation1_LIBS += automation1compiler
automation1c_DIR += /home/garrett/EPICS/synApps_6_1/support/motor-R7-1/bin/linux-x86_64
automation1_LIBS += automation1c

# Add all the support libraries needed by this IOC
automation1_LIBS += Automation1
automation1_LIBS += motor
#ifdef ASYN
automation1_LIBS += asyn
#endif
#ifdef BUSY
automation1_LIBS += busy
#endif
#ifdef SNCSEQ
automation1_LIBS += seq pv
#endif

# automation1_registerRecordDeviceDriver.cpp derives from automation1.dbd
automation1_SRCS += automation1_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
automation1_SRCS_DEFAULT += automation1Main.cpp
automation1_SRCS_vxWorks += -nil-

# Finally link to the EPICS Base libraries
automation1_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

