TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#DB += xxx.db

ifdef MOTOR_AUTOMATION1
### motorAutomation1 was built outside motor
DB_INSTALLS += $(MOTOR_AUTOMATION1)/db/Automation1MotorController.template
DB_INSTALLS += $(MOTOR_AUTOMATION1)/db/Automation1MotorAxis.template
else
### motorAutomation1 was built inside motor/modules
DB_INSTALLS += $(MOTOR)/db/Automation1MotorController.template
DB_INSTALLS += $(MOTOR)/db/Automation1MotorAxis.template
endif

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


