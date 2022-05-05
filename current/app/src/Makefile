TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build an ioc application
# Use PROD_IOC_RTEMS for RTEMS only
# Use PROD_IOC_Linux for Linux only
PROD_IOC = inficon

PROD_SYS_LIBS_DEFAULT += pcre

# vat590.dbd will be created and installed
DBD += inficon.dbd

# vat590.dbd will be made up from these files:
inficon_DBD += base.dbd
inficon_DBD += iocAdmin.dbd
inficon_DBD += asSupport.dbd
inficon_DBD += asyn.dbd
inficon_DBD += stream.dbd
inficon_DBD += drvAsynIPPort.dbd
inficon_DBD += drvAsynSerialPort.dbd
inficon_DBD += caPutLog.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
inficon_SRCS += inficon_registerRecordDeviceDriver.cpp
inficon_SRCS_DEFAULT += inficonMain.cpp

# Add locally compiled object code
#inficon_SRCS += dbSubExample.c

inficon_LIBS += devIocStats
inficon_LIBS += autosave
inficon_LIBS += asyn
inficon_LIBS += stream
inficon_LIBS += caPutLog
inficon_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
