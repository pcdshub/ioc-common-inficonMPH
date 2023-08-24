TOP = ..
-include $(TOP)/IOC_APPL_TOP
ifneq (,$(IOC_APPL_TOP))
TOP = $(IOC_APPL_TOP)
endif

include $(TOP)/configure/CONFIG
DIRS += $(wildcard *ioc*)
include $(EPICS_BASE)/configure/RULES_DIRS

