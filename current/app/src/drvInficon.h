//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH resifual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//

/* Record types */

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <epicsStdio.h>
#include <errlog.h>
#include <epicsMessageQueue.h>

#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <functional>
#include <list>

/* Forward declarations */
class devInficon;

class devInficon {
private:
    //
public:
	/* Device info */
	asynUser* pasynUser;
	char* m_portName;
	char* m_hostInfo;

	bool m_connected;
	bool m_init;

	/* Enable/disable debugging messages */
	bool m_debug;

public:
	drvInficon();

	/* Make  sure to free everything */
	~drvInficon();

public:
	//crete instance of inficon device
	static devInficon* Create(const char* portName, const char* hostinfo);

public:
	/* Utils for reading/writing */
};