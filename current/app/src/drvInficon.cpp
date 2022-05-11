//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH resifual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <iocsh.h>

/* Asyn includes */
#include <drvAsynIPPort.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "drvInficon.h"

/*Json parser includes*/
#include <json.hpp>
using nlohmann::json;

static const char *driverName = "INFICONMPH";

//==========================================================//
// class devInficon
//		Holds useful vars for interacting with Inficon MPH RGA****
//		hardware
//==========================================================//
devInficon::devInficon() {
	/* Initialize members*/
	pasynUser = NULL;
	m_portName = NULL;
	m_hostInfo = NULL;
	m_connected = false;
	m_init = false;
	m_debug = false;

	/* Lets make sure there are no nullptr issues */
	m_hostInfo = (char*)malloc(1);
	m_hostInfo[0] = '\0';
	m_portName = (char*)malloc(1);
	m_portName[0] = '\0';
}

devInficon::~devInficon() {
	if (m_hostInfo)
		free(m_hostInfo);
	if (m_portName)
		free(m_portName);
	
	//pasynManager->disconnect(pasynUser);
    //pasynManager->freeAsynUser(pasynUser);
    //pasynUser = NULL;
}


devInficon* devInficon::Create(const char* portName, const char* hostInfo) {
	if (!portName || !hostInfo)
		return NULL;
	
	devInficon* pinficon = new devInficon();
	
	/* Free the previously allocated stuff */
	free(pinficon->m_portName);
	free(pinficon->m_hostInfo);
	
	/* Copy portName */
	pinficon->m_portName = strdup(portName);
	/* Copy hostInfo */
	pinficon->m_hostInfo = strdup(hostInfo);
	
    //drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,noProcessEos)
	int status = drvAsynIPPortConfigure(pinficon->m_portName, pinficon->m_hostInfo, 0, 0, 0); //I think for the HTTP port the noAutoConnect should be set to 1

	if (status) {
		util::Error("devInficon::Create(): Unable to configure drvAsynIPPort.");
		return connected;
	}

	/* check connection */
	asynUser* usr = pasynManager->createAsynUser(NULL, NULL);
	pasynManager->connectDevice(usr, pinficon->m_portName, 0);
	int connected = 0;
	pasynManager->isConnected(usr, &connected);
	pasynManager->disconnect(usr);
	pasynManager->freeAsynUser(usr);

	if (!connected) {
		util::Error("devInficon::Create(): Error while connecting to device %s.", name);
		return NULL;
	}
	
	return pinficon;
}

//==========================================================//
// IOCsh functions here
//==========================================================//
void configCallFunc(const iocshArgBuf* args) {
	const char* portName = args[0].sval;
	const char* ip = args[1].sval;
	int port = args[2].ival;

	if (!portName) {
		epicsPrintf("Invalid port name passed.\n");
		return;
	}

	if (!ip) {
		epicsPrintf("Invalid IP passed.\n");
		return;
	}

	if (port <= 0) {
		epicsPrintf("The port %i is invalid.\n", port);
		return;
	}

	devInficon* dev;
	
	char hostInfo[64];
	sprintf(hostInfo, "%s:%i", ip, port);

    dev = devInficon::Create(portName, hostInfo);
	
	if (!dev) {
		epicsPrintf("Unable to create device: Unspecified error.\n");
		return;
	}
}


int drvInficonRegister() {
	
	/* inficonMPHConfigure("name", "ip", port) */
	{
		static const iocshArg arg1 = {"Port Name", iocshArgString};
		static const iocshArg arg2 = {"IP", iocshArgString};
		static const iocshArg arg3 = {"Port Number", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"inficonMPHConfigure", 3, args};
		iocshRegister(&func, configCallFunc);
	}
	
	return 0;
}
epicsExportRegistrar(drvInficonRegister);