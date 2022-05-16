//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH resifual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsPrint.h>
#include <iocsh.h>

/* Asyn includes */
#include <drvAsynIPPort.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "drvInficon.h"

/*Json parser includes*/
#include <json.hpp>
using nlohmann::json;

static const char *driverName = "INFICON";

struct inficonDrvUser_t {
    inficonCommandType_t commandType;
    int              len;
};

//==========================================================//
// class drvInficon
//		Holds useful vars for interacting with Inficon MPH RGA****
//		hardware
//==========================================================//
drvInficon::drvInficon(const char *portName, const char* hostInfo)

   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynUInt32DigitalMask | asynInt64Mask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynUInt32DigitalMask | asynInt64Mask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask,                   /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/

    inficonExiting_(false),
    initialized_(false),
	hostInfo_(epicsStrDup(hostinfo)),
    portName_(epicsStrDup(portName)),
    octetPortName_(NULL),
    isConnected_(false),
    ioStatus_(asynSuccess),
    commandType_(NULL),
    drvUser_(NULL),
    data_(0),
    readOK_(0),
    writeOK_(0),
    IOErrors_(0),
    currentIOErrors_(0),
    maxIOMsec_(0),
    lastIOMsec_(0)

{
    int status;
	int ipConfigureStatus;
    static const char *functionName = "drvInficon";

    createParam(MODBUS_DATA_STRING,                 asynParamInt32,       &P_Data);
    createParam(MODBUS_READ_STRING,                 asynParamInt32,       &P_Read);
    createParam(MODBUS_ENABLE_HISTOGRAM_STRING,     asynParamUInt32Digital, &P_EnableHistogram);
    createParam(MODBUS_READ_HISTOGRAM_STRING,       asynParamInt32,       &P_ReadHistogram);
    createParam(MODBUS_HISTOGRAM_BIN_TIME_STRING,   asynParamInt32,       &P_HistogramBinTime);
    createParam(MODBUS_HISTOGRAM_TIME_AXIS_STRING,  asynParamInt32Array,  &P_HistogramTimeAxis);
    createParam(MODBUS_POLL_DELAY_STRING,           asynParamFloat64,     &P_PollDelay);
    createParam(MODBUS_READ_OK_STRING,              asynParamInt32,       &P_ReadOK);
    createParam(MODBUS_WRITE_OK_STRING,             asynParamInt32,       &P_WriteOK);
    createParam(MODBUS_IO_ERRORS_STRING,            asynParamInt32,       &P_IOErrors);
    createParam(MODBUS_LAST_IO_TIME_STRING,         asynParamInt32,       &P_LastIOTime);
    createParam(MODBUS_MAX_IO_TIME_STRING,          asynParamInt32,       &P_MaxIOTime);

    setIntegerParam(P_ReadOK, 0);
    setIntegerParam(P_WriteOK, 0);
    setIntegerParam(P_IOErrors, 0);
    setIntegerParam(P_LastIOTime, 0);
    setIntegerParam(P_MaxIOTime, 0);
	
	
	if (octetPortName_ == NULL) octetPortName_ = "";
    /* Create octet port name */
	size_t prefixlen = strlen(PORT_PREFIX);
	size_t len = strlen(portName_) + strlen(PORT_PREFIX) + 1;
	octetPortName_ = (char*)malloc(len);
	memcpy(octetPortName_, PORT_PREFIX, prefixlen);
	memcpy(octetPortName_ + prefixlen, portName_, strlen(portName_) + 1);
	octetPortName_[len - 1] = '\0';
	
    // drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,noProcessEos)
	ipConfigureStatus = drvAsynIPPortConfigure(octetPortName_, hostInfo_, 0, 0, 0); //I think for the HTTP port the noAutoConnect should be set to 1

	if (ipConfigureStatus) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, Unable to configure drvAsynIPPort %s",
            driverName, functionName, octetPortName_);
        return;
	}

    /*Allocate data memory, need to do some testing to define max length*/
    //data_ = (epicsUInt16 *) callocMustSucceed(modbusLength_, sizeof(epicsUInt16), functionName);

    /* Allocate and initialize the default drvUser structure */
    drvUser_ = (inficonDrvUser_t *) callocMustSucceed(1, sizeof(inficonDrvUser_t), functionName);
    drvUser_->commandType = commandType_;
    drvUser_->len = -1;

    /* Connect to asyn octet port with asynOctetSyncIO */
    status = pasynOctetSyncIO->connect(octetPortName_, 0, &pasynUserOctet_, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s port %s can't connect to asynOctet on Octet server %s.\n",
            driverName, functionName, portName_, octetPortName_);
        return;
    }

    /* Connect to asyn octet port with asynCommonSyncIO */
    status = pasynCommonSyncIO->connect(octetPortName_, 0, &pasynUserCommon_, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s port %s can't connect to asynCommon on Octet server %s.\n",
        driverName, functionName, portName_, octetPortName_);
        return;
     }

    //epicsAtExit(modbusExitCallback, this);

    initialized_ = true;
}

drvInficon::~drvInficon() {
	if (hostInfo_)
		free(hostInfo_);
	if (portName_)
		free(portName_);
	if (octetPortName_)
		free(octetPortName_);
	
	//pasynManager->disconnect(pasynUser);
    //pasynManager->freeAsynUser(pasynUser);
    //pasynUser = NULL;
}

asynStatus drvInficon::VerifyConnection() {
	/* asynUsers should be pretty cheap to create */
	asynUser* usr = pasynManager->createAsynUser(NULL, NULL);
	usr->timeout = 0.5; /* 500ms timeout */

	/* Try for connection */
	pasynManager->connectDevice(usr, this->octetPortName_, 0);
	int yn = 0;
	pasynManager->isConnected(usr, &yn);
	pasynManager->disconnect(usr);

	pasynManager->freeAsynUser(usr);

	return (yn==1) ? asynSuccess : asynError;
}

drvInficon *drvInficon::Create(const char* portName, const char* hostInfo) {
	if (!portName || !hostInfo)
		return NULL;

	devInficon *pinficon = new devInficon();
	
	/* Free the previously allocated stuff */
	free(pinficon->portName_);
	free(pinficon->hostInfo_);
	free(pinficon->octetPortName_);
	
	/* Copy portName */
	pinficon->portName_ = strdup(portName);
	/* Copy hostInfo */
	pinficon->hostInfo_ = strdup(hostInfo);
	
	/* Create octet port name */
	size_t prefixlen = strlen(PORT_PREFIX);
	size_t len = strlen(portName) + strlen(PORT_PREFIX) + 1;
	pinficon->octetPortName_ = (char*)malloc(len);
	memcpy(pinficon->octetPortName_, PORT_PREFIX, prefixlen);
	memcpy(pinficon->octetPortName_ + prefixlen, portName, strlen(portName) + 1);
	pinficon->octetPortName_[len - 1] = '\0';
	
    // drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,noProcessEos)
	int status = drvAsynIPPortConfigure(pinficon->octetPortName_, pinficon->hostInfo_, 0, 0, 0);//I think for the HTTP port the noAutoConnect should be set to 1

	if (status) {
		epicsPrintf("devInficon::Create(): Unable to configure drvAsynIPPort.");
		return NULL;
	}

	/* check connection */
	int connected = drvInficon::VerifyConnection();

	if (!connected) {
		epicsPrintf("devInficon::Create(): Error while connecting to device %s.", hostInfo);
		return NULL;
	}
	
	return pinficon;
}

extern "C" {
/*
** drvInficonConfigure() - create and init an asyn port driver for a Inficon
**
*/

/** EPICS iocsh callable function to call constructor for the drvInficon class. */
asynStatus drvInficonConfigure(const char *portName, const char *hostInfo)
{
	if (!portName || !hostInfo)
		return asynError;
	
	new drvInficon(portName, hostInfo);
	
	return asynSuccess;
}

//==========================================================//
// IOCsh functions here
//==========================================================//
static void drvInficonConfigureCallFunc(const iocshArgBuf* args) {
	const char *portName = args[0].sval;
	const char *ip = args[1].sval;
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
	
	char hostInfo[64];
	sprintf(hostInfo,"%s:%i HTTP", ip, port);

	drvInficonConfigure(portName, hostInfo);
}


int drvInficonRegister() {
	
	/* drvInficonConfigure("ASYN_PORT", "IP", PORT_NUMBER) */
	{
		static const iocshArg arg1 = {"Port Name", iocshArgString};
		static const iocshArg arg2 = {"IP", iocshArgString};
		static const iocshArg arg3 = {"Port Number", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"drvInficonConfigure", 3, args};
		iocshRegister(&func, drvInficonConfigureCallFunc);
	}
	
	return 0;
}
epicsExportRegistrar(drvInficonRegister);

}// extern "C"