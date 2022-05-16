//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH residual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//

/* ANSI C includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* EPICS includes */
#include <dbAccess.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsPrint.h>
#include <epicsExit.h>
#include <errlog.h>
#include <iocsh.h>

/* Asyn includes */
#include <drvAsynIPPort.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>

#include "drvInficon.h"

/* Json parser includes */
#include <json.hpp>
using nlohmann::json;

static const char *driverName = "INFICON";

//==========================================================//
// class drvInficon
//		Holds useful vars for interacting with Inficon MPH RGA****
//		hardware
//==========================================================//
drvInficon::drvInficon(const char *portName, const char* hostInfo)

   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask,                   /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/

    inficonExiting_(false),
    initialized_(false),
	hostInfo_(NULL),
    portName_(NULL),
    octetPortName_(NULL),
    isConnected_(false),
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
    
    hostInfo_ = epicsStrDup(hostInfo);
    portName_ = epicsStrDup(portName);
	
    //Communication parameters
    createParam(INFICON_IP_STRING,              asynParamOctet,             &ip_); //what parameter type should be here if the readback value is string
    createParam(INFICON_MAC_STRING,             asynParamOctet,             &mac_); //what parameter type should be here if the readback value is string
    createParam(INFICON_ERROR_LOG_STRING,       asynParamOctet,             &errorLog_); //what parameter type should be here if the readback value is string
    //General control parameters
    createParam(INFICON_SET_EMI_STRING,         asynParamUInt32Digital,     &setEmi_);
    createParam(INFICON_GET_EMI_STRING,         asynParamUInt32Digital,     &getEmi_);
    createParam(INFICON_SET_EM_STRING,          asynParamUInt32Digital,     &setEm_);
    createParam(INFICON_GET_EM_STRING,          asynParamUInt32Digital,     &getEm_);
    createParam(INFICON_SET_RFGEN_STRING,       asynParamUInt32Digital,     &setRfGen_);
    createParam(INFICON_GET_RFGEN_STRING,       asynParamUInt32Digital,     &getRfGen_);
    createParam(INFICON_GET_FAN_STRING,         asynParamUInt32Digital,     &getFan_);
    createParam(INFICON_SHUTDOWN_STRING,        asynParamUInt32Digital,     &shutdown_);
    //Sensor info parameters
    createParam(INFICON_SENS_NAME_STRING,          asynParamOctet,          &sensName_);
    createParam(INFICON_SENS_DESC_STRING,          asynParamOctet,          &sensDesc_);
    createParam(INFICON_SENS_SN_STRING,            asynParamOctet,          &sensSn_);
    //Status parameters
    createParam(INFICON_SYST_STAT_STRING,          asynParamUInt32Digital,  &systStatus_);
    createParam(INFICON_HW_ERROR_STRING,           asynParamUInt32Digital,  &hwError_);
    createParam(INFICON_HW_WARN_STRING,            asynParamUInt32Digital,  &hwWarn_);
    createParam(INFICON_PWR_ON_TIME_STRING,        asynParamInt32,          &pwrOnTime_);
    createParam(INFICON_EMI_ON_TIME_STRING,        asynParamInt32,          &emiOnTime_);
    createParam(INFICON_EM_ON_TIME_STRING,         asynParamInt32,          &emOnTime_);
    createParam(INFICON_EMI_CML_ON_TIME_STRING,    asynParamInt32,          &emiCmlOnTime_);
    createParam(INFICON_EM_CML_ON_TIME_STRING,     asynParamInt32,          &emCmlOnTime_);
    createParam(INFICON_EMI_PRESS_TRIP_STRING,     asynParamInt32,          &emiPressTrip_);
    //Diagnostic data parameters
    createParam(INFICON_BOX_TEMP_STRING,           asynParamFloat64,        &boxTemp_);
    createParam(INFICON_ANODE_POTENTIAL_STRING,    asynParamFloat64,        &anodePotential_);
    createParam(INFICON_EMI_CURRENT_STRING,        asynParamFloat64,        &emiCurrent_);
    createParam(INFICON_FOCUS_POTENTIAL_STRING,    asynParamFloat64,        &focusPotential_);
    createParam(INFICON_ELECT_ENERGY_STRING,       asynParamFloat64,        &electEnergy_);
    createParam(INFICON_FIL_POTENTIAL_STRING,      asynParamFloat64,        &filPotential_);
    createParam(INFICON_FIL_CURRENT_STRING,        asynParamFloat64,        &filCurrent_);
    createParam(INFICON_EM_POTENTIAL_STRING,       asynParamFloat64,        &emPotential_);
    //Measurement parameters
    createParam(INFICON_GET_PRESS_STRING,          asynParamFloat64,        &getPress_);
    createParam(INFICON_GET_SCAN_STRING,           asynParamFloat32Array,   &getScan_);
    //Scan info parameters
    createParam(INFICON_FIRST_SCAN_STRING,         asynParamInt32,          &firstScan_);
    createParam(INFICON_LAST_SCAN_STRING,          asynParamInt32,          &lastScan_);
    createParam(INFICON_CURRENT_SCAN_STRING,       asynParamInt32,          &currentScan_);
    createParam(INFICON_PPSCAN_STRING,             asynParamInt32,          &ppscan_);
    createParam(INFICON_SCAN_STAT_STRING,          asynParamInt32,          &scanStat);
    //Sensor detector parameters
    createParam(INFICON_EM_VOLTAGE_MAX_STRING,     asynParamFloat64,        &emVoltageMax_);
    createParam(INFICON_EM_VOLTAGE_MIN_STRING,     asynParamFloat64,        &emVoltageMin_);
    //Sensor filter parameters
    createParam(INFICON_DWELL_MAX_STRING,          asynParamFloat64,        &dwelMax_);
    createParam(INFICON_DWELL_MIN_STRING,          asynParamFloat64,        &dwelMin_);
    //Scan setup parameters
    createParam(INFICON_SET_START_CH_STRING,       asynParamInt32,          &setStartCh_);
    createParam(INFICON_GET_START_CH_STRING,       asynParamInt32,          &getStartCh_);
    createParam(INFICON_SET_STOP_CH_STRING,        asynParamInt32,          &setStopCh_);
    createParam(INFICON_GET_STOP_CH_STRING,        asynParamInt32,          &getStopCh_);
    createParam(INFICON_SET_CH_MODE_STRING,        asynParamUInt32Digital,  &setChMode_);
    createParam(INFICON_GET_CH_MODE_STRING,        asynParamUInt32Digital,  &getChMode_);
    createParam(INFICON_SET_CH_PPAMU_STRING,       asynParamUInt32Digital,  &setChPpamu_);
    createParam(INFICON_GET_CH_PPAMU_STRING,       asynParamUInt32Digital,  &getChPpamu_);
    createParam(INFICON_SET_CH_DWELL_STRING,       asynParamFloat64,        &setChDwell_);
    createParam(INFICON_GET_CH_DWELL_STRING,       asynParamFloat64,        &getChDwell_);
    createParam(INFICON_SET_CH_START_MASS_STRING,  asynParamFloat64,        &setChStartMass_);
    createParam(INFICON_GET_CH_START_MASS_STRING,  asynParamFloat64,        &getChStartMass_);
    createParam(INFICON_SET_CH_STOP_MASS_STRING,   asynParamFloat64,        &setChStopMass_);
    createParam(INFICON_GET_CH_STOP_MASS_STRING,   asynParamFloat64,        &getChStopMass_);
    createParam(INFICON_SET_SCAN_COUNT_STRING,     asynParamInt32,          &setScanCount_);
    createParam(INFICON_GET_SCAN_COUNT_STRING,     asynParamInt32,          &getScanCount_);
    createParam(INFICON_SCAN_START_STRING,         asynParamUInt32Digital,  &scanStart_);
    createParam(INFICON_SCAN_STOP_STRING,          asynParamUInt32Digital,  &scanStop_);


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

    //epicsAtExit(inficonExitCallback, this);

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

/***********************/
/* asynCommon routines */
/***********************/

/* Report  parameters */
void drvInficon::report(FILE *fp, int details)
{
    fprintf(fp, "modbus port: %s\n", this->portName);
    if (details) {
        fprintf(fp, "    initialized:        %s\n", initialized_ ? "true" : "false");
        fprintf(fp, "    asynOctet server:   %s\n", octetPortName_);
        fprintf(fp, "    host info:          %s\n", hostInfo_);
    }
    asynPortDriver::report(fp, details);
}

asynStatus drvInficon::getAddress(asynUser *pasynUser, int *address)
{
    *address = 0;
    return asynSuccess;
}

/*
**  asynUInt32D support
*/
asynStatus drvInficon::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask)
{
    int function = pasynUser->reason;
    static const char *functionName = "readUInt32D";

    if (function == getEmi_) {
        ;
    }
    else {
        return asynPortDriver::readUInt32Digital(pasynUser, value, mask);
    }
}


asynStatus drvInficon::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    int function = pasynUser->reason;
    epicsUInt16 data = value;
    static const char *functionName = "writeUInt32D";

    if (function == setEmi_) {
        ;
    }
    else {
        return asynPortDriver::writeUInt32Digital(pasynUser, value, mask);
    }
    return asynSuccess;
}


/*
**  asynInt32 support
*/

asynStatus drvInficon::readInt32 (asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    static const char *functionName = "readInt32";

    *value = 0;

    if (function == getStartCh_) {
        ;
    }
    else {
        return asynPortDriver::readInt32(pasynUser, value);
    }
}


asynStatus drvInficon::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    epicsUInt16 buffer[4];
    int bufferLen;
    static const char *functionName = "writeInt32";

    if (function == setStartCh_) {
		;
    }
    else {
        return asynPortDriver::writeInt32(pasynUser, value);
    }
    return asynSuccess;
}


/*
**  asynFloat64 support
*/
asynStatus drvInficon::readFloat64 (asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    int bufferLen;
    static const char *functionName = "readFloat64";

    *value = 0;

    if (function == boxTemp_) {
        ;
    }
    else {
        return asynPortDriver::readFloat64(pasynUser, value);
    }

    return asynSuccess;
}


asynStatus drvInficon::writeFloat64 (asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    epicsUInt16 buffer[4];
    int bufferLen;
    static const char *functionName = "writeFloat64";


    if (function == setChDwell_) {
        ;
    }
    return asynSuccess;
}



/*
**  asynFloat32Array support
*/
asynStatus drvInficon::readFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans, size_t *nactual)
{
    int function = pasynUser->reason;
    int bufferLen;
	size_t i;
    static const char *functionName = "readFloat32Array";

    *nactual = 0;

    if (function == getScan_) {
        ;
    }
	else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }

    *nactual = i;
    return asynSuccess;
}


/*
**  asynOctet support
*/
asynStatus drvInficon::readOctet(asynUser *pasynUser, char *data, size_t maxChars, size_t *nactual, int *eomReason)
{
    int function = pasynUser->reason;
    static const char *functionName = "readOctet";

    *nactual = 0;

    if (function == ip_) {
        ;
    }
    else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }

    return asynSuccess;
}

/*
**  User functions
*/
asynStatus drvInficon::verifyConnection() {
	/* asynUsers should be pretty cheap to create */
	asynUser* usr = pasynManager->createAsynUser(NULL, NULL);
	usr->timeout = 0.5; /* 500ms timeout */

	/* Try for connection */
	pasynManager->connectDevice(usr, octetPortName_, 0);
	int yn = 0;
	pasynManager->isConnected(usr, &yn);
	pasynManager->disconnect(usr);

	pasynManager->freeAsynUser(usr);

	return (yn==1) ? asynSuccess : asynError;
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