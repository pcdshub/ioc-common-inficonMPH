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
//#include <dbAccess.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsPrint.h>
#include <epicsExit.h>
#include <cantProceed.h>
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
    isConnected_(false),
    portName_(epicsStrDup(portName)),
    octetPortName_(NULL),
    hostInfo_(epicsStrDup(hostInfo)),
    data_(NULL),
	ioStatus_(asynSuccess),
    prevIOStatus_(asynSuccess),
    readOK_(0),
    writeOK_(0),
	scanChannel_(1)

{
    int status;
	int ipConfigureStatus;
    static const char *functionName = "drvInficon";
	
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
    createParam(INFICON_SENS_NAME_STRING,          asynParamOctet,          &sensName_);  //what parameter type should be here if the readback value is string
    createParam(INFICON_SENS_DESC_STRING,          asynParamOctet,          &sensDesc_);  //what parameter type should be here if the readback value is string
    createParam(INFICON_SENS_SN_STRING,            asynParamOctet,          &sensSn_);  //what parameter type should be here if the readback value is string
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
    createParam(INFICON_SCAN_STAT_STRING,          asynParamUInt32Digital,  &scanStat);
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
    createParam(INFICON_SET_CH_MODE_STRING,        asynParamOctet,          &setChMode_);  //what parameter type should be here if the readback value is string // string only
    createParam(INFICON_GET_CH_MODE_STRING,        asynParamOctet,          &getChMode_);  //what parameter type should be here if the readback value is string // string only
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

    /*Allocate memory*/
    data_ = (char*)callocMustSucceed(HTTP_RESPONSE_SIZE, sizeof(char), functionName);

    /* Connect to asyn octet port with asynOctetSyncIO */
    status = pasynOctetSyncIO->connect(octetPortName_, 0, &pasynUserOctet_, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s port %s can't connect to asynOctet on Octet server %s.\n",
            driverName, functionName, portName_, octetPortName_);
        return;
    }

    /* Connect to asyn octet port with asynCommonSyncIO 
    status = pasynCommonSyncIO->connect(octetPortName_, 0, &pasynUserCommon_, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s port %s can't connect to asynCommon on Octet server %s.\n",
        driverName, functionName, portName_, octetPortName_);
        return;
     }*/

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
	
	pasynManager->disconnect(pasynUserOctet_);
    pasynManager->freeAsynUser(pasynUserOctet_);
    pasynUserOctet_ = NULL;
}

/***********************/
/* asynCommon routines */
/***********************/
/* Connect */
asynStatus drvInficon::connect(asynUser *pasynUser)
{
    if (initialized_ == false) return asynDisabled;

    pasynManager->exceptionConnect(pasynUser);
    return asynSuccess;
}

/* Report  parameters */
void drvInficon::report(FILE *fp, int details)
{
    fprintf(fp, "inficon port: %s\n", this->portName);
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
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "readUInt32D";
	
	pasynManager->getAddr(pasynUser, &chNumber);
    *value = 0;

    if (function == getEmi_) {
        sprintf(request,"GET /mmsp/generalControl/setEmission/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, setEmiCommand);
    } else if (function == getEm_) {
        sprintf(request,"GET /mmsp/generalControl/setEM/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, setEmCommand);
    } else if (function == getRfGen_) {
        sprintf(request,"GET /mmsp/generalControl/rfGeneratorSet/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == getFan_) {
        sprintf(request,"GET /mmsp/generalControl/fanState/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == systStatus_) {
        sprintf(request,
        "GET /mmsp/status/systemStatus/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == hwError_) {
        sprintf(request,"GET /mmsp/status/hardwareErrors/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == hwWarn_) {
        sprintf(request,"GET /mmsp/status/hardwareWarnings/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == getChPpamu_) {
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/ppamu/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == scanStat) {
        sprintf(request,"GET /mmsp/scanInfo/scanning/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, scanStatCommand);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    callParamCallbacks(chNumber);
	return status;
}


asynStatus drvInficon::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    int function = pasynUser->reason;
    static const char *functionName = "writeUInt32D";

    if (function == setEmi_) {
        ;
    } else if (function == setEmi_) {
        ;
    } else if (function == setEm_) {
        ;
    } else if (function == setRfGen_) {
        ;
    } else if (function == shutdown_) {
        ;
    } else if (function == setChPpamu_) {
        ;
    } else if (function == scanStart_) {
        ;
    } else if (function == scanStop_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
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

    if (function == pwrOnTime_) {
        ;
    } else if (function == emiOnTime_) {
        ;
    } else if (function == emOnTime_) {
        ;
    } else if (function == emiCmlOnTime_) {
        ;
    } else if (function == emCmlOnTime_) {
        ;
    } else if (function == emiPressTrip_) {
        ;
    } else if (function == firstScan_) {
        ;
    } else if (function == lastScan_) {
        ;
    } else if (function == currentScan_) {
        ;
    } else if (function == ppscan_) {
        ;
    } else if (function == getStartCh_) {
        ;
    } else if (function == getStopCh_) {
        ;
    } else if (function == getScanCount_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    return asynSuccess;
}


asynStatus drvInficon::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    //epicsUInt16 buffer[4];
    //int bufferLen;
    static const char *functionName = "writeInt32";

    if (function == setStartCh_) {
		;
    } else if (function == setStopCh_) {
		;
    } else if (function == setScanCount_) {
		;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    return asynSuccess;
}


/*
**  asynFloat64 support
*/
asynStatus drvInficon::readFloat64 (asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    static const char *functionName = "readFloat64";

    *value = 0;

    if (function == boxTemp_) {
        ;
    } else if (function == anodePotential_) {
        ;
    } else if (function == emiCurrent_) {
        ;
    } else if (function == focusPotential_) {
        ;
    } else if (function == electEnergy_) {
        ;
    } else if (function == filPotential_) {
        ;
    } else if (function == filCurrent_) {
        ;
    } else if (function == emPotential_) {
        ;
    } else if (function == getPress_) {
        ;
    } else if (function == emVoltageMax_) {
        ;
    } else if (function == emVoltageMin_) {
        ;
    } else if (function == dwelMax_) {
        ;
    } else if (function == dwelMin_) {
        ;
    } else if (function == getChDwell_) {
        ;
    } else if (function == getChStartMass_) {
        ;
    } else if (function == getChStopMass_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }

    return asynSuccess;
}


asynStatus drvInficon::writeFloat64 (asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    static const char *functionName = "writeFloat64";

    if (function == setChDwell_) {
        ;
    } else if (function == setChStartMass_) {
        ;
    } else if (function == setChStopMass_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    return asynSuccess;
}



/*
**  asynFloat32Array support
*/
asynStatus drvInficon::readFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans, size_t *nactual)
{
    int function = pasynUser->reason;
	size_t i;
    static const char *functionName = "readFloat32Array";

    *nactual = 0;

    if (function == getScan_) {
        ;
    } else {
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
asynStatus drvInficon::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nactual, int *eomReason)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "readOctet";

    pasynManager->getAddr(pasynUser, &chNumber);
    *nactual = 0;

    if (function == ip_) {
        sprintf(request,"GET /mmsp/communication/ipAddress/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else if (function == mac_) {
        sprintf(request,"GET /mmsp/communication/macAddress/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else if (function == errorLog_) {
        sprintf(request,"GET /mmsp/communication/errorLog/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, errorLogCommand);
    } else if (function == sensName_) {
        sprintf(request,"GET /mmsp/sensorInfo/name/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else if (function == sensDesc_) {
        sprintf(request,"GET /mmsp/sensorInfo/description/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else if (function == sensSn_) {
        sprintf(request,"GET /mmsp/sensorInfo/serialNumber/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else if (function == getChMode_) {
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/channelMode"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseString(data_, value, nactual, stringCommand);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    callParamCallbacks(chNumber);
    return status;
}

asynStatus drvInficon::writeOctet (asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    int function = pasynUser->reason;
    static const char *functionName = "writeOctet";

    if (function == setChMode_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, pasynUser->reason);
        return asynError;
    }
    return asynSuccess;
}


/*
**  User functions
*/
asynStatus drvInficon::inficonReadWrite(const char *request, char *response)
{
    asynStatus status = asynSuccess;
    int eomReason;
    int autoConnect;
    size_t nwrite, nread;
    int requestSize = 0;
	int responseSize = 0;
	char httpResponse[HTTP_RESPONSE_SIZE];

    static const char *functionName = "inficonReadWrite";
  
/*    // If the Octet driver is not set for autoConnect then do connection management ourselves
    status = pasynManager->isAutoConnect(pasynUserOctet_, &autoConnect);
    if (!autoConnect) {
        // See if we are connected
        int yn = 0;
        status = pasynManager->isConnected(pasynUserOctet_, &yn);
        isConnected_ = (yn != 0) ? true : false;
         // If we have an I/O error or are disconnected then disconnect device and reconnect 
        if ((ioStatus_ != asynSuccess) || !isConnected_) {
            if (ioStatus_ != asynSuccess)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s port %s has I/O error\n",
                          driverName, functionName, this->portName);
            if (!isConnected_)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s port %s is disconnected\n",
                          driverName, functionName, this->portName);
            status = pasynCommonSyncIO->disconnectDevice(pasynUserCommon_);
            if (status == asynSuccess) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s::%s port %s disconnect device OK\n",
                          driverName, functionName, this->portName);
            } else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s port %s disconnect error=%s\n",
                          driverName, functionName, this->portName, pasynUserOctet_->errorMessage);
            }
            status = pasynCommonSyncIO->connectDevice(pasynUserCommon_);
            if (status == asynSuccess) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s::%s port %s connect device OK\n",
                          driverName, functionName, this->portName);
            } else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s port %s connect device error=%s\n",
                          driverName, functionName, this->portName, pasynUserOctet_->errorMessage);
                goto done;
            }
        }
    }
*/
    /* Do the write/read cycle */
	requestSize = (int)strlen(request);
	responseSize = HTTP_RESPONSE_SIZE;
    status = pasynOctetSyncIO->writeRead(pasynUserOctet_,
                                         request, requestSize,
                                         httpResponse, responseSize,
                                         DEVICE_RW_TIMEOUT,
                                         &nwrite, &nread, &eomReason);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s port %s called pasynOctetSyncIO->writeRead, status=%d, requestSize=%d, responseSize=%d, nwrite=%d, nread=%d, eomReason=%d request:%s\n",
              driverName, functionName, this->portName, status, requestSize, responseSize, (int)nwrite, (int)nread, eomReason, request);
/*
    if (status != prevIOStatus_) {
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                     "%s::%s port %s error calling writeRead,"
                     " error=%s, nwrite=%d, nread=%d\n",
                     driverName, functionName, this->portName,
                     pasynUserOctet_->errorMessage, (int)nwrite, (int)nread);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                     "%s::%s port %s writeRead status back to normal,"
                     " nwrite=%d, nread=%d\n",
                     driverName, functionName, this->portName,
                     (int)nwrite, (int)nread);
        }
        prevIOStatus_ = status;
    }
*/
    if (status == asynSuccess && nread > 0) {
        httpResponse[nread +1] = '\0';
    } else if (status == asynTimeout && nread > 0) {
        httpResponse[nread +1] = '\0';
        status = asynSuccess;
    } else if (status == asynError && nread > 0) {
        httpResponse[nread +1] = '\0';
        status = asynSuccess;
    } else {
        goto done;
    }

    /* Make sure the function code in the response is 200 OK */
    /* if function code not 200 set error and go to done*/
	static const char *matchString = "HTTP/1.1";
	const char *substring;
	int responseCode;
    if (httpResponse == NULL) {
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                 "%s::%s port %s http response is empty string\n",
                 driverName, functionName, this->portName);
        goto done;
	}

    substring = strstr(httpResponse, matchString);
    if (substring == NULL) {
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                 "%s::%s port %s HTTP response not valid\n",
                 driverName, functionName, this->portName);
        goto done;
	} else {
        sscanf(substring, "HTTP/1.1 %3d ", &responseCode);
    }
	
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s httpResponse:%s\n",
              driverName, functionName, httpResponse);
	
    if (responseCode == 200) {
        const char *jsonStart;
		const char *jsonStop;
        jsonStart = strchr(httpResponse,'{');
		jsonStop = strrchr(httpResponse,'}');
        if (jsonStart == NULL){
            status = asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                 "%s::%s port %s json data not valid\n",
                 driverName, functionName, this->portName);
            goto done;
        } else if (jsonStop == NULL){
            status = asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                 "%s::%s port %s json data not valid\n",
                 driverName, functionName, this->portName);
            goto done;
        } else {
            size_t len = jsonStop - jsonStart + 1;
			memcpy(response, jsonStart, len);
            response[len] = '\0';
	
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s parsed response:%s, len:%d\n",
              driverName, functionName, response, (int)len);
        }
    } else {
        response[0] = '\0';
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
             "%s::%s port %s error response code %3d\n",
             driverName, functionName, this->portName, responseCode);
    }

    done:
    return status;
}

asynStatus drvInficon::parseInt32(const char *jsonData, epicsInt32 *value, commandType_t commandType)
{
    static const char *functionName = "parseInt32";

    return asynSuccess;
}

asynStatus drvInficon::parseUInt32(const char *jsonData, epicsUInt32 *value, commandType_t commandType)
{
	char *stemp;
	bool btemp;
    static const char *functionName = "parseUInt32";

    try {
        json j = json::parse(jsonData);
		std::string jstring;

        switch (commandType) {
            case uint32Command:
                *value = j["data"];
                break;
            case setEmiCommand:
                jstring = j["data"];
                strcpy(stemp, jstring.c_str());
				if(strcmp(stemp,"Off") == 0) {
                    *value = 0;
                } else if (strcmp(stemp,"On") == 0) {
                    *value = 1;
                } else {
                    return asynError;
                }
                break;
            case setEmCommand:
                jstring = j["data"];
                strcpy(stemp, jstring.c_str());
				if(strcmp(stemp,"Off") == 0) {
                    *value = 0;
                } else if (strcmp(stemp,"On") == 0) {
                    *value = 1;
                } else {
                    return asynError;
                }
                break;
            case scanStatCommand:
                btemp = j["data"];
				*value = (btemp != false) ? 1 : 0;
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s::%s, port %s unknown command type %d\n",
                        driverName, functionName, this->portName, commandType);
                return asynError;
        }
    }
	catch (const json::parse_error& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s JSON error parsing unsigned int: %s\n", driverName, functionName, e.what());
        return asynError;
    }
    catch (std::exception e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s other error parsing unsigned int: %s\n", driverName, functionName, e.what());
        return asynError;
    }
	//printf("%s::%s JSON data:%s value:%d\n", driverName, functionName, jsonData, *value);
	//*data = value;
    return asynSuccess;
}

asynStatus drvInficon::parseFloat64(const char *jsonData, epicsFloat64 *value, commandType_t commandType)
{
    static const char *functionName = "parseFloat64";

    return asynSuccess;
}

asynStatus drvInficon::parseString(const char *jsonData, char *data, size_t *dataLen, commandType_t commandType)
{
	size_t stempLen;
    static const char *functionName = "parseString";

    try {
        json j = json::parse(jsonData);
		std::string jstring;
		
        switch (commandType) {
            case stringCommand:
                jstring = j["data"];
                strcpy(data, jstring.c_str());
                *dataLen = strlen(data);
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s::%s, port %s unknown command type %d\n",
                        driverName, functionName, this->portName, commandType);
                return asynError;
        }
    }
	catch (const json::parse_error& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s JSON error parsing string: %s\n", driverName, functionName, e.what());
        return asynError;
    }
    catch (std::exception e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s other error parsing string: %s\n", driverName, functionName, e.what());
        return asynError;
    }
    printf("%s::%s JSON data:%s string:%s, dataLength:%d\n", driverName, functionName, jsonData, data, (int)*dataLen);
    return asynSuccess;
}

asynStatus drvInficon::parseScan(const char *jsonData, double *data, int *scanSize, int *scannum)
{
    static const char *functionName = "parseScan";
	
    return asynSuccess;
}

asynStatus drvInficon::verifyConnection() {
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
	sprintf(hostInfo,"%s:%i TCP", ip, port);

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