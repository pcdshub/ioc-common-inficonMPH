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
                    5, /* maxAddr */
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
    createParam(INFICON_GET_COMM_PARAM_STRING,     asynParamOctet,          &getCommParam_);
    createParam(INFICON_IP_STRING,                 asynParamOctet,          &ip_);
    createParam(INFICON_MAC_STRING,                asynParamOctet,          &mac_);
    //createParam(INFICON_ERROR_LOG_STRING,          asynParamOctet,          &errorLog_);
    //General control parameters
    createParam(INFICON_EMI_ON_STRING,             asynParamUInt32Digital,  &emiOn_);
    createParam(INFICON_EM_ON_STRING,              asynParamUInt32Digital,  &emOn_);
    createParam(INFICON_RFGEN_ON_STRING,           asynParamUInt32Digital,  &rfGenOn_);
    createParam(INFICON_FAN_CNTRL_STRING,          asynParamUInt32Digital,  &fanCntrl_);
    createParam(INFICON_SHUTDOWN_STRING,           asynParamUInt32Digital,  &shutdown_);
    //Sensor info parameters
    createParam(INFICON_GET_SENS_INFO_STRING,      asynParamOctet,          &getSensInfo_);
    createParam(INFICON_SENS_NAME_STRING,          asynParamOctet,          &sensName_);
    createParam(INFICON_SENS_DESC_STRING,          asynParamOctet,          &sensDesc_);
    createParam(INFICON_SENS_SN_STRING,            asynParamUInt32Digital,  &sensSn_);
    //Status parameters
    createParam(INFICON_GET_DEV_STAT_STRING,       asynParamOctet,          &getDevStatus_);
    createParam(INFICON_SYST_STAT_STRING,          asynParamUInt32Digital,  &systStatus_);
    createParam(INFICON_HW_ERROR_STRING,           asynParamUInt32Digital,  &hwError_);
    createParam(INFICON_HW_WARN_STRING,            asynParamUInt32Digital,  &hwWarn_);
    createParam(INFICON_PWR_ON_TIME_STRING,        asynParamUInt32Digital,  &pwrOnTime_);
    createParam(INFICON_EMI_ON_TIME_STRING,        asynParamUInt32Digital,  &emiOnTime_);
    createParam(INFICON_EM_ON_TIME_STRING,         asynParamUInt32Digital,  &emOnTime_);
    createParam(INFICON_EMI_CML_ON_TIME_STRING,    asynParamUInt32Digital,  &emiCmlOnTime_);
    createParam(INFICON_EM_CML_ON_TIME_STRING,     asynParamUInt32Digital,  &emCmlOnTime_);
    createParam(INFICON_EMI_PRESS_TRIP_STRING,     asynParamUInt32Digital,  &emiPressTrip_);
    createParam(INFICON_EM_PRESS_TRIP_STRING,      asynParamUInt32Digital,  &emPressTrip_);
    //Diagnostic data parameters
    createParam(INFICON_GET_DIAG_DATA_STRING,      asynParamOctet,          &getDiagData_);
    createParam(INFICON_BOX_TEMP_STRING,           asynParamFloat64,        &boxTemp_);
    createParam(INFICON_ANODE_POTENTIAL_STRING,    asynParamUInt32Digital,  &anodePotential_);
    createParam(INFICON_EMI_CURRENT_STRING,        asynParamUInt32Digital,  &emiCurrent_);
    createParam(INFICON_FOCUS_POTENTIAL_STRING,    asynParamUInt32Digital,  &focusPotential_);
    createParam(INFICON_ELECT_ENERGY_STRING,       asynParamUInt32Digital,  &electEnergy_);
    createParam(INFICON_FIL_POTENTIAL_STRING,      asynParamUInt32Digital,  &filPotential_);
    createParam(INFICON_FIL_CURRENT_STRING,        asynParamUInt32Digital,  &filCurrent_);
    createParam(INFICON_EM_POTENTIAL_STRING,       asynParamUInt32Digital,  &emPotential_);
    //Measurement parameters
    createParam(INFICON_GET_PRESS_STRING,          asynParamFloat64,        &getPress_);
    createParam(INFICON_GET_SCAN_STRING,           asynParamFloat32Array,   &getScan_);
    //Scan info parameters
    createParam(INFICON_GET_SCAN_INFO_STRING,      asynParamOctet,          &getScanInfo_);
    createParam(INFICON_FIRST_SCAN_STRING,         asynParamUInt32Digital,  &firstScan_);
    createParam(INFICON_LAST_SCAN_STRING,          asynParamUInt32Digital,  &lastScan_);
    createParam(INFICON_CURRENT_SCAN_STRING,       asynParamUInt32Digital,  &currentScan_);
    createParam(INFICON_PPSCAN_STRING,             asynParamUInt32Digital,  &ppscan_);
    createParam(INFICON_SCAN_STAT_STRING,          asynParamUInt32Digital,  &scanStatus_);
    //Sensor detector parameters
    createParam(INFICON_GET_SENS_DETECT_STRING,    asynParamOctet,          &getSensDetect_);
    createParam(INFICON_EM_VOLTAGE_MAX_STRING,     asynParamUInt32Digital,  &emVMax_);
    createParam(INFICON_EM_VOLTAGE_MIN_STRING,     asynParamUInt32Digital,  &emVMin_);
    createParam(INFICON_EM_VOLTAGE_STRING,         asynParamUInt32Digital,  &emV_);
    createParam(INFICON_EM_GAIN_STRING,            asynParamFloat64,        &emGain_);
    createParam(INFICON_EM_GAIN_MASS_STRING,       asynParamUInt32Digital,  &emGainMass_);
    //Sensor filter parameters
    createParam(INFICON_GET_SENS_FILT_STRING,      asynParamOctet,          &getSensFilt_);
    createParam(INFICON_MASS_MAX_STRING,           asynParamFloat64,        &massMax_);
    createParam(INFICON_MASS_MIN_STRING,           asynParamFloat64,        &massMin_);
    createParam(INFICON_DWELL_MAX_STRING,          asynParamUInt32Digital,  &dwelMax_);
    createParam(INFICON_DWELL_MIN_STRING,          asynParamUInt32Digital,  &dwelMin_);
    //Sensor Ion Source parameters
    createParam(INFICON_GET_SENS_ION_SRC_STRING,   asynParamOctet,          &getSensIonSrc_);
    createParam(INFICON_FIL_SEL_STRING,            asynParamUInt32Digital,  &filSel_);
    createParam(INFICON_EMI_LEVEL_STRING,          asynParamUInt32Digital,  &emiLevel_);
    createParam(INFICON_OPT_TYPE_STRING,           asynParamUInt32Digital,  &optType_);
    //Scan setup parameters
    createParam(INFICON_GET_CH_SCAN_SETUP_STRING,  asynParamOctet,          &getChScanSetup_);
    createParam(INFICON_SET_CH_SCAN_SETUP_STRING,  asynParamOctet,          &setChScanSetup_);
    createParam(INFICON_START_STOP_CH_STRING,      asynParamUInt32Digital,  &startStopCh_);
    createParam(INFICON_CH_MODE_STRING,            asynParamOctet,          &chMode_);
    createParam(INFICON_CH_PPAMU_STRING,           asynParamUInt32Digital,  &chPpamu_);
    createParam(INFICON_CH_DWELL_STRING,           asynParamUInt32Digital,  &chDwell_);
    createParam(INFICON_CH_START_MASS_STRING,      asynParamFloat64,        &chStartMass_);
    createParam(INFICON_CH_STOP_MASS_STRING,       asynParamFloat64,        &chStopMass_);
    createParam(INFICON_SCAN_COUNT_STRING,         asynParamInt32,          &scanCount_);
    createParam(INFICON_SCAN_MODE_STRING,          asynParamInt32,          &scanMode_);
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
	ipConfigureStatus = drvAsynIPPortConfigure(octetPortName_, hostInfo_, 0, 0, 0);

	if (ipConfigureStatus) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, Unable to configure drvAsynIPPort %s",
            driverName, functionName, octetPortName_);
        return;
	}

    /*Allocate memory*/
    data_ = (char*)callocMustSucceed(HTTP_RESPONSE_SIZE, sizeof(char), functionName);

    commParams_ = new commParamStruct;
    genCntrl_ = new genCntrlStruct;
    sensInfo_ = new sensInfoStruct;
    devStatus_ = new devStatusStruct;
    diagData_ = new diagDataStruct;
    scanInfo_ = new scanInfoStruct;
    sensDetect_ = new sensDetectStruct;
    sensFilt_ = new sensFiltStruct;
    chScanSetup_ = new chScanSetupStruct[5];
    scanData_ = new scanDataStruct;
    sensIonSource_ = new sensIonSourceStruct;

    /* Connect to asyn octet port with asynOctetSyncIO */
    status = pasynOctetSyncIO->connect(octetPortName_, 0, &pasynUserOctet_, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s port %s can't connect to asynOctet on Octet server %s.\n",
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
	if (data_)
		free(data_);
	
	pasynManager->disconnect(pasynUserOctet_);
    pasynManager->freeAsynUser(pasynUserOctet_);
    pasynUserOctet_ = NULL;

    delete commParams_;
    delete genCntrl_;
    delete sensInfo_;
    delete devStatus_;
    delete diagData_;
    delete scanInfo_;
    delete sensDetect_;
    delete sensFilt_;
    delete chScanSetup_;
    delete scanData_;
    delete sensIonSource_;
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

/*
**  asynUInt32D support
*/
asynStatus drvInficon::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask)
{
    //int function = pasynUser->reason;
    //static const char *functionName = "readUInt32D";
	
    *value = 0;

	return asynSuccess;
}


asynStatus drvInficon::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    int function = pasynUser->reason;
    //asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "writeUInt32D";
	
    pasynManager->getAddr(pasynUser, &chNumber);

    setUIntDigitalParam(chNumber, function, value, mask);
    if (function == emiOn_) {
        sprintf(request,"GET /mmsp/generalControl/setEmission/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
//maybe add emissionStandby command? This target puts the ion source filament in standby, a warm but not emitting state.
    } else if (function == emOn_) {
        sprintf(request,"GET /mmsp/generalControl/setEM/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == rfGenOn_) {
        sprintf(request,"GET /mmsp/generalControl/rfGeneratorSet/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == shutdown_) {
        sprintf(request,"GET /mmsp/generalControl/shutdown/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == emV_) {
        sprintf(request,"GET /mmsp/sensorDetector/emVoltage/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == startStopCh_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/set?startChannel=%d&stopChannel=%d\r\n"
        "\r\n", chNumber, chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == chPpamu_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/ppamu/set?%d\r\n"
        "\r\n", chNumber, value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == chDwell_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/dwell/set?%d\r\n"
        "\r\n", chNumber, value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == scanStart_) {
        sprintf(request,"GET /mmsp/scanSetup/scanStart/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == scanStop_) {
        if (value == 1) {
            sprintf(request,"GET /mmsp/scanSetup/scanStop/set?EndOfScan\r\n"
                    "\r\n");
        } else {
            sprintf(request,"GET /mmsp/scanSetup/scanStop/set?Immediately\r\n"
                    "\r\n");
        }
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == filSel_) {
        sprintf(request,"GET /mmsp/sensorIonSource/filamentSelected/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    callParamCallbacks(chNumber);
    return asynSuccess;
}


/*
**  asynInt32 support
*/
asynStatus drvInficon::readInt32 (asynUser *pasynUser, epicsInt32 *value)
{
    //int function = pasynUser->reason;
    //static const char *functionName = "readInt32";

    *value = 0;

    return asynSuccess;
}


asynStatus drvInficon::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "writeInt32";
	
    pasynManager->getAddr(pasynUser, &chNumber);

    setIntegerParam(chNumber, function, value);
    if (function == scanCount_) {
        sprintf(request,"GET /mmsp/scanSetup/scanCount/set?%d\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
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
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
    static const char *functionName = "readFloat64";

    *value = 0;

    if (function == getPress_) {
        sprintf(request,"GET /mmsp/measurement/totalPressure/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parsePressure(data_, value);
        if (status != asynSuccess) return(status);
        //printf("%s::%s pressure:%f json:%s\n", driverName, functionName, *value, data_);
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
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "writeFloat64";
	
    pasynManager->getAddr(pasynUser, &chNumber);

    setDoubleParam(chNumber, function, value);
    if (function == chStartMass_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/startMass/set?%.2f\r\n"
        "\r\n", chNumber, value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == chStopMass_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/stopMass/set?%.2f\r\n"
        "\r\n", chNumber, value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == emGain_) {
        sprintf(request,"GET /mmsp/sensorDetector/emGain/set?%.2f\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == emGainMass_) {
        sprintf(request,"GET /mmsp/sensorDetector/emGainMass/set?%.2f\r\n"
        "\r\n", value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
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
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
	int scanMode;
    static const char *functionName = "readFloat32Array";

    *nactual = 0;
	
    if (function == getScan_) {
        getIntegerParam(scanMode_, &scanMode);
        sprintf(request,"GET /mmsp/measurement/scans/%d/get\r\n"
        "\r\n", scanMode);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
		status = parseScan(data_, scanData_);
        if (status != asynSuccess) return(status);
		*nactual = scanData_->actualScanSize;
		memcpy(data, scanData_->scanValues, scanData_->actualScanSize * sizeof(float));
        //printf("%s::%s array0:%e array1:%e array2:%e nElements:%d scanNum:%d\n", driverName, functionName, data[0], data[1], scanData_->scanValues[2500], scanData_->actualScanSize, scanData_->scanNumber);
        //doCallbacksFloat32Array(scanData_->scanValues, scanData_->actualScanSize, getScan_, 0);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
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

    if (function == getCommParam_) {
        sprintf(request,"GET /mmsp/communication/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseCommParam(data_, commParams_);
        if (status != asynSuccess) return(status);
        setStringParam(ip_, commParams_->ip);
        setStringParam(mac_, commParams_->mac);
        //printf("%s::%s status:%d ip:%s mac:%s\n", driverName, functionName, status, commParams_->ip, commParams_->mac);
    } else if (function == getSensInfo_) {
        sprintf(request,"GET /mmsp/sensorInfo/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensInfo(data_, sensInfo_);
        if (status != asynSuccess) return(status);
        setStringParam(sensName_, sensInfo_->sensName);
        setStringParam(sensDesc_, sensInfo_->sensDesc);
        setUIntDigitalParam(sensSn_, sensInfo_->sensSN, 0xFFFFFFFF);
        //printf("%s::%s status:%d serial:%d name:%s desc:%s\n", driverName, functionName, status, sensInfo_->sensSN, sensInfo_->sensName, sensInfo_->sensDesc);
    } else if (function == getDevStatus_) {
        sprintf(request,"GET /mmsp/status/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseDevStatus(data_, devStatus_);
        if (status != asynSuccess) return(status);
        setUIntDigitalParam(systStatus_, devStatus_->systStatus, 0xFFFFFFFF);
        setUIntDigitalParam(hwError_, devStatus_->hwError, 0xFFFFFFFF);
        setUIntDigitalParam(hwWarn_, devStatus_->hwWarn, 0xFFFFFFFF);
        setUIntDigitalParam(pwrOnTime_, devStatus_->pwrOnTime, 0xFFFFFFFF);
        setUIntDigitalParam(emiOnTime_, devStatus_->emiOnTime, 0xFFFFFFFF);
        setUIntDigitalParam(emOnTime_, devStatus_->emOnTime, 0xFFFFFFFF);
        setUIntDigitalParam(emCmlOnTime_, devStatus_->emCmlOnTime, 0xFFFFFFFF);
        setUIntDigitalParam(emPressTrip_, devStatus_->emPressTrip, 0xFFFFFFFF);
    } else if (function == getDiagData_) {
        sprintf(request,"GET /mmsp/diagnosticData/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseDiagData(data_, diagData_);
        if (status != asynSuccess) return(status);
        setDoubleParam(boxTemp_, diagData_->boxTemp);
        setUIntDigitalParam(anodePotential_, diagData_->anodePot, 0xFFFFFFFF);
        setUIntDigitalParam(emiCurrent_, diagData_->emiCurrent, 0xFFFFFFFF);
        setUIntDigitalParam(focusPotential_, diagData_->focusPot, 0xFFFFFFFF);
        setUIntDigitalParam(electEnergy_, diagData_->electEng, 0xFFFFFFFF);
        setUIntDigitalParam(filPotential_, diagData_->filPot, 0xFFFFFFFF);
        setUIntDigitalParam(filCurrent_, diagData_->filCurrent, 0xFFFFFFFF);
        setUIntDigitalParam(emPotential_, diagData_->emPot, 0xFFFFFFFF);
        //printf("%s::%s boxTemp:%.3f anodePot:%d filCurrent:%d\n", driverName, functionName, diagData_->boxTemp, diagData_->anodePot, diagData_->filCurrent);
    } else if (function == getScanInfo_) {
        sprintf(request,"GET /mmsp/scanInfo/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseScanInfo(data_, scanInfo_);
        if (status != asynSuccess) return(status);
        setUIntDigitalParam(firstScan_, scanInfo_->firstScan, 0xFFFFFFFF);
        setUIntDigitalParam(lastScan_, scanInfo_->lastScan, 0xFFFFFFFF);
        setUIntDigitalParam(currentScan_, scanInfo_->currScan, 0xFFFFFFFF);
        setUIntDigitalParam(ppscan_, scanInfo_->ppScan, 0xFFFFFFFF);
        setUIntDigitalParam(scanStatus_, scanInfo_->scanStatus, 0xFFFFFFFF);
        //printf("%s::%s firstSan:%d currScan:%d scanStatus:%d\n", driverName, functionName, scanInfo_->firstScan, scanInfo_->currScan, scanInfo_->scanStatus);
    } else if (function == getSensDetect_) {
        sprintf(request,"GET /mmsp/sensorDetector/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensDetect(data_, sensDetect_);
        if (status != asynSuccess) return(status);
        setUIntDigitalParam(emVMax_, sensDetect_->emVMax, 0xFFFFFFFF);
        setUIntDigitalParam(emVMin_, sensDetect_->emVMin, 0xFFFFFFFF);
        setUIntDigitalParam(emV_, sensDetect_->emV, 0xFFFFFFFF);
        setDoubleParam(emGain_, sensDetect_->emGain);
        setUIntDigitalParam(emGainMass_, sensDetect_->emGainMass, 0xFFFFFFFF);
        //printf("%s::%s emVMax:%d emV:%d emGain:%.3f\n", driverName, functionName, sensDetect_->emVMax, sensDetect_->emV, sensDetect_->emGain);
    } else if (function == getSensFilt_) {
        sprintf(request,"GET /mmsp/sensorFilter/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensFilt(data_, sensFilt_);
        if (status != asynSuccess) return(status);
        setDoubleParam(massMax_, sensFilt_->massMax);
        setDoubleParam(massMin_, sensFilt_->massMin);
        setUIntDigitalParam(dwelMax_, sensFilt_->dwellMax, 0xFFFFFFFF);
        setUIntDigitalParam(dwelMin_, sensFilt_->dwellMin, 0xFFFFFFFF);
        //printf("%s::%s massMax:%.3f massMin:%.3f dwellMin:%d\n", driverName, functionName, sensFilt_->massMax, sensFilt_->massMin, sensFilt_->dwellMin);
    } else if (function == getSensIonSrc_) {
        sprintf(request,"GET /mmsp/sensorIonSource/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensIonSource(data_, sensIonSource_);
        if (status != asynSuccess) return(status);
        setUIntDigitalParam(filSel_, sensIonSource_->filSel, 0xFFFFFFFF);
        setUIntDigitalParam(emiLevel_, sensIonSource_->emiLevel, 0xFFFFFFFF);
        setUIntDigitalParam(optType_, sensIonSource_->optType, 0xFFFFFFFF);
    } else if (function == getChScanSetup_) {
		
        if (chNumber < 1 || chNumber > MAX_CHANNELS) 
            return asynError;

        sprintf(request,"GET /mmsp/scanSetup/channel/%d/get\r\n"
                        "\r\n", 
                        chNumber);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseChScanSetup(data_, chScanSetup_, chNumber);
        if (status != asynSuccess) return(status);
        setStringParam(chNumber, chMode_, chScanSetup_[chNumber].chMode);
        setDoubleParam(chNumber, chStartMass_, chScanSetup_[chNumber].chStartMass);
        setDoubleParam(chNumber, chStopMass_, chScanSetup_[chNumber].chStopMass);	
        setUIntDigitalParam(chNumber, chDwell_, chScanSetup_[chNumber].chDwell, 0xFFFFFFFF);
        setUIntDigitalParam(chNumber, chPpamu_, chScanSetup_[chNumber].chPpamu, 0xFFFFFFFF);
        //printf("%s::%s chNumber:%d chMode:%s chDwel:%d chppamu:%d chstartMass:%f chstopMass:%f\n", driverName, functionName, chNumber, chScanSetup_[chNumber].chMode, chScanSetup_[chNumber].chDwell, chScanSetup_[chNumber].chPpamu, chScanSetup_[chNumber].chStartMass, chScanSetup_[chNumber].chStopMass);
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
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
	//std::string tempChMode;
    //char chMode[32];
	//double startMass;
	//double stopMass;
	//unsigned int ppamu;
	//unsigned int dwell;
    static const char *functionName = "writeOctet";

    pasynManager->getAddr(pasynUser, &chNumber);

    *nActual = strlen(value);
    setStringParam(chNumber, function, value);
    if (function == chMode_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/channelMode/set?%s\r\n"
        "\r\n", chNumber, value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == setChScanSetup_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        /*getStringParam(chNumber, chMode_, tempChMode);
        strcpy(chMode, tempChMode.c_str());
        getDoubleParam(chNumber, chStartMass_, &startMass);
        getDoubleParam(chNumber, chStopMass_, &stopMass);
        getUIntDigitalParam(chNumber, chPpamu_, &ppamu, 0xFFFFFFFF);
        getUIntDigitalParam(chNumber, chDwell_, &dwell, 0xFFFFFFFF);
		if (strcmp(chMode, "Single") == 0) {
            sprintf(request,"GET /mmsp/scanSetup/channels/%d/set?channelMode=%s&startMass=%.2f&dwell=%d&enabled=True\r\n"
            "\r\n", chNumber, chMode, startMass, dwell);
        } else {
            sprintf(request,"GET /mmsp/scanSetup/channels/%d/set?channelMode=%s&startMass=%.2f&stopMass=%.2f&ppamu=%d&dwell=%d&enabled=True\r\n"
            "\r\n", chNumber, chMode, startMass, stopMass, ppamu, dwell);
        }*/
        sprintf(request,"GET /mmsp/scanSetup/channels/%d/set?enabled=True\r\n"
                "\r\n",
                chNumber);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else {
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
asynStatus drvInficon::inficonReadWrite(const char *request, char *response)
{
    asynStatus status = asynSuccess;
    int eomReason;
    //int autoConnect;
    size_t nwrite, nread;
    int requestSize = 0;
	//int responseSize = 0;
	char httpResponse[HTTP_RESPONSE_SIZE];

    static const char *functionName = "inficonReadWrite";
	
    memset(httpResponse, '\0', HTTP_RESPONSE_SIZE);
  
    /*// If the Octet driver is not set for autoConnect then do connection management ourselves
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
    }*/

    /*asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s::%s port %s starting with read\n",
            driverName, functionName, this->portName);*/

    /* Do the write/read cycle */
	requestSize = (int)strlen(request);
	//responseSize = HTTP_RESPONSE_SIZE;
    status = pasynOctetSyncIO->writeRead(pasynUserOctet_,
                                         request, requestSize,
                                         httpResponse, HTTP_RESPONSE_SIZE,
                                         DEVICE_RW_TIMEOUT,
                                         &nwrite, &nread, &eomReason);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s port %s called pasynOctetSyncIO->writeRead, status=%d, requestSize=%d, nwrite=%d, nread=%d, eomReason=%d request:%s\n",
              driverName, functionName, this->portName, status, requestSize, (int)nwrite, (int)nread, eomReason, request);

    /*if (status != prevIOStatus_) {
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
    }*/

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
    /* If function code not 200 set error and go to done*/
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
        sscanf(substring, "HTTP/1.1 %3d", &responseCode);
    }
	
    /*asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s httpResponse:%s\nhttp response code:%d\n",
              driverName, functionName, httpResponse, responseCode);*/
	
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
	
            /*asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s parsed response:%s, len:%d\n",
              driverName, functionName, response, (int)len);*/
        }
    } else {
        response[0] = '\0';
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
             "%s::%s port %s error response code %3d\n",
             driverName, functionName, this->portName, responseCode);
    }

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s::%s status=%d\n",
              driverName, functionName, status);
    done:
    return status;
}


asynStatus drvInficon::parseCommParam(const char *jsonData, commParamStruct *commParam)
{
    static const char *functionName = "parseCommParam";
    //printf("%s::%s JSON data:%s\n", driverName, functionName, jsonData);

    try {
        json j = json::parse(jsonData);
        std::string jstring;
		
		jstring = j["data"]["ipAddress"];
        strcpy(commParam->ip, jstring.c_str());
		jstring = j["data"]["macAddress"];
        strcpy(commParam->mac, jstring.c_str());
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

    //printf("%s::%s ip:%s mac:%s\n", driverName, functionName, commParam->ip, commParam->mac);
    return asynSuccess;
}

asynStatus drvInficon::parseSensInfo(const char *jsonData, sensInfoStruct *sensInfo)
{
    static const char *functionName = "parseSensInfo";

    try {
        json j = json::parse(jsonData);
        std::string jstring;

		jstring = j["data"]["name"];
        strcpy(sensInfo->sensName, jstring.c_str());
		jstring = j["data"]["description"];
        strcpy(sensInfo->sensDesc, jstring.c_str());
        sensInfo->sensSN = j["data"]["serialNumber"];
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
    //printf("%s::%s serial:%d name:%s desc:%s\n", driverName, functionName, sensInfo->serialNumber, sensInfo->sensName, sensInfo->sensDesc);
    return asynSuccess;
}

asynStatus drvInficon::parseDevStatus(const char *jsonData, devStatusStruct *devStatus)
{
    static const char *functionName = "parseDevStatus";

	char jsonDataSubstring[1500];
    const char *tempJsonData = jsonData;
    const char *cutAt;
    const char *cutTo;

	memset(jsonDataSubstring, '\0', 1500);

    cutAt = strstr(tempJsonData,"peakfind");
    cutTo = strstr(tempJsonData,"filaments");

    if(cutAt != NULL && cutTo != NULL) {
        size_t len = cutAt - tempJsonData - 1;
        strncpy(jsonDataSubstring, tempJsonData, len);
		jsonDataSubstring[len] = '\0';
		len = strlen(jsonDataSubstring);
		strcpy(jsonDataSubstring + len, cutTo - 1);
		//len = strlen(jsonDataSubstring);
        //printf("%s::%s len:%d, substring:%s\n", driverName, functionName, (int)len, jsonDataSubstring);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s JSON data corrupted\n", driverName, functionName);
        return asynError;
    }

    try {
        json j = json::parse(jsonDataSubstring);

        devStatus->systStatus = j["data"]["systemStatus"];
        devStatus->hwError = j["data"]["hardwareErrors"];
        devStatus->hwWarn = j["data"]["hardwareWarnings"];
        devStatus->pwrOnTime = j["data"]["powerSupplyPowerOnTime"];
        devStatus->emiOnTime = j["data"]["emissionStretch"];
        devStatus->emOnTime = j["data"]["emStretch"];
        devStatus->emCmlOnTime = j["data"]["emOnTime"];
        devStatus->emPressTrip = j["data"]["emPressTrip"];
        auto filaments = j["data"]["filaments"];
        unsigned int emiCmlOnTime = 0;
        for (json::iterator i = filaments.begin(); i != filaments.end(); i++) {
            devStatus->filament[*i].id = filaments["@id"];
            emiCmlOnTime = filaments["emisOnTime"];
            devStatus->filament[*i].emiCmlOnTime = emiCmlOnTime/3600;
            devStatus->filament[*i].emiPressTrip = filaments["emisPressTrip"];
            printf("%s::%s id:%d, emiCmlOnTime:%.1f, emiPressTrip:%d\n", driverName, functionName, devStatus->filament[*i].id, devStatus->filament[*i].emiCmlOnTime, devStatus->filament[*i].emiPressTrip);
        }
        //add emi press trip for every filamenet and cumulative power on time
        //printf("%s::%s systStatus:%d, pwrOnTime:%d\n", driverName, functionName, devStatus->systStatus, devStatus->pwrOnTime);
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

    //printf("%s::%s maxMass:%d\n", driverName, functionName, elecInfo->massMax);
    return asynSuccess;
}

asynStatus drvInficon::parseDiagData(const char *jsonData, diagDataStruct *diagData)
{
    static const char *functionName = "parseDiagData";

    try {
        json j = json::parse(jsonData);
		
        diagData->boxTemp = j["data"]["internalBoxTemperature"];
        diagData->anodePot = j["data"]["anodePotential"];
        diagData->focusPot = j["data"]["focusPotential"];
        diagData->filPot = j["data"]["filamentPotential"];
        diagData->emPot = j["data"]["electronMultiplierPotential"];
        diagData->emiCurrent = j["data"]["emissionCurrent"];
        diagData->filCurrent = j["data"]["filamentCurrent"];
        diagData->electEng = j["data"]["electronEnergy"];
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

    //printf("%s::%s maxMass:%d\n", driverName, functionName, elecInfo->massMax);
    return asynSuccess;
}

asynStatus drvInficon::parseScanInfo(const char *jsonData, scanInfoStruct *scanInfo)
{
    static const char *functionName = "parseScanInfo";

    try {
        json j = json::parse(jsonData);
		bool btemp;

        scanInfo->firstScan = j["data"]["firstScan"];
        scanInfo->lastScan = j["data"]["lastScan"];
        scanInfo->currScan = j["data"]["currentScan"];
        scanInfo->ppScan = j["data"]["pointsPerScan"];
        btemp = j["data"]["scanning"];		
        scanInfo->scanStatus = (btemp != false) ? 1 : 0;
        //printf("%s::%s scanStatus:%d\n", driverName, functionName, btemp);
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
    return asynSuccess;
}

asynStatus drvInficon::parseSensDetect(const char *jsonData, sensDetectStruct *sensDetect)
{
    static const char *functionName = "parseSensDetect";

    try {
        json j = json::parse(jsonData);
        unsigned int emGainMass;

        sensDetect->emVMax = j["data"]["emVoltageMax"];
        sensDetect->emVMin = j["data"]["emVoltageMin"];
        sensDetect->emV = j["data"]["emVoltage"];
        sensDetect->emGain = j["data"]["emGain"];
        emGainMass = j["data"]["emGainMass"];
        sensDetect->emGainMass = emGainMass/100;
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

    //printf("%s::%s maxMass:%d\n", driverName, functionName, elecInfo->massMax);
    return asynSuccess;
}

asynStatus drvInficon::parseSensFilt(const char *jsonData, sensFiltStruct *sensFilt)
{
    static const char *functionName = "parseSensFilt";

    try {
        json j = json::parse(jsonData);

        sensFilt->massMax = j["data"]["massMax"];
        sensFilt->massMin = j["data"]["massMin"];
        sensFilt->dwellMax = j["data"]["dwellMax"];
        sensFilt->dwellMin = j["data"]["dwellMin"];
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

    //printf("%s::%s maxMass:%d\n", driverName, functionName, elecInfo->massMax);
    return asynSuccess;
}

asynStatus drvInficon::parseChScanSetup(const char *jsonData, chScanSetupStruct *chScanSetup, unsigned int chNumber)
{
    static const char *functionName = "parseChScanSetup";

    try {
        json j = json::parse(jsonData);
        std::string jstring;

		jstring = j["data"][0]["channelMode"];
        strcpy(chScanSetup[chNumber].chMode, jstring.c_str());
        chScanSetup[chNumber].chStartMass = j["data"][0]["startMass"];
        chScanSetup[chNumber].chStopMass = j["data"][0]["stopMass"];
        chScanSetup[chNumber].chDwell = j["data"][0]["dwell"];
        chScanSetup[chNumber].chPpamu = j["data"][0]["ppamu"];
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

    //printf("%s::%s startMass:%.2f stopMass:%.2f\n", driverName, functionName, startMass, stopMass);
    return asynSuccess;
}

asynStatus drvInficon::parseScan(const char *jsonData, scanDataStruct *scanData)
{
    static const char *functionName = "parseScan";

    try {
        json j = json::parse(jsonData);

        scanData->scanSize = j["data"]["scansize"];
		scanData->actualScanSize = j["data"]["values"].size();
        scanData->scanNumber = j["data"]["scannum"];
        std::vector <float> values(MAX_SCAN_SIZE);
		values = j["data"]["values"].get<std::vector<float>>();
		memcpy(scanData->scanValues, &values[0], scanData->actualScanSize * sizeof(float));
        //printf("%s::%s value0:%e value1:%e copysize:%d\n", driverName, functionName, values[0], scanData->scanValues[2500], scanData->actualScanSize);
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
    return asynSuccess;
}

asynStatus drvInficon::parsePressure(const char *jsonData, double *value)
{
    static const char *functionName = "parsePressure";

    try {
        json j = json::parse(jsonData);

        *value = j["data"];
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
    return asynSuccess;
}

asynStatus drvInficon::parseSensIonSource(const char *jsonData, sensIonSourceStruct *sensIonSource)
{
    static const char *functionName = "parseSensIonSource";

	char jsonDataSubstring[8000];
	char stemp[32];
    const char *tempJsonData = jsonData;
    const char *cutAt;
    const char *cutTo;

	memset(jsonDataSubstring, '\0', 8000);
	memset(stemp, '\0', 32);	

    cutAt = strstr(tempJsonData,"ionSource");
    cutTo = strstr(tempJsonData,"calIndex");

    if(cutAt != NULL && cutTo != NULL) {
        size_t len = cutAt - tempJsonData - 1;
        strncpy(jsonDataSubstring, tempJsonData, len);
		jsonDataSubstring[len] = '\0';
		len = strlen(jsonDataSubstring);
		strcpy(jsonDataSubstring + len, cutTo - 1);
		//len = strlen(jsonDataSubstring);
        //printf("%s::%s len:%d, substring:%s\n", driverName, functionName, (int)len, jsonDataSubstring);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s JSON data corrupted\n", driverName, functionName);
        return asynError;
    }

    try {
        json j = json::parse(jsonDataSubstring);
        std::string jstring;

        sensIonSource->filSel = j["data"]["filamentSelected"];

        jstring = j["data"]["emissionLevel"];
        strcpy(stemp, jstring.c_str());
	    if(strcmp(stemp,"Lo") == 0) {
            sensIonSource->emiLevel = 0;
        } else if (strcmp(stemp,"Hi") == 0) {
            sensIonSource->emiLevel = 1;
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s JSON error parsing emiss level string: %s\n", driverName, functionName, stemp);
            return asynError;
        }

	    memset(stemp, '\0', 32);
        jstring = j["data"]["optimizationType"];
        strcpy(stemp, jstring.c_str());
	    if(strcmp(stemp,"Linearity") == 0) {
            sensIonSource->optType = 0;
        } else if (strcmp(stemp,"Sensitivity") == 0) {
            sensIonSource->optType = 1;
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s JSON error parsing optimization type string: %s\n", driverName, functionName, stemp);
            return asynError;
        }
        //printf("%s::%s filSel:%d, emiLevel:%d, optType:%d\n", driverName, functionName, sensIonSource->filSel, sensIonSource->emiLevel, sensIonSource->optType);
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