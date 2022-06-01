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
    createParam(INFICON_SET_EMI_STRING,            asynParamUInt32Digital,  &setEmi_);
    createParam(INFICON_SET_EM_STRING,             asynParamUInt32Digital,  &setEm_);
    createParam(INFICON_SET_RFGEN_STRING,          asynParamUInt32Digital,  &setRfGen_);
    createParam(INFICON_GET_FAN_STRING,            asynParamUInt32Digital,  &getFan_);
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
    createParam(INFICON_SCAN_STAT_STRING,          asynParamUInt32Digital,  &scanStat);
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
    //Scan setup parameters
    createParam(INFICON_GET_CH_SCAN_SETUP_STRING,  asynParamOctet,          &getChScanSetup_);
    createParam(INFICON_SET_START_CH_STRING,       asynParamUInt32Digital,  &setStartCh_);
    createParam(INFICON_GET_START_CH_STRING,       asynParamUInt32Digital,  &getStartCh_);
    createParam(INFICON_SET_STOP_CH_STRING,        asynParamUInt32Digital,  &setStopCh_);
    createParam(INFICON_GET_STOP_CH_STRING,        asynParamUInt32Digital,  &getStopCh_);
    createParam(INFICON_SET_CH_MODE_STRING,        asynParamOctet,          &setChMode_); //octet
    createParam(INFICON_GET_CH_MODE_STRING,        asynParamOctet,          &getChMode_); //octet
    createParam(INFICON_SET_CH_PPAMU_STRING,       asynParamUInt32Digital,  &setChPpamu_);
    createParam(INFICON_GET_CH_PPAMU_STRING,       asynParamUInt32Digital,  &getChPpamu_);
    createParam(INFICON_SET_CH_DWELL_STRING,       asynParamUInt32Digital,  &setChDwell_);
    createParam(INFICON_GET_CH_DWELL_STRING,       asynParamUInt32Digital,  &getChDwell_);
    createParam(INFICON_SET_CH_START_MASS_STRING,  asynParamUInt32Digital,  &setChStartMass_);
    createParam(INFICON_GET_CH_START_MASS_STRING,  asynParamUInt32Digital,  &getChStartMass_);
    createParam(INFICON_SET_CH_STOP_MASS_STRING,   asynParamUInt32Digital,  &setChStopMass_);
    createParam(INFICON_GET_CH_STOP_MASS_STRING,   asynParamUInt32Digital,  &getChStopMass_);
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
    }  else if (function == pwrOnTime_) {
        sprintf(request,"GET /mmsp/status/powerSupplyPowerOnTime/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emiOnTime_) {
        sprintf(request,"GET /mmsp/status/emissionStretch/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emOnTime_) {
        sprintf(request,"GET /mmsp/status/emStretch/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emiCmlOnTime_) {
        sprintf(request,"GET /mmsp/status/filaments/%d/emisOnTime/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emCmlOnTime_) {
        sprintf(request,"GET /mmsp/status/emOnTime/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emiPressTrip_) {
        sprintf(request,"GET /mmsp/status/filaments/%d/emisPressTrip/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == anodePotential_) {
        sprintf(request,"GET /mmsp/diagnosticData/anodePotential/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emiCurrent_) {
        sprintf(request,"GET /mmsp/diagnosticData/emissionCurrent/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == focusPotential_) {
        sprintf(request,"GET /mmsp/diagnosticData/focusPotential/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == electEnergy_) {
        sprintf(request,"GET /mmsp/diagnosticData/electronEnergy/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == filPotential_) {
        sprintf(request,"GET /mmsp/diagnosticData/filamentPotential/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == filCurrent_) {
        sprintf(request,"GET /mmsp/diagnosticData/filamentCurrent/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == emPotential_) {
        sprintf(request,"GET /mmsp/diagnosticData/electronMultiplierPotential/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == ppscan_) {
        sprintf(request,"GET /mmsp/scanInfo/pointsPerScan/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == scanStat) {
        sprintf(request,"GET /mmsp/scanInfo/scanning/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, scanStatCommand);
    } else if (function == dwelMax_) {
        sprintf(request,"GET /mmsp/sensorFilter/massMax/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == dwelMin_) {
        sprintf(request,"GET /mmsp/sensorFilter/massMin/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == getStartCh_) {
        ;
    } else if (function == getStopCh_) {
        ;
    } else if (function == getChPpamu_) {
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/ppamu/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
    } else if (function == getChDwell_) {
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/dwell/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseUInt32(data_, value, uint32Command);
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
    //printf("%s::%s status:%d chNumber:%d\n", driverName, functionName, status, chNumber);
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
    } else if (function == setStartCh_) {
		;
    } else if (function == setStopCh_) {
		;
    } else if (function == setChPpamu_) {
        ;
    } else if (function == setChDwell_) {
        ;
    } else if (function == setChStartMass_) {
        ;
    } else if (function == setChStopMass_) {
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
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    static const char *functionName = "readInt32";

	pasynManager->getAddr(pasynUser, &chNumber);
    *value = 0;

    if (function == firstScan_) {
        sprintf(request,"GET /mmsp/scanInfo/firstScan/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseInt32(data_, value, int32Command);
    } else if (function == lastScan_) {
        sprintf(request,"GET /mmsp/scanInfo/lastScan/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseInt32(data_, value, int32Command);
    } else if (function == currentScan_) {
        sprintf(request,"GET mmsp/scanInfo/currentScan/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseInt32(data_, value, int32Command);
    } else if (function == emVMax_) {
        ;
    } else if (function == emVMin_) {
        ;
    } else if (function == getScanCount_) {
        ;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    return status;
}


asynStatus drvInficon::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    //epicsUInt16 buffer[4];
    //int bufferLen;
    static const char *functionName = "writeInt32";

    if (function == setScanCount_) {
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
    asynStatus status = asynSuccess;
    char request[HTTP_REQUEST_SIZE];
    static const char *functionName = "readFloat64";

    *value = 0;

    if (function == getPress_) {
        sprintf(request,"GET /mmsp/measurement/totalPressure/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseFloat64(data_, value, float64Command);
        if (status != asynSuccess) return(status);
        //printf("%s::%s pressure:%f json:%s\n", driverName, functionName, *value, data_);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    //callParamCallbacks();
    return asynSuccess;
}


asynStatus drvInficon::writeFloat64 (asynUser *pasynUser, epicsFloat64 value)
{
    //int function = pasynUser->reason;
    //static const char *functionName = "writeFloat64";

/*    if (function == setChDwell_) {
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
    }*/
    //callParamCallbacks();
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
    static const char *functionName = "readFloat32Array";

    *nactual = 0;
	int scanSize = 0;
    int scanNum = 0;

    if (function == getScan_) {
        sprintf(request,"GET /mmsp/measurement/scans/-1/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
		status = parseScan(data_, data, &scanSize, &scanNum, scanData_);
        if (status != asynSuccess) return(status);
		*nactual = scanSize;
        printf("%s::%s array0:%e array1:%e array2:%e nElements:%d scanNum:%d\n", driverName, functionName, data[0], data[1], data[2], scanSize, scanNum);
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
    } else if (function == getSensInfo_) {
        sprintf(request,"GET /mmsp/sensorInfo/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensInfo(data_, sensInfo_);
        if (status != asynSuccess) return(status);
        setStringParam(sensName_, sensInfo_->sensName);
        setStringParam(sensDesc_, sensInfo_->sensDesc);
        setUIntDigitalParam(sensSn_, sensInfo_->serialNumber, 0xFFFFFFFF);
    } else if (function == getDevStatus_) {
        sprintf(request,"GET /mmsp/status/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseDevStatus(data_, devStatus_);
        if (status != asynSuccess) return(status);
    } else if (function == getChScanSetup_) {
        if (chNumber < 1 || chNumber > MAX_CHANNELS) return asynError;
        sprintf(request,"GET /mmsp/scanSetup/channel/%d/get\r\n"
        "\r\n", chNumber);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseChScanSetup(data_, chScanSetup_, chNumber);
        if (status != asynSuccess) return(status);
        //printf("%s::%s chNumber:%d chMode:%s chDwel:%f chppamu:%d chstartMass:%f chstopMass:%f\n", driverName, functionName, chNumber, chScanSetup_[chNumber].chMode, chScanSetup_[chNumber].chDwell, chScanSetup_[chNumber].chPpamu, chScanSetup_[chNumber].chStartMass, chScanSetup_[chNumber].chStopMass);
    } else if (function == getDiagData_) {
        sprintf(request,"GET /mmsp/diagnosticData/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseDiagData(data_, diagData_);
        if (status != asynSuccess) return(status);
        //printf("%s::%s boxTemp:%.3f anodePot:%d filCurrent:%d\n", driverName, functionName, diagData_->boxTemp, diagData_->anodePot, diagData_->filCurrent);
    } else if (function == getScanInfo_) {
        sprintf(request,"GET /mmsp/scanInfo/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseScanInfo(data_, scanInfo_);
        if (status != asynSuccess) return(status);
        //printf("%s::%s firstSan:%d currScan:%d scanStatus:%d\n", driverName, functionName, scanInfo_->firstScan, scanInfo_->currScan, scanInfo_->scanStatus);
    } else if (function == getSensDetect_) {
        sprintf(request,"GET /mmsp/sensorDetector/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensDetect(data_, sensDetect_);
        if (status != asynSuccess) return(status);
        //printf("%s::%s emVMax:%d emV:%d emGain:%.3f\n", driverName, functionName, sensDetect_->emVMax, sensDetect_->emV, sensDetect_->emGain);
    } else if (function == getSensFilt_) {
        sprintf(request,"GET /mmsp/sensorFilter/get\r\n"
        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        status = parseSensFilt(data_, sensFilt_);
        if (status != asynSuccess) return(status);
        //printf("%s::%s massMax:%.3f massMin:%.3f dwellMin:%d\n", driverName, functionName, sensFilt_->massMax, sensFilt_->massMin, sensFilt_->dwellMin);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s port %s invalid pasynUser->reason %d\n",
                  driverName, functionName, this->portName, function);
        return asynError;
    }
    //printf("%s::%s status:%d ip:%s mac:%s\n", driverName, functionName, status, commParams_->ip, commParams_->mac);
    //printf("%s::%s status:%d massMax:%d\n", driverName, functionName, status, elecInfo_.massMax);
    //printf("%s::%s status:%d serial:%d name:%s desc:%s\n", driverName, functionName, status, sensInfo_.serialNumber, sensInfo_.sensName, sensInfo_.sensDesc);
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
    int autoConnect;
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

asynStatus drvInficon::parseInt32(const char *jsonData, epicsInt32 *value, commandType_t commandType)
{
    static const char *functionName = "parseInt32";

    return asynSuccess;
}

asynStatus drvInficon::parseUInt32(const char *jsonData, epicsUInt32 *value, commandType_t commandType)
{
	char stemp[32];
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
    return asynSuccess;
}

asynStatus drvInficon::parseFloat64(const char *jsonData, epicsFloat64 *value, commandType_t commandType)
{
    static const char *functionName = "parseFloat64";

    try {
        json j = json::parse(jsonData);
		
        switch (commandType) {
            case float64Command:
                *value = j["data"];
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s::%s, unknown command type %d\n",
                        driverName, functionName, commandType);
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

    return asynSuccess;
}

asynStatus drvInficon::parseString(const char *jsonData, char *value, size_t *valueLen, commandType_t commandType)
{
    static const char *functionName = "parseString";

    try {
        json j = json::parse(jsonData);
		std::string jstring;
		
        switch (commandType) {
            case stringCommand:
                jstring = j["data"];
                strcpy(value, jstring.c_str());
                *valueLen = strlen(value);
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

    //printf("%s::%s JSON data:%s string:%s, dataLength:%d\n", driverName, functionName, jsonData, value, (int)*dataLen);
    return asynSuccess;
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
        sensInfo->serialNumber = j["data"]["serialNumber"];
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
    cutAt = strstr(tempJsonData,"peakfind");
    cutTo = strstr(tempJsonData,"filaments");

	memset(jsonDataSubstring, '\0', 1500);

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

asynStatus drvInficon::parseSensDetect(const char *jsonData, sensDetectStruct *sensDetect)
{
    static const char *functionName = "parseSensDetect";

    try {
        json j = json::parse(jsonData);

        sensDetect->emVMax = j["data"]["emVoltageMax"];
        sensDetect->emVMin = j["data"]["emVoltageMin"];
        sensDetect->emV = j["data"]["emVoltage"];
        sensDetect->emGain = j["data"]["emGain"];
        sensDetect->emGainMass = j["data"]["emGainMass"];
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

asynStatus drvInficon::parseScan(const char *jsonData, float *data, int *scanSize, int *scannum, scanDataStruct *scanData)
{
    static const char *functionName = "parseScan";

    try {
        json j = json::parse(jsonData);

        scanData->scanSize = j["data"]["scansize"];
        scanData->scanNumber = j["data"]["scannum"];
        std::vector <float> values(16384);
		values = j["data"]["values"].get<std::vector<float>>();
        scanData->scanData = values[0];
        printf("%s::%s value0:%e value1:%e\n", driverName, functionName, scanData->scanData[0], scanData->scanData[1]);
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