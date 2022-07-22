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
#include <epicsMutex.h>
#include <epicsEvent.h>
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

static void pollerThreadC(void *drvPvt);

//==========================================================//
// class drvInficon
//		Holds useful vars for interacting with Inficon MPH RGA****
//		hardware
//==========================================================//
drvInficon::drvInficon(const char *portName, const char* hostInfo)

   : asynPortDriver(portName,
                    MAX_CHANNELS, /* maxAddr */
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
    totalPressure_(0),
    pollTime_(DEFAULT_POLL_TIME),
    forceCallback_(true),
    mainState_(IDLE),
    startingLeakcheck_(false),
    startingMonitor_(false),
    leakChkValue_(0),
    lastPolledScan_(-1)
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
    createParam(INFICON_PWR_ON_TIME_STRING,        asynParamFloat64,        &pwrOnTime_);
    createParam(INFICON_EMI_ON_TIME_STRING,        asynParamFloat64,        &emiOnTime_);
    createParam(INFICON_EM_ON_TIME_STRING,         asynParamFloat64,        &emOnTime_);
    createParam(INFICON_EM_CML_ON_TIME_STRING,     asynParamFloat64,        &emCmlOnTime_);
    createParam(INFICON_EM_PRESS_TRIP_STRING,      asynParamUInt32Digital,  &emPressTrip_);
    createParam(INFICON_FIL1_CML_ON_TIME_STRING,   asynParamFloat64,        &fil1CmlOnTime_);
    createParam(INFICON_FIL1_PRESS_TRIP_STRING,    asynParamUInt32Digital,  &fil1PressTrip_);
    createParam(INFICON_FIL2_CML_ON_TIME_STRING,   asynParamFloat64,        &fil2CmlOnTime_);
    createParam(INFICON_FIL2_PRESS_TRIP_STRING,    asynParamUInt32Digital,  &fil2PressTrip_);
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
    createParam(INFICON_GET_XCOORD_STRING,         asynParamFloat32Array,   &getXCoord_);
    createParam(INFICON_GET_LEAKCHK_STRING,        asynParamFloat64,        &getLeakChk_);
    //Scan info parameters
    createParam(INFICON_GET_SCAN_INFO_STRING,      asynParamOctet,          &getScanInfo_);
    createParam(INFICON_FIRST_SCAN_STRING,         asynParamInt32,          &firstScan_);
    createParam(INFICON_LAST_SCAN_STRING,          asynParamInt32,          &lastScan_);
    createParam(INFICON_CURRENT_SCAN_STRING,       asynParamInt32,          &currentScan_);
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
    //User commands and parameters
    createParam(DRIVER_STATE_STRING,               asynParamUInt32Digital,  &driverState_);   
    createParam(MONITOR_START_STRING,              asynParamUInt32Digital,  &startMonitor_);
    createParam(LEAKCHECK_START_STRING,            asynParamUInt32Digital,  &startLeakcheck_);

    /* Create octet port name */
	size_t prefixlen = strlen(PORT_PREFIX);
	size_t len = strlen(portName_) + strlen(PORT_PREFIX) + 1;
	octetPortName_ = (char*)malloc(len);
	memcpy(octetPortName_, PORT_PREFIX, prefixlen);
	memcpy(octetPortName_ + prefixlen, portName_, strlen(portName_) + 1);
	octetPortName_[len - 1] = '\0';
	
    //drvAsynIPPortConfigure("portName","hostInfo",priority,noAutoConnect,noProcessEos)
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

    /* Create the epicsEvent to wake up the pollerThread.*/
    pollerEventId_ = epicsEventCreate(epicsEventEmpty);

    /* Create the thread to read registers if this is a read function code */
    pollerThreadId_ = epicsThreadCreate("InficonPoller",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)pollerThreadC,
            this);

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

    //setUIntDigitalParam(chNumber, function, value, mask);
    if (function == emiOn_) {
        sprintf(request,"GET /mmsp/generalControl/setEmission/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
        //maybe add emissionStandby command? This target puts the ion source filament in standby, a warm but not emitting state.
    } else if (function == emOn_) {
        sprintf(request,"GET /mmsp/generalControl/setEM/set?%d\r\n"
                        "\r\n",
                        value);
        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == rfGenOn_) {
        sprintf(request,"GET /mmsp/generalControl/rfGeneratorSet/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == shutdown_) {
        sprintf(request,"GET /mmsp/generalControl/shutdown/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == emV_) {
        sprintf(request,"GET /mmsp/sensorDetector/emVoltage/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == startStopCh_) {
        if (chNumber < 1 || chNumber >= MAX_CHANNELS)
            return asynError;

        sprintf(request,"GET /mmsp/scanSetup/set?startChannel=%d&stopChannel=%d\r\n"
                        "\r\n",
                        chNumber, chNumber);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == chPpamu_) {
        if (chNumber < 1 || chNumber >= MAX_CHANNELS)
            return asynError;

        sprintf(request,"GET /mmsp/scanSetup/channel/%d/ppamu/set?%d\r\n"
                        "\r\n",
                        chNumber, value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == chDwell_) {
        if (chNumber < 1 || chNumber >= MAX_CHANNELS)
            return asynError;

        sprintf(request,"GET /mmsp/scanSetup/channel/%d/dwell/set?%d\r\n"
                        "\r\n",
                        chNumber, value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == scanStart_) {
        sprintf(request,"GET /mmsp/scanSetup/scanStart/set?%d\r\n"
                        "\r\n",
                        value);

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
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

        //If we get up to here set the internal driver state
        mainState_ = IDLE;
        setUIntDigitalParam(driverState_, static_cast<unsigned int>(mainState_), 0xF);
        //printf("%s::%s mainState:%d\n", driverName, functionName, static_cast<unsigned int>(mainState_));

    } else if (function == filSel_) {
        sprintf(request,"GET /mmsp/sensorIonSource/filamentSelected/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess) return(ioStatus_);
    } else if (function == startMonitor_) {
        //check if we are in idle state
        if (mainState_ != IDLE && scanInfo_->scanStatus != 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s device not in idle state\n",
                      driverName, functionName);
            return asynError;
		}

        sprintf(request,"GET /mmsp/scanSetup/scanStop/set?Immediately\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/channels/3/set?channelMode=Sweep&enabled=True\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/set?startChannel=3&stopChannel=3\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/scanCount/set?-1\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/scanStart/set?1\r\n"
                        "\r\n");
        //ioStatus_ = inficonReadWrite(request, data_);
        inficonReadWrite(request, data_); //i get ioStatus error every time scanStart sent because it timeouts before getting data, guessing that it takes time

        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

        //If we get up to here set the internal driver state
        mainState_ = MONITORING;
        startingMonitor_ = true;
        setUIntDigitalParam(driverState_, static_cast<unsigned int>(mainState_), 0xF);
        //printf("%s::%s mainState:%d\n", driverName, functionName, static_cast<unsigned int>(mainState_));

    } else if (function == startLeakcheck_) {
        //check if we are in idle state
        if (mainState_ != IDLE && scanInfo_->scanStatus != 0) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s device not in idle state\n",
                      driverName, functionName);
            return asynError;
		}

        sprintf(request,"GET /mmsp/scanSetup/scanStop/set?Immediately\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/channels/4/set?channelMode=Single&enabled=True\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/set?startChannel=4&stopChannel=4\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/scanCount/set?-1\r\n"
                        "\r\n");
        ioStatus_ = inficonReadWrite(request, data_);

        sprintf(request,"GET /mmsp/scanSetup/scanStart/set?1\r\n"
                        "\r\n");
        //ioStatus_ = inficonReadWrite(request, data_);
        inficonReadWrite(request, data_); //i get ioStatus error every time scanStart sent because it timeouts before getting data, guessing that it takes time

        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

        //If we get up to here set the internal driver state
        mainState_ = LEAKCEHCK;
        startingLeakcheck_ = true;
        setUIntDigitalParam(driverState_, static_cast<unsigned int>(mainState_), 0xF);
        //printf("%s::%s mainState:%d\n", driverName, functionName, static_cast<unsigned int>(mainState_));

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

    //setIntegerParam(chNumber, function, value);
    if (function == scanCount_) {
        sprintf(request,"GET /mmsp/scanSetup/scanCount/set?%d\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

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
    //int function = pasynUser->reason;
    //static const char *functionName = "readFloat64";

    *value = 0;

    return asynSuccess;
}


asynStatus drvInficon::writeFloat64 (asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    char request[HTTP_REQUEST_SIZE];
	int chNumber;
    double startMass;
    double stopMass;
    static const char *functionName = "writeFloat64";
	
    pasynManager->getAddr(pasynUser, &chNumber);

    //setDoubleParam(chNumber, function, value);
    //get ch stop and start mass
    getDoubleParam(chNumber, chStartMass_, &startMass);
    getDoubleParam(chNumber, chStopMass_, &stopMass);

    if (function == chStartMass_) {
        //make sure that the chnumber doesn't exceed max available channels and that the chStartMass value is not higher than stop mass for that ch
        if (chNumber < 1 || chNumber >= MAX_CHANNELS) {
            return asynError;
	    } //else if (value > stopMass) {
          //  return asynError;
        //}

        sprintf(request,"GET /mmsp/scanSetup/channel/%d/startMass/set?%.2f\r\n"
                        "\r\n",
                        chNumber, value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

    } else if (function == chStopMass_) {
        //make sure that the chnumber doesn't exceed max available channels and that the chStopMass value is not lower than start mass for that ch
        if (chNumber < 1 || chNumber >= MAX_CHANNELS) {
            return asynError;
	    } //else if (value < startMass) {
          //  return asynError;
        //}

        sprintf(request,"GET /mmsp/scanSetup/channel/%d/stopMass/set?%.2f\r\n"
                        "\r\n",
                        chNumber, value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

    } else if (function == emGain_) {
        sprintf(request,"GET /mmsp/sensorDetector/emGain/set?%.2f\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

    } else if (function == emGainMass_) {
        sprintf(request,"GET /mmsp/sensorDetector/emGainMass/set?%.2f\r\n"
                        "\r\n",
                        value);

        ioStatus_ = inficonReadWrite(request, data_);
        if (ioStatus_ != asynSuccess)
            return(ioStatus_);

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
    //int function = pasynUser->reason;
    //static const char *functionName = "readFloat32Array";

    *nactual = 0;

    return asynSuccess;
}


/*
**  asynOctet support
*/
asynStatus drvInficon::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nactual, int *eomReason)
{
    //int function = pasynUser->reason;
    //static const char *functionName = "readOctet";

    *nactual = 0;
	
    return asynSuccess;
}

asynStatus drvInficon::writeOctet (asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    //int function = pasynUser->reason;
    //static const char *functionName = "writeOctet";

    *nActual = strlen(value);

    return asynSuccess;
}

static void pollerThreadC(void *drvPvt)
{
    drvInficon *pPvt = (drvInficon *)drvPvt;

    pPvt->pollerThread();
}


/*
****************************************************************************
** Poller thread for port reads
   One instance spawned per asyn port
****************************************************************************
*/

void drvInficon::pollerThread()
{
    char request[HTTP_REQUEST_SIZE];
    asynStatus status = asynSuccess;
    asynStatus prevIOStatus = asynSuccess;
    epicsTimeStamp currTime, cycleTimeFifeSec, cycleTimeTenSec;
    double dTFifeSec, dTTenSec;

    static const char *functionName="pollerThread";

    lock();

    /* Loop forever */
    while (1)
    {
        /* Sleep for the poll delay or waiting for epicsEvent with the port unlocked */
        unlock();

        epicsEventWaitWithTimeout(pollerEventId_, pollTime_);

        if (inficonExiting_) break;

        epicsTimeGetCurrent(&currTime);
        dTFifeSec = epicsTimeDiffInSeconds(&currTime, &cycleTimeFifeSec);
        dTTenSec = epicsTimeDiffInSeconds(&currTime, &cycleTimeTenSec);

        /* Lock the port.  It is important that the port be locked so other threads cannot access the Inficon
         * structure while the poller thread is running. */
        lock();

        if(dTFifeSec >= 5.) {
            /*Get diagnostic data*/
            sprintf(request,"GET /mmsp/diagnosticData/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseDiagData(data_, diagData_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing device diagnostic data, status=%d\n",
                          driverName, functionName, status);
            setDoubleParam(boxTemp_, diagData_->boxTemp);
            setUIntDigitalParam(anodePotential_, diagData_->anodePot, 0xFFFFFFFF);
            setUIntDigitalParam(emiCurrent_, diagData_->emiCurrent, 0xFFFFFFFF);
            setUIntDigitalParam(focusPotential_, diagData_->focusPot, 0xFFFFFFFF);
            setUIntDigitalParam(electEnergy_, diagData_->electEng, 0xFFFFFFFF);
            setUIntDigitalParam(filPotential_, diagData_->filPot, 0xFFFFFFFF);
            setUIntDigitalParam(filCurrent_, diagData_->filCurrent, 0xFFFFFFFF);
            setUIntDigitalParam(emPotential_, diagData_->emPot, 0xFFFFFFFF);

            /*Get Sensor detector data*/
            sprintf(request,"GET /mmsp/sensorDetector/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseSensDetect(data_, sensDetect_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing sensor detector data, status=%d\n",
                          driverName, functionName, status);
            setUIntDigitalParam(emVMax_, sensDetect_->emVMax, 0xFFFFFFFF);
            setUIntDigitalParam(emVMin_, sensDetect_->emVMin, 0xFFFFFFFF);
            setUIntDigitalParam(emV_, sensDetect_->emV, 0xFFFFFFFF);
            setDoubleParam(emGain_, sensDetect_->emGain);
            setUIntDigitalParam(emGainMass_, sensDetect_->emGainMass, 0xFFFFFFFF);

            /*Get Sensor Ion Source data*/
            sprintf(request,"GET /mmsp/sensorIonSource/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseSensIonSource(data_, sensIonSource_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing sens Ion source data, status=%d\n",
                          driverName, functionName, status);
            setUIntDigitalParam(filSel_, sensIonSource_->filSel, 0xFFFFFFFF);
            setUIntDigitalParam(emiLevel_, sensIonSource_->emiLevel, 0xFFFFFFFF);
            setUIntDigitalParam(optType_, sensIonSource_->optType, 0xFFFFFFFF);


            /*Get CH3 Scan setup data*/
            sprintf(request,"GET /mmsp/scanSetup/channel/3/get\r\n"
                            "\r\n");
            /* Read the data */
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseChScanSetup(data_, chScanSetup_, 3);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing ch3 scan setup, status=%d\n",
                          driverName, functionName, status);
            setStringParam(3, chMode_, chScanSetup_[3].chMode);
            setDoubleParam(3, chStartMass_, chScanSetup_[3].chStartMass);
            setDoubleParam(3, chStopMass_, chScanSetup_[3].chStopMass);	
            setUIntDigitalParam(3, chDwell_, chScanSetup_[3].chDwell, 0xFFFFFFFF);
            setUIntDigitalParam(3, chPpamu_, chScanSetup_[3].chPpamu, 0xFFFFFFFF);

            /*Get CH4 Scan setup data*/
            sprintf(request,"GET /mmsp/scanSetup/channel/4/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseChScanSetup(data_, chScanSetup_, 4);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing ch4 scan setup, status=%d\n",
                          driverName, functionName, status);
            setStringParam(4, chMode_, chScanSetup_[4].chMode);
            setDoubleParam(4, chStartMass_, chScanSetup_[4].chStartMass);
            setDoubleParam(4, chStopMass_, chScanSetup_[4].chStopMass);	
            setUIntDigitalParam(4, chDwell_, chScanSetup_[4].chDwell, 0xFFFFFFFF);
            setUIntDigitalParam(4, chPpamu_, chScanSetup_[4].chPpamu, 0xFFFFFFFF);

            /*Update cycle time*/
            epicsTimeGetCurrent(&cycleTimeFifeSec);
        }

        if(dTTenSec >= 10.) {
            /*Get communication parameters*/
            sprintf(request,"GET /mmsp/communication/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseCommParam(data_, commParams_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing communication parameters, status=%d\n",
                          driverName, functionName, status);
            setStringParam(ip_, commParams_->ip);
            setStringParam(mac_, commParams_->mac);

            /*Get Sensor info*/
            sprintf(request,"GET /mmsp/sensorInfo/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseSensInfo(data_, sensInfo_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing sensor info parameters, status=%d\n",
                          driverName, functionName, status);
            setStringParam(sensName_, sensInfo_->sensName);
            setStringParam(sensDesc_, sensInfo_->sensDesc);
            setUIntDigitalParam(sensSn_, sensInfo_->sensSN, 0xFFFFFFFF);


            /*Get device status*/
            sprintf(request,"GET /mmsp/status/get\r\n"
                            "\r\n");
            /*Read the data*/
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseDevStatus(data_, devStatus_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing device status parameters, status=%d\n",
                          driverName, functionName, status);
            setUIntDigitalParam(systStatus_, devStatus_->systStatus, 0xFFFFFFFF);
            setUIntDigitalParam(hwError_, devStatus_->hwError, 0xFFFFFFFF);
            setUIntDigitalParam(hwWarn_, devStatus_->hwWarn, 0xFFFFFFFF);
            setDoubleParam(pwrOnTime_, devStatus_->pwrOnTime);
            setDoubleParam(emiOnTime_, devStatus_->emiOnTime);
            setDoubleParam(emOnTime_, devStatus_->emOnTime);
            setDoubleParam(emCmlOnTime_, devStatus_->emCmlOnTime);
            setUIntDigitalParam(emPressTrip_, devStatus_->emPressTrip, 0xFFFFFFFF);
            setDoubleParam(fil1CmlOnTime_, devStatus_->filament[1].emiCmlOnTime);
            setUIntDigitalParam(fil1PressTrip_, devStatus_->filament[1].emiPressTrip, 0xFFFFFFFF);
            setDoubleParam(fil2CmlOnTime_, devStatus_->filament[2].emiCmlOnTime);
            setUIntDigitalParam(fil2PressTrip_, devStatus_->filament[2].emiPressTrip, 0xFFFFFFFF);

            /*Get Sensor Filter data*/
            sprintf(request,"GET /mmsp/sensorFilter/get\r\n"
                            "\r\n");
            /* Read the data */
            ioStatus_ = inficonReadWrite(request, data_);

            status = parseSensFilt(data_, sensFilt_);
            if (status)
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: ERROR parsing sensor filter parameters, status=%d\n",
                          driverName, functionName, status);
            setDoubleParam(massMax_, sensFilt_->massMax);
            setDoubleParam(massMin_, sensFilt_->massMin);
            setUIntDigitalParam(dwelMax_, sensFilt_->dwellMax, 0xFFFFFFFF);
            setUIntDigitalParam(dwelMin_, sensFilt_->dwellMin, 0xFFFFFFFF);

            /*Update cycle time*/
            epicsTimeGetCurrent(&cycleTimeTenSec);
        }

        /*****************Do this every cycle******************************/
        /*Get scan info data*/
        sprintf(request,"GET /mmsp/scanInfo/get\r\n"
                        "\r\n");
        /*Read the data*/
        ioStatus_ = inficonReadWrite(request, data_);

        status = parseScanInfo(data_, scanInfo_);
        if (status)
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: ERROR parsing scan info, status=%d\n",
                      driverName, functionName, status);
        setIntegerParam(firstScan_, scanInfo_->firstScan);
        setIntegerParam(lastScan_, scanInfo_->lastScan);
        setIntegerParam(currentScan_, scanInfo_->currScan);
        setUIntDigitalParam(ppscan_, scanInfo_->ppScan, 0xFFFFFFFF);
        setUIntDigitalParam(scanStatus_, scanInfo_->scanStatus, 0x1);

        /*Get pressure value*/
        sprintf(request,"GET /mmsp/measurement/totalPressure/get\r\n"
                        "\r\n");
        /*Read the data*/
        ioStatus_ = inficonReadWrite(request, data_);

        status = parsePressure(data_, &totalPressure_);
        if (status)
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: ERROR parsing total pressure data, status=%d\n",
                      driverName, functionName, status);
        setDoubleParam(getPress_, totalPressure_);

        //let's check if the leakcheck is running, and start pulling leakcheck data
        if(mainState_ == LEAKCEHCK && scanInfo_->scanStatus == 1) {
            if (startingLeakcheck_) {
                startingLeakcheck_ = false;
                lastPolledScan_ = -1;
            }

            if (scanInfo_->lastScan > lastPolledScan_) {
                /*Get leakcheck value from last successfull scan*/
                sprintf(request,"GET /mmsp/measurement/scans/-1/get\r\n"
                                "\r\n");
                /*Read the data*/
                ioStatus_ = inficonReadWrite(request, data_);

                status = parseLeakChk(data_, &leakChkValue_);
                if (status)
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: ERROR parsing leakcheck data, status=%d\n",
                              driverName, functionName, status);
                setDoubleParam(getLeakChk_, leakChkValue_);

                //update last polled scan number 
                lastPolledScan_ = scanInfo_->lastScan;
            }
        }

        //let's check if the monitoring is running, and start pulling data
        if(mainState_ == MONITORING && scanInfo_->scanStatus == 1) {
            if (startingMonitor_) {
                startingMonitor_ = false;
                lastPolledScan_ = -1;
                //set elements of scan array to 0
                memset(scanData_->scanValues, 0, MAX_SCAN_SIZE*sizeof(float));
                //clear screen for the user, array size from previous scan
                doCallbacksFloat32Array(scanData_->scanValues, scanData_->scanSize, getScan_, 0);

                //set elements of x cooridnate array to 0
                memset(scanData_->amuValues, 0, MAX_SCAN_SIZE*sizeof(float));
                //clear screen for the user, array size from previous scan
                doCallbacksFloat32Array(scanData_->amuValues, scanData_->scanSize, getXCoord_, 0);
            }

            if (scanInfo_->lastScan > lastPolledScan_) {
                /*Get scan values from last successfull scan*/
                sprintf(request,"GET /mmsp/measurement/scans/-1/get\r\n"
                                "\r\n");
                /*Read the data*/
                ioStatus_ = inficonReadWrite(request, data_);

		        status = parseScan(data_, scanData_);
                if (status)
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: ERROR parsing leakcheck data, status=%d\n",
                              driverName, functionName, status);

                //update x coordinate data
                doCallbacksFloat32Array(scanData_->amuValues, scanData_->scanSize, getXCoord_, 0);

                //update scan/measurement data
                doCallbacksFloat32Array(scanData_->scanValues, scanData_->scanSize, getScan_, 0);

                //update last polled scan number 
                lastPolledScan_ = scanInfo_->lastScan;
            }
        }


        /* If we have an I/O error this time and the previous time, just try again */
        if (ioStatus_ != asynSuccess &&
            ioStatus_ == prevIOStatus) {
            epicsThreadSleep(1.0);
            continue;
        }

        /* If the I/O status has changed then force callbacks */
        if (ioStatus_ != prevIOStatus) forceCallback_ = true;

        /* Don't start polling until EPICS interruptAccept flag is set,
         * because it does callbacks to device support. */
        while (!interruptAccept) {
            unlock();
            epicsThreadSleep(0.1);
            lock();
        }

        for (int i=0; i<MAX_CHANNELS; i++) {
            callParamCallbacks(i);
        }
        //unlock();
        /* Reset the forceCallback flag */
        forceCallback_ = false;

        /* Set the previous I/O status */
        prevIOStatus = ioStatus_;
    }
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
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                 "%s::%s port %s http response is empty string or invalid asynStatus\n",
                 driverName, functionName, this->portName);
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
    unsigned int uintValue = 0;

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
        uintValue = j["data"]["powerSupplyPowerOnTime"];
        devStatus->pwrOnTime = uintValue/3600.0;
        uintValue = j["data"]["emissionStretch"];
        devStatus->emiOnTime = uintValue/3600.0;
        uintValue = j["data"]["emStretch"];
        devStatus->emOnTime = uintValue/3600.0;
        uintValue = j["data"]["emOnTime"];
        devStatus->emCmlOnTime = uintValue/3600.0;
        devStatus->emPressTrip = j["data"]["emPressTrip"];
        //auto filaments = j["data"]["filaments"];
		int i = 0;
        for (auto& filaments : j["data"]["filaments"]) {
            if (i > 2) 
                return asynError;

            devStatus->filament[i].id = filaments["@id"];
            uintValue = filaments["emisOnTime"];
            devStatus->filament[i].emiCmlOnTime = uintValue/3600.0;
            devStatus->filament[i].emiPressTrip = filaments["emisPressTrip"];
            i++;
            //printf("%s::%s id:%d, emiCmlOnTime:%.1f, emiPressTrip:%d\n", driverName, functionName, devStatus->filament[i].id, devStatus->filament[i].emiCmlOnTime, devStatus->filament[i].emiPressTrip);
            //printf("%s::%s iterator:%d\n", driverName, functionName, i);
        }
        printf("%s::%s systStatus:%d\n", driverName, functionName, devStatus->systStatus);
        printf("%s::%s json:%s\n", driverName, functionName, jsonDataSubstring);
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
    unsigned int ppAMU = 0;
    double dAMU = 0;
    double startMass = 0;
    double stopMass = 0;
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
	
    /*calculate x coordinate data points*/
    getDoubleParam(3, chStartMass_, &startMass);
    getDoubleParam(3, chStopMass_, &stopMass);
    getUIntDigitalParam(3, chPpamu_, &ppAMU, 0xFFFFFFFF);

	if (ppAMU <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s ppAMU value not valid\n",
                  driverName, functionName);
        return asynError;

    } else if (scanData->scanSize <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s scanSize value not valid\n",
                  driverName, functionName);
        return asynError;

    } else if (startMass > stopMass) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s startMass value higher than stopMass value\n",
                  driverName, functionName);
        return asynError;
    }

    //calculate delta AMU
    dAMU = 1/(double)ppAMU;

	//calculate array of values for x coordinate
    for(int i = 0; i < (int)scanData->scanSize; i++) {
        scanData->amuValues[i] = startMass + (i*dAMU);
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

asynStatus drvInficon::parseLeakChk(const char *jsonData, double *value)
{
    static const char *functionName = "parseLeakChk";
    unsigned int actualScanSize = 0;

    try {
        json j = json::parse(jsonData);

		actualScanSize = j["data"]["values"].size();
        if (actualScanSize == 1) {
		    *value = j["data"]["values"][0];
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Error parsing leakcheck data, array size not valid\n",
                      driverName, functionName);   
            return asynError;				 
        }
        //printf("%s::%s value:%e\n", driverName, functionName, *value);
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