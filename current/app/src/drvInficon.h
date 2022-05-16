//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH resifual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//
#pragma once

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
//#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
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
class drvInficon;

#define PORT_PREFIX "PORT_"

/* These are the strings that device support passes to drivers via
 * the asynDrvUser interface.
 * Drivers must return a value in pasynUser->reason that is unique
 * for that command.
 */
// These are the parameters we register with asynPortDriver
#define MODBUS_DATA_STRING                "MODBUS_DATA"
#define MODBUS_READ_STRING                "MODBUS_READ"
#define MODBUS_ENABLE_HISTOGRAM_STRING    "ENABLE_HISTOGRAM"
#define MODBUS_READ_HISTOGRAM_STRING      "READ_HISTOGRAM"
#define MODBUS_HISTOGRAM_BIN_TIME_STRING  "HISTOGRAM_BIN_TIME"
#define MODBUS_HISTOGRAM_TIME_AXIS_STRING "HISTOGRAM_TIME_AXIS"
#define MODBUS_POLL_DELAY_STRING          "POLL_DELAY"
#define MODBUS_READ_OK_STRING             "READ_OK"
#define MODBUS_WRITE_OK_STRING            "WRITE_OK"
#define MODBUS_IO_ERRORS_STRING           "IO_ERRORS"
#define MODBUS_LAST_IO_TIME_STRING        "LAST_IO_TIME"
#define MODBUS_MAX_IO_TIME_STRING         "MAX_IO_TIME"

// These are the data type strings that are used in the drvUser parameter
// They are not registered with asynPortDriver
//Communication
#define INFICON_IP_STRING                 "IP"
#define INFICON_MAC_STRING                "MAC"
#define INFICON_LOGIN_STRING              "LOGIN"
#define INFICON_ERROR_LOG_STRING          "ERROR_LOG"
//General control
#define INFICON_SET_EMI_STRING            "SET_EMI"
#define INFICON_GET_EMI_STRING            "GET_EMI"
#define INFICON_SET_EM_STRING             "SET_EM"
#define INFICON_GET_EM_STRING             "GET_EM"
#define INFICON_SET_RFGEN_STRING          "SET_RFGEN"
#define INFICON_GET_RFGEN_STRING          "GET_RFGEN"
#define INFICON_GET_FAN_STRING            "GET_FAN"
#define INFICON_SHUTDOWN_STRING           "SHUTDOWN"
//Sensor info
#define INFICON_SENS_NAME_STRING          "SENS_NAME"
#define INFICON_SENS_DESC_STRING          "SENS_DESC"
#define INFICON_SENS_SN_STRING            "SENS_SN"
//Status
#define INFICON_SYST_STAT_STRING          "SYST_STAT"
#define INFICON_HW_ERROR_STRING           "HW_ERROR"
#define INFICON_HW_WARN_STRING            "HW_WARN"
#define INFICON_PWR_ON_TIME_STRING        "PWR_ON_T"
#define INFICON_EMI_ON_TIME_STRING        "EMI_ON_T"
#define INFICON_EM_ON_TIME_STRING         "EM_ON_T"
#define INFICON_EMI_CML_ON_TIME_STRING    "EMI_CML_ON_T"
#define INFICON_EM_CML_ON_TIME_STRING     "EM_CML_ON_T"
#define INFICON_EMI_PRESS_TRIP_STRING     "EMI_PRESS_TRIP"
//Diagnostic data
#define INFICON_BOX_TEMP_STRING           "BOX_TEMP"
#define INFICON_ANODE_POTENTIAL_STRING    "ANODE_POTENTIAL"
#define INFICON_EMI_CURRENT_STRING        "EMI_CURRENT"
#define INFICON_FOCUS_POTENTIAL_STRING    "FOCUS_POTENTIAL"
#define INFICON_ELECT_ENERGY_STRING       "ELECT_ENERGY"
#define INFICON_FIL_POTENTIAL_STRING      "FIL_POTENTIAL"
#define INFICON_FIL_CURRENT_STRING        "FIL_CURRENT"
#define INFICON_EM_POTENTIAL_STRING       "EM_POTENTIAL"
//Measurement
#define INFICON_GET_PRESS_STRING          "GET_PRESS"
#define INFICON_GET_SCAN_STRING           "GET_SCAN"
//Scan info
#define INFICON_FIRST_SCAN_STRING         "FIRST_SCAN"
#define INFICON_LAST_SCAN_STRING          "LAST_SCAN"
#define INFICON_CURRENT_SCAN_STRING       "CURRENT_SCAN"
#define INFICON_PPSCAN_STRING             "PPSCAN"
#define INFICON_SCAN_STAT_STRING          "SCAN_STAT"
//Sensor detector
#define INFICON_EM_VOLTAGE_MAX_STRING     "EM_V_MAX"
#define INFICON_EM_VOLTAGE_MIN_STRING     "EM_V_MIN"
//Sensor filter
#define INFICON_DWELL_MAX_STRING          "DWELL_MAX"
#define INFICON_DWELL_MIN_STRING          "DWELL_MIN"
//Scan setup
#define INFICON_SET_START_CH_STRING       "SET_START_CH"
#define INFICON_GET_START_CH_STRING       "GET_START_CH"
#define INFICON_SET_STOP_CH_STRING        "SET_STOP_CH"
#define INFICON_GET_STOP_CH_STRING        "GET_STOP_CH"
#define INFICON_SET_CH_MODE_STRING        "SET_CH_MODE"
#define INFICON_GET_CH_MODE_STRING        "GET_CH_MODE"
#define INFICON_SET_CH_PPAMU_STRING       "SET_CH_PPAMU"
#define INFICON_GET_CH_PPAMU_STRING       "GET_CH_PPAMU"
#define INFICON_SET_CH_DWELL_STRING       "SET_CH_DWELL"
#define INFICON_GET_CH_DWELL_STRING       "GET_CH_DWELL"
#define INFICON_SET_CH_START_MASS_STRING  "SET_CH_START_MASS"
#define INFICON_GET_CH_START_MASS_STRING  "GET_CH_START_MASS"
#define INFICON_SET_CH_STOP_MASS_STRING   "SET_CH_STOP_MASS"
#define INFICON_GET_CH_STOP_MASS_STRING   "GET_CH_STOP_MASS"
#define INFICON_SET_SCAN_COUNT_STRING     "SET_SCAN_COUNT"
#define INFICON_GET_SCAN_COUNT_STRING     "GET_SCAN_COUNT"
#define INFICON_SCAN_START_STRING         "SCAN_START"
#define INFICON_SCAN_STOP_STRING          "SCAN_STOP"

#define MAX_INFICON_COMMAND_TYPES          73   

typedef enum {
    dataTypeInt16,
    dataTypeInt16SM,
    dataTypeBCDUnsigned,
    dataTypeBCDSigned,
    dataTypeUInt16,
    dataTypeInt32LE,
    dataTypeInt32LEBS,
    dataTypeInt32BE,
    dataTypeInt32BEBS,
    dataTypeUInt32LE,
    dataTypeUInt32LEBS,
    dataTypeUInt32BE,
    dataTypeUInt32BEBS,
    dataTypeInt64LE,
    dataTypeInt64LEBS,
    dataTypeInt64BE,
    dataTypeInt64BEBS,
    dataTypeUInt64LE,
    dataTypeUInt64LEBS,
    dataTypeUInt64BE,
    dataTypeUInt64BEBS,
    dataTypeFloat32LE,
    dataTypeFloat32LEBS,
    dataTypeFloat32BE,
    dataTypeFloat32BEBS,
    dataTypeFloat64LE,
    dataTypeFloat64LEBS,
    dataTypeFloat64BE,
    dataTypeFloat64BEBS,
    dataTypeStringHigh,
    dataTypeStringLow,
    dataTypeStringHighLow,
    dataTypeStringLowHigh,
    dataTypeZStringHigh,
    dataTypeZStringLow,
    dataTypeZStringHighLow,
    dataTypeZStringLowHigh,
	MAX_INFICON_COMMAND_TYPES
} inficonCommandType_t;

struct inficonDrvUser_t;

class drvInficon : public asynPortDriver {
public:
	drvInficon(const char *portName, const char* hostInfo);
	
	/* Make  sure to free everything */
	~drvInficon();

    /* These are the methods that we override from asynPortDriver */

    /* These functions are in the asynCommon interface */
    virtual void report(FILE *fp, int details);
    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus getAddress(asynUser *pasynUser, int *address);

   /* These functions are in the asynDrvUser interface */
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
    virtual asynStatus drvUserDestroy(asynUser *pasynUser);

    /* These functions are in the asynUInt32Digital interface */
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);

    /* These functions are in the asynInt32 interface */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);

    /* These functions are in the asynInt64 interface */
    virtual asynStatus writeInt64(asynUser *pasynUser, epicsInt64 value);
    virtual asynStatus readInt64(asynUser *pasynUser, epicsInt64 *value);

    /* These functions are in the asynFloat64 interface */
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

    /* These functions are in the asynInt32Array interface */
    virtual asynStatus readFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans, size_t *nactual);
    //virtual asynStatus writeFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans);

    /* These functions are in the asynOctet interface */
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

    /* These are the methods that are new to this class */
    inficonCommandType_t getCommandType(asynUser *pasynUser);
    int getStringLen(asynUser *pasynUser, size_t maxChars);
    asynStatus doInficonIO(int slave, int function, int start, epicsUInt16 *data, int len);
    asynStatus readPlcInt32(inficonCommandType_t dataType, int offset, epicsInt32 *value, int *bufferLen);
    asynStatus writePlcInt32(inficonCommandType_t dataType, int offset, epicsInt32 value, epicsUInt16 *buffer, int *bufferLen);
    asynStatus readPlcInt64(inficonCommandType_t dataType, int offset, epicsInt64 *value, int *bufferLen);
    asynStatus writePlcInt64(inficonCommandType_t dataType, int offset, epicsInt64 value, epicsUInt16 *buffer, int *bufferLen);
    asynStatus readPlcFloat(inficonCommandType_t dataType, int offset, epicsFloat64 *value, int *bufferLen);
    asynStatus writePlcFloat(inficonCommandType_t dataType, int offset, epicsFloat64  value, epicsUInt16 *buffer, int *bufferLen);
    asynStatus readPlcString (inficonCommandType_t dataType, int offset, char *value, size_t maxChars, int *bufferLen);
    asynStatus writePlcString(inficonCommandType_t dataType, int offset, const char *value, size_t maxChars, size_t *nActual, int *bufferLen);
    bool inficonExiting_;
	asynStatus verifyConnection();   // Verify connection using asynUser //Return asynSuccess for connect
/*
public:
	//crete instance of inficon device
	static drvInficon* Create(const char* portName, const char* hostinfo);*/
protected:
    /* Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_Data;
    int P_Read;
    int P_EnableHistogram;
    int P_ReadHistogram;
    int P_HistogramBinTime;
    int P_HistogramTimeAxis;
    int P_PollDelay;
    int P_ReadOK;
    int P_WriteOK;
    int P_IOErrors;
    int P_LastIOTime;
    int P_MaxIOTime;

private:
    /* Our data */
    bool initialized_;           /* If initialized successfully */
	char *portName_;             /* asyn port name for the user driver */
    char *octetPortName_;        /* asyn port name for the asyn octet port */
	char *hostInfo_;
    bool isConnected_;           /* Connection status */
    asynStatus ioStatus_;        /* I/O error status */
    asynUser  *pasynUserOctet_;  /* asynUser for asynOctet interface to asyn octet port */
    asynUser  *pasynUserCommon_; /* asynUser for asynCommon interface to asyn octet port */
    asynUser  *pasynUserTrace_;  /* asynUser for asynTrace on this port */
    inficonCommandType_t commandType_;  /* Command type */
    inficonDrvUser_t *drvUser_;   /* Drv user structure */
    epicsUInt16 *data_;          /* Memory buffer */
    char inficonRequest_[MAX_REQUEST_SIZE];       /* Modbus request message */
    char inficonReply_[MAX_RESPONSE_SIZE];        /* Modbus reply message */
    int readOK_;
    int writeOK_;
    int IOErrors_;
    int currentIOErrors_; /* IO Errors since last successful writeRead cycle */
    int maxIOMsec_;
    int lastIOMsec_;

	/* Enable/disable debugging messages */
	bool debug_;
};