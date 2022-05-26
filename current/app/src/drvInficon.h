//======================================================//
// Name: drvInficon.cpp
// Purpose: Device support for Inficon MPH resifual gas analyzer
//
// Authors: Janez G.
// Date Created: May 10, 2022

//======================================================//
#ifndef drvInficon_H
#define drvInficon_H

#include <epicsThread.h>
#include <epicsEvent.h>

#include <asynPortDriver.h>

//User defines
#define PORT_PREFIX "PORT_"
#define HTTP_OK_CODE "200"
#define DEVICE_RW_TIMEOUT 0.1
#define HTTP_REQUEST_SIZE 512
#define HTTP_RESPONSE_SIZE 150000

/* These are the strings that device support passes to drivers via
 * the asynDrvUser interface.
 * Drivers must return a value in pasynUser->reason that is unique
 * for that command.
 */
//Electronics Info
#define INFICON_MASS_RANGE_STRING         "MASS_MAX"
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

#define MAX_INFICON_COMMAND_TYPES          10

typedef enum {
    stringCommand,
    uint32Command,
    int32Command,
	float64Command,
	setEmiCommand,
	setEmCommand,
	shutdownCommand,
	scanStatCommand,
	setChModeCommand,
	scanStartCommand,
	scanStopCommand,
	errorLogCommand
} commandType_t;

/* Forward declarations */
class drvInficon;

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

    /* These functions are in the asynUInt32Digital interface */
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);

    /* These functions are in the asynInt32 interface */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);

    /* These functions are in the asynInt64 interface */
    //virtual asynStatus writeInt64(asynUser *pasynUser, epicsInt64 value);
    //virtual asynStatus readInt64(asynUser *pasynUser, epicsInt64 *value);

    /* These functions are in the asynFloat64 interface */
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

    /* These functions are in the asynFloat32Array interface */
    virtual asynStatus readFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans, size_t *nactual);
    //virtual asynStatus writeFloat32Array(asynUser *pasynUser, epicsFloat32 *data, size_t maxChans);

    /* These functions are in the asynOctet interface */
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

    /* These are the methods that are new to this class */
    asynStatus inficonReadWrite(const char *request, char *response);
	asynStatus parseInt32(const char *jsonData, epicsInt32 *value, commandType_t commandType);
    asynStatus parseUInt32(const char *jsonData, epicsUInt32 *value, commandType_t commandType);
    asynStatus parseFloat64(const char *jsonData, epicsFloat64 *value, commandType_t commandType);
    asynStatus parseString(const char *jsonData, char *data, size_t *dataLen, commandType_t commandType);
    asynStatus parseScan(const char *jsonData, double *scanValues, int *scanSize, int *scannum);
	asynStatus verifyConnection();   // Verify connection using asynUser //Return asynSuccess for connect
    bool inficonExiting_;
	
protected:
    /* Values used for pasynUser->reason, and indexes into the parameter library. */
    //Electronics Info
    int massRange_;
    //Communication parameters
    int ip_;
    int mac_;
    int errorLog_;
    //General control parameters
    int setEmi_;
    int getEmi_;
    int setEm_;
    int getEm_;
    int setRfGen_;
    int getRfGen_;
    int getFan_;
    int shutdown_;
    //Sensor info parameters
    int sensName_;
    int sensDesc_;
    int sensSn_;
    //Status parameters
    int systStatus_;
    int hwError_;
    int hwWarn_;
    int pwrOnTime_;
    int emiOnTime_;
    int emOnTime_;
    int emiCmlOnTime_;
    int emCmlOnTime_;
    int emiPressTrip_;
    //Diagnostic data parameters
    int boxTemp_;
    int anodePotential_;
    int emiCurrent_;
    int focusPotential_;
    int electEnergy_;
    int filPotential_;
    int filCurrent_;
    int emPotential_;
    //Measurement parameters
    int getPress_;
    int getScan_;
    //Scan info parameters
    int firstScan_;
    int lastScan_;
    int currentScan_;
    int ppscan_;
    int scanStat;
    //Sensor detector parameters
    int emVoltageMax_;
    int emVoltageMin_;
    //Sensor filter parameters
    int dwelMax_;
    int dwelMin_;
    //Scan setup parameters
    int setStartCh_;
    int getStartCh_;
    int setStopCh_;
    int getStopCh_;
    int setChMode_;
    int getChMode_;
    int setChPpamu_;
    int getChPpamu_;
    int setChDwell_;
    int getChDwell_;
    int setChStartMass_;
    int getChStartMass_;
    int setChStopMass_;
    int getChStopMass_;
    int setScanCount_;
    int getScanCount_;
    int scanStart_;
    int scanStop_;

private:
    /* Our data */
    bool initialized_;           /* If initialized successfully */
    bool isConnected_;           /* Connection status */
	char *portName_;             /* asyn port name for the user driver */
    char *octetPortName_;        /* asyn port name for the asyn octet port */
	char *hostInfo_;             /* host info (IP address,connection type, port)*/
    asynUser  *pasynUserOctet_;  /* asynUser for asynOctet interface to asyn octet port */
    asynUser  *pasynUserCommon_; /* asynUser for asynCommon interface to asyn octet port */
    asynUser  *pasynUserTrace_;  /* asynUser for asynTrace on this port */
    char *data_;                 /* Memory buffer */
    //char inficonRequest_[HTTP_REQUEST_SIZE];       /* Inficon request */
    //char inficonResponse_[HTTP_RESPONSE_SIZE];        /* Inficon response */
	asynStatus ioStatus_;
    asynStatus prevIOStatus_;
    int readOK_;
    int writeOK_;
	int scanChannel_;
};

#endif /* drvInficon_H */