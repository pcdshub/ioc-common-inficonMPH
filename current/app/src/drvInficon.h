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
#define DEVICE_RW_TIMEOUT 0.2
#define HTTP_REQUEST_SIZE 512
#define HTTP_RESPONSE_SIZE 150000

/* These are the strings that device support passes to drivers via
 * the asynDrvUser interface.
 * Drivers must return a value in pasynUser->reason that is unique
 * for that command.
 */
//Communication
#define INFICON_GET_COMM_PARAM_STRING     "GET_COMM_PARAM"
#define INFICON_IP_STRING                 "IP"
#define INFICON_MAC_STRING                "MAC"
//#define INFICON_ERROR_LOG_STRING          "ERROR_LOG"
//General control
#define INFICON_EMI_ON_STRING             "EMI_ON"
#define INFICON_EM_ON_STRING              "EM_ON"
#define INFICON_RFGEN_ON_STRING           "RFGEN_ON"
#define INFICON_FAN_CNTRL_STRING          "FAN_CNTRL"
#define INFICON_SHUTDOWN_STRING           "SHUTDOWN"
//Sensor info
#define INFICON_GET_SENS_INFO_STRING      "GET_SENS_INFO"
#define INFICON_SENS_NAME_STRING          "SENS_NAME"
#define INFICON_SENS_DESC_STRING          "SENS_DESC"
#define INFICON_SENS_SN_STRING            "SENS_SN"
//Status
#define INFICON_GET_DEV_STAT_STRING       "GET_DEV_STAT"
#define INFICON_SYST_STAT_STRING          "SYST_STAT"
#define INFICON_HW_ERROR_STRING           "HW_ERROR"
#define INFICON_HW_WARN_STRING            "HW_WARN"
#define INFICON_PWR_ON_TIME_STRING        "PWR_ON_T"
#define INFICON_EMI_ON_TIME_STRING        "EMI_ON_T"
#define INFICON_EM_ON_TIME_STRING         "EM_ON_T"
#define INFICON_EMI_CML_ON_TIME_STRING    "EMI_CML_ON_T"
#define INFICON_EM_CML_ON_TIME_STRING     "EM_CML_ON_T"
#define INFICON_EMI_PRESS_TRIP_STRING     "EMI_PRESS_TRIP"
#define INFICON_EM_PRESS_TRIP_STRING      "EM_PRESS_TRIP"
//Diagnostic data
#define INFICON_GET_DIAG_DATA_STRING      "GET_DIAG_DATA"
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
#define INFICON_GET_SCAN_INFO_STRING      "GET_SCAN_INFO"
#define INFICON_FIRST_SCAN_STRING         "FIRST_SCAN"
#define INFICON_LAST_SCAN_STRING          "LAST_SCAN"
#define INFICON_CURRENT_SCAN_STRING       "CURRENT_SCAN"
#define INFICON_PPSCAN_STRING             "PPSCAN"
#define INFICON_SCAN_STAT_STRING          "SCAN_STAT"
//Sensor detector
#define INFICON_GET_SENS_DETECT_STRING    "GET_SENS_DETECT"
#define INFICON_EM_VOLTAGE_STRING         "EM_V"
#define INFICON_EM_VOLTAGE_MAX_STRING     "EM_V_MAX"
#define INFICON_EM_VOLTAGE_MIN_STRING     "EM_V_MIN"
#define INFICON_EM_GAIN_STRING            "EM_GAIN"
#define INFICON_EM_GAIN_MASS_STRING       "EM_GAIN_MASS"
//Sensor filter
#define INFICON_GET_SENS_FILT_STRING      "GET_SENS_FILT"
#define INFICON_MASS_MAX_STRING           "MASS_MAX"
#define INFICON_MASS_MIN_STRING           "MASS_MIN"
#define INFICON_DWELL_MAX_STRING          "DWELL_MAX"
#define INFICON_DWELL_MIN_STRING          "DWELL_MIN"
//Scan setup
#define INFICON_GET_CH_SCAN_SETUP_STRING  "GET_CH_SCAN_SETUP"
#define INFICON_SET_CH_SCAN_SETUP_STRING  "SET_CH_SCAN_SETUP"
#define INFICON_START_STOP_CH_STRING      "START_STOP_CH"
#define INFICON_CH_MODE_STRING            "CH_MODE"
#define INFICON_CH_PPAMU_STRING           "CH_PPAMU"
#define INFICON_CH_DWELL_STRING           "CH_DWELL"
#define INFICON_CH_DWELL_STRING           "CH_DWELL"
#define INFICON_CH_START_MASS_STRING      "CH_START_MASS"
#define INFICON_CH_STOP_MASS_STRING       "CH_STOP_MASS"
#define INFICON_SCAN_COUNT_STRING         "SCAN_COUNT"
#define INFICON_SCAN_MODE_STRING          "SCAN_MODE"
#define INFICON_SCAN_START_STRING         "SCAN_START"
#define INFICON_SCAN_STOP_STRING          "SCAN_STOP"

#define MAX_INFICON_COMMAND_TYPES          10
#define MAX_CHANNELS                       4
#define MAX_SCAN_SIZE                      16384

typedef struct {
    char ip[32];
    char mac[32];
} commParamStruct;

typedef struct {
    unsigned int emiSetStatus;
    unsigned int emSetStatus;
    unsigned int rfSetGenStatus;
    unsigned int fanStatus;
} genCntrlStruct;

typedef struct {
    char sensName[20];
    char sensDesc[40];
    unsigned int sensSN;
} sensInfoStruct;

typedef struct {
    unsigned int systStatus;
    unsigned int hwError;
    unsigned int hwWarn;
    unsigned int pwrOnTime;
    unsigned int emiOnTime;
    unsigned int emOnTime;
    unsigned int emCmlOnTime;
    unsigned int emPressTrip;
    unsigned int emiCmlOnTime;
    unsigned int emiPressTrip;
} devStatusStruct;

typedef struct {
    double boxTemp;
    unsigned int anodePot;
    unsigned int emiCurrent;
    unsigned int focusPot;
    unsigned int electEng;
    unsigned int filPot;
    unsigned int filCurrent;
    unsigned int emPot;
} diagDataStruct;

typedef struct {
    unsigned int firstScan;
    unsigned int lastScan;
    unsigned int currScan;
    unsigned int ppScan;
    unsigned int scanStatus;
} scanInfoStruct;

typedef struct {
    unsigned int emVMax;
    unsigned int emVMin;
    unsigned int emV;
    double emGain;
    unsigned int emGainMass;
} sensDetectStruct;

typedef struct {
    double massMax;
    double massMin;
    unsigned int dwellMax;
    unsigned int dwellMin;
} sensFiltStruct;

typedef struct {
    char chMode[32];
    double chStartMass;
    double chStopMass;
    double chDwell;
    unsigned int chPpamu;
} chScanSetupStruct;

typedef struct {
    unsigned int scanSize;
    unsigned int actualScanSize;
    unsigned int scanNumber;
	float scanValues[MAX_SCAN_SIZE];
} scanDataStruct;

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
    asynStatus parseScan(const char *jsonData, scanDataStruct *scanData);
    asynStatus parseCommParam(const char *jsonData, commParamStruct *commParam);
    asynStatus parseSensInfo(const char *jsonData, sensInfoStruct *sensInfo);
    asynStatus parseDevStatus(const char *jsonData, devStatusStruct *devStatus);
    asynStatus parseDiagData(const char *jsonData, diagDataStruct *diagData);
    asynStatus parseScanInfo(const char *jsonData, scanInfoStruct *scanInfo);
    asynStatus parseSensDetect(const char *jsonData, sensDetectStruct *sensDetect);
    asynStatus parseSensFilt(const char *jsonData, sensFiltStruct *sensFilt);
    asynStatus parseChScanSetup(const char *jsonData, chScanSetupStruct *chScanSetup, unsigned int chNumber);
    asynStatus parsePressure(const char *jsonData, double *value);
    asynStatus verifyConnection();   // Verify connection using asynUser //Return asynSuccess for connect
    bool inficonExiting_;

protected:
    /* Values used for pasynUser->reason, and indexes into the parameter library. */
    //Communication parameters
    int getCommParam_;
    int ip_;
    int mac_;
    //int errorLog_;
    //General control parameters
    int emiOn_;
    int emOn_;
    int rfGenOn_;
    int fanCntrl_;
    int shutdown_;
    //Sensor info parameters
    int getSensInfo_;
    int sensName_;
    int sensDesc_;
    int sensSn_;
    //Status parameters
    int getDevStatus_;
    int systStatus_;
    int hwError_;
    int hwWarn_;
    int pwrOnTime_;
    int emiOnTime_;
    int emOnTime_;
    int emiCmlOnTime_;
    int emCmlOnTime_;
    int emiPressTrip_;
    int emPressTrip_;
    //Diagnostic data parameters
    int getDiagData_;
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
    int getScanInfo_;
    int firstScan_;
    int lastScan_;
    int currentScan_;
    int ppscan_;
    int scanStatus_;
    //Sensor detector parameters
    int getSensDetect_;
    int emVMax_;
    int emVMin_;
    int emV_;
    int emGain_;
    int emGainMass_;
    //Sensor filter parameters
    int getSensFilt_;
    int massMax_;
    int massMin_;
    int dwelMax_;
    int dwelMin_;
    //Scan setup parameters
    int getChScanSetup_;
    int setChScanSetup_;
    int startStopCh_;
    int chMode_;
    int chPpamu_;
    int chDwell_;
    int chStartMass_;
    int chStopMass_;
    int scanCount_;
    int scanMode_;
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
    //char inficonRequest_[HTTP_REQUEST_SIZE];          /* Inficon request */
    //char inficonResponse_[HTTP_RESPONSE_SIZE];        /* Inficon response */
    asynStatus ioStatus_;
    asynStatus prevIOStatus_;
    int readOK_;
    int writeOK_;
    int scanChannel_;
    commParamStruct *commParams_;
    genCntrlStruct *genCntrl_;
    sensInfoStruct *sensInfo_;
    devStatusStruct *devStatus_;
    diagDataStruct *diagData_;
    scanInfoStruct *scanInfo_;
    sensDetectStruct *sensDetect_;
    sensFiltStruct *sensFilt_;
    chScanSetupStruct *chScanSetup_;
    scanDataStruct *scanData_;
	float totalPressure_;
};

#endif /* drvInficon_H */