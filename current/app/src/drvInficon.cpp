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

#include "drvInficon.h"

/*Json parser includes*/
#include <json.hpp>
using nlohmann::json;

static const char *driverName = "INFICONMPH";