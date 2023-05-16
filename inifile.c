
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "configIO.h"
#include "log4c.h"
#include "ws_structs.h"


#ifndef FALSE
#define FALSE  (0)
#define TRUE   (!FALSE)
#endif


//
// we keep a copy of the INI file we're using
static  char    *iniFileName;


// ----------------------------------------------------------------------------
static
int fileExists (const char * fileName)
{
    struct  stat    buf;

    int i = stat ( fileName, &buf );
    if ( i == 0 )
        return TRUE;

    return FALSE;
}


// ------------------------------------------------------------------
void    findIniFile (WS2308System_t *aSystem)
{
    iniFileName = "";
    
    //
    //  Look in some common places for the INI file
    if (fileExists(  aSystem->INIFileName ))                      // 1. Local takes precedence
        iniFileName = aSystem->INIFileName;
    
    else {
        Logger_LogError( "Cannot find the INI file [%s]\n", aSystem->INIFileName );
        //
        // Logger might not be initialized...
        fprintf( stderr, "Cannot find the INI file [%s]. Will look in other standard places for the INI file.\n", aSystem->INIFileName );
    
        //
        // Look in some more common places
        if (fileExists( "WS2308.ini" )) {
            iniFileName = "WS2308.ini";
        
        } else if (fileExists( "~/WS2308.ini" )) {
            iniFileName = "~/WS2308.ini";
        
        } else if (fileExists( "/usr/local/INI/WS2308.ini" )) {
            iniFileName = "/usr/local/INI/WS2308.ini";
        } else {
            iniFileName = "";
        }
        
        strncpy( &(aSystem->INIFileName[0]), iniFileName, MAX_INI_STRLEN );
        fprintf( stderr, "Using the INI file [%s]\n", iniFileName );
    }
}

// ------------------------------------------------------------------
void    readIniFile (WS2308System_t *aSystem)
{
    int     result;

    //
    //  Go looking for it first!!!
    findIniFile( aSystem );

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "deviceName", &(aSystem->ws2308DeviceName[0]), Cfg_String );
    if (!result) {
        Logger_LogWarning( "Could not open the INI File [%s]. Using program defaults for everything!\n", aSystem->INIFileName );        
        return;
    }
    

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "debugLevel", &aSystem->debugLevel, Cfg_Ushort );

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "deviceName", &(aSystem->ws2308DeviceName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "readIntervalSecs", &aSystem->readInterval, Cfg_Ushort );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "discardBadReadings", &aSystem->discardBadReadings, Cfg_Boolean );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "ignoreAtomicClock", &aSystem->ignoreAtomicClock, Cfg_Boolean );

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "logFile", &(aSystem->logFileName[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "absPressureCorrection", &aSystem->absPressureCorrection, Cfg_Double );

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "logToMQTT", &aSystem->logToMQTT, Cfg_Boolean );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "logToDatabase", &aSystem->logToDatabase, Cfg_Boolean );
            
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTServer", &(aSystem->MQTTServerName[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTParentTopic", &(aSystem->MQTTParentTopic[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTStatusTopic", &(aSystem->MQTTStatusTopic[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTAlarmTopic", &(aSystem->MQTTAlarmTopic[0]), Cfg_String );
    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTPort", &aSystem->MQTTPort, Cfg_Short );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTQoS", &aSystem->MQTTQoS, Cfg_Short );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTRetainMsgs", &aSystem->MQTTRetainMsgs, Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTKeepAliveValue", &aSystem->MQTTKeepAliveValue, Cfg_Short );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTUseJson", &aSystem->MQTTUseJSON, Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "MQTTUseISO8601", &aSystem->MQTTUseISO8601, Cfg_Boolean );    

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_IndoorTempLow", &aSystem->Alarm_IndoorTempLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_IndoorTempHigh", &aSystem->Alarm_IndoorTempHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_IndoorHumidityLow", &aSystem->Alarm_IndoorHumidityLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_IndoorHumidityHigh", &aSystem->Alarm_IndoorHumidityHigh, Cfg_Double );    

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorTempLow", &aSystem->Alarm_OutdoorTempLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorTempHigh", &aSystem->Alarm_OutdoorTempHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorHumidityLow", &aSystem->Alarm_OutdoorHumidityLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorHumidityHigh", &aSystem->Alarm_OutdoorHumidityHigh, Cfg_Double );    

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorWindspeedHigh", &aSystem->Alarm_OutdoorWindspeedHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_OutdoorWindchillLow", &aSystem->Alarm_OutdoorWindchillLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "Alarm_Raining", &aSystem->Alarm_Raining, Cfg_Boolean );   

    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.hostName", &(aSystem->db.hostName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.userName", &(aSystem->db.userName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.password", &(aSystem->db.password[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.schema", &(aSystem->db.schema[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.logAlarms", &(aSystem->db.logAlarms), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.logReadings", &(aSystem->db.logReadings), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.logHistory", &(aSystem->db.logHistory), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.dropAlarmTable", &(aSystem->db.dropAlarmTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.createAlarmTable", &(aSystem->db.createAlarmTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.dropReadingsTable", &(aSystem->db.dropReadingsTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.createReadingsTable", &(aSystem->db.createReadingsTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.dropHistoryTable", &(aSystem->db.dropHistoryTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "WS2308", "db.createHistoryTable", &(aSystem->db.createHistoryTable), Cfg_Boolean );
    
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneWindspeed", &sanityValues.minSaneWindspeed, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneWindspeed", &sanityValues.maxSaneWindspeed, Cfg_Double );
    
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneOTemp", &sanityValues.minSaneOTemp, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneOTemp", &sanityValues.maxSaneOTemp, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneOHumidity", &sanityValues.minSaneOHumidity, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneOHumidity", &sanityValues.maxSaneOHumidity, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneOPressure", &sanityValues.minSaneOPressure, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneOPressure", &sanityValues.maxSaneOPressure, Cfg_Double );
    
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneITemp", &sanityValues.minSaneITemp, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneITemp", &sanityValues.maxSaneITemp, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "minSaneIHumidity", &sanityValues.minSaneIHumidity, Cfg_Double );
    //(void) IniFiler_SearchCfg( iniFileName, "Errors", "maxSaneIHumidity", &sanityValues.maxSaneIHumidity, Cfg_Double );
}

// ------------------------------------------------------------------
void    readIniFile2 (WS2308System_t *aSystem)
{
    int     result;

    //
    //  Go looking for it first!!!
    findIniFile( aSystem );

    (void) IniFiler_SearchCfg( iniFileName, "System", "deviceName", &(aSystem->ws2308DeviceName[0]), Cfg_String );
    if (!result) {
        Logger_LogWarning( "Could not open the INI File [%s]. Using program defaults for everything!\n", aSystem->INIFileName );        
        return;
    }
    

    (void) IniFiler_SearchCfg( iniFileName, "Debugging", "debugLevel", &aSystem->debugLevel, Cfg_Ushort );
    (void) IniFiler_SearchCfg( iniFileName, "Debugging", "logFile", &(aSystem->logFileName[0]), Cfg_String );

    (void) IniFiler_SearchCfg( iniFileName, "System", "deviceName", &(aSystem->ws2308DeviceName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "System", "readIntervalSecs", &aSystem->readInterval, Cfg_Ushort );
    (void) IniFiler_SearchCfg( iniFileName, "System", "discardBadReadings", &aSystem->discardBadReadings, Cfg_Boolean );
    (void) IniFiler_SearchCfg( iniFileName, "System", "absPressureCorrection", &aSystem->absPressureCorrection, Cfg_Double );

    (void) IniFiler_SearchCfg( iniFileName, "System", "logToMQTT", &aSystem->logToMQTT, Cfg_Boolean );
    (void) IniFiler_SearchCfg( iniFileName, "System", "logToDatabase", &aSystem->logToDatabase, Cfg_Boolean );
            
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "BrokerHostName", &(aSystem->MQTTServerName[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "ParentTopic", &(aSystem->MQTTParentTopic[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "StatusTopic", &(aSystem->MQTTStatusTopic[0]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "AlarmTopic", &(aSystem->MQTTAlarmTopic[0]), Cfg_String );
    
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "PortNumber", &aSystem->MQTTPort, Cfg_Short );
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "QoSValue", &aSystem->MQTTQoS, Cfg_Short );
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "RetainMsgs", &aSystem->MQTTRetainMsgs, Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "MQTT", "KeepAliveValue", &aSystem->MQTTKeepAliveValue, Cfg_Short );
    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "IndoorTemp_Low", &aSystem->Alarm_IndoorTempLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "IndoorTemp_High", &aSystem->Alarm_IndoorTempHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "IndoorHumidity_Low", &aSystem->Alarm_IndoorHumidityLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "IndoorHumidity_High", &aSystem->Alarm_IndoorHumidityHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorTemp_Low", &aSystem->Alarm_OutdoorTempLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorTemp_High", &aSystem->Alarm_OutdoorTempHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorHumidity_Low", &aSystem->Alarm_OutdoorHumidityLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorHumidity_High", &aSystem->Alarm_OutdoorHumidityHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorWindspeed_High", &aSystem->Alarm_OutdoorWindspeedHigh, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "OutdoorWindchill_Low", &aSystem->Alarm_OutdoorWindchillLow, Cfg_Double );    
    (void) IniFiler_SearchCfg( iniFileName, "Alarming", "IsRaining", &aSystem->Alarm_Raining, Cfg_Boolean );   

    (void) IniFiler_SearchCfg( iniFileName, "Database", "hostName", &(aSystem->db.hostName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "Database", "userName", &(aSystem->db.userName[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "Database", "password", &(aSystem->db.password[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "Database", "schema", &(aSystem->db.schema[ 0 ]), Cfg_String );
    (void) IniFiler_SearchCfg( iniFileName, "Database", "logAlarms", &(aSystem->db.logAlarms), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "logReadings", &(aSystem->db.logReadings), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "logHistory", &(aSystem->db.logHistory), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "dropAlarmTable", &(aSystem->db.dropAlarmTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "createAlarmTable", &(aSystem->db.createAlarmTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "dropReadingsTable", &(aSystem->db.dropReadingsTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "createReadingsTable", &(aSystem->db.createReadingsTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "dropHistoryTable", &(aSystem->db.dropHistoryTable), Cfg_Boolean );    
    (void) IniFiler_SearchCfg( iniFileName, "Database", "createHistoryTable", &(aSystem->db.createHistoryTable), Cfg_Boolean );
}

