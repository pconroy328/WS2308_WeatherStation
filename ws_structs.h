/* 
 * File:   wssstructs.h
 * Author: patrick.conroy
 *
 * Created on April 5, 2014, 11:48 AM
 */

#ifndef WSSSTRUCTS_H
#define	WSSSTRUCTS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

//
//
// Structure that holds information system wide
#define     MAX_INI_STRLEN      256 

    
typedef struct  Database_Parameters {
    char    hostName[ MAX_INI_STRLEN ];
    char    userName[ MAX_INI_STRLEN ];
    char    password[ MAX_INI_STRLEN ];
    char    schema[ MAX_INI_STRLEN ];
    int     logAlarms;
    int     logReadings;
    int     logHistory;
    int     dropAlarmTable; 
    int     createAlarmTable;
    int     dropReadingsTable; 
    int     createReadingsTable;
    int     dropHistoryTable; 
    int     createHistoryTable;
    int     maxMinutesOfHistoryStored;
} Database_Parameters_t;    
    
   
typedef struct  MQTT_Parameters {
    //
    // MQTT Specific Information
    int     logEventsToMQTT;
    char    mqttBrokerHost[ MAX_INI_STRLEN ];
    int     mqttPortNumber;
    int     mqttKeepAliveValue;
    int     mqttQoS;
    char    mqttParentTopic[ MAX_INI_STRLEN ];
    char    mqttStatusTopic[ MAX_INI_STRLEN ];
    char    mqttAlarmTopic[ MAX_INI_STRLEN ];
    int     mqttRetainMsgs;
    int     useJSON;
} MQTT_Parameters_t;
    

typedef struct WS2308System {
    char            INIFileName[ MAX_INI_STRLEN ];
    char            logFileName[ MAX_INI_STRLEN ];
    int             debugLevel;
    char            ws2308DeviceName[ MAX_INI_STRLEN ];
    int             readInterval;
    int             discardBadReadings;
    double          absPressureCorrection;
    int             ignoreAtomicClock;
    
    int             sendToHomeAssistant;
    
    int             logToMQTT;
    int             logToDatabase;

    char            MQTTServerName[ MAX_INI_STRLEN ];
    int             MQTTPort;
    char            MQTTParentTopic[ MAX_INI_STRLEN ];
    char            MQTTStatusTopic[ MAX_INI_STRLEN ];
    char            MQTTAlarmTopic[ MAX_INI_STRLEN ];
    int             MQTTQoS;
    int             MQTTRetainMsgs;
    int             MQTTKeepAliveValue;
    int             MQTTUseJSON;
    int             MQTTUseISO8601;

    
    double          Alarm_IndoorTempLow;
    double          Alarm_IndoorTempHigh;
    double          Alarm_IndoorHumidityLow;
    double          Alarm_IndoorHumidityHigh;
    double          Alarm_OutdoorTempLow;
    double          Alarm_OutdoorTempHigh;
    double          Alarm_OutdoorHumidityLow;
    double          Alarm_OutdoorHumidityHigh;
    double          Alarm_OutdoorWindspeedHigh;
    double          Alarm_OutdoorWindchillLow;
    int             Alarm_Raining;
    
    
    Database_Parameters_t   db;
} WS2308System_t;   


//
// Some defines for which Alarm we're reporting (if any)
// Note - pick a scheme that allows these to be "OR"d together
//  for example - it could be COLD and raining!
#define ALARM_NONE                      0
#define ALARM_INDOORTEMPLOW             0b0000000000000001
#define ALARM_INDOORTEMPHIGH            0b0000000000000010
#define ALARM_INDOORHUMIDITYLOW         0b0000000000000100
#define ALARM_INDOORHUMIDITYHIGH        0b0000000000001000
#define ALARM_OUTDOORTEMPLOW            0b0000000000010000
#define ALARM_OUTDOORTEMPHIGH           0b0000000000100000
#define ALARM_OUTDOORHUMIDITYLOW        0b0000000001000000
#define ALARM_OUTDOORHUMIDITYHIGH       0b0000000010000000
#define ALARM_OUTDOORWINDSPEEDHIGH      0b0000000100000000
#define ALARM_RAINING                   0b0000001000000000
#define ALARM_OUTDOORWINDCHILLLOW       0b0000010000000000

//
// By packaging up each reading in a structure, it'll be easier to handle
typedef struct  weatherDatum {
    char            date[ 20 ];
    char            time[ 20 ];
    char            tendency[ 80 ];         // "Rising", "Falling"...
    char            forecast[ 80 ];         // "Sunny, Cloudy, Rain..."

    struct  indoorData {
        double      temp;                   // indoor temp
        double      humidity;
    } indoor;

    struct  outdoorData {
        double      temp;                   // outdoor temp
        double      humidity;
        double      windspeed;              // lets hope wind blows only outdoors
        double      windheading;
        double      pressure;
        double      dewpoint;
        double      windchill;

        struct  rainData {
            double  lastHour;
            double  lastDay;
            double  total;
        } rain;
    } outdoor;
} weatherDatum_t;



#ifdef	__cplusplus
}
#endif

#endif	/* WSSSTRUCTS_H */

