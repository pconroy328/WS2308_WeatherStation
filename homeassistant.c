#include <stdio.h>
#include <stdlib.h>
#include <log4c.h>
#include <cjson/cJSON.h>
#include <string.h>

#include "mqtt.h"
#include "homeassistant.h"
    
#include <uthash/utlist.h>
    


// -----------------------------------------------------------------------------

//static  void                decodeJSONMessage( weatherStation_t *aStation, const char *jsonPayload );
static  void        sendConfigurationMessages( );
static  void        sendStation_OnlineStatus (void);

static  const int   createDataTopic_TemperatureOutdoor( char *configTopic, const int bufSize, int stationID );
static  const int   createDataTopic_HumidityOutdoor( char *configTopic, const int bufSize, int stationID );
static  const int   createDataTopic_Wind( char *configTopic, const int bufSize, int stationID );
static  const int   createDataTopic_StationState( char *configTopic, const int bufSize, int stationID );



/*
 * This is the MQTT packet with the weather data that we're telling Home Assistant how
 * to parse
 * 
 * { "topic":"WS2308/STATUS", 
        "version":"1.1", 
        "dateTime":"2023-05-11T20:59:17-0600" ,
        "tendency":"Falling" ,
        "forecast":"Rainy  " ,
        "indoorTemp":65.8 ,                         // degrees F
        "indoorHumdity":62.0 ,                      // percent
        "outdoorTemp":50.2 ,
        "outdoorHumdity":92.0 ,
        "outdoorPressure":29.7 ,                    // inches of Hg
        "outdoorWindChill":50.2 ,
        "windSpeed":5.6 ,                           // mph
        "windHeading":22.5 ,                        // degrees
        "rainLastHour":0.04 ,                       // inches
        "rainLastDay":1.37 ,
        "rainTotal":31.94 ,
        "stationDate":"05/11/23" ,"stationTime":"20:58:00" }        // the date/time pulled from the station clock
*/


void HomeAssistant_initialize (const WS2308System_t *aSystem)
{
    sendConfigurationMessages();
    sendStation_OnlineStatus();
}

void HomeAssistant_sendData (const WS2308System_t *aSystem, weatherDatum_t *datum)
{
    char    topic[ MAX_CONFIGMSG_LEN ];
    char    msg[ MAX_CONFIGMSG_LEN ];
    int     msgSize = 0;
    
    return;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/* This is what we're after...
 * Our unique station ID, is 28. We got that value from the RTL433 software.
 * It just needs to be unique from other WS2310 stations in range of our receiver
 *
 *  "homeassistant/sensor/LaCrosse-WS2310-28/temperature_outdoor/config"
 * 
 */
static
const int   createConfigTopic_TemperatureOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/temperature_outdoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
/* This is what we're after...
 * Our unique station ID, is 28. We got that value from the RTL433 software.
 * It just needs to be unique from other WS2310 stations in range of our receiver
 * 
    {
      "availability":[
         {
            "topic":"home/LaCrosse-WS2310-28/state",                // tells HA to watch for online/offline msgs here
            "value_template":"{{ value_json.state }}"               // tells HA it'll be a json "state" attribute
         }
      ],
      "device":{
         "identifiers":[
            "LaCrosse-WS2310-28"
         ],
         "manufacturer":"LaCrosse",
         "model":"Weather Station",
         "name":"LaCrosse-WS2310-28"
      },
      "device_class":"temperature",                             // simply picks an icon for the UI
      "enabled_by_default":true,
      "expire_after":3600,                                      // if no new data after 3600 seconds, HA will mark it offline
      "force_update":true,
      "name":"Weather Station 28 temperature",
      "state_class":"measurement",
      "state_topic":"WS2308/STATUS",                            // STATE really means the Data. Data will be sent on this topic
      "unique_id":"LaCrosse-WS2310-28_temperature",
      "unit_of_measurement":"°F",
      "value_template":"{{ value_json.outdoorTemp}}"            // parse the JSON looking for "outdoorTemp" attribute
   }
 */
static
const int   createConfigMsg_TemperatureOutdoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"temperature\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d temperature outdoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_temperature_outdoor\","
                        "\"unit_of_measurement\":\"°F\", \"value_template\":\"{{ value_json.outdoorTemp}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

static
const int   createConfigTopic_TemperatureIndoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/temperature_indoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_TemperatureIndoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"temperature\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d temperature indoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_temperature_indoor\","
                        "\"unit_of_measurement\":\"°F\", \"value_template\":\"{{ value_json.indoorTemp}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_HumidityOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/humidity_outdoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_HumidityOutdoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"humidity\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d humidity outdoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_humidity_outdoor\","
                        "\"unit_of_measurement\":\"%\", \"value_template\":\"{{ value_json.outdoorHumdity}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

static
const int   createConfigTopic_HumidityIndoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/humidity_indoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_HumidityIndoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"humidity\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d humidity indoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_humidity_indoor\","
                        "\"unit_of_measurement\":\"%\", \"value_template\":\"{{ value_json.indoorHumdity}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_PressureOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/pressure_outdoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_PressureOutdoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"pressure\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d pressure outdoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_pressure_outdoor\","
                        "\"unit_of_measurement\":\"in Hg\", \"value_template\":\"{{ value_json.outdoorPressure}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_WindchillOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/windchill_outdoor/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_WindchillOutdoor (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"temperature\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d windchill outdoor\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_windchill_outdoor\","
                        "\"unit_of_measurement\":\"°F\", \"value_template\":\"{{ value_json.outdoorWindchill}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_WindDirection (char *configTopic, const int bufSize, int stationID)
{
    //  Create the MQTT Device Discovery TOPIC for the wind direction sensor
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/winddirection/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_WindDirection (char *configMsg, const int bufSize, int stationID)
{
    //  Create the MQTT Device Discovery MESSAGE for the wind direction sensor
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d/windirection\" },"
                        "\"device_class\":\"speed\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d winddirection\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_winddirection\","
                        "\"unit_of_measurement\":\"°\", \"value_template\":\"{{ value_json.windHeading}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_WindSpeed (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/windspeed/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_WindSpeed (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"speed\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d windspeed\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_windspeed\","
                        "\"unit_of_measurement\":\"mph\", \"value_template\":\"{{ value_json.windSpeed}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
static
const int   createConfigTopic_RainLastHour (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/rain_last_hour/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_RainLastHour (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"speed\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d rain last hour\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_rain_last_hour\","
                        "\"unit_of_measurement\":\"in\", \"value_template\":\"{{ value_json.rainLastHour}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

//------------------------------------------------------------------------------
static
const int   createConfigTopic_RainLastDay (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/rain_last_day/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_RainLastDay (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"speed\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d rain last day\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_rain_last_day\","
                        "\"unit_of_measurement\":\"in\", \"value_template\":\"{{ value_json.rainLastDay}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigTopic_RainTotal (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/sensor/LaCrosse-WS2310-%d/rain_total/config";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    HOMEASSISTANT_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createConfigMsg_RainTotal (char *configMsg, const int bufSize, int stationID)
{
    const   char    *configTemplate =  "{\"availability\":[ { \"topic\":\"home/LaCrosse-WS2310-%d/state\", \"value_template\":\"{{ value_json.state }}\" } ],"
                        "\"device\":{ \"identifiers\":[ \"LaCrosse-WS2310-%d\" ],"
                        "\"manufacturer\":\"LaCrosse\", \"model\":\"Weather Station\", "
                        "\"name\":\"LaCrosse-WS2310-%d\" },"
                        "\"device_class\":\"speed\", \"enabled_by_default\":true, \"expire_after\" : 3600, \"force_update\" : true,"
                        "\"name\":\"Weather Station %d rain total\", \"state_class\":\"measurement\","
    
                        "\"state_topic\":\"WS2308/STATUS\","
    
    
                        "\"unique_id\":\"LaCrosse-WS2310-%d_rain_total\","
                        "\"unit_of_measurement\":\"in\", \"value_template\":\"{{ value_json.rainTotal}}\"}";

    int num = snprintf( configMsg, bufSize,
                    configTemplate,
                    stationID, stationID,
                    stationID, stationID,
                    stationID, stationID
                );
    
    return num;
}


// -----------------------------------------------------------------------------
void    HomeAssistant_handleMessage (const char *model, const int stationID, char *jsonPayload)
{
}

#if 0
// -----------------------------------------------------------------------------
static
const int   createDataTopic_TemperatureOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/LaCrosse-WS2310-%d/temperature_outdoor";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    STATIONDATA_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
const int   createDataTopic_HumidityOutdoor (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/LaCrosse-WS2310-%d/humidity_outdoor";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    STATIONDATA_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}
// -----------------------------------------------------------------------------
static
const int   createDataTopic_Wind (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/LaCrosse-WS2310-%d/wind";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    STATIONDATA_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}
#endif 

// -----------------------------------------------------------------------------
static
const int   createDataTopic_StationState (char *configTopic, const int bufSize, int stationID)
{
    const char  *topicTemplate = "%s/LaCrosse-WS2310-%d/state";

    int num = snprintf( configTopic, bufSize,
                    topicTemplate,
                    STATIONDATA_ROOT_TOPIC,
                    stationID
                );
    
    return num;
}

// -----------------------------------------------------------------------------
static
void    sendStation_OnlineStatus (void)
{
    char    topic[ MAX_CONFIGMSG_LEN ];
    char    msg[ MAX_CONFIGMSG_LEN ];
    int     msgSize = 0;
    
    //
    //  Station State - online/offline
    (void) createDataTopic_StationState( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = snprintf( msg, MAX_CONFIGMSG_LEN, "{ \"state\" : \"online\" }");
    
    Logger_LogDebug( "Sending Station Status: OnLine\n" );
    MQTT_Publish( topic, msg, msgSize, FALSE );    
}

// -----------------------------------------------------------------------------
static  
void    sendConfigurationMessages (void)
{
    Logger_LogDebug( "Publishing configuration data\n" );
    
    char    topic[ MAX_CONFIGMSG_LEN ];
    char    msg[ MAX_CONFIGMSG_LEN ];
    int     msgSize = 0;
    
    //
    // Temperature - Config
    (void) createConfigTopic_TemperatureOutdoor( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_TemperatureOutdoor( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Temperature Outdoor\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    (void) createConfigTopic_TemperatureIndoor( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_TemperatureIndoor( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Temperature Indoor\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );
    
    //
    // Humidity - Config
    (void) createConfigTopic_HumidityOutdoor( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_HumidityOutdoor( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Humidity Outdoor\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    //
    // Pressure - Config
    (void) createConfigTopic_PressureOutdoor( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_PressureOutdoor( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Pressure Outdoor\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );


    //
    // Windchill - Config
    (void) createConfigTopic_WindchillOutdoor( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_WindchillOutdoor( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Windchill Outdoor\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    
    //
    // Windspeed - Config
    (void) createConfigTopic_WindSpeed( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_WindSpeed( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Windspeed\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    //
    // Wind direction - Config
    (void) createConfigTopic_WindDirection( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_WindDirection( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Wind Direction\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );


    //
    // Rain - Config
    (void) createConfigTopic_RainLastHour( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_RainLastHour( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Rain Last Hour\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    (void) createConfigTopic_RainLastDay( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_RainLastDay( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Rain Last Day\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );

    (void) createConfigTopic_RainTotal( topic, MAX_CONFIGMSG_LEN, STATION_ID );
    msgSize = createConfigMsg_RainTotal( msg, MAX_CONFIGMSG_LEN, STATION_ID );
    
    Logger_LogDebug( "Rain Total\n" );
    Logger_LogDebug( "  Topic:[%s]\n", topic );
    Logger_LogDebug( "  Payload:[%s]\n", msg );
    MQTT_Publish( topic, msg, msgSize, FALSE );
}

// -----------------------------------------------------------------------------
