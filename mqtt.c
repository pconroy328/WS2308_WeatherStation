
/* 
 * File:   mqtt.c
 * Author: patrick conroy
 *
 * Created on September 17, 2013, 10:03 AM
 * (C) 2013 Patrick Conroy
 * 
 * 12Mar2104    Adding MAC address to MQTT Payload
 */


// ----------------------------------------------------------------------------
//
//  mqtt.c      code to connect to a MQTT broker and send off events
//
//      This code uses Mosquitto v3.1.  If you install the Ubuntu packages you may
//      get the older versioN!  (I did). Follow the instructions on mosquitto.org
//      to install the latest version
//

//
// Don't forget to download and build mosquitto from source!
//      Don't forget you'll need libssl-dev from Synaptic
//      And g++ (sudo apt-get install g++)

#include <mosquitto.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include "mqtt.h"
#include "logger.h"

static  int                     MQTT_Connected;
static  struct  mosquitto       *mosq = NULL;
static  int                     totalMosquittoErrors = 0;
static  int                     consecutiveMosquittoErrors = 0;
static  int                     useISO8601 = FALSE;



//
// Forward declarations
static  void    my_message_callback( struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message );
static  void    my_connect_callback( struct mosquitto *mosq, void *userdata, int result );
static  void    my_log_callback( struct mosquitto *mosq, void *userdata, int level, const char *str );
static  char    *getCurrentDateTime( void );


// ------------------------------------------------------------------------------
static
void my_message_callback (struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    Logger_LogDebug ( "MQTT Message Callback. Payload length: %d\n", message->payloadlen );
    if (message->payloadlen > 0) {
        Logger_LogDebug( "Topic [%s] Data [%s]\n", (char *) message->topic, (char *) message->payload);
        
    } else {
        Logger_LogDebug(" MQTT Topic [%s] with no payload\n", message->topic );
    }
}


//
//      Needs to be of type 
// void mosquitto_connect_callback_set(struct mosquitto *mosq, void (*on_connect)(void *, int));
 
// ------------------------------------------------------------------------------
static
void my_connect_callback (struct mosquitto *mosq, void *userdata, int result)
{
    Logger_LogInfo( "MQTT Connection Callback. Result: %d\n", result );

    if(!result) {
        /* Subscribe to broker information topics on successful connect. */
        //mosquitto_subscribe( mosq, NULL, "$SYS/#", 2 );
        MQTT_Connected = TRUE;
        
    } else {
        Logger_LogError( "MQTT Connection failed\n" );
        MQTT_Connected = FALSE;
    }
}

// ------------------------------------------------------------------------------
void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;

    Logger_LogDebug( "MQTT Subscriber Callback (mid: %d): %d", mid, granted_qos[ 0 ] );
    for(i = 1; i < qos_count; i++){
        Logger_LogDebug( ", %d", granted_qos[ i ] );
    }
    
    Logger_LogDebug( "\n", 0 );
}

// ------------------------------------------------------------------------------
static
void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    /* Print all log messages regardless of level. */
    // debug_print("MQTT Logging: [%s]\n", str );
}



// ---------------------------------------------------------------------------
void    MQTT_setDefaults (WS2308System_t *aSystem, char *brokerHostName)
{
    assert( brokerHostName != NULL ); 
    
    aSystem->MQTTServerName;
    
    strncpy( &(aSystem->MQTTServerName[ 0 ]),
             brokerHostName, 
             sizeof aSystem->MQTTServerName );
    
    aSystem->MQTTPort = (aSystem->MQTTPort == 0 ? 1883 : aSystem->MQTTPort);
    aSystem->MQTTKeepAliveValue = (aSystem->MQTTKeepAliveValue == 0 ? 60 : aSystem->MQTTKeepAliveValue);
    
    aSystem->MQTTRetainMsgs = false;
    aSystem->MQTTQoS = 0;
    
    useISO8601 = aSystem->MQTTUseISO8601;
    // QoS of zero is ok
}
// ----------------------------------------------------------------------------
void    MQTT_initialize (WS2308System_t *aSystem)
{
    char    id[30];
    int     i;
    bool    clean_session = true;

    MQTT_Connected = FALSE;
    
    if (!aSystem->logToMQTT)
        return;
    
      
    //
    // If you get errors complaining about arguments and mismatched prototypes
    //      odds are you're not pulling down the latest version of mosquitto
    //
    Logger_LogInfo( "MQTT Initialization starting.\n" );
    mosquitto_lib_init();
    
    //
    // Example code shows the first parameter is "id" - but I'm getting EINVAL so I
    // switched it to NULL and it's working     
    mosq = mosquitto_new( NULL, clean_session, NULL );
    
    if(!mosq) {
        Logger_LogWarning( "Unable to instantiate an MQTT instance!\n" );
        Logger_LogWarning( "MQTT Broker Host: [%s], Port: %d, KeepAlive: %d\n", 
                            aSystem->MQTTServerName,
                            aSystem->MQTTPort,
                            aSystem->MQTTKeepAliveValue );

        aSystem->logToMQTT = FALSE;
        return;
    }
    
    mosquitto_log_callback_set( mosq, my_log_callback );
    mosquitto_connect_callback_set( mosq, my_connect_callback );
    mosquitto_message_callback_set( mosq, my_message_callback );
    mosquitto_subscribe_callback_set( mosq, my_subscribe_callback );

    if (mosquitto_connect(  mosq, 
                            aSystem->MQTTServerName,
                            aSystem->MQTTPort,
                            aSystem->MQTTKeepAliveValue ) ) {
        Logger_LogWarning( "Unable to connect MQTT to broker!\n" );
        Logger_LogWarning( "MQTT Broker Host: [%s], Port: %d, KeepAlive: %d\n", 
                            aSystem->MQTTServerName,
                            aSystem->MQTTPort,
                            aSystem->MQTTKeepAliveValue );
        
        aSystem->logToMQTT = FALSE;
        return;
    }
    
    MQTT_Connected = TRUE;
}

// ----------------------------------------------------------------------------
void    MQTT_teardown ()
{
    if (MQTT_Connected) {
        mosquitto_destroy( mosq );
        mosquitto_lib_cleanup();
    }
    Logger_LogInfo( "MQTT Terminated.\n" );
}

// ----------------------------------------------------------------------------
int MQTT_sendReceive (WS2308System_t *aSystem)
{
    if (!MQTT_Connected)
        return MQTT_NOT_CONNECTED;
    
    //
    // Last parameter cannot be ZERO!
    int error = mosquitto_loop( mosq, -1, 10 );
    if (error != MOSQ_ERR_SUCCESS) {
        Logger_LogError( "MQTT SendReceive - error Code : %d\n", error  );
        switch (error) {
            case    MOSQ_ERR_INVAL:     Logger_LogError( "ERROR: mosquitto_loop() says INVAL\n", 0 ); break;
            case    MOSQ_ERR_NOMEM:     Logger_LogError( "ERROR: mosquitto_loop() says NOMEM\n", 0 ); break;
            case    MOSQ_ERR_NO_CONN:   Logger_LogError( "ERROR: mosquitto_loop() says NO CONN\n", 0 ); break;
            case    MOSQ_ERR_CONN_LOST: Logger_LogError( "ERROR: mosquitto_loop() says CONN LOST\n", 0 ); break;
            case    MOSQ_ERR_PROTOCOL:  Logger_LogError( "ERROR: mosquitto_loop() says PROTOCOL\n", 0 ); break;
            case    MOSQ_ERR_ERRNO:     Logger_LogError( "ERROR: mosquitto_loop() says ERRNO\n", 0 ); break;
                
        }
    }
    
    //
    // Let's see if we can do something about the errors that might occur
    if (error != MOSQ_ERR_SUCCESS) {
        totalMosquittoErrors += 1;
        consecutiveMosquittoErrors += 1;
        Logger_LogWarning( "MQTT error: %d  Total Number of MQTT Errors since initialization: %d\n", error, totalMosquittoErrors );
                
        //
        // Was it intermittent or consistent?
        if (consecutiveMosquittoErrors > 2) {
            Logger_LogError( "MQTT reporting errors (at least) twice in a row. Last MQTT error: %d\n", error );
            Logger_LogError( "Total Number of MQTT Errors since initialization: %d\n", totalMosquittoErrors );
            Logger_LogError( "Attempting to re-initialize MQTT systems\n" );
            
            MQTT_handleError( aSystem, error );             
        }
    } else {
        consecutiveMosquittoErrors = 0;
    }
    return error;
}

// -----------------------------------------------------------------------------
static  char    currentDateTimeBuffer[ 80 ];
static
char    *getCurrentDateTime (void)
{
    //
    // Something quick and dirty... Fix this later - thread safe
    time_t  current_time;
    struct  tm  *tmPtr;
 
    memset( currentDateTimeBuffer, '\0', sizeof currentDateTimeBuffer );
    
    /* Obtain current time as seconds elapsed since the Epoch. */
    current_time = time( NULL );
    if (current_time > 0) {
        /* Convert to local time format. */
        tmPtr = localtime( &current_time );
 
        if (tmPtr != NULL) {
            if (useISO8601) {
                strftime( currentDateTimeBuffer,
                    sizeof currentDateTimeBuffer,
                    "%FT%T%z",
                    tmPtr );
                      
            } else {
                strftime( currentDateTimeBuffer,
                    sizeof currentDateTimeBuffer,
                    "%F %T %z",
                    tmPtr );
            }
        }
    }
    
    return &currentDateTimeBuffer[ 0 ];
}

// -----------------------------------------------------------------------------
int MQTT_handleError (WS2308System_t *aSystem, int errorCode) 
{
    Logger_LogError( "ERROR detected in MQTT [%d]\n", errorCode );
    Logger_LogError( "Disconnecting...\n" );
    MQTT_teardown();
    sleep( 1 );
    Logger_LogError( "Reconnecting...\n" );
    MQTT_initialize( aSystem );
}


// -----------------------------------------------------------------------------
static
int     sendNumericAlarm (WS2308System_t *aSystem, char *alarmMessage, double temp)
{
    int     duration = 0;
    int     length = 0;
    char    payload[ 1024 ];
    char    combinedTopic[ (MAX_INI_STRLEN * 2) + 1 ];
    
    memset( payload, '\0', sizeof payload );

    //      WS2308/ALARM | <datetime> | OUTDOOR TEMP HOT | xx.x  |
    //      WS2308/ALARM | <datetime> | OUTDOOR TEMP COLD | xx.x  |
    
    //
    // Put the parent and Alarm topic strings together
    snprintf( combinedTopic, sizeof combinedTopic, "%s/%s", aSystem->MQTTParentTopic, aSystem->MQTTAlarmTopic );
    
    if (!aSystem->MQTTUseJSON) {
        //
        // Now the message
        length = snprintf( payload, sizeof payload,
                    "%s | %s | %s | %0.1f |",
                    combinedTopic,
                    getCurrentDateTime(),
                    alarmMessage,
                    temp );
    } else {
        //
        // Send the data in a JSON packet: ' { "topic": Topic, datetime":"<dt in ISO8601>", "alarmMsg":"<msg>", "value":<value> }'
        
        length = snprintf( payload, sizeof payload,
                    "{ \"topic\":\"%s\", \"datetime\":\"%s\" , \"alarmMsg\":\"%s\" , \"value\":%0.1f }",
                    combinedTopic,
                    getCurrentDateTime(),
                    alarmMessage,
                    temp );
    }
    
    int     messageID;
    int result = mosquitto_publish( mosq, &messageID, 
                                    combinedTopic, 
                                    length, 
                                    payload, 
                                    aSystem->MQTTQoS, 
                                    aSystem->MQTTRetainMsgs );
    
    if (result != 0) {
        Logger_LogWarning( "MQTT Create Alarm - attempt to publish an alarm failed. Result: %d  Message: [%s]\n", result, payload );
        MQTT_handleError( aSystem, result ); 
    }
    
    return result;
}


// -----------------------------------------------------------------------------
static
int     sendWindAlarm (WS2308System_t *aSystem, char *alarmMessage, double windspeed, double windHeading)
{
    int     duration = 0;
    int     length = 0;
    char    payload[ 1024 ];
    char    combinedTopic[ (MAX_INI_STRLEN * 2) + 1 ];
    
    memset( payload, '\0', sizeof payload );

    //
    // Put the parent and Alarm topic strings together
    snprintf( combinedTopic, sizeof combinedTopic, "%s/%s", aSystem->MQTTParentTopic, aSystem->MQTTAlarmTopic );
    
    //
    // Now the message
    if (!aSystem->MQTTUseJSON) {
        length = snprintf( payload, sizeof payload,
                "%s | %s | %s | %0.1f | %03.1f |",
                combinedTopic,
                getCurrentDateTime(),
                alarmMessage,
                windspeed,
                windHeading );
    
    } else {
        length = snprintf( payload, sizeof payload,
                "{ \"topic\":\"%s\", \"datetime\":\"%s\" , \"alarmMsg\":\"%s\" , \"speed\":%0.1f , \"heading\":%03.1f }",
                combinedTopic,
                getCurrentDateTime(),
                alarmMessage,
                windspeed,
                windHeading );
    }
    
    int     messageID;
    int result = mosquitto_publish( mosq, &messageID, 
                                    combinedTopic, 
                                    length, 
                                    payload, 
-                                   aSystem->MQTTQoS, 
                                    aSystem->MQTTRetainMsgs );
    
    if (result != 0) {
        Logger_LogWarning( "MQTT Create Alarm - attempt to publish an alarm failed. Result: %d  Message: [%s]\n", result, payload );
        MQTT_handleError( aSystem, result ); 
    }
    
    return result;
}

// -----------------------------------------------------------------------------
static
int     sendRainAlarm (WS2308System_t *aSystem, char *alarmMessage, double lastHour, double lastDay, double total)
{
    int     duration = 0;
    int     length = 0;
    char    payload[ 1024 ];
    char    combinedTopic[ (MAX_INI_STRLEN * 2) + 1 ];
    
    memset( payload, '\0', sizeof payload );

    //
    // Put the parent and Alarm topic strings together
    snprintf( combinedTopic, sizeof combinedTopic, "%s/%s", aSystem->MQTTParentTopic, aSystem->MQTTAlarmTopic );
    
    //
    // Now the message
    if (!aSystem->MQTTUseJSON) {
        length = snprintf( payload, sizeof payload,
                "%s | %s | %s | %0.2f | %0.2f | %0.2f |",
                combinedTopic,
                getCurrentDateTime(),
                alarmMessage,
                lastHour,
                lastDay,
                total );
    } else {
        length = snprintf( payload, sizeof payload,
                "{ \"topic\":\"%s\", \"datetime\":\"%s\" , \"alarmMsg\":\"%s\" , \"lastHour\":%0.2f , \"lastDay\":%0.2f , \"total\":%0.2f }",
                combinedTopic,
                getCurrentDateTime(),
                alarmMessage,
                lastHour,
                lastDay,
                total );
    }    
    
    int     messageID;
    int result = mosquitto_publish( mosq, &messageID, 
                                    combinedTopic, 
                                    length, 
                                    payload, 
                                    aSystem->MQTTQoS, 
                                    aSystem->MQTTRetainMsgs );
    
    if (result != 0) {
        Logger_LogWarning( "MQTT Create Alarm - attempt to publish an alarm failed. Result: %d  Message: [%s]\n", result, payload );
        MQTT_handleError( aSystem, result ); 
    }
    
    return result;
}

// -----------------------------------------------------------------------------
int    MQTT_createWeatherAlarm (WS2308System_t *aSystem, int alarmCode, weatherDatum_t *weatherRec)
{
    if (!MQTT_Connected)
        return MQTT_NOT_CONNECTED;

    //
    //
    //  Going to be one of:
    //      WS2308/ALARM | <datetime> | WINDSPEED | xx.xx mph | xxx.x * |
    //      WS2308/ALARM | <datetime> | OUTDOOR TEMP HOT | xx.x F |
    //      WS2308/ALARM | <datetime> | OUTDOOR TEMP COLD | xx.x F |
    //      WS2308/ALARM | <datetime> | OUTDOOR HUMIDTY LOW | xx % |
    //      WS2308/ALARM | <datetime> | OUTDOOR HUMIDTY HIGH | xx % |
    //      WS2308/ALARM | <datetime> | INDOOR HUMIDTY LOW | xx % |
    //      WS2308/ALARM | <datetime> | INDOOR HUMIDTY HIGH | xx % |
    //      WS2308/ALARM | <datetime> | INDOOR TEMP HOT | xx.x F |
    //      WS2308/ALARM | <datetime> | INDOOR TEMP COLD | xx.x F |
    //      WS2308/ALARM | <datetime> | RAIN | xx.x | xx.x | xx.x |
    
    if (alarmCode == ALARM_NONE)
        return TRUE;
    
    if ((alarmCode & ALARM_INDOORTEMPLOW) != 0)
        (void) sendNumericAlarm( aSystem, "INDOOR TEMP LOW", weatherRec->indoor.temp );
    if ((alarmCode & ALARM_INDOORTEMPHIGH) != 0)
        (void) sendNumericAlarm( aSystem, "INDOOR TEMP HIGH", weatherRec->indoor.temp );

    if ((alarmCode & ALARM_INDOORHUMIDITYLOW) != 0)
        (void) sendNumericAlarm( aSystem, "INDOOR HUMIDITY LOW", weatherRec->indoor.humidity );
    if ((alarmCode & ALARM_INDOORHUMIDITYHIGH) != 0)
        (void) sendNumericAlarm( aSystem, "INDOOR HUMIDITY HIGH", weatherRec->indoor.humidity );

    if ((alarmCode & ALARM_OUTDOORTEMPLOW) != 0)
        (void) sendNumericAlarm( aSystem, "OUTDOOR TEMP LOW", weatherRec->outdoor.temp );
    if ((alarmCode & ALARM_OUTDOORTEMPHIGH) != 0)
        (void) sendNumericAlarm( aSystem, "OUTDOOR TEMP HIGH", weatherRec->outdoor.temp );

    if ((alarmCode & ALARM_OUTDOORHUMIDITYLOW) != 0)
        (void) sendNumericAlarm( aSystem, "OUTDOOR HUMIDITY LOW", weatherRec->outdoor.humidity );
    if ((alarmCode & ALARM_OUTDOORHUMIDITYHIGH) != 0)
        (void) sendNumericAlarm( aSystem, "OUTDOOR HUMIDITY HIGH", weatherRec->outdoor.humidity );
    
    if ((alarmCode & ALARM_OUTDOORWINDSPEEDHIGH) != 0)
        (void) sendWindAlarm( aSystem, "OUTDOOR WINDSPEED HIGH", weatherRec->outdoor.windspeed, weatherRec->outdoor.windheading );
    if ((alarmCode & ALARM_OUTDOORWINDCHILLLOW) != 0)
        (void) sendNumericAlarm( aSystem, "OUTDOOR HUMIDITY HIGH", weatherRec->outdoor.windchill );

    if ((alarmCode & ALARM_RAINING) != 0)
        (void) sendRainAlarm( aSystem, "RAIN", weatherRec->outdoor.rain.lastHour, weatherRec->outdoor.rain.lastDay, weatherRec->outdoor.rain.total );
}

// -----------------------------------------------------------------------------
int    MQTT_createWeatherStatus (WS2308System_t *aSystem, weatherDatum_t *datum)
{
    char    combinedTopic[ (MAX_INI_STRLEN * 2) + 1 ];
    char    buffer[ 2048 ];
    
    
    if (!MQTT_Connected)
        return MQTT_NOT_CONNECTED;
    
    //
    // Put the parent and Alarm topic strings together
    snprintf( combinedTopic, sizeof combinedTopic, "%s/%s", aSystem->MQTTParentTopic, aSystem->MQTTStatusTopic );
    memset( buffer, '\0', sizeof buffer );
    
    int length = 0;
    if (!aSystem->MQTTUseJSON) {
        length = snprintf( buffer, sizeof buffer,
                "%s | %s | %s | %s | iTemp %3.1f | iHum %3.1f | oTemp %3.1f | oHum %3.1f | oPress %3.1f | oWChill %3.1f | oWSpd %3.1f | oWHead %3.1f | hrRain %2.2f | dayRain %2.2f | totRain %2.2f | %s | %s |",
                combinedTopic,
                getCurrentDateTime(),
                
                datum->tendency, datum->forecast,
                datum->indoor.temp, datum->indoor.humidity,
                datum->outdoor.temp, datum->outdoor.humidity, datum->outdoor.pressure , datum->outdoor.windchill,
                datum->outdoor.windspeed, datum->outdoor.windheading,
                datum->outdoor.rain.lastHour,datum->outdoor.rain.lastDay,datum->outdoor.rain.total,
                datum->date, datum->time
            );
        
    } else {
        length = snprintf( buffer, sizeof buffer,
                "{ \"topic\":\"%s\", \"datetime\":\"%s\" ,\"tendency\":\"%s\" ,\"forecast\":\"%s\" ,\
\"indoorTemp\":%3.1f ,\"indoorHumdity\":%3.1f ,\
\"outdoorTemp\":%3.1f ,\"outdoorHumdity\":%3.1f ,\"outdoorPressure\":%3.1f ,\"outdoorWindChill\":%3.1f ,\
\"windSpeed\":%3.1f ,\"windHeading\":%3.1f ,\
\"rainLastHour\":%2.2f ,\"rainLastDay\":%2.2f ,\"rainTotal\":%2.2f ,\
\"stationDate\":\"%s\" ,\"stationTime\":\"%s\" }",
                combinedTopic,
                getCurrentDateTime(),
                
                datum->tendency, datum->forecast,
                datum->indoor.temp, datum->indoor.humidity,
                datum->outdoor.temp, datum->outdoor.humidity, datum->outdoor.pressure , datum->outdoor.windchill,
                datum->outdoor.windspeed, datum->outdoor.windheading,
                datum->outdoor.rain.lastHour,datum->outdoor.rain.lastDay,datum->outdoor.rain.total,
                datum->date, datum->time
            );
        
    }
    //
    //   MQTT defines three levels of Quality of Service (QoS). The QoS defines how hard the broker/client will try 
    //  to ensure that a message is received. Messages may be sent at any QoS level, and clients may attempt to subscribe 
    //  to topics at any QoS level. This means that the client chooses the maximum QoS it will receive. For example, 
    //  if a message is published at QoS 2 and a client is subscribed with QoS 0, the message will be delivered 
    //  to that client with QoS 0. If a second client is also subscribed to the same topic, but with QoS 2, 
    //  then it will receive the same message but with QoS 2. For a second example, if a client is subscribed with QoS 2 
    //  and a message is published on QoS 0, the client will receive it on QoS 0.
    //  
    //  Higher levels of QoS are more reliable, but involve higher latency and have higher bandwidth requirements.
    //      0: The broker/client will deliver the message once, with no confirmation.
    //      1: The broker/client will deliver the message at least once, with confirmation required.
    //      2: The broker/client will deliver the message exactly once by using a four step handshake.
    int messageID;
    int result = mosquitto_publish( mosq, &messageID, 
                                    combinedTopic, 
                                    length, 
                                    buffer, 
                                    aSystem->MQTTQoS, 
                                    aSystem->MQTTRetainMsgs );
    
    if (result != 0) {
        Logger_LogWarning( "MQTT Create Status - attempt to publish an event failed. Result: %d  Message: [%s]\n", result, buffer );
        MQTT_handleError( aSystem, result ); 
    }
    
    return result;
}


    
    


