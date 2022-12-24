/* 
 * File:   main.c
 * Author: conroy.patrick
 *
 * Created on October 1, 2009, 3:46 PM
 * 
 * 03-Dec-2013  Switching over from SmartSockets to MQTT!
 *      Using mosquitto v1.2 - expecting to see it in /usr/local/include and 
 *      /usr/local/lib
 * 
 */
// #define _POSIX_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <getopt.h>
#include <errno.h>

#include <time.h>
#include <signal.h>


#include "ws2300.h"
#include "mqtt.h"
#include "logger.h"
#include "ws_structs.h"
#include "database.h"
#include "stats.h"


// WHEN STARTED AT BOOT - IT CANNOT FIND THE INI FILE

//
// Holds all relevant parameters to the whole system
WS2308System_t  aSystem;


//
//  Forward Declarations
static  int     isBefore( char[], char[] );
extern  void    readIniFile( WS2308System_t * );



// ------------------------------------------------------------------------------------
static  char                *version = "v4.2 [ Alarm Raining Bug ]";
int                         debugLevel = 0;

//
//  Fastest the WS2308 can spit out need data is once every 8 seconds.
//  that's 8 per minute (7.5).  But every 8 seconds is only if the remote sensor
//  is wire-connected to the base unit. If the readings are sent over the wireless
//  connection - then the rate varies between 16 seconds and 120 seconds. If the wind
//  speed is above some threshold, the frequency of updates increases
static  WEATHERSTATION      ws2308;

#define MAX_READINGS        (8 * 60 * 60 * 24)
static  weatherDatum_t      dailyData[ MAX_READINGS ];
static  weatherStats_t      *weatherStatsPtr;


static  unsigned long       numDailyReadings;
static  unsigned long       numTotalReadings;
static  int                 cacheReadings = FALSE;
static  int                 useJSON = FALSE;

//
//  Those variables that can be set by the INI file are global to make things easier.
//int                         discardBadReadings = TRUE;
int                         readInterval;                  // every 16 seconds
double                      absPressureCorrection;


char            *INIFileName = "../INI/WS2308.ini";
static  int     ignoreIniFile = FALSE;



// -------------------------------------------------------------------------------------
static
double  absolutePressureToRelative (double absPressure)
{
    // we're about 3 inHG lower at this elevation
    // It may be silly to do a "add" and force them to put in a negative number
    // if relative is always <= absolute.
    return (absPressure + absPressureCorrection );
}

// -------------------------------------------------------------------------------------
static
int     takeReading (WEATHERSTATION ws2308, weatherDatum_t *datum)
{
    char    aBuffer[ 1024 ];
    double  newWindChill;

   Logger_LogDebug( "WS2308::takeReading() - start\n" );
   (void) read_date( ws2308, datum->date );
   (void) read_time( ws2308, datum->time );

   //                                                                  0....5
   // When the weatherstation goes offline - the date and time will be 00/00/00  00:00:00
   if ((datum->date[ 0 ] == '0') && (datum->date[ 1 ] == '0'))
    return FALSE;



   tendency_forecast( ws2308, datum->tendency, datum->forecast );

   datum->indoor.temp       = temperature_indoor( ws2308, FARENHEIT );
   datum->indoor.humidity   = humidity_indoor( ws2308 );
   datum->outdoor.temp      = temperature_outdoor( ws2308, FARENHEIT );
   datum->outdoor.humidity  = humidity_outdoor( ws2308 );
   datum->outdoor.dewpoint  = dewpoint( ws2308, FARENHEIT );

   datum->outdoor.windchill = windchill( ws2308, FARENHEIT );
   datum->outdoor.windspeed = wind_current( ws2308, ms_TO_MPH, &(datum->outdoor.windheading) );

   datum->outdoor.pressure  = abs_pressure( ws2308, mmHg_TO_inHG );
   // datum->outdoor.pressure  = rel_pressure( ws2308, hPA_TO_inHG );
   datum->outdoor.pressure = absolutePressureToRelative( datum->outdoor.pressure );

   datum->outdoor.rain.lastHour  = rain_1h( ws2308, mm_TO_in );
   datum->outdoor.rain.lastDay = rain_24h( ws2308, mm_TO_in );
   datum->outdoor.rain.total = rain_total( ws2308, mm_TO_in );

   //
   //   While exploring spurious windchill values coming from the unit
   //   I noticed that their windchill values didn't match up with the
   //   NOAA/NWS 2001 WincChills. So I wrote the new algorithm
   newWindChill = nws_2001_windchill( datum->outdoor.temp, datum->outdoor.windspeed );

   // Use the more accurate value anyway...
   datum->outdoor.windchill = newWindChill;

   sprintf( aBuffer, "%s %s %3.1f %3.1f %3.1f\n", datum->date, datum->time, datum->indoor.temp, datum->outdoor.temp, datum->outdoor.windspeed );
   Logger_LogInfo( aBuffer );
   //logMsg( "WS2308::takeReading() - stop" );

   //
   // Wow - it takes about 15-25 seconds to get one reading out!!!
   return TRUE;
}// takeReading

// -----------------------------------------------------------------------------
static
int     openWeatherStation (void)
{
    int     portIsOpen = FALSE;
    char    date[ 16 ];
    char    time[ 16 ];

    Logger_LogInfo( "Opening WS2308 communication port [%s]\n", aSystem.ws2308DeviceName );

    //
    // We've added the capability now for the software to try to find the port
    //  that the WS2308 is connected to.  I suppose we should try the set portname
    //  first and then search if that fails...

    //
    //
    ws2308 = open_weatherstation( aSystem.ws2308DeviceName );

    if (reset_06( ws2308 )) {
        date[ 0 ] = '\0';
        time[ 0 ] = '\0';
        read_date( ws2308, date );
        read_time( ws2308, time );

        //logMsg( "Ready" );
        //sprintf( aBuffer, "System date is: %s", read_date( ws2308, date ) );    logMsg( aBuffer );
        //sprintf( aBuffer, "System time is: %s", read_time( ws2308, time ) );    logMsg( aBuffer );

        if (date[ 0 ] != '\0' && time[ 0 ] != '\0')
            portIsOpen = TRUE;
        
        //
        // New in V2.5 - The Atomic Clock stopped working
        
    } else {
        Logger_LogError( "Unable to open that port, closing and invoking the findWS2300() call.\n" );
        close_weatherstation( ws2308 );
        
        if (findWS2300( &(aSystem.ws2308DeviceName[0]), sizeof( aSystem.ws2308DeviceName ), 2, 10 )) {
            Logger_LogInfo( "Found the WS2308 on port [%s]\n", aSystem.ws2308DeviceName );;

            ws2308 = open_weatherstation( aSystem.ws2308DeviceName );
            date[ 0 ] = '\0';
            time[ 0 ] = '\0';
            read_date( ws2308, date );
            read_time( ws2308, time );

            //logMsg( "Ready" );
            //sprintf( aBuffer, "System date is: %s", read_date( ws2308, date ) );    logMsg( aBuffer );
            //sprintf( aBuffer, "System time is: %s", read_time( ws2308, time ) );    logMsg( aBuffer );

            if (date[ 0 ] != '\0' && time[ 0 ] != '\0')
                portIsOpen = TRUE;
        }
    }

    return  portIsOpen;
}

// -------------------------------------------------------------------------------------
static
void    closeWeatherStation (void)
{
    close_weatherstation( ws2308 );
    ws2308 = 0;
}


// -------------------------------------------------------------------------------------
static
int     readingIsValid (weatherDatum_t *datum, weatherStats_t *wsPtr)
{
    Logger_LogInfo( "Calling reading is valid on datums at [%s] [%s]\n", datum->date, datum->time );
    
    //
    // Do some simple validity checking...
    
    //
    //  2014/2015 : I'm seeing some common errors now --
    //      -- every now and then the time is off (the clock moved backwards)
    //      -- and I'm seeing readings spike way high or way low)
    
    //
    //  We saw some weird windspeeds when things went all haywire one night
    //  What I saw was windspeeds of 114.1 first time.
    //  It's happened again and this time I see:
    //      mysql> select oTemp, oHumidity, oWindChill, oWindSpeed from DailyReading where oWindSpeed > 90 limit 20;
    //      +-------+-----------+------------+------------+
    //      | oTemp | oHmidity | oWindChill | oWindSpeed |
    //      +-------+-----------+------------+------------+
    //      |   0.0 |       0.0 |      177.1 |      114.1 |
    //      |   0.0 |       0.0 |      177.1 |      114.1 |
    //      |   0.0 |       0.0 |      177.1 |      114.1 |
    //

    if (isBefore( datum->date, datum->time )) {
        Logger_LogWarning( "Reading's date time appears to be in error. Will be discarded. Date [%s], Time [%s]\n", datum->date, datum->time );
        return FALSE;
    }

    //
    // After running for awhile - 50% is too big
    //
    if (!checkAndAddValue_OTemp( wsPtr, datum->outdoor.temp, 30.0 )) {
        Logger_LogWarning( "Reading's outdoor temperature appears to be in error. Will be discarded. Value [%f]\n", datum->outdoor.temp );
        return FALSE;
    }

    if (!checkAndAddValue_OHumidity( wsPtr, datum->outdoor.humidity, 30.0 )) {
        Logger_LogWarning( "Reading's outdoor humidity appears to be in error. Will be discarded. Value [%f]\n", datum->outdoor.humidity );
        return FALSE;
    }

    if (!checkAndAddValue_OPressure( wsPtr, datum->outdoor.pressure, 30.0 )) {
        Logger_LogWarning( "Reading's outdoor pressure appears to be in error. Will be discarded. Value [%f]\n", datum->outdoor.pressure );
        return FALSE;
    }

    if (!checkAndAddValue_ITemp( wsPtr, datum->indoor.temp, 15.0 )) {
        Logger_LogWarning( "Reading's indoor temperature appears to be in error. Will be discarded. Value [%f]\n", datum->indoor.temp );
        return FALSE;
    }

    if (!checkAndAddValue_IHumidity( wsPtr, datum->indoor.humidity, 25.0 )) {
        Logger_LogWarning( "Reading's indoor humidity appears to be in error. Will be discarded. Value [%f]\n", datum->indoor.humidity );
        return FALSE;
    }

    if (!checkAndAddValue_Windspeed( wsPtr, datum->outdoor.windspeed, 50.0 )) {
        Logger_LogWarning( "Reading's windspeed appears to be in error. Will be discarded. Value [%f]\n", datum->outdoor.windspeed );
        return FALSE;
    }

    
    return TRUE;
} // checkReading

// -------------------------------------------------------------------------------------
static
int     checkForAlarms (WS2308System_t *aSystem, weatherDatum_t *datum)
{
    int alarmCode = ALARM_NONE;

    //
    //  Indoor temp and humidity
    if (datum->indoor.temp < aSystem->Alarm_IndoorTempLow)
        alarmCode |= ALARM_INDOORTEMPLOW;
    if (datum->indoor.temp > aSystem->Alarm_IndoorTempHigh)
        alarmCode |= ALARM_INDOORTEMPHIGH;

    
    if (datum->indoor.humidity < aSystem->Alarm_IndoorHumidityLow)
        alarmCode |= ALARM_INDOORHUMIDITYLOW;
    if (datum->indoor.humidity > aSystem->Alarm_IndoorHumidityHigh)
        alarmCode |= ALARM_INDOORHUMIDITYHIGH;
    

    
    //
    //  Outdoor temp and humidity
    if (datum->outdoor.temp < aSystem->Alarm_OutdoorTempLow)
        alarmCode |= ALARM_OUTDOORTEMPLOW;
    if (datum->outdoor.temp > aSystem->Alarm_OutdoorTempHigh)
        alarmCode |= ALARM_OUTDOORTEMPHIGH;

    if (datum->outdoor.humidity < aSystem->Alarm_OutdoorHumidityLow)
        alarmCode |= ALARM_OUTDOORHUMIDITYLOW;
    if (datum->outdoor.humidity > aSystem->Alarm_OutdoorHumidityHigh)
        alarmCode |= ALARM_OUTDOORHUMIDITYHIGH;

    //
    //  Outdoor Wind and Windchill
    if (datum->outdoor.windspeed > aSystem->Alarm_OutdoorWindspeedHigh)
        alarmCode |= ALARM_OUTDOORWINDSPEEDHIGH;
    if (datum->outdoor.windchill < aSystem->Alarm_OutdoorWindchillLow)
        alarmCode |= ALARM_OUTDOORWINDCHILLLOW;

    //
    // Do they want to know if it's raining?
    if (aSystem->Alarm_Raining) {
        if (datum->outdoor.rain.lastHour > 0)
            alarmCode |= ALARM_RAINING;
    }
    return alarmCode;
}


// ------------------------------------------------------------------
void     terminationHandler (int signalValue)
{
    Logger_LogInfo( "SIGINT received... Shutting down.\n" );
    
    closeWeatherStation();
    MQTT_teardown();
    Database_closeDatabase();
    Logger_Terminate();
    exit( 1 );
}

// ------------------------------------------------------------------
void     sigUsr1Handler (int signalValue)
{
    //
    //  Close everything, reset, reopen
    //  
    Logger_LogInfo( "Signal USR1 received - Stopping, restarting, re-reading...\n" );

    sleep( 1 );
    Logger_LogInfo( "Shutting down MQTT and the database connection\n" );
    MQTT_teardown();
    Database_closeDatabase();
    

    Logger_LogInfo( "Closing and reopening the weather station\n" );
    closeWeatherStation();

    MQTT_setDefaults( &aSystem, aSystem.MQTTServerName );
    
    if (!ignoreIniFile) {
        Logger_LogInfo( "Reloading INI file...\n" );
        readIniFile( &aSystem );
    }

    Logger_LogInfo( "Restarting MQTT and the database connection...\n" );
    MQTT_initialize( &aSystem );
    Database_initialize( &aSystem );                    // Init calls "open()")
    // Database_openDatabase( &aSystem );
    

    if (!openWeatherStation())
        Logger_LogError( "ERROR - COULD NOT REOPEN THE WEATHERSTATION PORT\n" );

    Logger_LogInfo( "Reintialization complete!\n" );
}


// ------------------------------------------------------------------
void     sigUsr2Handler (int signalValue)
{
    //  
    // Just re-read INI file
    Logger_LogInfo( "Signal USR2 received - Reading INI file but not restarting or restting anything\n" );
    readIniFile( &aSystem );
}

// -----------------------------------------------------------------------------
void    setupSignalHandlers (void)
{
    struct  sigaction sa;

    //
    // Set up the structure to specify the new action.
    sa.sa_handler = terminationHandler;
    sigemptyset( &sa.sa_mask );
    sa.sa_flags = 0;

    //
    //  SIGINT and SIGTERM - terminate the programs
    if (sigaction( SIGINT, &sa, NULL ) == -1)
        Logger_LogError( "Unable to install SIGNINT handler!\n" );
    if (sigaction( SIGTERM, &sa, NULL ) == -1)
        Logger_LogError( "Unable to install SIGTERM handler!\n" );

    //
    // SIGUSR1 causes us to re-read the INI file
    sa.sa_handler = sigUsr1Handler;
    sigemptyset( &sa.sa_mask );
    sa.sa_flags = 0;
    if (sigaction( SIGUSR1, &sa, NULL ) == -1)
        Logger_LogError( "Unable to install SIGUSR1 handler!\n" );

    //
    // SIGUSR2
    sa.sa_handler = sigUsr2Handler;
    sigemptyset( &sa.sa_mask );
    sa.sa_flags = 0;
    if (sigaction( SIGUSR2, &sa, NULL ) == -1)
        Logger_LogError( "Unable to install SIGUSR2 handler!\n" );


    //
    //   SIGHUPs are ignored - we keep on running
    sa.sa_handler = SIG_IGN;
    sigaction( SIGHUP, &sa, NULL );
}


// -------------------------------------------------------------------------------------
static
int     isBefore (char newDate[], char newTime[])
{
    static  char    oldDate[ 10 ];               // static because we're going to
    static  char    oldTime[ 10 ];               // use these values next time
    static  int     badTimeCount;
    
    int     returnVal = FALSE;
    
    //
    //  First time thru, assume the data and time is good...
    if (oldDate[ 0 ] == '\0') {
        Logger_LogDebug( "In isBefore - oldDateStr is null. First time we've checked a date. Returning false.\n" );
        memmove( &oldDate[ 0 ], newDate, 10 );
        memmove( &oldTime[ 0 ], newTime, 10 );
        badTimeCount = 0;
        return FALSE;
    }
    
    //  0.2..5..
    //  HH:MM:SS
    int newSeconds = ((newTime[ 6 ] - '0') * 10) + (newTime[ 7 ] - '0' );
    newSeconds += (((newTime[ 3 ] - '0') * 10) + (newTime[ 4 ] - '0' )) * 60;
    newSeconds += (((newTime[ 0 ] - '0') * 10) + (newTime[ 1 ] - '0' )) * 60 * 60;
    
    int oldSeconds = ((oldTime[ 6 ] - '0') * 10) + (oldTime[ 7 ] - '0' );
    oldSeconds += (((oldTime[ 3 ] - '0') * 10) + (oldTime[ 4 ] - '0' )) * 60;
    oldSeconds += (((oldTime[ 0 ] - '0') * 10) + (oldTime[ 1 ] - '0' )) * 60 * 60;
  
    
    int difference = newSeconds - oldSeconds;

    //
    // Did time move backwards?
    if (difference < 0) {
        Logger_LogWarning( "In isBefore - new date [%s] [%s] is *before* last, old date [%s] [%s]. Seconds difference: %d Returning true.\n", newDate, newTime, oldDate, oldTime, difference );
        badTimeCount += 1;
        returnVal = TRUE;
    }

    //
    // Did time move forwards but too far to be reasonable? Like more than 20 minutes?
    if (difference > (20 * 60)) {
        Logger_LogWarning( "In isBefore - new date [%s] [%s] too far forward of old date [%s] [%s]. Seconds difference: %d Returning true.\n", newDate, newTime, oldDate, oldTime, difference );
        badTimeCount += 1;
        returnVal = TRUE;
    }
    
    //
    // As some point if we get too many badTimeCounts, we need to just reset and start over
    if (badTimeCount > 5) {
        Logger_LogError( "Too many successive bad time calculations - we're going to reset and start over.\n" );
        returnVal = FALSE;
    }
    
    
    //
    // Old Date is NOT before New Date - so we're good. Save it off.
    if (!returnVal) {
        memcpy( &oldDate[ 0 ], newDate, 10 );
        memcpy( &oldTime[ 0 ], newTime, 10 );
        badTimeCount = 0;
    }
    
    return returnVal;
}

// -------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------
int showHelp()
{
    printf( "Version %s\n", version );
    puts( "Command line options:" );
    puts( "-p <com port>      WS2308 is on device <com port> (eg. '/dev/ttyUSB0')" );
    puts( "-s <server>        send MQTT data to this MQTT server, (eg '192.168.1.11')" );
    puts( "-m <mqtt port num> use this port number for MQTT (eg 1883)" );
    puts( "-j                 put MQTT packets in JSON format" );
    puts( "-b <topic>         use <topic> as the ROOT MQTT topic string" );
    puts( "-a <subtopic>      use <subtopic> as the ALARM MQTT sub-topic string" );
    puts( "-e <subtopic>      use <subtopic> as the STATUS MQTT sub-topic string" );
    puts( "-l <seconds>       take reading every <secconds> seconds (eg 120" );
    puts( "-v                 turn on verbose/debug output" );
    puts( "-c                 cache readings locally" );
    puts( "-r                 do NOT discard bad readings - just take what comes" );
    puts( "-f <fileName>      write debug and log data to this file" );


    puts( "-i <fileName>      pull INI parameters from this file" );
    puts( "-x                 ignore all INI files" );
    puts( "INI File parameters:");
    printf( "    Ini File Name: [%s]\n", INIFileName );
    puts( "  [WS2308] - debug - see -v" );

    puts( "  [WS2308] - port - see dash p" );
    puts( "  [WS2308] - MQTTServer - see dash s" );
    puts( "  [WS2308] - MQTTPort - see dash m" );
    puts( "  [WS2308] - MQTTQoS - MQTT Quality of Service: 0, 1 or 2" );
    puts( "  [WS2308] - MQTTRetainMsgs - MQTT Retain Messages: True or False" );
    puts( "  [WS2308] - readIntervalSecs - see dash l" );
    puts( "  [WS2308] - discardBadReadings - see dash r" );
    puts( "  [WS2308] - absPressureCorrection - value to add to absolute pressure to get relative" );
    puts( "  There are many more INI file parameters than are listed here."  );
    
    puts( "" );
    puts( "Signals" );
    puts( "  Sending USR1 causes the application to close, reset, reopen reread everything" );
    puts( "  Sending USR2 causes the application to read the INI file" );
    

}   // showHelp

// -------------------------------------------------------------------------------------
int parseCommandLineArgs (int argc, char **argv)
{
    opterr = 0;
    int     ch;
    
    //
    // getopt() has an issue on the Raspberry Pi!
    while ( ((ch = getopt( argc, argv, "p:l:s:t:m:v:ncrij:x" )) != -1) && (ch != 255))
       switch (ch) {
            case 'l':   aSystem.readInterval = atoi( optarg );
                        break;
            case 'v':   aSystem.debugLevel = atoi( optarg );
                        break;
            case 'c':   cacheReadings = TRUE;
                        break;
            case 'r':   aSystem.discardBadReadings = FALSE;
                        break;
                        
            case 's':   strncpy( &(aSystem.MQTTServerName[0]), optarg, sizeof( aSystem.MQTTServerName ) );            
                        break;
                        
            case 'b':   strncpy( &(aSystem.MQTTParentTopic[0]), optarg, sizeof( aSystem.MQTTParentTopic ) );
                        break;
            case 'a':   strncpy( &(aSystem.MQTTAlarmTopic[0]), optarg, sizeof( aSystem.MQTTAlarmTopic ) );
                        break;
            case 'e':   strncpy( &(aSystem.MQTTStatusTopic[0]), optarg, sizeof( aSystem.MQTTStatusTopic ) );
                        break;

            case 'm':   aSystem.MQTTPort = atoi( optarg );
                        break;
                        
            case 'p':   strncpy( &(aSystem.ws2308DeviceName[0]), optarg, sizeof( aSystem.ws2308DeviceName ) );
                        break;
                        
            case 'i':   strncpy( &(aSystem.INIFileName[0]), optarg, sizeof( aSystem.INIFileName ) );
                        break;
                        
            case 'j':   aSystem.MQTTUseJSON = TRUE;
                        break;
                        
            case 'x':   ignoreIniFile = TRUE;
                        break;
                        
            case 'f':   strncpy( &(aSystem.logFileName[0]), optarg, sizeof( aSystem.logFileName ) );
                        break;

           default:     showHelp();
                        exit( 1 );
                        break;
        }

    return 0;
}   // parseCommnadLineArgs

// -------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    weatherDatum_t      datum;
    int                 done;
    int                 readingMakesSense;


    numDailyReadings = 0L;
    numTotalReadings = 0L;


    puts( "WS2308 Weather Station Data Reader" );
    printf( "Version: %s\n", version );
    
    setupSignalHandlers();
    

    //
    //  Initialize defaults
    //strcpy( &(aSystem.MQTTServerName[ 0 ]), "192.168.1.11" );
    strcpy( &(aSystem.MQTTParentTopic[ 0 ]), "WS2308" );
    strcpy( &(aSystem.MQTTStatusTopic[ 0 ]), "STATUS" );
    strcpy( &(aSystem.MQTTAlarmTopic[ 0 ]), "ALARM" );
    strcpy( &(aSystem.ws2308DeviceName[ 0 ]), "/dev/ttyUSB0" );

    
    aSystem.debugLevel = 3;                 // Start off with logging almost everything!
    aSystem.readInterval = 16;
    aSystem.discardBadReadings = TRUE;
    aSystem.ignoreAtomicClock = FALSE;
    aSystem.MQTTUseJSON = FALSE;
    aSystem.MQTTUseISO8601 = TRUE;
    
    absPressureCorrection = -3.0;
    strncpy( &(aSystem.logFileName[ 0 ]), "/tmp/ws2308.log", sizeof aSystem.logFileName );
    aSystem.logToMQTT = TRUE;
    
    
    MQTT_setDefaults( &aSystem, aSystem.MQTTServerName );
    Database_setDefaults( &aSystem );

   
    //
    // Get command line arguments, if any...
    parseCommandLineArgs( argc, argv );
 
    //
    //  Add INI file reading!!!
    if (!ignoreIniFile)
        readIniFile( &aSystem );

   
    Logger_Initialize( aSystem.logFileName, aSystem.debugLevel );
    Logger_LogInfo( "WS2308 Weather Station Data Reader Version: %s\n", version );
    
    MQTT_initialize( &aSystem );
    Database_initialize( &aSystem );
    
    if (!openWeatherStation()) {
        Logger_LogFatal( "Unable to open the weather station port. Shutting down.\n" );
        MQTT_teardown();
        Database_closeDatabase();
        exit( 0 );
    }

    weatherStatsPtr = initializeStatsModule( 10 );
    
    done = FALSE;
    while (!done) {
        if (takeReading( ws2308, &dailyData[ numDailyReadings ] ) ) {

            readingMakesSense = TRUE;
            if (aSystem.discardBadReadings && !readingIsValid( &dailyData[ numDailyReadings ], weatherStatsPtr ))
                readingMakesSense = FALSE;

            if (readingMakesSense) {
                if (aSystem.logToDatabase)
                    Database_insertReadingsRecord( &aSystem, &dailyData[ numDailyReadings ] );
                if (aSystem.logToMQTT)
                    MQTT_createWeatherStatus( &aSystem, &dailyData[ numDailyReadings ] );
                
                //
                // Check to see if any thresholds are exceeded, if so - publish alarm event
                int alarmCode = checkForAlarms( &aSystem, &dailyData[ numDailyReadings ] );
                if (alarmCode != ALARM_NONE && aSystem.logToMQTT)
                    MQTT_createWeatherAlarm( &aSystem, alarmCode, &dailyData[ numDailyReadings ] );
            }

            //
            // Need to tell MQTT - time to send/receive messages!
            MQTT_sendReceive( &aSystem );
            
            numTotalReadings += 1L;
            sleep( readInterval );
            
       } else {
           //
           //   takeReading failed - It'll be the darned port died again. Close it and
           //   try to reopen it
           Logger_LogError( "takeReading() call reporting a failure. Closing the port.\n" );
           closeWeatherStation();
           if (!openWeatherStation()) {
               Logger_LogFatal( "Attempt to reopen weather station port failed - exiting\n" );
               exit( 1 );
           }
       }
    }

    closeWeatherStation();
    teardownStatsModule( weatherStatsPtr );
    MQTT_teardown();
    Database_closeDatabase();
    Logger_Terminate();
}
