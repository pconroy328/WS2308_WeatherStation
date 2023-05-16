


/* 
 * File:   database.c
 * Author: patrick.conroy
 *
 * Created on September 3, 2013, 2:01 PM
 * (C) 2013 Patrick Conroy
 */


// -----------------------------------------------------------------------------
//
//
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
// #include <mysql/mysql.h>
#include <mariadb/mysql.h>


#include "database.h"
#include "log4c.h"
#include "ws_structs.h"


//
// Forward Declarations
static  int     createHistoryTable();
static  int     createAlarmTable();
static  int     createReadingsTable();

static  void    dropHistoryTable();
static  void    dropAlarmTable();
static  void    dropReadingsTable();



//
//
static  char    *readingsTableName = "WeatherReadings";
static  char    *createReadingsTableSQL = "CREATE TABLE `%s`.`%s` \
 ( ID INT NOT NULL AUTO_INCREMENT , \
   aDate DATE, \
   aTime TIME, \
   tendency VARCHAR( 10 ), \
   forecast VARCHAR( 10 ), \
    iTemp DECIMAL(4,1), \
   iHumidity DECIMAL(4,1), \
   oTemp DECIMAL(4,1), \
   oHumidity DECIMAL(4,1), \
    oPressure DECIMAL(4,2), \
   oWindChill DECIMAL(4,1), \
   oWindSpeed DECIMAL(4,1), \
   oWindHeading DECIMAL(4,1),   \
    rainLastHour DECIMAL(4,2), \
   rainLastDay DECIMAL(4,2), \
   rainTotal  DECIMAL(4,2), \
   lastUpdate TIMESTAMP NOT NULL DEFAULT NOW() ,\
   PRIMARY KEY (`ID`) ); ";



static  char    *insertReadingsRecordSQL = "INSERT INTO `%s`.`%s` ( \
   aDate,aTime,tendency,forecast, \
   iTemp,iHumidity,oTemp,oHumidity, \
   oPressure,oWindChill,oWindSpeed,oWindHeading, \
   rainLastHour,rainLastDay,rainTotal)VALUES( \
   STR_TO_DATE('%s','%%m/%%d/%%y'), \
   '%s', \
   TRIM('%s'),TRIM('%s'),\
   %f,%f,%f,%f, \
   %f,%f,%f,%f, \
   %f,%f,%f );";

static  char    *insertReadingsRecordSQL_NoClock = "INSERT INTO `%s`.`%s` ( \
   tendency,forecast, \
   iTemp,iHumidity,oTemp,oHumidity, \
   oPressure,oWindChill,oWindSpeed,oWindHeading, \
   rainLastHour,rainLastDay,rainTotal)VALUES( \
   CURDATE(), \
   CURTIME(), \
   TRIM('%s'),TRIM('%s'),\
   %f,%f,%f,%f, \
   %f,%f,%f,%f, \
   %f,%f,%f );";

    
    
// -----------------------------------------------------------------------------
static  int     databaseIsOpen = FALSE;
static  MYSQL   *connection = NULL;
static  char    dbHostName[ MAX_INI_STRLEN ];
static  char    dbUserName[ MAX_INI_STRLEN ];
static  char    dbPassword[ MAX_INI_STRLEN ];
static  char    dbSchemaName[ MAX_INI_STRLEN ];
static  int     dbFailOnErrors = FALSE;
static  int     logAlarms = FALSE;
static  int     logReadings = FALSE;
static  int     logHistory = FALSE;
static  int     maxMinutesOfHistoryStored = 180;

// -----------------------------------------------------------------------------
void    Database_setDatabaseHost (char *hostName)
{
    assert( hostName != NULL );
    strncpy( dbHostName, hostName, sizeof( dbHostName ) );
}
// -----------------------------------------------------------------------------
void    Database_setDatabaseUserName (char *userName)
{
    assert( userName != NULL );
    strncpy( dbUserName, userName, sizeof( dbUserName ) );
}
// -----------------------------------------------------------------------------
void    Database_setDatabasePassword (char *password)
{
    assert( password != NULL );
    strncpy( dbPassword, password, sizeof( dbPassword ) );
}
// -----------------------------------------------------------------------------
void    Database_setDatabaseSchema (char *schemaName)
{
    assert( schemaName != NULL );
    strncpy( dbSchemaName, schemaName, sizeof( dbSchemaName ) );
}

// -----------------------------------------------------------------------------
void    Database_setFailOnDatabaseErrors (int newValue)
{
    dbFailOnErrors = newValue;
}

// -----------------------------------------------------------------------------
int     Database_openDatabase (WS2308System_t *aSystem)
{
    Logger_LogInfo( "Database_openDatabase [%s] [%s] [%s]\n", 
            aSystem->db.userName, 
            aSystem->db.schema, 
            aSystem->db.hostName );
    
    if (databaseIsOpen) {
        Logger_LogWarning( "Database is already open!\n" );
        return TRUE;
    }    
    databaseIsOpen = FALSE;
    connection = mysql_init( NULL );
    if (connection == NULL) {
        Logger_LogError( "Unable to create a client database session (mysql_init call failed).\n" );
        if (dbFailOnErrors)
            exit( 1 );
    }

    if (mysql_real_connect( connection, aSystem->db.hostName, aSystem->db.userName, aSystem->db.password, aSystem->db.schema, 0, NULL, 0) == NULL) {
        Logger_LogError( "Unable to establish a connection to the database. Host: [%s], User: [%s], Schema: [%s].\n", aSystem->db.hostName, aSystem->db.userName, aSystem->db.schema );
        Logger_LogError( "%s\n", mysql_error( connection ));
        fprintf( stderr, "Unable to establish a connection to the database. Host: [%s], User: [%s], Schema: [%s].\n", aSystem->db.hostName, aSystem->db.userName, aSystem->db.schema );
        fprintf( stderr, "%s\n", mysql_error( connection ));
        if (dbFailOnErrors) {
            mysql_close( connection );
            exit( 1 );
        }
    } else {
        databaseIsOpen = TRUE;
        Logger_LogInfo( "Database is open and connected.\n" );
    }
    
    return databaseIsOpen;
}        

// -----------------------------------------------------------------------------
int     Database_closeDatabase (void)
{
    Logger_LogInfo( "Disconnecting from database\n" );
    if (!databaseIsOpen)
        return TRUE;
    
    mysql_close( connection );
    databaseIsOpen = FALSE;
    connection = NULL;
    return TRUE;
}

// -----------------------------------------------------------------------------
void    Database_setDefaults (WS2308System_t *aSystem)
{
    Logger_LogInfo( "Database setting safe defaults..." );
    strncpy( &(aSystem->db.hostName[0]), "foo", MAX_INI_STRLEN );
    strncpy( &(aSystem->db.userName[0]), "foo", MAX_INI_STRLEN );
    strncpy( &(aSystem->db.password[0]), "foo", MAX_INI_STRLEN );
    strncpy( &(aSystem->db.schema[0]), "HomeAutomation", MAX_INI_STRLEN );
    
    aSystem->db.dropAlarmTable = FALSE;
    aSystem->db.createAlarmTable = FALSE;
    aSystem->db.dropReadingsTable = FALSE;
    aSystem->db.createReadingsTable = FALSE;
    aSystem->db.dropHistoryTable = FALSE;
    aSystem->db.createHistoryTable = FALSE;
    aSystem->db.maxMinutesOfHistoryStored = 180;
    aSystem->db.logAlarms = FALSE;
    aSystem->db.logReadings = TRUE;
    aSystem->db.logHistory = FALSE;
}

// -----------------------------------------------------------------------------
void    Database_initialize (WS2308System_t *aSystem)
{
    Logger_LogInfo( "Database initialization with INI file parameters\n" );
    // Logger_LogInfo( "  -- createReadingsTable is [%d]\n", aSystem->db.createReadingsTable );
    
    
    if (aSystem->logToDatabase || aSystem->db.logAlarms || aSystem->db.logReadings || aSystem->db.logHistory) {
        Database_setDatabaseHost( &(aSystem->db.hostName[ 0 ] ) );
        Database_setDatabaseUserName( &(aSystem->db.userName[ 0 ] ) );
        Database_setDatabasePassword( &(aSystem->db.password[ 0 ] ) );
        Database_setDatabaseSchema( &(aSystem->db.schema[ 0 ] ) );
        Database_setFailOnDatabaseErrors( 1 );
        
        if (!Database_openDatabase( aSystem )) {
            Logger_LogWarning( "Unable to open the database -- continuing\n" );
            return;
        }
        
        if (aSystem->db.dropAlarmTable)
            dropAlarmTable();
        if (aSystem->db.createAlarmTable)
            createAlarmTable();

        if (aSystem->db.dropReadingsTable)
            dropReadingsTable();
        if (aSystem->db.createReadingsTable)
            createReadingsTable();

        if (aSystem->db.dropHistoryTable)
            dropHistoryTable();
        if (aSystem->db.createHistoryTable)
            createHistoryTable();

        
        //
        // Make some local copies of the data...
        logAlarms = (aSystem->db.logAlarms);
        logHistory = (aSystem->db.logHistory);
        logReadings = (aSystem->db.logReadings);

        maxMinutesOfHistoryStored = aSystem->db.maxMinutesOfHistoryStored;
    }
}

// -----------------------------------------------------------------------------
void    Database_updateDeviceTables (weatherDatum_t *recPtr)
{
    if (!databaseIsOpen)
        return;
}

// -----------------------------------------------------------------------------
static
int     createAlarmTable()
{
    char  buffer[ 1024 ];
    
    if (!databaseIsOpen)
        return FALSE;
    
    return TRUE;
}

// -----------------------------------------------------------------------------
static
void   dropAlarmTable ()
{
    char buffer[ 1024 ];
    
    if (!databaseIsOpen)
        return;    
}

// -----------------------------------------------------------------------------
static
void    insertAlarmRecord (weatherDatum_t *recPtr)
{
    char    *stateString;
    char    buffer[ 8192 ];

    if (!databaseIsOpen)
        return;
    
}    

// -----------------------------------------------------------------------------
static
int     createHistoryTable()
{
}

// -----------------------------------------------------------------------------
static
void   dropHistoryTable ()
{
}

// -----------------------------------------------------------------------------
static
void    insertHistoryRecord (weatherDatum_t *recPtr)
{
}

// -----------------------------------------------------------------------------
static
void    deleteHistoryRecords (void)
{
}

// -----------------------------------------------------------------------------
static
int     createReadingsTable (void)
{
    char    buffer[ 4096 ];
    
    Logger_LogInfo( "createReadingsTable\n" );

        
    if (!databaseIsOpen)
        return FALSE;
    
    memset( buffer, '\0', sizeof buffer );
    int bufferLength = snprintf( buffer, sizeof buffer, createReadingsTableSQL,  dbSchemaName, readingsTableName );
    
    if (mysql_query( connection, buffer ) != 0) {
        Logger_LogError( "Unable to create the table.\n" );
        Logger_LogError( "%s\n", mysql_error( connection ));
        return FALSE;
    }
    return TRUE;
}

// -----------------------------------------------------------------------------
static
void    dropReadingsTable (void)
{
    char buffer[ 1024 ];
    
    Logger_LogInfo( "dropReadingsTable\n" );

    memset( buffer, '\0', sizeof buffer );
    
    snprintf( buffer, sizeof buffer, "DROP TABLE `%s`.`%s` ", dbSchemaName, readingsTableName );
    
    if (mysql_query( connection, buffer ) != 0) {
        Logger_LogError( "Unable to drop the table.\n" );
        Logger_LogError( "%s\n", mysql_error( connection ));
        if (dbFailOnErrors) {
            //mysql_close( connection );
            //exit( 1 );
        }
    }
}


// -----------------------------------------------------------------------------
void    Database_insertReadingsRecord (WS2308System_t *aSystem, weatherDatum_t *recPtr)
{
    char    buffer[ 8192 ];
    
    Logger_LogInfo( "insertReadingsRecord\n" );

    if (!databaseIsOpen)
        return;

    int bufferLength = 0;
    memset( buffer, '\0', sizeof buffer );

    //
    // Need a hack for when the Atomic Clock is busted...
    if  (aSystem->ignoreAtomicClock) {
        bufferLength = snprintf( buffer, sizeof buffer, 
                        insertReadingsRecordSQL_NoClock,
                        dbSchemaName, readingsTableName,
                        recPtr->tendency, recPtr->forecast,
                        recPtr->indoor.temp, recPtr->indoor.humidity, recPtr->outdoor.temp, recPtr->outdoor.humidity,
                        recPtr->outdoor.pressure,recPtr->outdoor.windchill,recPtr->outdoor.windspeed,recPtr->outdoor.windheading,
                        recPtr->outdoor.rain.lastHour,recPtr->outdoor.rain.lastDay,recPtr->outdoor.rain.total );
        
    } else {
    
        bufferLength = snprintf( buffer, sizeof buffer, 
                        insertReadingsRecordSQL,
                        dbSchemaName, readingsTableName,
                        recPtr->date, recPtr->time, 
                        recPtr->tendency, recPtr->forecast,
                        recPtr->indoor.temp, recPtr->indoor.humidity, recPtr->outdoor.temp, recPtr->outdoor.humidity,
                        recPtr->outdoor.pressure,recPtr->outdoor.windchill,recPtr->outdoor.windspeed,recPtr->outdoor.windheading,
                        recPtr->outdoor.rain.lastHour,recPtr->outdoor.rain.lastDay,recPtr->outdoor.rain.total );
    }
    
    Logger_LogDebug( "insertReadingsRecord SQL [%s]\n", buffer );
    
    if (mysql_query( connection, buffer ) != 0) {
        Logger_LogError( "Unable to insert the record into the table.\n" );
        Logger_LogError( "%s\n", mysql_error( connection ));
        Logger_LogError( "SQL: [%s]\n", buffer );
        if (dbFailOnErrors) {
            mysql_close( connection );
            exit( 1 );
        }
    }
}

