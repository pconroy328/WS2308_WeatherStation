/* 
 * File:   logger.c
 * Author: patrick conroy
 *
 * Created on September 17, 2013, 10:03 AM
 * 
 *  Super simple logging framework.  The functionality I need with 10% the complexity of
 *  other 'C' logging frameworks
 * 
 *  Nota Bene:
 *  (1) I use "localtime_r" so consult platform docs for required macros (I use -D_POSIX_SOURCE)
 *  (2) getCurrentDateTime() not thread safe yet.
 * 
 * (C) 2013, 2015 Patrick Conroy
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "logger.h"



//static  int     Logger_Log (char *format, char *level, ...);
static  char    *getCurrentDateTime( void );


static  char    currentDateTimeBuffer[ 80 ];



static  FILE    *fp;
static  char    *currentFileName;
static  int     logFileOpen = FALSE;
static  int     debugValue;                     // from the INI file



// ----------------------------------------------------------------------------
//
// How "debugValue" works.  It's an integer
// 0 = Log nothing         
// 5 = Log Fatal and Error and Warning and Debug and Info
// 4 = Log Fatal and Error and Warning and Debug
// 3 = Log Fatal and Error and Warning
// 2 = Log Fatal and Error
// 1 = Log Fatal

// ----------------------------------------------------------------------------
void    Logger_Initialize (char *fileName, int debugLevel)
{
    printf( "ATTEMPTING TO OPEN [%s] DEBUG LEVEL [%d]\n", fileName, debugLevel ); fflush( stdout );
    debugValue = debugLevel;
    currentFileName = fileName;
  
    //
    // Remember UNIX filesystems often do not keep a "Creation Date" for a file.
    // At best you'll have 'accessed (read)', 'modified (written to)', changed...
    // 
    // So we need to figure out a way to determine if the file we're about to open should be
    // rolled before we open it.
    //

    
    //fprintf( stderr, "[%s] Logger_Initialize() called - Opening up the file.\n", getCurrentDateTime() );
    if (fileName != (char *) 0 ) {
        fp = fopen( fileName, "a" );
        if (fp != (FILE *) 0) {
            logFileOpen = TRUE;
        
        }
    }
    
    if (logFileOpen)
        //fprintf( stderr, "[%s] Logger_Initialize() called - Log File is open.\n", getCurrentDateTime() );
        ;
}

// ----------------------------------------------------------------------------
void    Logger_UpdateDebugLevel (int newLevel)
{
    debugValue = newLevel;
}


// ----------------------------------------------------------------------------
void    Logger_Terminate()
{
    if (logFileOpen)
        fclose( fp );
}

// ----------------------------------------------------------------------------
void    Logger_LogDebug (char *format, ... )
{
    if (!logFileOpen || debugValue < 4)
        return;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
    int numWritten = fprintf( fp, "DEBUG|%s|", getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );
    va_end( args );
    
    fflush( fp );
}

// ----------------------------------------------------------------------------
void    Logger_LogWarning (char *format, ... )
{
    if (!logFileOpen || debugValue < 3)
        return;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
    int numWritten = fprintf( fp, "WARNING|%s|", getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );
    va_end( args );
    
    fflush( fp );
}

// ----------------------------------------------------------------------------
void    Logger_LogError (char *format, ... )
{
    if (!logFileOpen || debugValue < 2)
        return;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
    int numWritten = fprintf( fp, "ERROR|%s|", getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );
    va_end( args );
    
    fflush( fp );
}

// ----------------------------------------------------------------------------
void    Logger_LogFatal (char *format, ... )
{
    if (!logFileOpen || debugValue < 1)
        return;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
    int numWritten = fprintf( fp, "FATAL|%s|", getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );
    va_end( args );
    
    fflush( fp );
    Logger_Terminate();
    exit( 1 );
}

// ----------------------------------------------------------------------------
void    Logger_LogInfo (char *format, ... )
{
    if (!logFileOpen || debugValue < 5)
        return;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
    int numWritten = fprintf( fp, "INFO|%s|", getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );
    va_end( args );
    
    fflush( fp );
}
/*
// ----------------------------------------------------------------------------
static  int    Logger_Log (char *level, char *format, ...)
{
    if (!logFileOpen)
        return 0;
    
    va_list args;
    
    va_start( args, format );                   // the last fixed parameter
   
    int numWritten = fprintf( fp, "%s|%s|", level, getCurrentDateTime() );
    numWritten += vfprintf( fp, format, args );

    va_end( args );
    
    return numWritten;
}
*/

// ----------------------------------------------------------------------------
static  char    *getCurrentDateTime ()
{
    //
    // Something quick and dirty... Fix this later - it's NOT thread safe
    time_t  current_time;
    struct  tm  aTmStruct;
    struct  tm  *tmPtr;
 
    memset( currentDateTimeBuffer, '\0', sizeof currentDateTimeBuffer );
    
    current_time = time( NULL );
    if (current_time > 0) {
        
        
        tmPtr = localtime_r( &current_time, &aTmStruct );
 
        if (tmPtr != NULL) {
            strftime( currentDateTimeBuffer,
                    sizeof currentDateTimeBuffer,
                    "%F %T",
                    tmPtr );
            
        }
    }

    
    return &currentDateTimeBuffer[ 0 ];    
}
