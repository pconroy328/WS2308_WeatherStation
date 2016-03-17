#include <assert.h>
#include <stdlib.h>
#include "stats.h"
#include "logger.h"

#ifndef FALSE
#define     FALSE       0
#define     TRUE        (!FALSE)
#endif


//
// Forward declarations
static  MovingAverage_t *getRightMAPtr( weatherStats_t  *wsPtr, int statType );
static  int             *getRightInvalidCountPtr( weatherStats_t *wsPtr, int statType );





// -----------------------------------------------------------------------------
weatherStats_t  *initializeStatsModule (int maBufferSize)
{
    Logger_LogDebug( "Initializing stats module\n" );
    
    weatherStats_t      *wsPtr = malloc( sizeof( weatherStats_t ));
    if (wsPtr != (weatherStats_t *) 0 ) {
        wsPtr->oTempMA = MovingAverage_CreateMovingAverage( maBufferSize );
        wsPtr->oHumidityMA = MovingAverage_CreateMovingAverage( maBufferSize );
        wsPtr->oPressureMA = MovingAverage_CreateMovingAverage( maBufferSize );
        wsPtr->iTempMA = MovingAverage_CreateMovingAverage( maBufferSize );
        wsPtr->iHumidityMA = MovingAverage_CreateMovingAverage( maBufferSize );
        
        //
        // A Moving average for Windspeed - just doesn't work
        //wsPtr->windspeedMA = MovingAverage_CreateMovingAverage( maBufferSize );
    }
    
    assert( wsPtr->oTempMA != NULL );    
    assert( wsPtr->oHumidityMA != NULL );
    assert( wsPtr->oPressureMA != NULL );
    assert( wsPtr->iTempMA != NULL );
    assert( wsPtr->iHumidityMA != NULL );
    //assert( wsPtr->windspeedMA != NULL );

    wsPtr->invalidOTempCount = 0;
    wsPtr->invalidOHumidityCount = 0;
    wsPtr->invalidOPressureCount = 0;
    wsPtr->invalidiTempCount = 0;
    wsPtr->invalidiHumidityCount = 0;
    //wsPtr->invalidWindspeedCount = 0;

    
    return wsPtr;
}

// -----------------------------------------------------------------------------
void    teardownStatsModule (weatherStats_t  *wsPtr)
{
    assert( wsPtr != NULL );

    MovingAverage_DestroyMovingAverage( wsPtr->iHumidityMA );
    MovingAverage_DestroyMovingAverage( wsPtr->iTempMA );
    MovingAverage_DestroyMovingAverage( wsPtr->oHumidityMA );
    MovingAverage_DestroyMovingAverage( wsPtr->oTempMA );
    MovingAverage_DestroyMovingAverage( wsPtr->oPressureMA );
    //MovingAverage_DestroyMovingAverage( wsPtr->windspeedMA );
    
    free( wsPtr );
    Logger_LogDebug( "Stats module torn down and memory freed\n" );
}


// -----------------------------------------------------------------------------
static
void    addValue (weatherStats_t *wsPtr, int statType, double value)
{
    assert( wsPtr != NULL );

    MovingAverage_t     *maPtr = getRightMAPtr( wsPtr, statType );
    
    if (maPtr != (MovingAverage_t *) 0) {
        Logger_LogDebug( "In addValue - Added value: %f. StatsType: %d\n", value, statType );
        MovingAverage_AddValue( maPtr, value );
    }
}

// -----------------------------------------------------------------------------
static
int     checkValue (weatherStats_t *wsPtr, int statType, double value, double percentage)
{
    assert( percentage >= 0.0 && percentage <= 100.0);
    assert( wsPtr != NULL );
    

    //
    // return TRUE if the value is within range
    MovingAverage_t     *maPtr = getRightMAPtr( wsPtr, statType );
    
    if (maPtr != (MovingAverage_t *) 0) {
        int     valueCount = MovingAverage_GetElementCount( maPtr );
        
        //
        // What do we want to do if there aren't enough values in the average yet?
        if (valueCount == 0) {
            Logger_LogDebug( "In checkValue - zero elements in moving average. Returning true for value: %f. StatsType: %d\n", value, statType );
            return TRUE;
        }
        
        double  average = MovingAverage_GetAverage( maPtr );
        
        double  upperBound = average * (1.0 + (percentage / 100.0));
        double  lowerBound = average * (1.0 - (percentage / 100.0));
        
        //
        // If the average is zero then we've got a problem - the upper and lower bounds will be zero if we multiply
        if (average == 0.0) {
            upperBound = 50.0;                  // Could the temp, pressure humidity go from 0 to 50? Maybe, maybe not - but windspeed probably could
            lowerBound = -10.0;                 // Could the temp, pressure humidity go down from 0 to -10? Yeah, probably
        }
        
        
        if (value < lowerBound || value > upperBound) {
            Logger_LogInfo( "Discarding this value: %f.  StatsType: %d.  Upper: %f, Lower: %f\n", value, statType, upperBound, lowerBound );
            return FALSE;
        } else {
            Logger_LogDebug( "In checkValue - Returning true for value: %f. StatsType: %d. Avg: %f  Upper: %f  Lower: %f\n", value, statType, average, upperBound, lowerBound );
            return TRUE;
        }
    }
    
    return FALSE;
}

// -----------------------------------------------------------------------------
static
int     checkAndAddValue (weatherStats_t  *wsPtr, int statType, double value, double percentage)
{
    assert( wsPtr != NULL );

    MovingAverage_t     *maPtr = getRightMAPtr( wsPtr, statType );
    
    assert( maPtr != NULL );
    
    if (checkValue( wsPtr, statType, value, percentage )) {
        addValue( wsPtr, statType, value );
        
    } else {
        Logger_LogInfo( "Value: %f  StatsType: %d  did not pass check and will not be added to the MA\n", value, statType );
        //
        // Value did NOT pass our check
        int *invalidCountPtr = getRightInvalidCountPtr( wsPtr, statType );
        *invalidCountPtr += 1;
        
        //
        //  After five discards in a row - reset things
        if (*invalidCountPtr > 5) {
            Logger_LogWarning( "5 invalid readings detected for statType: %d\n", statType );
            Logger_LogWarning( "     Flushing the data and restarting the moving average calculations.\n" );
            MovingAverage_Reset( maPtr );
            
            Logger_LogWarning( "     MA flushed - adding this value: %f\n", value );
            *invalidCountPtr = 0;
            addValue( wsPtr, statType, value );
        } else {
            Logger_LogWarning( "This value: %f failed the check - it will be discarded (statType: %d)\n", value, statType );
        }
        
        return FALSE;
    }
    
    return TRUE;
}

// -----------------------------------------------------------------------------
int     checkAndAddValue_OTemp (weatherStats_t *wsPtr, double value, double percentage)
{ 
    Logger_LogInfo( "Checking Outside Temperature value: %f\n", value );
    return checkAndAddValue( wsPtr, STATS_OTEMP, value, percentage );
}
// -----------------------------------------------------------------------------
int     checkAndAddValue_OHumidity (weatherStats_t *wsPtr, double value, double percentage)
{ 
    Logger_LogInfo( "Checking Outside Humidity value: %f\n", value );
    return checkAndAddValue( wsPtr, STATS_OHUMIDITY, value, percentage );
}
// -----------------------------------------------------------------------------
int     checkAndAddValue_OPressure (weatherStats_t *wsPtr, double value, double percentage)
{ 
    Logger_LogInfo( "Checking Outside Pressure value: %f\n", value );
    return checkAndAddValue( wsPtr, STATS_OPRESSURE, value, percentage );
}
// -----------------------------------------------------------------------------
int     checkAndAddValue_ITemp (weatherStats_t *wsPtr, double value, double percentage)
{ 
    Logger_LogInfo( "Checking Inside Temperature value: %f\n", value );
    return checkAndAddValue( wsPtr, STATS_ITEMP, value, percentage );
}
// -----------------------------------------------------------------------------
int     checkAndAddValue_IHumidity (weatherStats_t *wsPtr, double value, double percentage)
{ 
    Logger_LogInfo( "Checking Inside Humidity value: %f\n", value );
    return checkAndAddValue( wsPtr, STATS_IHUMIDITY, value, percentage );
}
// -----------------------------------------------------------------------------
int     checkAndAddValue_Windspeed (weatherStats_t *wsPtr, double value, double percentage)
{ 
    //
    // Update - not going to use a Moving Average for wind speeds - they're too spiky!
    Logger_LogInfo( "Checking Windpseed value: %f\n", value );
    if (value >= 0.0 && value < 80.0)
        return TRUE;
    //return checkAndAddValue( wsPtr, STATS_WINDSPEED, value, percentage );
    return FALSE;
}



// -----------------------------------------------------------------------------
static 
MovingAverage_t *getRightMAPtr (weatherStats_t *wsPtr, int statType)
{
    assert( wsPtr != NULL );
    
    //
    MovingAverage_t *maPtr = (MovingAverage_t *) 0;
    
    switch (statType) {
        //case    STATS_WINDSPEED:    maPtr = wsPtr->windspeedMA;     break;
        case    STATS_OTEMP:        maPtr = wsPtr->oTempMA;         break;
        case    STATS_OHUMIDITY:    maPtr = wsPtr->oHumidityMA;     break;
        case    STATS_OPRESSURE:    maPtr = wsPtr->oPressureMA;     break;
        case    STATS_ITEMP:        maPtr = wsPtr->iTempMA;         break;
        case    STATS_IHUMIDITY:    maPtr = wsPtr->iHumidityMA;     break;
            
        default:
            Logger_LogError( "Invalid stats type passed: %d\n", statType );
    }
    
    return maPtr;
}

// -----------------------------------------------------------------------------
static 
int     *getRightInvalidCountPtr (weatherStats_t *wsPtr, int statType)
{
    assert( wsPtr != NULL );
    
    //
    int *iPtr = (int *) 0;
    
    switch (statType) {
        //case    STATS_WINDSPEED:    iPtr = &(wsPtr->invalidWindspeedCount); break;
        case    STATS_OTEMP:        iPtr = &(wsPtr->invalidOTempCount);     break;
        case    STATS_OHUMIDITY:    iPtr = &(wsPtr->invalidOHumidityCount); break;
        case    STATS_OPRESSURE:    iPtr = &(wsPtr->invalidOPressureCount); break;
        case    STATS_ITEMP:        iPtr = &(wsPtr->invalidiTempCount);     break;
        case    STATS_IHUMIDITY:    iPtr = &(wsPtr->invalidiHumidityCount); break;
            
        default:
            Logger_LogError( "Invalid stats type passed: %d\n", statType );
    }
    
    return iPtr;
}