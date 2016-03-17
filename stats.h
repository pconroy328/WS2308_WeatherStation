/* 
 * File:   stats.h
 * Author: pconroy
 *
 * Created on January 19, 2015, 1:53 PM
 */

#ifndef STATS_H
#define	STATS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "movingaverage.h"



typedef struct  weatherStats {
    MovingAverage_t     *oTempMA;
    MovingAverage_t     *oHumidityMA;
    MovingAverage_t     *oPressureMA;

    MovingAverage_t     *iTempMA;
    MovingAverage_t     *iHumidityMA;
    
    //MovingAverage_t     *windspeedMA;
    
    unsigned    int     invalidOTempCount;
    unsigned    int     invalidOHumidityCount;
    unsigned    int     invalidOPressureCount;
    unsigned    int     invalidiTempCount;
    unsigned    int     invalidiHumidityCount;
    //unsigned    int     invalidWindspeedCount;
} weatherStats_t;



typedef  struct  sanityBounds {
    double  minSaneWindspeed;
    double  maxSaneWindspeed;
    double  minSaneOTemp;
    double  maxSaneOTemp;
    double  minSaneOHumidity;
    double  maxSaneOHumidity;
    double  minSaneOPressure;
    double  maxSaneOPressure;
    double  minSaneITemp;
    double  maxSaneITemp;
    double  minSaneIHumidity;
    double  maxSaneIHumidity;
    
    //
    // Need a place to store last sane date and time
    char    lastDate[ 20 ];
    char    lastTime[ 20 ];    
} sanityBounds_t;


#define STATS_WINDSPEED         1
#define STATS_OTEMP             2
#define STATS_OHUMIDITY         3
#define STATS_OPRESSURE         4
#define STATS_ITEMP             5
#define STATS_IHUMIDITY         6



extern  weatherStats_t  *initializeStatsModule (int maBufferSize);
extern  void    teardownStatsModule (weatherStats_t  *wsPtr);
extern  int     checkAndAddValue_OTemp (weatherStats_t *wsPtr, double value, double percentage);
extern  int     checkAndAddValue_OHumidity (weatherStats_t *wsPtr, double value, double percentage);
extern  int     checkAndAddValue_OPressure (weatherStats_t *wsPtr, double value, double percentage);
extern  int     checkAndAddValue_ITemp (weatherStats_t *wsPtr, double value, double percentage);
extern  int     checkAndAddValue_IHumidity (weatherStats_t *wsPtr, double value, double percentage);
extern  int     checkAndAddValue_Windspeed (weatherStats_t *wsPtr, double value, double percentage);



#ifdef	__cplusplus
}
#endif

#endif	/* STATS_H */

