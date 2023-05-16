/* 
 * File:   homeassistant.h
 * Author: pconroy
 *
 * Created on October 6, 2022, 2:16 PM
 */

#ifndef HOMEASSISTANT_H
#define HOMEASSISTANT_H

#ifdef __cplusplus
extern "C" {
#endif

//#define       DEBUG
    
#ifdef  DEBUG
# define HOMEASSISTANT_ROOT_TOPIC    "debug"
# define STATIONDATA_ROOT_TOPIC      "debug"
#else
# define STATIONDATA_ROOT_TOPIC      "home"
# define HOMEASSISTANT_ROOT_TOPIC    "homeassistant"
#endif

    
#ifndef FALSE
# define FALSE  0
# define TRUE (!FALSE)
#endif
    
#include <uthash/utlist.h>
    
// This is the "ID" of my WS2308
#define STATION_ID          28
    
#define STATION_MODEL_LEN   1024
#define MAX_CONFIGMSG_LEN   1024
#define MAX_CHANNEL_LEN     16
#define MAX_DATETIME_LEN    20
    
    
extern void HomeAssistant_initialize( const WS2308System_t *aSystem );
extern void HomeAssistant_sendData( const WS2308System_t *aSystem, weatherDatum_t *datum );

#ifdef __cplusplus
}
#endif

#endif /* HOMEASSISTANT_H */

