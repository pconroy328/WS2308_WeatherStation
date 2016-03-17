/* 
 * File:   mqtt.h
 * Author: patrick.conroy
 *
 * Created on April 2, 2014, 7:41 PM
 */

#ifndef MQTT_H
#define	MQTT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "ws_structs.h"
    
#define MQTT_NOT_CONNECTED      (-1)
    
    
extern  void    MQTT_setDefaults( WS2308System_t *aSystem, char *brokerHostName );
extern  void    MQTT_initialize (WS2308System_t *aSystem );
extern  void    MQTT_teardown( void );
extern  int     MQTT_sendReceive( WS2308System_t *aSystem );
extern  int     MQTT_createWeatherStatus( WS2308System_t *aSystem, weatherDatum_t *weatherRec );
extern  int     MQTT_createWeatherAlarm( WS2308System_t *aSystem, int errorCode, weatherDatum_t *weatherRec );
extern  int     MQTT_handleError( WS2308System_t *aSystem, int errorCode );




#ifdef	__cplusplus
}
#endif

#endif	/* MQTT_H */

