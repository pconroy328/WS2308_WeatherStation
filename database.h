/* 
 * File:   database.h
 * Author: patrick.conroy
 *
 * Created on April 5, 2014, 11:45 AM
 */

#ifndef DATABASE_H
#define	DATABASE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "ws_structs.h"
    
//extern  void    Database_setDatabaseHost( char *hostName );
//extern  void    Database_setDatabaseUserName( char *userName );
//extern  void    Database_setDatabasePassword( char *password );
//extern  void    Database_setDatabaseSchema( char *schemaName );
extern  void    Database_setFailOnDatabaseErrors( int newValue );
extern  void    Database_setDefaults( WS2308System_t *aSystem );
extern  void    Database_initialize( WS2308System_t *aSystem );
    
    
extern  int     Database_openDatabase ( WS2308System_t * );
extern  int     Database_closeDatabase ();
extern  void    Database_insertReadingsRecord( WS2308System_t *aSystem, weatherDatum_t *recPtr );


static  int     createAlarmTable( void );
static  void    dropAlarmTable( void );
static  void    insertAlarmRecord( weatherDatum_t *recPtr );


static  int     createReadingsTable( void );
static  void    dropReadingsTable( void );



#ifdef	__cplusplus
}
#endif

#endif	/* DATABASE_H */

