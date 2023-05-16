/* 
 * File:   logger.h
 * Author: patrick.conroy
 *
 * Created on April 2, 2014, 7:39 PM
 */

#ifndef LOGGER_H
#define	LOGGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef  FALSE
# define FALSE 0
# define TRUE  (!FALSE)
#endif
    
    
extern  void    OldLogger_Initialize( char *fileName, int debugValue );
extern  void    OldLogger_Terminate();
extern  void    OldLogger_LogInfo( char *format, ... );
extern  void    OldLogger_LogDebug( char *format, ... );
extern  void    OldLogger_LogWarning( char *format, ... );
extern  void    OldLogger_LogError( char *format, ... );
extern  void    OldLogger_LogFatal( char *format, ... );


#define OldLogger_FunctionStart(x)     OldLogger_LogDebug( "%s[%d] :: %s() - enter\n", __FILE__, __LINE__, __func__)
#define OldLogger_FunctionEnd(x)       OldLogger_LogDebug( "%s[%d] :: %s() - exit\n", __FILE__, __LINE__, __func__ )





#ifdef	__cplusplus
}
#endif

#endif	/* LOGGER_H */

