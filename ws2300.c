/*  open2300  - Linux specific library functions
 *  This file contains the common functions that are unique to
 *  Linux. The entire file is ignored in case of Windows
 *
 *  Version 1.10
 *
 *  Control WS2300 weather station
 *
 *  Copyright 2003-2005, Kenneth Lavrsen
 *  This program is published under the GNU General Public license
 */
// #define _SVID_SOURCE                // finds CRTSCTS in the header files
#define _BSD_SOURCE                 // finds CRTSCTS in the header files and usleep()

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/file.h>

#include "ws2300.h"
#include "logger.h"


#define BIT_IS_SET(var,pos) ((var) & (1<<(pos)))

//extern void    logMsg (const char *msg);
//static  char    logBuffer[ 4096 ];


struct  alarmSetting {
    unsigned    char    isSetAddress;           // WS2308 memory map address to set alarm on/off
    unsigned    char    isOnAddress;            // WS2308 memory map address to see if alarm is beeping
    unsigned    int     bit;                    // which bit to test [0..3]
    unsigned    int     ID;                     // the ID number of the alarm (ALARM_ID_xxx)
    char                *name;                  // Human readable name for the alarm
    void        (*fname)( const int );          // Callback function, if you want to be called

} alarmSettings[] = {
    { 0x19, 0x20,   2,  ALARM_ID_STORM_WARNING, "Storm Warning", NULL },
    { 0x19, 0x20,   0,  ALARM_ID_TIME, "Time", NULL },

    { 0x1A, 0x21,   3,  ALARM_ID_PRESSURE_HI, "Pressure High", NULL },
    { 0x1A, 0x21,   2,  ALARM_ID_PRESSURE_HO, "Pressure Low(Ho)", NULL },
    { 0x1B, 0x22,   3,  ALARM_ID_OTEMP_HI, "Outside Temp High", NULL },
    { 0x1B, 0x22,   2,  ALARM_ID_OTEMP_LO, "Outside Temp Low", NULL },
    { 0x1B, 0x22,   1,  ALARM_ID_ITEMP_HI, "Inside Temp High", NULL },
    { 0x1B, 0x22,   0,  ALARM_ID_ITEMP_LO, "Inside Temp Low", NULL },
    { 0x1C, 0x23,   3,  ALARM_ID_DEWTEMP_HI, "Outside Temp High", NULL },
    { 0x1C, 0x23,   2,  ALARM_ID_DEWTEMP_LO, "Outside Temp Low", NULL },
    { 0x1C, 0x23,   1,  ALARM_ID_WINDCHILL_HI, "Windchill Temp High", NULL },
    { 0x1C, 0x23,   0,  ALARM_ID_WINDCHILL_LO, "Windchill Temp Low", NULL },

    { 0x1D, 0x24,   3,  ALARM_ID_IHUMID_HI, "Inside Humidity Hi", NULL },
    { 0x1D, 0x24,   2,  ALARM_ID_IHUMID_LO, "Inside Humidity Low", NULL },
    { 0x1D, 0x24,   1,  ALARM_ID_OHUMID_HI, "Outside Humidity Hi", NULL },
    { 0x1D, 0x24,   0,  ALARM_ID_OHUMID_LO, "Outside Humidity Low", NULL },

    { 0x1E, 0x25,   1,  ALARM_ID_RAIN1H_HI, "1H Rain Min", NULL },
    { 0x1E, 0x25,   0,  ALARM_ID_RAIN24H_LO, "24H Rain Min", NULL },

    { 0x1F, 0x26,   2,  ALARM_ID_WINDDIR, "Wind Direction", NULL },
    { 0x1F, 0x26,   1,  ALARM_ID_WINSPEED_HI, "Windspeed Hi", NULL },
    { 0x1F, 0x26,   0,  ALARM_ID_WINDPSEED_LO, "Windspeed Low", NULL },


    { 0x1A, 0x27,   3,  ALARM_ID_PRESSURE_HI2, "2Pressure High", NULL },
    { 0x1A, 0x27,   2,  ALARM_ID_PRESSURE_HO2, "2Pressure Low(Ho)", NULL },
    { 0x1B, 0x28,   3,  ALARM_ID_OTEMP_HI2, "2Outside Temp High", NULL },
    { 0x1B, 0x28,   2,  ALARM_ID_OTEMP_LO2, "2Outside Temp Low", NULL },
    { 0x1B, 0x28,   1,  ALARM_ID_ITEMP_HI2, "2Inside Temp High", NULL },
    { 0x1B, 0x28,   0,  ALARM_ID_ITEMP_LO2, "2Inside Temp Low", NULL },
    { 0x1C, 0x29,   3,  ALARM_ID_DEWTEMP_HI2, "2Outside Temp High", NULL },
    { 0x1C, 0x29,   2,  ALARM_ID_DEWTEMP_LO2, "2Outside Temp Low", NULL },
    { 0x1C, 0x29,   1,  ALARM_ID_WINDCHILL_HI2, "2Windchill Temp High", NULL },
    { 0x1C, 0x29,   0,  ALARM_ID_WINDCHILL_LO2, "2Windchill Temp Low", NULL },

    { 0x1D, 0x2A,   3,  ALARM_ID_IHUMID_HI2, "2Inside Humidity Hi", NULL },
    { 0x1D, 0x2A,   2,  ALARM_ID_IHUMID_LO2, "2Inside Humidity Low", NULL },
    { 0x1D, 0x2A,   1,  ALARM_ID_OHUMID_HI2, "2Outside Humidity Hi", NULL },
    { 0x1D, 0x2A,   0,  ALARM_ID_OHUMID_LO2, "2Outside Humidity Low", NULL },

    { 0x1E, 0x2B,   1,  ALARM_ID_RAIN1H_HI2,  "21H Rain Min", NULL },
    { 0x1E, 0x2B,   0,  ALARM_ID_RAIN24H_LO2, "224H Rain Min", NULL },

    { 0x1F, 0x2C,   2,  ALARM_ID_WINDDIR2, "2Wind Direction", NULL },
    { 0x1F, 0x2C,   1,  ALARM_ID_WINSPEED_HI2, "2Windspeed Hi", NULL },
    { 0x1F, 0x2C,   0,  ALARM_ID_WINDPSEED_LO2, "2Windspeed Low", NULL },

    { 0, 0, 0, 0 }
};


static  int         DEBUG   = 0;

#define NUM_ALARM_SETTINGS  ( (sizeof alarmSettings ) / (sizeof alarmSettings[ 0 ] )  - 1)


/********************************************************************
 * open_weatherstation, Linux version
 *
 * Input:   devicename (/dev/tty0, /dev/tty1 etc)
 *
 * Returns: Handle to the weatherstation (type WEATHERSTATION)
 *
 ********************************************************************/
WEATHERSTATION open_weatherstation(char *device)
{
	WEATHERSTATION ws2300;
	struct termios adtio;
	int portstatus;

	//Setup serial port

	if ((ws2300 = open(device, O_RDWR | O_NOCTTY)) < 0)
	{
		Logger_LogError( "Unable to open serial device %s\n", device );
		//exit(EXIT_FAILURE);
        return 0;
	}

	//if ( flock(ws2300, LOCK_EX) < 0 ) {
	//	perror("\nSerial device is locked by other program\n");
	//	exit(EXIT_FAILURE);
	// }

	//We want full control of what is set and simply reset the entire adtio struct
	memset(&adtio, 0, sizeof(adtio));

	//tcgetattr(ws2300, &adtio);   // Commented out and replaced by the memset above

	// Serial control options
	adtio.c_cflag &= ~PARENB;      // No parity
	adtio.c_cflag &= ~CSTOPB;      // One stop bit
	adtio.c_cflag &= ~CSIZE;       // Character size mask
	adtio.c_cflag |= CS8;          // Character size 8 bits
	adtio.c_cflag |= CREAD;        // Enable Receiver
	adtio.c_cflag &= ~HUPCL;       // No "hangup"
	adtio.c_cflag &= ~CRTSCTS;     // No flowcontrol
	adtio.c_cflag |= CLOCAL;       // Ignore modem control lines

	// Baudrate, for newer systems
	cfsetispeed(&adtio, BAUDRATE);
	cfsetospeed(&adtio, BAUDRATE);

	// Serial local options: adtio.c_lflag
	// Raw input = clear ICANON, ECHO, ECHOE, and ISIG
	// Disable misc other local features = clear FLUSHO, NOFLSH, TOSTOP, PENDIN, and IEXTEN
	// So we actually clear all flags in adtio.c_lflag
	adtio.c_lflag = 0;

	// Serial input options: adtio.c_iflag
	// Disable parity check = clear INPCK, PARMRK, and ISTRIP
	// Disable software flow control = clear IXON, IXOFF, and IXANY
	// Disable any translation of CR and LF = clear INLCR, IGNCR, and ICRNL
	// Ignore break condition on input = set IGNBRK
	// Ignore parity errors just in case = set IGNPAR;
	// So we can clear all flags except IGNBRK and IGNPAR
	adtio.c_iflag = IGNBRK|IGNPAR;

	// Serial output options
	// Raw output should disable all other output options
	adtio.c_oflag &= ~OPOST;

	adtio.c_cc[VTIME] = 10;		// timer 1s
	adtio.c_cc[VMIN] = 0;		// blocking read until 1 char

	if (tcsetattr(ws2300, TCSANOW, &adtio) < 0)
	{
		Logger_LogError( "Unable to initialize serial device\n" ); 
		// exit(0);
        return 0;
	}

	tcflush(ws2300, TCIOFLUSH);

	// Set DTR low and RTS high and leave other ctrl lines untouched

	ioctl(ws2300, TIOCMGET, &portstatus);	// get current port status
	portstatus &= ~TIOCM_DTR;
	portstatus |= TIOCM_RTS;
	ioctl(ws2300, TIOCMSET, &portstatus);	// set current port status

	return ws2300;
}

/********************************************************************
 * close_weatherstation, Linux version
 *
 * Input: Handle to the weatherstation (type WEATHERSTATION)
 *
 * Returns nothing
 *
 ********************************************************************/
void close_weatherstation(WEATHERSTATION ws)
{
	close(ws);
	return;
}

/********************************************************************
 * reset_06 WS2300 by sending command 06 (Linux version)
 *
 * Input:   device number of the already open serial port
 *
 * Returns: 1 if reset is successful; else 0
 *
 ********************************************************************/
int reset_06(WEATHERSTATION serdevice)
{
    unsigned char command = 0x06;
    unsigned char answer;
    int i;

    for (i = 0; i < 10; i++){
        //sprintf( logBuffer, "reset_06 - DEBUG - looping [%d]\n", i );

        // Discard any garbage in the input buffer
        tcflush(serdevice, TCIFLUSH);

        // puts( "reset_06 - DEBUG - writing 0x06" );
        write_device(serdevice, &command, 1);

        // Occasionally 0, then 2 is returned.  If zero comes back, continue
        // reading as this is more efficient than sending an out-of sync
        // reset and letting the data reads restore synchronization.
        // Occasionally, multiple 2's are returned.  Read with a fast timeout
        // until all data is exhausted, if we got a two back at all, we
        // consider it a success

        // puts( "reset_06 - DEBUG - attempting read of one byte" );
        while (1 == read_device(serdevice, &answer, 1))	{
            // sprintf( logBuffer, "reset_06 - DEBUG - read one byte [%X]\n", answer );
            if (answer == 2) {
                return 1;
            }
        }

        // puts( "reset_06 - DEBUG - sleeping" );
        usleep(50000 * i);   //we sleep longer and longer for each retry
    }


    Logger_LogError( "Could not reset\n" );
    return 0;
}

/********************************************************************
 * read_device in the Linux version is identical
 * to the standard Linux read()
 *
 * Inputs:  serdevice - opened file handle
 *          buffer - pointer to the buffer to read into (unsigned char)
 *          size - number of bytes to read
 *
 * Output:  *buffer - modified on success (pointer to unsigned char)
 *
 * Returns: number of bytes read
 *
 ********************************************************************/
int read_device (WEATHERSTATION serdevice, unsigned char *buffer, int size)
{
	int ret;

	for (;;) {
		ret = read(serdevice, buffer, size);
		if (ret == 0 && errno == EINTR)
			continue;
		return ret;
	}
}

/********************************************************************
 * write_device in the Linux version is identical
 * to the standard Linux write()
 *
 * Inputs:  serdevice - opened file handle
 *          buffer - pointer to the buffer to write from
 *          size - number of bytes to write
 *
 * Returns: number of bytes written
 *
 ********************************************************************/
int write_device(WEATHERSTATION serdevice, unsigned char *buffer, int size)
{
	int ret = write(serdevice, buffer, size);
	tcdrain(serdevice);	// wait for all output written
	return ret;
}

// -----------------------------------------------------------------------------
//  PMC
/* readWithTimeout -- add a timeout to the read() call so it doesn't block
 *      This will be handle for the initial open() call since I'm no longer
 *      sure which USB/RS232 port the weather station is on, anymore
 * 
 * Inputs:  serdevice - opened file handle
 *          buffer - pointer to the buffer to read into (unsigned char)
 *          size - number of bytes to read
 *
 * Output:  *buffer - modified on success (pointer to unsigned char)
 *
 * Returns: number of bytes read
*/
int readWithTimeout (WEATHERSTATION serdevice, unsigned char *buffer, int size, int timeoutSecs)
{
    struct  timeval     tv;
    fd_set              aSet;

    tv.tv_sec = timeoutSecs;
    tv.tv_usec = 0;                     // eg. 2,000,000 would be two seconds
    FD_ZERO( &aSet );
    FD_SET( serdevice, &aSet );

    //
    // select() will return before "timeout" if data is there.
    if (!select( (serdevice + 1), &aSet, NULL, NULL, &tv ) ) {
        //
        //  time'd out with no data
        return 0;
    }

    //
    //  data is there - let's just call the normal read_device() routine
    return read_device( serdevice, buffer, size );
}

/********************************************************************/
/* findWS2300
 * Try to locate the device that the WS2300's connected to
 *
 * Much to my annoyance, using my cheap USB/RS232 adapters has become
 * a pain.  I have two on the system and I haven't figure out a way to tell
 * Linux which one is which, after a boot. So sometimes the WS2300 will boot on
 * /dev/ttyUSB0, and the next time its on /dev/ttyUSB1.  I've also had a port
 * disappear (/dev/ttyUSB1) and then reappear on (/dev/ttyUSB2).
 * Grrrrrrrrrrrrrrrrrrrr...
 *
 *
 * Input: Nothing
 * Output: device name where I found the WS2300
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int findWS2300 (char *deviceName, int devNameLength, int sleepInterval, int maxDevNum)
{
    int             done = FALSE;
    int             devNum = 0;
    int             found = FALSE;
    WEATHERSTATION  ws = 0;
    unsigned char   aByte = 0x06;               // reset command
    char            devBuffer[ 20 ];

    //
    //  Sanity check some of the input values
    if (devNameLength < strlen( "/dev/ttyUSBx"))
        return 0;
    if (sleepInterval > 10)
        sleepInterval = 10;
    if (maxDevNum > 255)
        maxDevNum = 255;

    //
    //  loop thru making device names, try to see if there's a WS2300 there
    while (!done) {
        sprintf( devBuffer, "/dev/ttyUSB%d", devNum );

        Logger_LogDebug( "findWS2300 - DEBUG - opening [%s]\n", devBuffer );
        if ( (ws = open_weatherstation( devBuffer )) != 0) {
            Logger_LogDebug( "findWS2300 - DEBUG - resetting [%s]\n", devBuffer );
            found = (reset_06( ws ) != 0);
            Logger_LogDebug( "findWS2300 - DEBUG - reset returned [%d]\n", found );
            //found = initialize( ws );
            if (found) {
                strncpy( deviceName, devBuffer, devNameLength );
                Logger_LogDebug( "findWS2300 - DEBUG - found the WS2300 on [%s]\n", deviceName );
            }

            close_weatherstation( ws );
        }

        if (!found)
            devNum += 1;

        done = (found || (devNum > maxDevNum));
        if (!done)
            sleep( sleepInterval );
    }

    return found;
}


/********************************************************************
 * sleep_short - Linux version
 *
 * Inputs: Time in milliseconds (integer)
 *
 * Returns: nothing
 *
 ********************************************************************/
void sleep_short(int milliseconds)
{
	usleep(milliseconds/1000);
}

/********************************************************************
 * sleep_long - Linux version
 *
 * Inputs: Time in seconds (integer)
 *
 * Returns: nothing
 *
 ********************************************************************/
void sleep_long(int seconds)
{
	sleep(seconds);
}


/********************************************************************
 * http_request_url - Linux version
 *
 * Inputs: urlline - URL to Weather Underground with path and data
 *                   as a pointer to char array (string)
 *
 * Returns: 0 on success and -1 if fail.
 *
 * Action: Send a http request to Weather Underground
 *
 ********************************************************************/
int http_request_url(char *urlline)
{
	int sockfd;
	struct hostent *hostinfo;
	struct sockaddr_in urladdress;
	char buffer[1024];
	int bytes_read;

	if ( (hostinfo = gethostbyname(WEATHER_UNDERGROUND_BASEURL)) == NULL )
	{
		perror("Host not known by DNS server or DNS server not working");
		return(-1);
	}

	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	{
		perror("Cannot open socket");
		return(-1);
	}

	memset(&urladdress, 0, sizeof(urladdress));
	urladdress.sin_family = AF_INET;
	urladdress.sin_port = htons(80); /*default HTTP Server port */

	urladdress.sin_addr = *(struct in_addr *)*hostinfo->h_addr_list;

	if (connect(sockfd,(struct sockaddr*)&urladdress,sizeof(urladdress)) != 0)
	{
		perror("Cannot connect to host");
		return(-1);
	}
	sprintf(buffer, "GET %s\nHTTP/1.0\n\n", urlline);
	send(sockfd, buffer, strlen(buffer), 0);

	/* While there's data, read and print it */
	do
	{
		memset(buffer, 0, sizeof(buffer));
		bytes_read = recv(sockfd, buffer, sizeof(buffer), 0);
		if ( bytes_read > 0 )
			if (DEBUG) printf("%s", buffer);
	}
	while ( bytes_read > 0 );

	/* Close socket and clean up winsock */
	close(sockfd);

	return(0);
}


/********************************************************************
 * citizen_weather_send - Linux version
 *
 * Inputs: config structure (pointer to) - containing CW ID
 *         datastring (pointer to) - containing all the data
 *
 * Returns: 0 on success and -1 if fail.
 *
 * Action: Send data to Citizen Weather
 *
 ********************************************************************/
int citizen_weather_send(struct config_type *config, char *aprsline)
{
	int sockfd = -1; // just to eliminate a warning we'll set this
	int bytes_read;
	struct hostent *hostinfo;
	struct sockaddr_in urladdress;
	char buffer[1024];          //Enough to hold a response
	int hostnum;

	// Connect to server and send the record
	// loop trying all of the defined servers
	for (hostnum = 0; hostnum <= config->num_hosts; hostnum++)
	{
		if ( hostnum == config->num_hosts )
			return(-1);          // tried 'em all, fail exit

		if ( (hostinfo = gethostbyname(config->aprs_host[hostnum].name) ) == NULL )
		{
			sprintf(buffer,"Host, %s, not known ", config->aprs_host[hostnum].name);
			perror(buffer);
			continue;
		}

		if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
		{
			sprintf(buffer,"Cannot open socket on %s ", config->aprs_host[hostnum].name);
			perror(buffer);
			continue;
		}

		memset(&urladdress, 0, sizeof(urladdress)); // clear the structure
		urladdress.sin_family = AF_INET;
		urladdress.sin_port = htons(config->aprs_host[hostnum].port);
		urladdress.sin_addr = *(struct in_addr *)*hostinfo->h_addr_list;

		if ( connect(sockfd, (struct sockaddr*)&urladdress, sizeof(urladdress)) != 0 )
		{
			sprintf(buffer,"Cannot connect to host: %s ", config->aprs_host[hostnum].name);
			perror(buffer);
			continue;
		}
		else
		{
			break;   // success
		}
	}

	if (DEBUG) printf("%d: %s: ",hostnum, config->aprs_host[hostnum].name);

	memset(buffer, 0, sizeof(buffer));

	if ( (recv(sockfd, buffer, sizeof(buffer), 0) > 0) && (DEBUG != 0) )                 // read login prompt
	{
		printf("%s", buffer);	// display prompt - if debug
	}

	// The login/header line
        // sprintf(buffer,"user %s pass -1 vers open2300 %s\n", config->citizen_weather_id, VERSION );
        // PMC fix - version is an int
        sprintf(buffer,"user %s pass -1 vers open2300 %d\n", config->citizen_weather_id, VERSION );
	send(sockfd, buffer, strlen(buffer), 0);
	if (DEBUG)
		printf("%s\n", buffer);

	// now the data
	sprintf(buffer,"%s\n", aprsline);
	send(sockfd, buffer, strlen(buffer), 0);
	if (DEBUG)
		printf("%s\n", buffer);

	/* While there's data, read and print it - Not sure it is needed */
	do
	{
		memset(buffer, 0, sizeof(buffer));
		bytes_read = recv(sockfd, buffer, sizeof(buffer), 0);
		if ( bytes_read > 0 )
		{
			if (DEBUG)
				printf("Data returned from server\n%s\n", buffer);
			break;
		}
	}
	while ( bytes_read > 0 );

	/* Close socket*/
	close(sockfd);

	return(0);
}

/*  #!/bin/bash
 * # Script by Ralph Beckmann (12.01.2006)
 *      SS_MM_HH=$(/opt/open2300/open2300 0200 r 3 | grep Data: | cut -c 28-)
 *      dd_mm_YY=$(/opt/open2300/open2300 0240 r 3 | grep Data: | cut -c 28-)
 *      if [ ${#SS_MM_HH} = 8 -a ${#dd_mm_YY} = 8 ] then
 *          # set system clock echo -n "Adjusting date from '$(date)' to: " date ${dd_mm_YY:3:2}${dd_mm_YY:0:2}${SS_MM_HH:6:2}${SS_MM_HH:3:2}${dd_mm_YY:6:2}.${SS_MM_HH:0:2}
 *          # set hardware to value of system clock
 *          /sbin/hwclock -w
 *      fi
 */

/*
0019 	0 	Alarm set flags: -/Storm warning/-/Time
001A 	0 	Alarm set flags: Pressure Hi/Pressure Ho/-/-
001B 	0 	Alarm set flags: T out Hi/T out Lo/T in Hi/T in Lo
001C 	0 	Alarm set flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
001D 	0 	Alarm set flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
001E 	0 	Alarm set flags: -/-/Rain 1h/Rain 24h
001F 	0 	Alarm set flags: -/Wind dir/Wind speed Hi/Wind speed Lo
0020 	4 	Alarm active flags: -/Alarm Icon/-/Time?
0021 	0 	Alarm active flags: Pressure Hi/Pressure Ho/-/-
0022 	0 	Alarm active flags: T out Hi/T out Lo/T in Hi/T in Lo
0023 	0 	Alarm active flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
0024 	0 	Alarm active flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
0025 	0 	Alarm active flags: -/-/Rain 1h/Rain 24h
0026 	0 	Alarm active flags: -/Wind dir/Wind speed Hi/Wind speed Lo
0027 	0 	Alarm active flags: Pressure Hi/Pressure Ho/-/-
0028 	0 	Alarm active flags: T out Hi/T out Lo/T in Hi/T in Lo
0029 	0 	Alarm active flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
002A 	0 	Alarm active flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
002B 	0 	Alarm active flags: -/-/Rain 1h/Rain 24h
002C 	0 	Alarm active flags: -/-/Wind speed Hi/Wind speed Lo
 */

// ------------------------------------
static
int findAlarmIndex (const int alarmID)
{
    int i;
    for (i = 0; i < NUM_ALARM_SETTINGS; i += 1)
        if (alarmSettings[ i ].ID == alarmID)
            return i;

    return -1;
}

// -------------------------------------
int isAlarmSet (const WEATHERSTATION ws2300, const int alarmID)
{
    char    data[ 20 ];
    char    command[ 20 ];

    int     idx = findAlarmIndex( alarmID );
    int     address = 0;
    int     bit = 0;
    int     numRead = 0;
    int     isSet = FALSE;

    // fprintf( stderr, "isAlarmSet(%d)\n", alarmID );
    if (idx >= 0) {
        address = alarmSettings[ idx ].isSetAddress;
        bit = alarmSettings[ idx ].bit;

        // fprintf( stderr, "   -- idx: %d, addr: %x, bit %d, name :[%s]\n", idx, address, bit, alarmSettings[idx].name );

        memset( data, '\0', sizeof data );
        memset( command, '\0', sizeof command );

        numRead = read_safe( ws2300, address, 1, data, command );
        //fprintf( stderr, "   -- read_safe() returned %d. data[0]=%x,data[1]=%x; cmd[0]=%x,cmd[1]=%x\n", numRead, data[0],data[1],command[0],command[1] );
        //fprintf( stderr, "   -- bit is " );
        isSet = (BIT_IS_SET( data[ 0 ], bit ));
    } else
        // fprintf( stderr, "BAD ALARM ID!\n" );
        ;
    
    return isSet;
}   

// -------------------------------------
int setAlarm (WEATHERSTATION ws2300, const void *data, const void (*callback)( int ) )
{

}

// -------------------------------------
int unsetAlarm (WEATHERSTATION ws2300)
{

}

// -------------------------------------
int isAlarmRinging (WEATHERSTATION ws2300, int alarmID)
{
    char    data[ 20 ];
    char    command[ 20 ];
    int     idx = findAlarmIndex( alarmID );
    int     address = alarmSettings[ idx ].isOnAddress;
    int     bit = alarmSettings[ idx ].bit;
    int     numRead = 0;

    //fprintf( stderr, "isAlarmBeeping(%d)\n", alarmID );
    if (idx >= 0) {
        //fprintf( stderr, "   -- idx: %d, addr: %x, bit %d, name :[%s]\n", idx, address, bit, alarmSettings[idx].name );

        memset( data, '\0', sizeof data );
        memset( command, '\0', sizeof command );

        numRead = read_safe( ws2300, address, 1, data, command );
        //fprintf( stderr, "   -- read_safe() returned %d. data[0]=%x,data[1]=%x; cmd[0]=%x,cmd[1]=%x\n", numRead, data[0],data[1],command[0],command[1] );

        //fprintf( stderr, "   -- bit is " );
        if (BIT_IS_SET( data[ 0 ], bit ))
            //fprintf( stderr, "SET!  Alarm is BEEPING\n" );
            ;
        else
            //fprintf( stderr, "RESET! Alarm is SILENT\n" );
            ;
    } else
        //fprintf( stderr, "BAD ALARM ID!" );
        ;
}

// ----------------------------------------------------------------------------
double  getAlarm_IndoorTempLo ( const WEATHERSTATION ws2300,
                                const int convToFarenheit )
{
    unsigned char   data[ 20 ];
    unsigned char   command[ 25 ];	//room for write data also
    int             address = 0x369;
    int             bytes = 2;
    double          temp = 0.0;

    memset( data, '\0', sizeof data );
    memset( command, '\0', sizeof command );
    
    if (read_safe( ws2300, address, bytes, data, command ) != bytes)
        read_error_exit();

    //fprintf( stderr, ">>> data [%x][%x][%x][%x]\n", data[0],data[1],data[2],data[3] );
    //fprintf( stderr, ">>> command [%x][%x][%x][%x]\n", command[0],command[1],command[2],command[3] );


    temp = ((data[ 1 ] >> 4) * 10.0 +
            (data[ 1 ] & 0xF) +
            (data[ 0 ] >> 4) / 10.0 +
            (data[ 0 ] & 0xF ) /100.0) - 30.0;

    //fprintf( stderr, ">>> Temp before: %f", temp  );
    if (convToFarenheit)
        temp = temp  * 9.0 / 5.0 + 32.0;
    
    //fprintf( stderr, " after: %f\n", temp  );
    return temp;
}

// ----------------------------------------------------------------------------
void setAlarm_IndoorTempLo ( const WEATHERSTATION ws2300,
                                const int isFarenheit,
                                const double newTemp)
{
    int             address = 0x369;
    int             bytes = 4;
    double          temp = newTemp;
    int             digit = 0;
    unsigned char   data[ 20 ];
    unsigned char   command[ 25 ];	//room for write data also

    /* ** code starts ** */
    if (isFarenheit)                    // device stores data as Centigrade
        temp = ((newTemp - 32.0) * 5.0 / 9.0);

    if (temp > 99.99)
        temp = 99.99;                   // we can't store anything larger!


    //fprintf( stderr, "Temp in was %f, temp converted is: %f\n", newTemp, temp );
    
    //
    // data[0] = hundreds, tens digits; data[1]= ones, tenth's digits
    memset( data, '\0', sizeof data );
    memset( command, '\0', sizeof command );

    //
    // data[0] = hundreds, tens digits; data[1]= ones, tenth's digits
    /*
     	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;
    */
    //
    // When you write - each nybble goes into it's own byte
    digit = (int) trunc( temp / 10.0 );
    data[ 0 ] = digit;

    temp = temp - (digit * 10.0);
    digit = (int) trunc( temp / 1.0 );
    data[ 1 ] = digit;

    temp = temp - (digit * 1.0);
    digit = (int) ( temp / 0.1 );
    data[ 2 ] = digit;

    temp = temp - (digit * 0.1);
    digit = (int) ( temp / 0.01 );
    data[ 3 ] = digit;

    //fprintf( stderr, "data [%x][%x][%x][%x]\n", data[0],data[1],data[2],data[3] );
    
    bytes = 4;
    digit = write_safe( ws2300, address, bytes, WRITENIB, data, command );
    //fprintf( stderr, "write_safe() returned: %d\n", digit );

    //fprintf( stderr, "command [%x][%x][%x][%x]\n", command[0],command[1],command[2],command[3] );
}   /* setAlarm_IndoorTempLo */



// -------------------------------------
char *read_date (WEATHERSTATION ws2300, char *date)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int     address=0x240;
	int     bytes = 3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();


        // no checking to make sure it's big nuff...
        sprintf( date, "%02X/%02X/%02X%c", data[ 1 ], data[ 0 ], data[ 2 ], '\0' );
        return date;
}   // read_date

//--------------------------------------------------------------
char *read_time (WEATHERSTATION ws2300, char *time)
{
    unsigned char data[20];
    unsigned char command[25];	//room for write data also
    //      int     address=0x200;      // 0x200 is UTC time
    int     address=0x239;              // 0x239 is Local time
    int     bytes = 3;
    int     i;

    if (read_safe(ws2300, address, bytes, data, command) != bytes)
        read_error_exit();

    // for (i = 0; i < bytes; i +=1)
    //    printf( "Data[%d] = %x\n", i, data[ i ] );

    // no checking to make sure it's big nuff...
    sprintf( time, "%02X:%02X:%02X%c", data[ 2 ], data[ 1 ], data[ 0 ], '\0' );
    return time;
}   // read_time

//--------------------------------------------------------------
int read_dow (WEATHERSTATION ws2300)
{
    // this isn't working yet...
    unsigned char data[20];
    unsigned char command[25];	//room for write data also
    int     address=0x23F;
    int     bytes = 3;
    int     i;

    if (read_safe(ws2300, address, bytes, data, command) != bytes)
        read_error_exit();

    for (i = 0; i < bytes; i +=1)
        printf( "Data[%d] = %x\n", i, data[ i ] );

    // no checking to make sure it's big nuff...
    return data[ 0 ];
}   // read_time



/********************************************************************/
/* temperature_indoor
 * Read indoor temperature, current temperature only
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Returns: Temperature (deg C if temperature_conv is 0)
 *                      (deg F if temperature_conv is 1)
 *
 ********************************************************************/
double temperature_indoor(WEATHERSTATION ws2300, int temperature_conv)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x346;
	int bytes=2;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	if (temperature_conv)
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) -
		          30.0) * 9 / 5 + 32);
	else
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) - 30.0));
}


/********************************************************************/
/* temperature_indoor_minmax
 * Read indoor min/max temperatures with timestamps
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Output: Temperatures temp_min and temp_max
 *                (deg C if temperature_conv is 0)
 *                (deg F if temperature_conv is 1)
 *         Timestamps for temp_min and temp_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 ********************************************************************/
void temperature_indoor_minmax(WEATHERSTATION ws2300,
                               int temperature_conv,
                               double *temp_min,
                               double *temp_max,
                               struct timestamp *time_min,
                               struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x34B;
	int bytes=15;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*temp_min = ((data[1]>>4)*10 + (data[1]&0xF) + (data[0]>>4)/10.0 +
	             (data[0]&0xF)/100.0) - 30.0;

	*temp_max = ((data[4]&0xF)*10 + (data[3]>>4) + (data[3]&0xF)/10.0 +
	             (data[2]>>4)/100.0) - 30.0;

	if (temperature_conv)
	{
		*temp_min = *temp_min * 9/5 + 32;
		*temp_max = *temp_max * 9/5 + 32;
	}

	time_min->minute = ((data[5] & 0xF) * 10) + (data[4] >> 4);
	time_min->hour = ((data[6] & 0xF) * 10) + (data[5] >> 4);
	time_min->day = ((data[7] & 0xF) * 10) + (data[6] >> 4);
	time_min->month = ((data[8] & 0xF) * 10) + (data[7] >> 4);
	time_min->year = 2000 + ((data[9] & 0xF) * 10) + (data[8] >> 4);

	time_max->minute = ((data[10] & 0xF) * 10) + (data[9] >> 4);
	time_max->hour = ((data[11] & 0xF) * 10) + (data[10] >> 4);
	time_max->day = ((data[12] & 0xF) * 10) + (data[11] >> 4);
	time_max->month = ((data[13] & 0xF) * 10) + (data[12] >> 4);
	time_max->year = 2000 + ((data[14] & 0xF) * 10) + (data[13] >> 4);

	return;
}

/********************************************************************/
/* temperature_indoor_reset
 * Reset indoor min/max temperatures with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int temperature_indoor_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current temperature into data_value
	address=0x346;
	number=2;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x34B;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x354;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x350;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x35E;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************/
/* temperature_outdoor
 * Read indoor temperature, current temperature only
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Returns: Temperature (deg C if temperature_conv is 0)
 *                      (deg F if temperature_conv is 1)
 *
 ********************************************************************/
double temperature_outdoor(WEATHERSTATION ws2300, int temperature_conv)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x373;
	int bytes=2;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	if (temperature_conv)
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) -
		          30.0) * 9 / 5 + 32);
	else
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) - 30.0));
}


/********************************************************************
 * temperature_outdoor_minmax
 * Read outdoor min/max temperatures with timestamps
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Output: Temperatures temp_min and temp_max
 *                (deg C if temperature_conv is 0)
 *                (deg F if temperature_conv is 1)
 *         Timestamps for temp_min and temp_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 ********************************************************************/
void temperature_outdoor_minmax(WEATHERSTATION ws2300,
                                int temperature_conv,
                                double *temp_min,
                                double *temp_max,
                                struct timestamp *time_min,
                                struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x378;
	int bytes=15;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*temp_min = ((data[1]>>4)*10 + (data[1]&0xF) + (data[0]>>4)/10.0 +
	             (data[0]&0xF)/100.0) - 30.0;

	*temp_max = ((data[4]&0xF)*10 + (data[3]>>4) + (data[3]&0xF)/10.0 +
	             (data[2]>>4)/100.0) - 30.0;

	if (temperature_conv)
	{
		*temp_min = *temp_min * 9/5 + 32;
		*temp_max = *temp_max * 9/5 + 32;
	}

	time_min->minute = ((data[5] & 0xF) * 10) + (data[4] >> 4);
	time_min->hour = ((data[6] & 0xF) * 10) + (data[5] >> 4);
	time_min->day = ((data[7] & 0xF) * 10) + (data[6] >> 4);
	time_min->month = ((data[8] & 0xF) * 10) + (data[7] >> 4);
	time_min->year = 2000 + ((data[9] & 0xF) * 10) + (data[8] >> 4);

	time_max->minute = ((data[10] & 0xF) * 10) + (data[9] >> 4);
	time_max->hour = ((data[11] & 0xF) * 10) + (data[10] >> 4);
	time_max->day = ((data[12] & 0xF) * 10) + (data[11] >> 4);
	time_max->month = ((data[13] & 0xF) * 10) + (data[12] >> 4);
	time_max->year = 2000 + ((data[14] & 0xF) * 10) + (data[13] >> 4);

	return;
}


/********************************************************************/
/* temperature_outdoor_reset
 * Reset outdoor min/max temperatures with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int temperature_outdoor_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current temperature into data_value
	address=0x373;
	number=2;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x378;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x381;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x37D;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x38B;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * dewpoint
 * Read dewpoint, current value only
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Returns: Dewpoint    (deg C if temperature_conv is 0)
 *                      (deg F if temperature_conv is 1)
 *
 ********************************************************************/
double dewpoint(WEATHERSTATION ws2300, int temperature_conv)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3CE;
	int bytes=2;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	if (temperature_conv)
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) -
		          30.0) * 9 / 5 + 32);
	else
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) - 30.0));
}


/********************************************************************
 * dewpoint_minmax
 * Read outdoor min/max dewpoint with timestamps
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Output: Dewpoints dp_min and dp_max
 *                (deg C if temperature_conv is 0),
 *                (deg F if temperature_conv is 1)
 *         Timestamps for dp_min and dp_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 ********************************************************************/
void dewpoint_minmax(WEATHERSTATION ws2300,
                     int temperature_conv,
                     double *dp_min,
                     double *dp_max,
                     struct timestamp *time_min,
                     struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3D3;
	int bytes=15;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*dp_min = ((data[1]>>4)*10 + (data[1]&0xF) + (data[0]>>4)/10.0 +
	           (data[0]&0xF)/100.0) - 30.0;

	*dp_max = ((data[4]&0xF)*10 + (data[3]>>4) + (data[3]&0xF)/10.0 +
	           (data[2]>>4)/100.0) - 30.0;

	if (temperature_conv)
	{
		*dp_min = *dp_min * 9/5 + 32;
		*dp_max = *dp_max * 9/5 + 32;
	}

	time_min->minute = ((data[5] & 0xF) * 10) + (data[4] >> 4);
	time_min->hour = ((data[6] & 0xF) * 10) + (data[5] >> 4);
	time_min->day = ((data[7] & 0xF) * 10) + (data[6] >> 4);
	time_min->month = ((data[8] & 0xF) * 10) + (data[7] >> 4);
	time_min->year = 2000 + ((data[9] & 0xF) * 10) + (data[8] >> 4);

	time_max->minute = ((data[10] & 0xF) * 10) + (data[9] >> 4);
	time_max->hour = ((data[11] & 0xF) * 10) + (data[10] >> 4);
	time_max->day = ((data[12] & 0xF) * 10) + (data[11] >> 4);
	time_max->month = ((data[13] & 0xF) * 10) + (data[12] >> 4);
	time_max->year = 2000 + ((data[14] & 0xF) * 10) + (data[13] >> 4);

	return;
}


/********************************************************************/
/* dewpoint_reset
 * Reset min/max dewpoint with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int dewpoint_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current dewpoint into data_value
	address=0x3CE;
	number=2;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x3D3;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x3DC;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x3D8;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x3E6;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * humidity_indoor
 * Read indoor relative humidity, current value only
 *
 * Input: Handle to weatherstation
 * Returns: relative humidity in percent (integer)
 *
 ********************************************************************/
int humidity_indoor(WEATHERSTATION ws2300)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3FB;
	int bytes=1;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	return ((data[0] >> 4) * 10 + (data[0] & 0xF));
}


/********************************************************************
 * humidity_indoor_all
 * Read both current indoor humidity and min/max values with timestamps
 *
 * Input: Handle to weatherstation
 * Output: Relative humidity in % hum_min and hum_max (integers)
 *         Timestamps for hum_min and hum_max in pointers to
 *                timestamp structures for time_min and time_max
 * Returns: releative humidity current value in % (integer)
 *
 ********************************************************************/
int humidity_indoor_all(WEATHERSTATION ws2300,
                        int *hum_min,
                        int *hum_max,
                        struct timestamp *time_min,
                        struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3FB;
	int bytes=13;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*hum_min = (data[1] >> 4) * 10 + (data[1] & 0xF);
	*hum_max = (data[2] >> 4) * 10 + (data[2] & 0xF);

	time_min->minute = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_min->hour = ((data[4] >> 4) * 10) + (data[4] & 0xF);
	time_min->day = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_min->month = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_min->year = 2000 + ((data[7] >> 4) * 10) + (data[7] & 0xF);

	time_max->minute = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->hour = ((data[9] >> 4) * 10) + (data[9] & 0xF);
	time_max->day = ((data[10] >> 4) * 10) + (data[10] & 0xF);
	time_max->month = ((data[11] >> 4) * 10) + (data[11] & 0xF);
	time_max->year = 2000 + ((data[12] >> 4) * 10) + (data[12] & 0xF);

	return ((data[0] >> 4) * 10 + (data[0] & 0xF));
}


/********************************************************************/
/* humidity_indoor_reset
 * Reset min/max indoor humidity with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int humidity_indoor_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current humidity into data_value
	address=0x3FB;
	number=1;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x3FD;
		number=2;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x401;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x3FF;
		number=2;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x40B;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * humidity_outdoor
 * Read relative humidity, current value only
 *
 * Input: Handle to weatherstation
 * Returns: relative humidity in percent (integer)
 *
 ********************************************************************/
int humidity_outdoor(WEATHERSTATION ws2300)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x419;
	int bytes=1;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	return ((data[0] >> 4) * 10 + (data[0] & 0xF));
}


/********************************************************************
 * humidity_outdoor_all
 * Read both current outdoor humidity and min/max values with timestamps
 *
 * Input: Handle to weatherstation
 * Output: Relative humidity in % hum_min and hum_max (integers)
 *         Timestamps for hum_min and hum_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: releative humidity current value in % (integer)
 *
 ********************************************************************/
int humidity_outdoor_all(WEATHERSTATION ws2300,
                         int *hum_min,
                         int *hum_max,
                         struct timestamp *time_min,
                         struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x419;
	int bytes=13;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*hum_min = (data[1] >> 4) * 10 + (data[1] & 0xF);
	*hum_max = (data[2] >> 4) * 10 + (data[2] & 0xF);

	time_min->minute = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_min->hour = ((data[4] >> 4) * 10) + (data[4] & 0xF);
	time_min->day = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_min->month = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_min->year = 2000 + ((data[7] >> 4) * 10) + (data[7] & 0xF);

	time_max->minute = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->hour = ((data[9] >> 4) * 10) + (data[9] & 0xF);
	time_max->day = ((data[10] >> 4) * 10) + (data[10] & 0xF);
	time_max->month = ((data[11] >> 4) * 10) + (data[11] & 0xF);
	time_max->year = 2000 + ((data[12] >> 4) * 10) + (data[12] & 0xF);

	return ((data[0] >> 4) * 10 + (data[0] & 0xF));
}


/********************************************************************/
/* humidity_outdoor_reset
 * Reset min/max outdoor humidity with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int humidity_outdoor_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current humidity into data_value
	address=0x419;
	number=1;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x41B;
		number=2;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x41F;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x41D;
		number=2;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x429;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * wind_current
 * Read wind speed, wind direction and last 5 wind directions
 *
 * Input: Handle to weatherstation
 *        wind_speed_conv_factor controlling convertion to other
 *             units than m/s
 *
 * Output: winddir - pointer to double in degrees
 *
 * Returns: Wind speed (double) in the unit given in the loaded config
 *
 ********************************************************************/
double wind_current(WEATHERSTATION ws2300,
                    double wind_speed_conv_factor,
                    double *winddir)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int i;
	int address=0x527; //Windspeed and direction
	int bytes=3;

	for (i=0; i<MAXWINDRETRIES; i++)
	{
		if (read_safe(ws2300, address, bytes, data, command)!=bytes) //Wind
			read_error_exit();

		if ( (data[0]!=0x00) ||                            //Invalid wind data
		    ((data[1]==0xFF) && (((data[2]&0xF)==0)||((data[2]&0xF)==1))) )
		{
            fprintf( stderr, "wind_current:: Invalid wind data. [%X], [%X], [%x]\n", data[0], data[1], data[2] );
			sleep_long(10); //wait 10 seconds for new wind measurement
			continue;
		}
		else
		{
			break;
		}
	}

	//Calculate wind directions

	*winddir = (data[2]>>4)*22.5;

	//Calculate raw wind speed 	- convert from m/s to whatever
	return( (((data[2]&0xF)<<8)+(data[1])) / 10.0 * wind_speed_conv_factor );
}


/********************************************************************
 * wind_all
 * Read wind speed, wind direction and last 5 wind directions
 *
 * Input: Handle to weatherstation
 *        wind_speed_conv_factor controlling convertion to other
 *             units than m/s
 *
 * Output: winddir_index
 *              Current wind direction expressed as ticks from North
 *              where North=0. Used to convert to direction string
 *         winddir
 *              Array of doubles containing current winddirection
 *              in winddir[0] and the last 5 in the following
 *              positions all given in degrees
 *
 * Returns: Wind speed (double) in the unit given in the loaded config
 *
 ********************************************************************/
double wind_all(WEATHERSTATION ws2300,
                double wind_speed_conv_factor,
                int *winddir_index,
                double *winddir)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int i;
	int address=0x527; //Windspeed and direction
	int bytes=6;

	for (i=0; i<MAXWINDRETRIES; i++)
	{
		if (read_safe(ws2300, address, bytes, data, command)!=bytes) //Wind
			read_error_exit();

		if ( (data[0]!=0x00) ||                             //Invalid wind data
		   ((data[1]==0xFF) && (((data[2]&0xF)==0)||( (data[2]&0xF)==1))) )
		{
			sleep_long(10); //wait 10 seconds for new wind measurement
			continue;
		}
		else
		{
			break;
		}
	}

	//Calculate wind directions

	*winddir_index = (data[2]>>4);
	winddir[0] = (data[2]>>4)*22.5;
	winddir[1] = (data[3]&0xF)*22.5;
	winddir[2] = (data[3]>>4)*22.5;
	winddir[3] = (data[4]&0xF)*22.5;
	winddir[4] = (data[4]>>4)*22.5;
	winddir[5] = (data[5]&0xF)*22.5;

	//Calculate raw wind speed - convert from m/s to whatever
	return ( (((data[2]&0xF)<<8)+(data[1]) ) / 10.0 * wind_speed_conv_factor);
}


/********************************************************************
 * wind_minmax
 * Read min/max wind speeds with timestamps
 *
 * Input: Handle to weatherstation
 *        wind_speed_conv_factor controlling convertion to other
 *             units than m/s
 *
 * Output: Wind wind_min and wind_max (double)
 *                unit defined by config conversion factor
 *         Timestamps for wind_min and wind_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: wind max (double)
 *
 * Note: The function is made so that if a pointer to
 *       wind_min/max and time_min/max is a NULL pointer the function
 *       ignores this parameter. Example: if you only need wind_max
 *       use the function like this..
 *       windmax = wind_minmax(ws2300,METERS_PER_SECOND,NULL,NULL,NULL,NULL);
 *
 ********************************************************************/
double wind_minmax(WEATHERSTATION ws2300,
                   double wind_speed_conv_factor,
                   double *wind_min,
                   double *wind_max,
                   struct timestamp *time_min,
                   struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x4EE;
	int bytes=15;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
			read_error_exit();

	if (wind_min != NULL)
		*wind_min = (data[1]*256 + data[0])/360.0 * wind_speed_conv_factor;
	if (wind_max != NULL)
		*wind_max = (data[4]*256 + data[3])/360.0 * wind_speed_conv_factor;

	if (time_min != NULL)
	{
		time_min->minute = ((data[5] >> 4) * 10) + (data[5] & 0xF);
		time_min->hour = ((data[6] >> 4) * 10) + (data[6] & 0xF);
		time_min->day = ((data[7] >> 4) * 10) + (data[7] & 0xF);
		time_min->month = ((data[8] >> 4) * 10) + (data[8] & 0xF);
		time_min->year = 2000 + ((data[9] >> 4) * 10) + (data[9] & 0xF);
	}

	if (time_max != NULL)
	{
		time_max->minute = ((data[10] >> 4) * 10) + (data[10] & 0xF);
		time_max->hour = ((data[11] >> 4) * 10) + (data[11] & 0xF);
		time_max->day = ((data[12] >> 4) * 10) + (data[12] & 0xF);
		time_max->month = ((data[13] >> 4) * 10) + (data[13] & 0xF);
		time_max->year = 2000 + ((data[14] >> 4) * 10) + (data[14] & 0xF);
	}

	return ((data[4]*256 + data[3])/360.0 * wind_speed_conv_factor);
}


/********************************************************************/
/* wind_reset
 * Reset min/max wind with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int wind_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;
	int i;
	int current_wind;

	address=0x527; //Windspeed
	number=3;

	for (i=0; i<MAXWINDRETRIES; i++)
	{
		if (read_safe(ws2300, address, number, data_read, command)!=number)
			read_error_exit();

		if ((data_read[0]!=0x00) ||                            //Invalid wind data
		    ((data_read[1]==0xFF)&&(((data_read[2]&0xF)==0)||((data_read[2]&0xF)==1))))
		{
			sleep_long(10); //wait 10 seconds for new wind measurement
			continue;
		}
		else
		{
			break;
		}
	}

	current_wind = ( ((data_read[2]&0xF)<<8) + (data_read[1]) ) * 36;

	data_value[0] = current_wind&0xF;
	data_value[1] = (current_wind>>4)&0xF;
	data_value[2] = (current_wind>>8)&0xF;
	data_value[3] = (current_wind>>12)&0xF;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x4EE;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x4F8;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x4F4;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x502;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * windchill
 * Read wind chill, current value only
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Returns: wind chill  (deg C if config.temperature_conv is not set)
 *                      (deg F if config.temperature_conv is set)
 *
 * It is recommended to run this right after a wind speed reading
 * to enhance the likelyhood that the wind speed is valid
 *
 ********************************************************************/
double windchill(WEATHERSTATION ws2300, int temperature_conv)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3A0;
	int bytes=2;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	if (temperature_conv)
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) -
		          30.0) * 9 / 5 + 32);
	else
		return ((((data[1] >> 4) * 10 + (data[1] & 0xF) +
		          (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) - 30.0));
}


/********************************************************************
 * windchill_minmax
 * Read wind chill min/max with timestamps
 *
 * Input: Handle to weatherstation
 *        temperature_conv flag (integer) controlling
 *            convertion to deg F
 *
 * Output: Windchill wc_min and wc_max
 *                (deg C if config.temperature_conv is not set)
 *                (deg F if config.temperature_conv is set)
 *         Timestamps for wc_min and wc_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: Nothing
 *
 ********************************************************************/
void windchill_minmax(WEATHERSTATION ws2300,
                      int temperature_conv,
                      double *wc_min,
                      double *wc_max,
                      struct timestamp *time_min,
                      struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x3A5;
	int bytes=15;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*wc_min = ( (data[1]>>4)*10 + (data[1]&0xF) + (data[0]>>4)/10.0 +
	            (data[0]&0xF)/100.0 ) - 30.0;

	*wc_max = ( (data[4]&0xF)*10 + (data[3]>>4) + (data[3]&0xF)/10.0 +
	            (data[2]>>4)/100.0 ) - 30.0;

	if (temperature_conv)
	{
		*wc_min = *wc_min * 9/5 + 32;
		*wc_max = *wc_max * 9/5 + 32;
	}

	time_min->minute = ((data[5] & 0xF) * 10) + (data[4] >> 4);
	time_min->hour = ((data[6] & 0xF) * 10) + (data[5] >> 4);
	time_min->day = ((data[7] & 0xF) * 10) + (data[6] >> 4);
	time_min->month = ((data[8] & 0xF) * 10) + (data[7] >> 4);
	time_min->year = 2000 + ((data[9] & 0xF) * 10) + (data[8] >> 4);

	time_max->minute = ((data[10] & 0xF) * 10) + (data[9] >> 4);
	time_max->hour = ((data[11] & 0xF) * 10) + (data[10] >> 4);
	time_max->day = ((data[12] & 0xF) * 10) + (data[11] >> 4);
	time_max->month = ((data[13] & 0xF) * 10) + (data[12] >> 4);
	time_max->year = 2000 + ((data[14] & 0xF) * 10) + (data[13] >> 4);

	return;
}


/********************************************************************/
/* windchill_reset
 * Reset min/max windchill with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int windchill_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current windchill into data_value
	address=0x3A0;
	number=2;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min value to current value
		address=0x3A5;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x3AE;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max value to current value
		address=0x3AA;
		number=4;

		if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x3B8;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * rain_1h
 * Read rain last 1 hour, current value only
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_1h(WEATHERSTATION ws2300, double rain_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x4B4;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	return ( ((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	          (data[1] >> 4) * 10 + (data[1] & 0xF) + (data[0] >> 4) / 10.0 +
	          (data[0] & 0xF) / 100.0 ) / rain_conv_factor);
}

/********************************************************************
 * rain_1h_all
 * Read rain last 1 hourand maximum with timestamp
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Output: Rain maximum in rain_max (double)
 *                unit defined by config conversion factor
 *         Timestamps for rain_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_1h_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   double *rain_max,
                   struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x4B4;
	int bytes=11;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*rain_max = ((data[5] >> 4) * 1000 + (data[5] & 0xF) * 100 +
	             (data[4] >> 4) * 10 + (data[4] & 0xF) + (data[3]>>4)/10.0 +
	             (data[3] & 0xF) / 100.0) / rain_conv_factor;

	time_max->minute = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_max->hour = ((data[7] >> 4) * 10) + (data[7] & 0xF);
	time_max->day = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->month = ((data[9] >> 4) * 10) + (data[9] & 0xF);
	time_max->year = 2000 + ((data[10] >> 4) * 10) + (data[10] & 0xF);

	return (((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	         (data[1] >> 4) * 10 + (data[1] & 0xF) + (data[0] >> 4) / 10.0 +
	         (data[0] & 0xF) / 100.0) / rain_conv_factor);
}


/********************************************************************/
/* rain_1h_max_reset
 * Reset max rain 1h with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int rain_1h_max_reset(WEATHERSTATION ws2300)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current rain 1h into data_value
	address=0x4B4;
	number=3;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;
	data_value[4] = data_read[2]&0xF;
	data_value[5] = data_read[2]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	// Set max value to current value
	address=0x4BA;
	number=6;

	if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
		write_error_exit();

	// Set max value timestamp to current time
	address=0x4C0;
	number=10;

	if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
		write_error_exit();

	return 1;
}

/********************************************************************/
/* rain_1h_reset
 * Reset current rain 1h
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int rain_1h_reset(WEATHERSTATION ws2300)
{
	unsigned char data[50];
	unsigned char command[60];	//room for write data also
	int address;
	int number;

	// First overwrite the 1h rain history with zeros
	address=0x479;
	number=30;
	memset(&data, 0, sizeof(data));

	if (write_safe(ws2300, address, number, WRITENIB, data, command) != number)
		write_error_exit();

	// Set value to zero
	address=0x4B4;
	number=6;

	if (write_safe(ws2300, address, number, WRITENIB, data, command) != number)
		write_error_exit();

	return 1;
}


/********************************************************************
 * rain_24h
 * Read rain last 24 hours, current value only
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_24h(WEATHERSTATION ws2300, double rain_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x497;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	return (((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	         (data[1] >> 4) * 10 + (data[1] & 0xF) + (data[0] >> 4) / 10.0 +
	         (data[0] & 0xF) / 100.0) / rain_conv_factor);
}


/********************************************************************
 * rain_24h_all
 * Read rain last 24 hours and maximum with timestamp
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Output: Rain maximum in rain_max (double)
 *                unit defined by config conversion factor
 *         Timestamp for rain_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_24h_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   double *rain_max,
                   struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x497;
	int bytes=11;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*rain_max = ((data[5] >> 4) * 1000 + (data[5] & 0xF) * 100 +
	             (data[4] >> 4) * 10 + (data[4] & 0xF) + (data[3]>>4)/10.0 +
	             (data[3] & 0xF) / 100.0) / rain_conv_factor;

	time_max->minute = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_max->hour = ((data[7] >> 4) * 10) + (data[7] & 0xF);
	time_max->day = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->month = ((data[9] >> 4) * 10) + (data[9] & 0xF);
	time_max->year = 2000 + ((data[10] >> 4) * 10) + (data[10] & 0xF);

	return (((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	         (data[1] >> 4) * 10 + (data[1] & 0xF) + (data[0] >> 4) / 10.0 +
	         (data[0] & 0xF) / 100.0) / rain_conv_factor);
}


/********************************************************************/
/* rain_24h_max_reset
 * Reset max rain 24h with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int rain_24h_max_reset(WEATHERSTATION ws2300)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current rain 24h into data_value
	address=0x497;
	number=3;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value[0] = data_read[0]&0xF;
	data_value[1] = data_read[0]>>4;
	data_value[2] = data_read[1]&0xF;
	data_value[3] = data_read[1]>>4;
	data_value[4] = data_read[2]&0xF;
	data_value[5] = data_read[2]>>4;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	// Set max value to current value
	address=0x49D;
	number=6;

	if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
		write_error_exit();

	// Set max value timestamp to current time
	address=0x4A3;
	number=10;

	if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
		write_error_exit();

	return 1;
}


/********************************************************************/
/* rain_24h_reset
 * Reset current rain 24h
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int rain_24h_reset(WEATHERSTATION ws2300)
{
	unsigned char data[50];
	unsigned char command[60];	//room for write data also
	int address;
	int number;

	// First overwrite the 24h rain history with zeros
	address=0x446;
	number=48;
	memset(&data, 0, sizeof(data));

	if (write_safe(ws2300, address, number, WRITENIB, data, command) != number)
		write_error_exit();

	// Set value to zero
	address=0x497;
	number=6;

	if (write_safe(ws2300, address, number, WRITENIB, data, command) != number)
		write_error_exit();

	return 1;
}


/********************************************************************
 * rain_total
 * Read rain accumulated total, current value only
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_total(WEATHERSTATION ws2300, double rain_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x4D2;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	return (((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	         (data[1] >> 4) * 10 + (data[1] & 0xF) +
	         (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) /
	         rain_conv_factor);
}


/********************************************************************
 * rain_total_all
 * Read rain total accumulated with timestamp
 *
 * Input: Handle to weatherstation
 *        rain_conv_factor controlling convertion to other
 *             units than mm
 *
 * Output: Timestamp for rain total in pointers to
 *                timestamp structures for time_since
 *
 * Returns: rain (double) converted to unit given in config
 *
 ********************************************************************/
double rain_total_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   struct timestamp *time_since)
{
	unsigned char data[20];
	unsigned char command[25];	//room for write data also
	int address=0x4D2;
	int bytes=8;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	time_since->minute = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_since->hour = ((data[4] >> 4) * 10) + (data[4] & 0xF);
	time_since->day = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_since->month = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_since->year = 2000 + ((data[7] >> 4) * 10) + (data[7] & 0xF);

	return (((data[2] >> 4) * 1000 + (data[2] & 0xF) * 100 +
	         (data[1] >> 4) * 10 + (data[1] & 0xF) +
	         (data[0] >> 4) / 10.0 + (data[0] & 0xF) / 100.0) /
	         rain_conv_factor);
}


/********************************************************************/
/* rain_total_reset
 * Reset current total rain
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int rain_total_reset(WEATHERSTATION ws2300)
{
	unsigned char data_read[20];
	unsigned char data_value[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	// Set value to zero
	address=0x4D1;
	number=7;
	memset(&data_value, 0, sizeof(data_value));

	if (write_safe(ws2300, address, number, WRITENIB, data_value, command) != number)
		write_error_exit();

	// Set max value timestamp to current time
	address=0x4D8;
	number=10;

	if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
		write_error_exit();

	return 1;
}


/********************************************************************
 * rel_pressure
 * Read relaive air pressure, current value only
 *
 * Input: Handle to weatherstation
 *        pressure_conv_factor controlling convertion to other
 *             units than hPa
 *
 * Returns: pressure (double) converted to unit given in config
 *
 ********************************************************************/
double rel_pressure(WEATHERSTATION ws2300, double pressure_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x5E2;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();


	return (((data[2] & 0xF) * 1000 + (data[1] >> 4) * 100 +
	         (data[1] & 0xF) * 10 + (data[0] >> 4) +
	         (data[0] & 0xF) / 10.0) / pressure_conv_factor);
}


/********************************************************************
 * rel_pressure_minmax
 * Read relative pressure min/max with timestamps
 *
 * Input: Handle to weatherstation
 *        pressure_conv_factor controlling convertion to other
 *             units than hPa
 *
 * Output: Pressure pres_min and pres_max (double)
 *                unit defined by config conversion factor
 *         Timestamps for pres_min and pres_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: nothing
 *
 ********************************************************************/
void rel_pressure_minmax(WEATHERSTATION ws2300,
                         double pressure_conv_factor,
                         double *pres_min,
                         double *pres_max,
                         struct timestamp *time_min,
                         struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x600;
	int bytes=13;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*pres_min = ((data[2]&0xF)*1000 + (data[1]>>4)*100 +
	            (data[1]&0xF)*10 + (data[0]>>4) +
	            (data[0]&0xF)/10.0) / pressure_conv_factor;

	*pres_max = ((data[12]&0xF)*1000 + (data[11]>>4)*100 +
	            (data[11]&0xF)*10 + (data[10]>>4) +
	            (data[10]&0xF)/10.0) / pressure_conv_factor;

	address=0x61E; //Relative pressure time and date for min/max
	bytes=10;

	if (read_safe(ws2300, address, bytes, data, command)!=bytes)
		read_error_exit();

	time_min->minute = ((data[0] >> 4) * 10) + (data[0] & 0xF);
	time_min->hour = ((data[1] >> 4) * 10) + (data[1] & 0xF);
	time_min->day = ((data[2] >> 4) * 10) + (data[2] & 0xF);
	time_min->month = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_min->year = 2000 + ((data[4] >> 4) * 10) + (data[4] & 0xF);

	time_max->minute = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_max->hour = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_max->day = ((data[7] >> 4) * 10) + (data[7] & 0xF);
	time_max->month = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->year = 2000 + ((data[9] >> 4) * 10) + (data[9] & 0xF);

	return;
}


/********************************************************************
 * abs_pressure
 * Read absolute air pressure, current value only
 *
 * Input: Handle to weatherstation
 *        pressure_conv_factor controlling convertion to other
 *             units than hPa
 *
 * Returns: pressure (double) converted to unit given in config
 *
 ********************************************************************/
double abs_pressure(WEATHERSTATION ws2300, double pressure_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x5D8;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();


	return (((data[2] & 0xF) * 1000 + (data[1] >> 4) * 100 +
	         (data[1] & 0xF) * 10 + (data[0] >> 4) +
	         (data[0] & 0xF) / 10.0) / pressure_conv_factor);
}


/********************************************************************
 * abs_pressure_minmax
 * Read absolute pressure min/max with timestamps
 *
 * Input: Handle to weatherstation
 *        pressure_conv_factor controlling convertion to other
 *             units than hPa
 *
 * Output: Pressure pres_min and pres_max (double)
 *                unit defined by config conversion factor
 *         Timestamps for pres_min and pres_max in pointers to
 *                timestamp structures for time_min and time_max
 *
 * Returns: nothing
 *
 ********************************************************************/
void abs_pressure_minmax(WEATHERSTATION ws2300,
                         double pressure_conv_factor,
                         double *pres_min,
                         double *pres_max,
                         struct timestamp *time_min,
                         struct timestamp *time_max)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x5F6;
	int bytes=13;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();

	*pres_min = ((data[2]&0xF)*1000 + (data[1]>>4)*100 +
	            (data[1]&0xF)*10 + (data[0]>>4) +
	            (data[0]&0xF)/10.0) / pressure_conv_factor;

	*pres_max = ((data[12]&0xF)*1000 + (data[11]>>4)*100 +
	            (data[11]&0xF)*10 + (data[10]>>4) +
	            (data[10]&0xF)/10.0) / pressure_conv_factor;

	address=0x61E; //Relative pressure time and date for min/max
	bytes=10;

	if (read_safe(ws2300, address, bytes, data, command)!=bytes)
		read_error_exit();

	time_min->minute = ((data[0] >> 4) * 10) + (data[0] & 0xF);
	time_min->hour = ((data[1] >> 4) * 10) + (data[1] & 0xF);
	time_min->day = ((data[2] >> 4) * 10) + (data[2] & 0xF);
	time_min->month = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_min->year = 2000 + ((data[4] >> 4) * 10) + (data[4] & 0xF);

	time_max->minute = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_max->hour = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_max->day = ((data[7] >> 4) * 10) + (data[7] & 0xF);
	time_max->month = ((data[8] >> 4) * 10) + (data[8] & 0xF);
	time_max->year = 2000 + ((data[9] >> 4) * 10) + (data[9] & 0xF);

	return;
}



/********************************************************************/
/* pressure_reset
 * Reset min/max pressure (relative and absolute) with timestamps
 *
 * Input: Handle to weatherstation
 *        minmax - char (8 bit integer) that controls if minimum,
 *                 maximum or both are reset
 * Output: None
 *
 * Returns: 1 if success
 *
 ********************************************************************/
int pressure_reset(WEATHERSTATION ws2300, char minmax)
{
	unsigned char data_read[20];
	unsigned char data_value_abs[20];
	unsigned char data_value_rel[20];
	unsigned char data_time[20];
	unsigned char command[25];	//room for write data also
	int address;
	int number;

	// First read current abs/rel pressure into data_value_abs/rel
	address=0x5D8;
	number=8;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_value_abs[0] = data_read[0]&0xF;
	data_value_abs[1] = data_read[0]>>4;
	data_value_abs[2] = data_read[1]&0xF;
	data_value_abs[3] = data_read[1]>>4;
	data_value_abs[4] = data_read[2]&0xF;

	data_value_rel[0] = data_read[5]&0xF;
	data_value_rel[1] = data_read[5]>>4;
	data_value_rel[2] = data_read[6]&0xF;
	data_value_rel[3] = data_read[6]>>4;
	data_value_rel[4] = data_read[7]&0xF;

	// Get current time from station
	address=0x23B;
	number=6;

	if (read_safe(ws2300, address, number, data_read, command) != number)
		read_error_exit();

	data_time[0] = data_read[0]&0xF;
	data_time[1] = data_read[0]>>4;
	data_time[2] = data_read[1]&0xF;
	data_time[3] = data_read[1]>>4;
	data_time[4] = data_read[2]>>4;
	data_time[5] = data_read[3]&0xF;
	data_time[6] = data_read[3]>>4;
	data_time[7] = data_read[4]&0xF;
	data_time[8] = data_read[4]>>4;
	data_time[9] = data_read[5]&0xF;

	if (minmax & RESET_MIN) // minimum
	{
		// Set min abs value to current abs value
		address=0x5F6;
		number=5;

		if (write_safe(ws2300, address, number, WRITENIB, data_value_abs, command) != number)
			write_error_exit();

		// Set min rel value to current rel value
		address=0x600;
		number=5;

		if (write_safe(ws2300, address, number, WRITENIB, data_value_rel, command) != number)
			write_error_exit();

		// Set min value timestamp to current time
		address=0x61E;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	if (minmax & RESET_MAX) // maximum
	{
		// Set max abs value to current abs value
		address=0x60A;
		number=5;

		if (write_safe(ws2300, address, number, WRITENIB, data_value_abs, command) != number)
			write_error_exit();

		// Set max rel value to current rel value
		address=0x614;
		number=5;

		if (write_safe(ws2300, address, number, WRITENIB, data_value_rel, command) != number)
			write_error_exit();

		// Set max value timestamp to current time
		address=0x628;
		number=10;

		if (write_safe(ws2300, address, number, WRITENIB, data_time, command) != number)
			write_error_exit();
	}

	return 1;
}


/********************************************************************
 * pressure_correction
 * Read the correction from absolute to relaive air pressure
 *
 * Input: Handle to weatherstation
 *        pressure_conv_factor controlling convertion to other
 *             units than hPa
 *
 * Returns: pressure (double) converted to unit given in conv factor
 *
 ********************************************************************/
double pressure_correction(WEATHERSTATION ws2300, double pressure_conv_factor)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x5EC;
	int bytes=3;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
		read_error_exit();


	return ((data[2] & 0xF) * 1000 +
	        (data[1] >> 4) * 100 +
	        (data[1] & 0xF) * 10 +
	        (data[0] >> 4) +
	        (data[0] & 0xF) / 10.0 -
	        1000
	       ) / pressure_conv_factor;
}


/********************************************************************
 * tendency_forecast
 * Read Tendency and Forecast
 *
 * Input: Handle to weatherstation
 *
 * Output: tendency - string Steady, Rising or Falling
 *         forecast - string Rainy, Cloudy or Sunny
 *
 * Returns: nothing
 *
 ********************************************************************/
void tendency_forecast(WEATHERSTATION ws2300, char *tendency, char *forecast)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x26B;
	int bytes=1;
	const char *tendency_values[] = { "Steady ", "Rising ", "Falling" };
	const char *forecast_values[] = { "Rainy  ", "Cloudy ", "Sunny  " };

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
	    read_error_exit();

	strcpy(tendency, tendency_values[data[0] >> 4]);
	strcpy(forecast, forecast_values[data[0] & 0xF]);

	return;
}


/********************************************************************
 * read_history_info
 * Read the history information like interval, countdown, time
 * of last record, pointer to last record.
 *
 * Input:  Handle to weatherstation
 *
 * Output: interval - Current interval in minutes (integer)
 *         countdown - Countdown to next measurement (integer)
 *         timelast - Time/Date for last measurement (timestamp struct)
 *         no_records - number of valid records (integer)
 *
 * Returns: interger pointing to last written record. [0x00-0xAE]
 *
 ********************************************************************/
int read_history_info(WEATHERSTATION ws2300, int *interval, int *countdown,
                 struct timestamp *time_last, int *no_records)
{
	unsigned char data[20];
	unsigned char command[25];
	int address=0x6B2;
	int bytes=10;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
	    read_error_exit();

	*interval = (data[1] & 0xF)*256 + data[0] + 1;
	*countdown = data[2]*16 + (data[1] >> 4) + 1;
	time_last->minute = ((data[3] >> 4) * 10) + (data[3] & 0xF);
	time_last->hour = ((data[4] >> 4) * 10) + (data[4] & 0xF);
	time_last->day = ((data[5] >> 4) * 10) + (data[5] & 0xF);
	time_last->month = ((data[6] >> 4) * 10) + (data[6] & 0xF);
	time_last->year = 2000 + ((data[7] >> 4) * 10) + (data[7] & 0xF);
	*no_records = data[9];

	return data[8];

}


/********************************************************************
 * read_history_record
 * Read the history information like interval, countdown, time
 * of last record, pointer to last record.
 *
 * Input:  Handle to weatherstation
 *         config structure with conversion factors
 *         record - record index number to be read [0x00-0xAE]
 *
 * Output: temperature_indoor (double)
 *         temperature_indoor (double)
 *         pressure (double)
 *         humidity_indoor (integer)
 *         humidity_outdoor (integer)
 *         raincount (double)
 *         windspeed (double)
 *         windir_degrees (double)
 *         dewpoint (double) - calculated
 *         windchill (double) - calculated, new post 2001 formula
 *
 * Returns: interger index number pointing to next record
 *
 ********************************************************************/
int read_history_record(WEATHERSTATION ws2300,
                        int record,
                        struct config_type *config,
                        double *temperature_indoor,
                        double *temperature_outdoor,
                        double *pressure,
                        int *humidity_indoor,
                        int *humidity_outdoor,
                        double *raincount,
                        double *windspeed,
                        double *winddir_degrees,
                        double *dewpoint,
                        double *windchill)
{
	unsigned char data[20];
	unsigned char command[25];
	int address;
	int bytes=10;
	long int tempint;
	double A, B, C; // Intermediate values used for dewpoint calculation
	double wind_kmph;

	address = 0x6C6 + record*19;

	if (read_safe(ws2300, address, bytes, data, command) != bytes)
	    read_error_exit();

	tempint = (data[4]<<12) + (data[3]<<4) + (data[2] >> 4);

	*pressure = 1000 + (tempint % 10000)/10.0;

	if (*pressure >= 1502.2)
		*pressure = *pressure - 1000;

	*pressure = *pressure / config->pressure_conv_factor;

	*humidity_indoor = (tempint - (tempint % 10000)) / 10000.0;

	*humidity_outdoor = (data[5]>>4)*10 + (data[5]&0xF);

	*raincount = ((data[7]&0xF)*256 + data[6]) * 0.518 / config->rain_conv_factor;

	*windspeed = (data[8]*16 + (data[7]>>4))/ 10.0; //Need metric for WC

	*winddir_degrees = (data[9]&0xF)*22.5;

	// Temperatures	in Celcius. Cannot convert until WC is calculated
	tempint = ((data[2] & 0xF)<<16) + (data[1]<<8) + data[0];
	*temperature_indoor = (tempint % 1000)/10.0 - 30.0;
	*temperature_outdoor = (tempint - (tempint % 1000))/10000.0 - 30.0;

	// Calculate windchill using new post 2001 USA/Canadian formula
	// Twc = 13.112 + 0.6215*Ta -11.37*V^0.16 + 0.3965*Ta*V^0.16 [Celcius and km/h]

	wind_kmph = 3.6 * *windspeed;
	if (wind_kmph > 4.8)
	{
		*windchill = 13.12 + 0.6215 * *temperature_outdoor -
		             11.37 * pow(wind_kmph, 0.16) +
		             0.3965 * *temperature_outdoor * pow(wind_kmph, 0.16);
	}
	else
	{
		*windchill = *temperature_outdoor;
	}

	// Calculate dewpoint
	// REF http://www.faqs.org/faqs/meteorology/temp-dewpoint/
	A = 17.2694;
	B = (*temperature_outdoor > 0) ? 237.3 : 265.5;
	C = (A * *temperature_outdoor)/(B + *temperature_outdoor) + log((double)*humidity_outdoor/100);
	*dewpoint = B * C / (A - C);

	// Now that WC/DP is calculated we can convert all temperatures and winds
	if (config->temperature_conv)
	{
		*temperature_indoor = *temperature_indoor * 9/5 + 32;
		*temperature_outdoor = *temperature_outdoor * 9/5 + 32;
		*windchill = *windchill * 9/5 + 32;
		*dewpoint = *dewpoint * 9/5 + 32;
	}

	*windspeed *= config->wind_speed_conv_factor;

	return (++record)%0xAF;
}


/********************************************************************
 * light
 * Turns display light on and off
 *
 * Input: control - integer -   0 = off, Anything else = on
 *
 * Returns: Nothing
 *
 ********************************************************************/
void light(WEATHERSTATION ws2300, int control)
{
	unsigned char data;
	unsigned char command[25];  //Data returned is just ignored
	int address=0x016;
	int number=1;
	unsigned char encode_constant;

	data = 0;
	encode_constant = UNSETBIT;
	if (control != 0)
		encode_constant = SETBIT;

	if (write_safe(ws2300, address, number, encode_constant, &data, command)!=number)
		write_error_exit();

	return;
}


/********************************************************************
 * read_error_exit
 * exit location for all calls to read_safe for error exit.
 * includes error reporting.
 *
 ********************************************************************/
void read_error_exit(void)
{
	perror("read_safe() error");
	exit(0);
}

/********************************************************************
 * write_error_exit
 * exit location for all calls to write_safe for error exit.
 * includes error reporting.
 *
 ********************************************************************/
void write_error_exit(void)
{
	perror("write_safe() error");
	exit(0);
}


/********************************************************************
 * get_configuration()
 *
 * read setup parameters from ws2300.conf
 * It searches in this sequence:
 * 1. Path to config file including filename given as parameter
 * 2. ./open2300.conf
 * 3. /usr/local/etc/open2300.conf
 * 4. /etc/open2300.conf
 *
 * See file open2300.conf-dist for the format and option names/values
 *
 * input:    config file name with full path - pointer to string
 *
 * output:   struct config populated with valid settings either
 *           from config file or defaults
 *
 * returns:  0 = OK
 *          -1 = no config file or file open error
 *
 ********************************************************************/
int get_configuration(struct config_type *config, char *path)
{
	FILE *fptr;
	char inputline[1000] = "";
	char token[100] = "";
	char val[100] = "";
	char val2[100] = "";

	// First we set everything to defaults - faster than many if statements
	strcpy(config->serial_device_name, DEFAULT_SERIAL_DEVICE);  // Name of serial device
	strcpy(config->citizen_weather_id, "CW0000");               // Citizen Weather ID
	strcpy(config->citizen_weather_latitude, "5540.12N");       // latitude default Glostrup, DK
	strcpy(config->citizen_weather_longitude, "01224.60E");     // longitude default, Glostrup, DK
	strcpy(config->aprs_host[0].name, "aprswest.net");         // host1 name
	config->aprs_host[0].port = 23;                            // host1 port
	strcpy(config->aprs_host[1].name, "indiana.aprs2.net");    // host2 name
	config->aprs_host[1].port = 23;                            // host2 port
	config->num_hosts = 2;                                     // default number of defined hosts
	strcpy(config->weather_underground_id, "WUID");             // Weather Underground ID
	strcpy(config->weather_underground_password, "WUPassword"); // Weather Underground Password
	strcpy(config->timezone, "1");                              // Timezone, default CET
	config->wind_speed_conv_factor = 1.0;                   // Speed dimention, m/s is default
	config->temperature_conv = 0;                           // Temperature in Celcius
	config->rain_conv_factor = 1.0;                         // Rain in mm
	config->pressure_conv_factor = 1.0;                     // Pressure in hPa (same as millibar)
	strcpy(config->mysql_host, "localhost");            // localhost, IP or domainname of server
	strcpy(config->mysql_user, "open2300");             // MySQL database user name
	strcpy(config->mysql_passwd, "mysql2300");          // Password for MySQL database user
	strcpy(config->mysql_database, "open2300");         // Name of MySQL database
	config->mysql_port = 0;                             // MySQL port. 0 means default port/socket
	strcpy(config->pgsql_connect, "hostaddr='127.0.0.1'dbname='open2300'user='postgres'"); // connection string
	strcpy(config->pgsql_table, "weather");             // PgSQL table name
	strcpy(config->pgsql_station, "open2300");          // Unique station id

	// open the config file

	fptr = NULL;
	if (path != NULL)
		fptr = fopen(path, "r");       //first try the parameter given
	if (fptr == NULL)                  //then try default search
	{
		if ((fptr = fopen("open2300.conf", "r")) == NULL)
		{
			if ((fptr = fopen("/usr/local/etc/open2300.conf", "r")) == NULL)
			{
				if ((fptr = fopen("/etc/open2300.conf", "r")) == NULL)
				{
					//Give up and use defaults
					return(-1);
				}
			}
		}
	}

	while (fscanf(fptr, "%[^\n]\n", inputline) != EOF)
	{
		sscanf(inputline, "%[^= \t]%*[ \t=]%s%*[, \t]%s%*[^\n]", token, val, val2);

		if (token[0] == '#')	// comment
			continue;

		if ((strcmp(token,"SERIAL_DEVICE")==0) && (strlen(val) != 0))
		{
			strcpy(config->serial_device_name,val);
			continue;
		}

		if ((strcmp(token,"CITIZEN_WEATHER_ID")==0) && (strlen(val) != 0))
		{
			strcpy(config->citizen_weather_id, val);
			continue;
		}

		if ((strcmp(token,"CITIZEN_WEATHER_LATITUDE")==0) && (strlen(val)!=0))
		{
			strcpy(config->citizen_weather_latitude, val);
			continue;
		}

		if ((strcmp(token,"CITIZEN_WEATHER_LONGITUDE")==0) && (strlen(val)!=0))
		{
			strcpy(config->citizen_weather_longitude, val);
			continue;
		}

		if ((strcmp(token,"APRS_SERVER")==0) && (strlen(val)!=0) && (strlen(val2)!=0))
		{
			if ( config->num_hosts >= MAX_APRS_HOSTS)
				continue;           // ignore host definitions over the defined max
			strcpy(config->aprs_host[config->num_hosts].name, val);
			config->aprs_host[config->num_hosts].port = atoi(val2);
			config->num_hosts++;    // increment for next
			continue;
		}

		if ((strcmp(token,"WEATHER_UNDERGROUND_ID")==0) && (strlen(val)!=0))
		{
			strcpy(config->weather_underground_id, val);
			continue;
		}

		if ((strcmp(token,"WEATHER_UNDERGROUND_PASSWORD")==0)&&(strlen(val)!=0))
		{
			strcpy(config->weather_underground_password, val);
			continue;
		}

		if ((strcmp(token,"TIMEZONE")==0) && (strlen(val) != 0))
		{
			strcpy(config->timezone, val);
			continue;
		}

		if ((strcmp(token,"WIND_SPEED") == 0) && (strlen(val) != 0))
		{
			if (strcmp(val, "m/s") == 0)
				config->wind_speed_conv_factor = METERS_PER_SECOND;
			else if (strcmp(val, "km/h") == 0)
				config->wind_speed_conv_factor = KILOMETERS_PER_HOUR;
			else if (strcmp(val, "MPH") == 0)
				config->wind_speed_conv_factor = MILES_PER_HOUR;
			continue; //else default remains
		}

		if ((strcmp(token,"TEMPERATURE") == 0) && (strlen(val) != 0))
		{
			if (strcmp(val, "C") == 0)
				config->temperature_conv = CELCIUS;
			else if (strcmp(val, "F") == 0)
				config->temperature_conv = FAHRENHEIT;
			continue; //else default remains
		}

		if ((strcmp(token,"RAIN") == 0) && (strlen(val) != 0))
		{
			if (strcmp(val, "mm") == 0)
				config->rain_conv_factor = MILLIMETERS;
			else if (strcmp(val, "IN") == 0)
				config->rain_conv_factor = INCHES;
			continue; //else default remains
		}

		if ((strcmp(token,"PRESSURE") == 0) && (strlen(val) != 0))
		{
			if ( (strcmp(val, "hPa") == 0) || (strcmp(val, "mb") == 0))
				config->pressure_conv_factor = HECTOPASCAL;
			else if (strcmp(val, "INHG") == 0)
				config->pressure_conv_factor = INCHES_HG;
			continue; //else default remains
		}

		if ((strcmp(token,"MYSQL_HOST") == 0) && (strlen(val) != 0))
		{
			strcpy(config->mysql_host, val);
			continue;
		}

		if ( (strcmp(token,"MYSQL_USERNAME") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->mysql_user, val);
			continue;
		}

		if ( (strcmp(token,"MYSQL_PASSWORD") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->mysql_passwd, val);
			continue;
		}

		if ( (strcmp(token,"MYSQL_DATABASE") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->mysql_database, val);
			continue;
		}

		if ( (strcmp(token,"MYSQL_PORT") == 0) && (strlen(val) != 0) )
		{
			config->mysql_port = atoi(val);
			continue;
		}

		if ( (strcmp(token,"PGSQL_CONNECT") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->pgsql_connect, val);
			continue;
		}

		if ( (strcmp(token,"PGSQL_TABLE") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->pgsql_table, val);
			continue;
		}

		if ( (strcmp(token,"PGSQL_STATION") == 0) && (strlen(val) != 0) )
		{
			strcpy(config->pgsql_station, val);
			continue;
		}

	}

	return (0);
}


 /********************************************************************
 * address_encoder converts an 16 bit address to the form needed
 * by the WS-2300 when sending commands.
 *
 * Input:   address_in (interger - 16 bit)
 *
 * Output:  address_out - Pointer to an unsigned character array.
 *          3 bytes, not zero terminated.
 *
 * Returns: Nothing.
 *
 ********************************************************************/
void address_encoder(int address_in, unsigned char *address_out)
{
	int i = 0;
	int adrbytes = 4;
	unsigned char nibble;

	for (i = 0; i < adrbytes; i++)
	{
		nibble = (address_in >> (4 * (3 - i))) & 0x0F;
		address_out[i] = (unsigned char) (0x82 + (nibble * 4));
	}

	return;
}


/********************************************************************
 * data_encoder converts up to 15 data bytes to the form needed
 * by the WS-2300 when sending write commands.
 *
 * Input:   number - number of databytes (integer)
 *          encode_constant - unsigned char
 *                            0x12=set bit, 0x32=unset bit, 0x42=write nibble
 *          data_in - char array with up to 15 hex values
 *
 * Output:  address_out - Pointer to an unsigned character array.
 *
 * Returns: Nothing.
 *
 ********************************************************************/
void data_encoder(int number, unsigned char encode_constant,
                  unsigned char *data_in, unsigned char *data_out)
{
	int i = 0;

	for (i = 0; i < number; i++)
	{
		data_out[i] = (unsigned char) (encode_constant + (data_in[i] * 4));
	}

	return;
}


/********************************************************************
 * numberof_encoder converts the number of bytes we want to read
 * to the form needed by the WS-2300 when sending commands.
 *
 * Input:   number interger, max value 15
 *
 * Returns: unsigned char which is the coded number of bytes
 *
 ********************************************************************/
unsigned char numberof_encoder(int number)
{
	int coded_number;

	coded_number = (unsigned char) (0xC2 + number * 4);
	if (coded_number > 0xfe)
		coded_number = 0xfe;

	return coded_number;
}


/********************************************************************
 * command_check0123 calculates the checksum for the first 4
 * commands sent to WS2300.
 *
 * Input:   pointer to char to check
 *          sequence of command - i.e. 0, 1, 2 or 3.
 *
 * Returns: calculated checksum as unsigned char
 *
 ********************************************************************/
unsigned char command_check0123(unsigned char *command, int sequence)
{
	int response;

	response = sequence * 16 + ((*command) - 0x82) / 4;

	return (unsigned char) response;
}


/********************************************************************
 * command_check4 calculates the checksum for the last command
 * which is sent just before data is received from WS2300
 *
 * Input: number of bytes requested
 *
 * Returns: expected response from requesting number of bytes
 *
 ********************************************************************/
unsigned char command_check4(int number)
{
	int response;

	response = 0x30 + number;

	return response;
}


/********************************************************************
 * data_checksum calculates the checksum for the data bytes received
 * from the WS2300
 *
 * Input:   pointer to array of data to check
 *          number of bytes in array
 *
 * Returns: calculated checksum as unsigned char
 *
 ********************************************************************/
unsigned char data_checksum(unsigned char *data, int number)
{
	int checksum = 0;
	int i;

	for (i = 0; i < number; i++)
	{
		checksum += data[i];
	}

	checksum &= 0xFF;

	return (unsigned char) checksum;
}


/********************************************************************
 * initialize resets WS2300 to cold start (rewind and start over)
 *
 * Input:   device number of the already open serial port
 *
 * Returns: 0 if fail, 1 if success
 *
 ********************************************************************/
int initialize(WEATHERSTATION ws2300)
{
	unsigned char command = 0x06;
	unsigned char answer;

	write_device(ws2300, &command, 1);

	if (read_device(ws2300, &answer, 1) != 1)
		return 0;

	write_device(ws2300, &command, 1);
	write_device(ws2300, &command, 1);

	if (read_device(ws2300, &answer, 1) != 1)
		return 0;

	write_device(ws2300, &command, 1);

	if (read_device(ws2300, &answer, 1) != 1)
		return 0;

	write_device(ws2300, &command, 1);

	if (read_device(ws2300, &answer, 1) != 1)
		return 0;

	if (answer != 2)
		return 0;

	return 1;
}


/********************************************************************
 * read_data reads data from the WS2300 based on a given address,
 * number of data read, and a an already open serial port
 *
 * Inputs:  serdevice - device number of the already open serial port
 *          address (interger - 16 bit)
 *          number - number of bytes to read, max value 15
 *
 * Output:  readdata - pointer to an array of chars containing
 *                     the just read data, not zero terminated
 *          commanddata - pointer to an array of chars containing
 *                     the commands that were sent to the station
 *
 * Returns: number of bytes read, -1 if failed
 *
 ********************************************************************/
int read_data(WEATHERSTATION ws2300, int address, int number,
			  unsigned char *readdata, unsigned char *commanddata)
{

	unsigned char answer;
	int i;

	// First 4 bytes are populated with converted address range 0000-13B0
	address_encoder(address, commanddata);
	// Last populate the 5th byte with the converted number of bytes
	commanddata[4] = numberof_encoder(number);

	for (i = 0; i < 4; i++)
	{
		if (write_device(ws2300, commanddata + i, 1) != 1)
			return -1;
		if (read_device(ws2300, &answer, 1) != 1)
			return -1;
		if (answer != command_check0123(commanddata + i, i))
			return -1;
	}

	//Send the final command that asks for 'number' of bytes, check answer
	if (write_device(ws2300, commanddata + 4, 1) != 1)
		return -1;
	if (read_device(ws2300, &answer, 1) != 1)
		return -1;
	if (answer != command_check4(number))
		return -1;

	//Read the data bytes
	for (i = 0; i < number; i++)
	{
		if (read_device(ws2300, readdata + i, 1) != 1)
			return -1;
	}

	//Read and verify checksum
	if (read_device(ws2300, &answer, 1) != 1)
		return -1;
	if (answer != data_checksum(readdata, number))
		return -1;

	return i;

}


/********************************************************************
 * write_data writes data to the WS2300.
 * It can both write nibbles and set/unset bits
 *
 * Inputs:      ws2300 - device number of the already open serial port
 *              address (interger - 16 bit)
 *              number - number of nibbles to be written/changed
 *                       must 1 for bit modes (SETBIT and UNSETBIT)
 *                       max 80 for nibble mode (WRITENIB)
 *              encode_constant - unsigned char
 *                                (SETBIT, UNSETBIT or WRITENIB)
 *              writedata - pointer to an array of chars containing
 *                          data to write, not zero terminated
 *                          data must be in hex - one digit per byte
 *                          If bit mode value must be 0-3 and only
 *                          the first byte can be used.
 *
 * Output:      commanddata - pointer to an array of chars containing
 *                            the commands that were sent to the station
 *
 * Returns:     number of bytes written, -1 if failed
 *
 ********************************************************************/
int write_data(WEATHERSTATION ws2300, int address, int number,
			   unsigned char encode_constant, unsigned char *writedata,
			   unsigned char *commanddata)
{
    unsigned char answer;
    unsigned char encoded_data[80];
    int i = 0;
    unsigned char ack_constant = WRITEACK;

    if (encode_constant == SETBIT)
    {
        ack_constant = SETACK;
    }
    else if (encode_constant == UNSETBIT)
    {
        ack_constant = UNSETACK;
    }

    // First 4 bytes are populated with converted address range 0000-13XX
    address_encoder(address, commanddata);

    // populate the encoded_data array
    data_encoder(number, encode_constant, writedata, encoded_data);

    //Write the 4 address bytes
    for (i = 0; i < 4; i++)
    {
        if (write_device(ws2300, commanddata + i, 1) != 1)
            return -1;

        if (read_device(ws2300, &answer, 1) != 1)
            return -1;

	if (answer != command_check0123(commanddata + i, i))
            return -1;
    }

	//Write the data nibbles or set/unset the bits
	for (i = 0; i < number; i++)
	{
		if (write_device(ws2300, encoded_data + i, 1) != 1)
			return -1;
		if (read_device(ws2300, &answer, 1) != 1)
			return -1;
		if (answer != (writedata[i] + ack_constant))
			return -1;
		commanddata[i + 4] = encoded_data[i];
	}

	return i;
}


/********************************************************************
 * read_safe Read data, retry until success or maxretries
 * Reads data from the WS2300 based on a given address,
 * number of data read, and a an already open serial port
 * Uses the read_data function and has same interface
 *
 * Inputs:  ws2300 - device number of the already open serial port
 *          address (interger - 16 bit)
 *          number - number of bytes to read, max value 15
 *
 * Output:  readdata - pointer to an array of chars containing
 *                     the just read data, not zero terminated
 *          commanddata - pointer to an array of chars containing
 *                     the commands that were sent to the station
 *
 * Returns: number of bytes read, -1 if failed
 *
 ********************************************************************/
int read_safe(WEATHERSTATION ws2300, int address, int number,
			  unsigned char *readdata, unsigned char *commanddata)
{
	int j;

	for (j = 0; j < MAXRETRIES; j++)
	{
		reset_06(ws2300);

		// Read the data. If expected number of bytes read break out of loop.
		if (read_data(ws2300, address, number, readdata, commanddata)==number)
		{
			break;
		}
	}

	// If we have tried MAXRETRIES times to read we expect not to
	// have valid data
	if (j == MAXRETRIES)
	{
		return -1;
	}

	return number;
}


/********************************************************************
 * write_safe Write data, retry until success or maxretries
 * Writes data to the WS2300 based on a given address,
 * number of data to write, and a an already open serial port
 * Uses the write_data function and has same interface
 *
 * Inputs:      serdevice - device number of the already open serial port
 *              address (interger - 16 bit)
 *              number - number of nibbles to be written/changed
 *                       must 1 for bit modes (SETBIT and UNSETBIT)
 *                       unlimited for nibble mode (WRITENIB)
 *              encode_constant - unsigned char
 *                               (SETBIT, UNSETBIT or WRITENIB)
 *              writedata - pointer to an array of chars containing
 *                          data to write, not zero terminated
 *                          data must be in hex - one digit per byte
 *                          If bit mode value must be 0-3 and only
 *                          the first byte can be used.
 *
 * Output:      commanddata - pointer to an array of chars containing
 *                            the commands that were sent to the station
 *
 * Returns: number of bytes written, -1 if failed
 *
 ********************************************************************/
int write_safe(WEATHERSTATION ws2300, int address, int number,
               unsigned char encode_constant, unsigned char *writedata,
               unsigned char *commanddata)
{
	int j;

	for (j = 0; j < MAXRETRIES; j++)
	{
		// sprintf( logBuffer,"Iteration = %d\n",j); // debug
		reset_06(ws2300);

		// Read the data. If expected number of bytes read break out of loop.
		if (write_data(ws2300, address, number, encode_constant, writedata,
		    commanddata)==number)
		{
			break;
		}
	}

	// If we have tried MAXRETRIES times to read we expect not to
	// have valid data
	if (j == MAXRETRIES)
	{
		return -1;
	}

	return number;
}

//--------------------------------------------------------------------
// My WS2308 is sending the occasional weird WindChill values (278*)
// When investigating, I see that the reported windchills don't match
// the values coming from the NOAA charts.
// Here's an attempt to rewrite the algorithm used by the device
double  nws_2001_windchill (double oTempF, double oWindSpeedMPH)
{
    //
    //  Website says:
    //      WindChill (*F) = 35.74 + 0.6215T - 35.75( V^0.16) + 0.4275T(V^0.16)
    //  Where T = Ait temp *F, V = WindSpeed (MPH)
    double  V016;
    double  a, b, c;
    double  windchill;

    //
    //  But windspeed must be above 3MPH
    if (oWindSpeedMPH <= 3.0)
        return oTempF;
    //
    //  And only good for Temps < 50
    if (oTempF >= 50.0)
        return oTempF;

    // sprintf( logBuffer, ">>> DEBUG %f, %f\n", oTempF, oWindSpeedMPH );

    V016 = pow( oWindSpeedMPH, 0.16 );
    a = (0.6215 * oTempF);
    b = (35.75 * V016);
    c = (0.4275 * oTempF) * V016;

    windchill = 35.74 + a - b + c;

    //sprintf( logBuffer, ">>> V016 %f\n", V016 );
    //sprintf( logBuffer, ">>> DEBUG A %f, B %f, C %f, wc %f\n", a,b,c,windchill );

    return windchill;
}


// ---------------------------------------------------------------------
// Lets add in some routines we can use to sanity check values...
double  d_in_range (double value, double minV, double maxV, double defaultV )
{
    if (value < minV || value > maxV)
        return defaultV;
    return value;
}   // d_in_range

//  Indoor Temp validity checking
double  check_iTemp (double iTemp)
{
    return d_in_range( iTemp, 10, 120, 0 );
}
double  check_iHumidity (double dVal)
{
    return d_in_range( dVal, 1, 100, 0 );
}
double  check_oTemp (double dVal)
{
    return d_in_range( dVal, -60, 150, 0 );
}
double  check_oHumidity (double dVal)
{
    return d_in_range( dVal, 1, 100, 0 );
}
double  check_oWindSpeed (double dVal)
{
    return d_in_range( dVal, 0, 150, 0 );
}


