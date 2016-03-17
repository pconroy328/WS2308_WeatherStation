/* 
 * File:   ws2300.h
 * Author: pconroy
 *
 * Created on August 23, 2010, 9:08 AM
 */

#ifndef _WS2300_H
#define	_WS2300_H

#ifdef	__cplusplus
extern "C" {
#endif


//
//  rw2300.h and linxux2300.h
//
#ifndef  FALSE
# define FALSE 0
# define TRUE  (!FALSE)
#endif

#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>


/*
 0019 	0 	Alarm set flags: -/Storm warning/-/Time
001A 	0 	Alarm set flags: Pressure Hi/Pressure Ho/-/-
001B 	0 	Alarm set flags: T out Hi/T out Lo/T in Hi/T in Lo
001C 	0 	Alarm set flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
001D 	0 	Alarm set flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
001E 	0 	Alarm set flags: -/-/Rain 1h/Rain 24h
001F 	0 	Alarm set flags: -/Wind dir/Wind speed Hi/Wind speed Lo
 *
0020 	4 	Alarm active flags: -/Alarm Icon/-/Time?
0021 	0 	Alarm active flags: Pressure Hi/Pressure Ho/-/-
0022 	0 	Alarm active flags: T out Hi/T out Lo/T in Hi/T in Lo
0023 	0 	Alarm active flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
0024 	0 	Alarm active flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
0025 	0 	Alarm active flags: -/-/Rain 1h/Rain 24h
0026 	0 	Alarm active flags: -/Wind dir/Wind speed Hi/Wind speed Lo
 *
 * why is this a repeat of 0x21?
0027 	0 	Alarm active flags: Pressure Hi/Pressure Ho/-/-
0028 	0 	Alarm active flags: T out Hi/T out Lo/T in Hi/T in Lo
0029 	0 	Alarm active flags: Dew Hi/Dew Lo/Windchill Hi/Windchill Lo
002A 	0 	Alarm active flags: Hum in Hi/Hum in Lo/Hum out Hi/Hum out Lo
002B 	0 	Alarm active flags: -/-/Rain 1h/Rain 24h
002C 	0 	Alarm active flags: -/-/Wind speed Hi/Wind speed Lo
 */

#define ALARM_ID_UNUSED                     0
#define ALARM_ID_STORM_WARNING              1
#define ALARM_ID_TIME                       2
#define ALARM_ID_PRESSURE_HI                3
#define ALARM_ID_PRESSURE_HO                4
#define ALARM_ID_OTEMP_HI                   5
#define ALARM_ID_OTEMP_LO                   6
#define ALARM_ID_ITEMP_HI                   7
#define ALARM_ID_ITEMP_LO                   8
#define ALARM_ID_DEWTEMP_HI                 9
#define ALARM_ID_DEWTEMP_LO                 10
#define ALARM_ID_WINDCHILL_HI               11
#define ALARM_ID_WINDCHILL_LO               12
#define ALARM_ID_IHUMID_HI                  13
#define ALARM_ID_IHUMID_LO                  14
#define ALARM_ID_OHUMID_HI                  15
#define ALARM_ID_OHUMID_LO                  16
#define ALARM_ID_RAIN1H_HI                  17
#define ALARM_ID_RAIN24H_LO                 18
#define ALARM_ID_WINDDIR                    19
#define ALARM_ID_WINSPEED_HI                20
#define ALARM_ID_WINDPSEED_LO               21

#define ALARM_ID_PRESSURE_HI2               23
#define ALARM_ID_PRESSURE_HO2               24
#define ALARM_ID_OTEMP_HI2                  25
#define ALARM_ID_OTEMP_LO2                  26
#define ALARM_ID_ITEMP_HI2                  27
#define ALARM_ID_ITEMP_LO2                  28
#define ALARM_ID_DEWTEMP_HI2                29
#define ALARM_ID_DEWTEMP_LO2                30
#define ALARM_ID_WINDCHILL_HI2              31
#define ALARM_ID_WINDCHILL_LO2              32
#define ALARM_ID_IHUMID_HI2                 33
#define ALARM_ID_IHUMID_LO2                 34
#define ALARM_ID_OHUMID_HI2                 35
#define ALARM_ID_OHUMID_LO2                 36
#define ALARM_ID_RAIN1H_HI2                 37
#define ALARM_ID_RAIN24H_LO2                38
#define ALARM_ID_WINDDIR2                   39
#define ALARM_ID_WINSPEED_HI2               40
#define ALARM_ID_WINDPSEED_LO2              41






//
// The temperature routines take a second parameter - 0 for Celcius and non-zero for Farenheit
#define FARENHEIT           1

// Wind speeds are in meters/sec - convert to MPH
#define  ms_TO_MPH      2.237

// Pressure readings come back in hPa - convert to in Hg
#define mmHg_TO_inHG    25.4

// Rain data is in mm
#define mm_TO_in        25.4



#define BAUDRATE B2400
#define DEFAULT_SERIAL_DEVICE "/dev/ttyS0"
typedef int WEATHERSTATION;

#define VERSION             1


#define MAXRETRIES          50
#define MAXWINDRETRIES      20
#define WRITENIB            0x42
#define SETBIT              0x12
#define UNSETBIT            0x32
#define WRITEACK            0x10
#define SETACK              0x04
#define UNSETACK            0x0C
#define RESET_MIN           0x01
#define RESET_MAX           0x02


#define METERS_PER_SECOND   1.0
#define KILOMETERS_PER_HOUR 3.6
#define MILES_PER_HOUR      2.23693629
#define CELCIUS             0
#define FAHRENHEIT          1
#define MILLIMETERS         1
#define INCHES              25.4
#define HECTOPASCAL         1.0
#define MILLIBARS           1.0
#define INCHES_HG           33.8638864


/* ONLY EDIT THESE IF WEATHER UNDERGROUND CHANGES URL */
#define WEATHER_UNDERGROUND_BASEURL "weatherstation.wunderground.com"
#define WEATHER_UNDERGROUND_PATH "/weatherstation/updateweatherstation.php"

#define WEATHER_UNDERGROUND_SOFTWARETYPE   "open2300%20v"

#define MAX_APRS_HOSTS	6

typedef struct {
	char name[50];
	int port;
} hostdata;

struct config_type
{
	char   serial_device_name[50];
	char   citizen_weather_id[30];
	char   citizen_weather_latitude[20];
	char   citizen_weather_longitude[20];
	hostdata aprs_host[MAX_APRS_HOSTS]; // max 6 possible aprs hosts 1 primary and 5 alternate
	int    num_hosts;					// total defined hosts
	char   weather_underground_id[30];
	char   weather_underground_password[50];
	char   timezone[6];                //not integer because of half hour time zones
	double wind_speed_conv_factor;     //from m/s to km/h or miles/hour
	int    temperature_conv;           //0=Celcius, 1=Fahrenheit
	double rain_conv_factor;           //from mm to inch
	double pressure_conv_factor;       //from hPa (=millibar) to mmHg
	char   mysql_host[50];             //Either localhost, IP address or hostname
	char   mysql_user[25];
	char   mysql_passwd[25];
	char   mysql_database[30];
	int    mysql_port;                 //0 works for local connection
	char   pgsql_connect[128];
	char   pgsql_table[25];
	char   pgsql_station[25];
};

struct timestamp
{
	int minute;
	int hour;
	int day;
	int month;
	int year;
};



extern  int findWS2300 ( char *deviceName, int devNameLength, int sleepInterval, int maxDevNum );


/* Weather data functions */

extern  double temperature_indoor(WEATHERSTATION ws2300, int temperature_conv);

extern  void temperature_indoor_minmax(WEATHERSTATION ws2300,
                               int temperature_conv,
                               double *temp_min,
                               double *temp_max,
                               struct timestamp *time_min,
                               struct timestamp *time_max);

extern  int temperature_indoor_reset(WEATHERSTATION ws2300, char minmax);

extern  double temperature_outdoor(WEATHERSTATION ws2300, int temperature_conv);

extern  void temperature_outdoor_minmax(WEATHERSTATION ws2300,
                                int temperature_conv,
                                double *temp_min,
                                double *temp_max,
                                struct timestamp *time_min,
                                struct timestamp *time_max);

extern  int temperature_outdoor_reset(WEATHERSTATION ws2300, char minmax);

extern  double dewpoint(WEATHERSTATION ws2300, int temperature_conv);

extern  void dewpoint_minmax(WEATHERSTATION ws2300,
					int temperature_conv,
					double *dp_min,
					double *dp_max,
					struct timestamp *time_min,
					struct timestamp *time_max);

extern  int dewpoint_reset(WEATHERSTATION ws2300, char minmax);

extern  int humidity_indoor(WEATHERSTATION ws2300);

extern  int humidity_indoor_all(WEATHERSTATION ws2300,
					int *hum_min,
					int *hum_max,
					struct timestamp *time_min,
					struct timestamp *time_max);

extern  int humidity_indoor_reset(WEATHERSTATION ws2300, char minmax);

extern  int humidity_outdoor(WEATHERSTATION ws2300);

extern  int humidity_outdoor_all(WEATHERSTATION ws2300,
					int *hum_min,
					int *hum_max,
					struct timestamp *time_min,
					struct timestamp *time_max);

extern  int humidity_outdoor_reset(WEATHERSTATION ws2300, char minmax);

extern  double wind_current(WEATHERSTATION ws2300,
                    double wind_speed_conv_factor,
                    double *winddir);

extern  double wind_all(WEATHERSTATION ws2300,
                double wind_speed_conv_factor,
                int *winddir_index,
                double *winddir);

extern  double wind_minmax(WEATHERSTATION ws2300,
                 double wind_speed_conv_factor,
                 double *wind_min,
                 double *wind_max,
                 struct timestamp *time_min,
                 struct timestamp *time_max);

extern  int wind_reset(WEATHERSTATION ws2300, char minmax);

extern  double windchill(WEATHERSTATION ws2300, int temperature_conv);

extern  void windchill_minmax(WEATHERSTATION ws2300,
                      int temperature_conv,
                      double *wc_min,
                      double *wc_max,
                      struct timestamp *time_min,
                      struct timestamp *time_max);

extern  int windchill_reset(WEATHERSTATION ws2300, char minmax);

extern  double rain_1h(WEATHERSTATION ws2300, double rain_conv_factor);

extern  double rain_1h_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   double *rain_max,
                   struct timestamp *time_max);

extern  double rain_24h(WEATHERSTATION ws2300, double rain_conv_factor);

extern  int rain_1h_max_reset(WEATHERSTATION ws2300);

extern  int rain_1h_reset(WEATHERSTATION ws2300);

extern  double rain_24h_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   double *rain_max,
                   struct timestamp *time_max);

extern  int rain_24h_max_reset(WEATHERSTATION ws2300);

extern  int rain_24h_reset(WEATHERSTATION ws2300);

extern  double rain_total(WEATHERSTATION ws2300, double rain_conv_factor);

extern  double rain_total_all(WEATHERSTATION ws2300,
                   double rain_conv_factor,
                   struct timestamp *time_max);

extern  int rain_total_reset(WEATHERSTATION ws2300);

extern  double rel_pressure(WEATHERSTATION ws2300, double pressure_conv_factor);

extern  void rel_pressure_minmax(WEATHERSTATION ws2300,
                         double pressure_conv_factor,
                         double *pres_min,
                         double *pres_max,
                         struct timestamp *time_min,
                         struct timestamp *time_max);

extern  double abs_pressure(WEATHERSTATION ws2300, double pressure_conv_factor);

extern  void abs_pressure_minmax(WEATHERSTATION ws2300,
                         double pressure_conv_factor,
                         double *pres_min,
                         double *pres_max,
                         struct timestamp *time_min,
                         struct timestamp *time_max);

extern  int pressure_reset(WEATHERSTATION ws2300, char minmax);

extern  double pressure_correction(WEATHERSTATION ws2300, double pressure_conv_factor);

extern  void tendency_forecast(WEATHERSTATION ws2300, char *tendency, char *forecast);

extern  int read_history_info(WEATHERSTATION ws2300, int *interval, int *countdown,
                      struct timestamp *time_last, int *no_records);

extern  int read_history_record(WEATHERSTATION ws2300,
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
                        double *windchill);

extern  void light(WEATHERSTATION ws2300, int control);


/* Generic functions */

extern  void read_error_exit(void);

extern  void write_error_exit(void);

//extern  void print_usage(void);

extern  int get_configuration(struct config_type *, char *path);

extern  WEATHERSTATION open_weatherstation(char *device);

extern  void close_weatherstation(WEATHERSTATION ws);

static  void address_encoder(int address_in, unsigned char *address_out);

static  void data_encoder(int number, unsigned char encode_constant,
				  unsigned char *data_in, unsigned char *data_out);

static  unsigned char numberof_encoder(int number);

static  unsigned char command_check0123(unsigned char *command, int sequence);

static  unsigned char command_check4(int number);

static  unsigned char data_checksum(unsigned char *data, int number);

static  int initialize(WEATHERSTATION ws2300);

extern int reset_06(WEATHERSTATION ws2300);

static  int read_data(WEATHERSTATION ws2300, int address, int number,
			  unsigned char *readdata, unsigned char *commanddata);

static  int write_data(WEATHERSTATION ws2300, int address, int number,
			   unsigned char encode_constant, unsigned char *writedata,
			   unsigned char *commanddata);

static  int read_safe(WEATHERSTATION ws2300, int address, int number,
			  unsigned char *readdata, unsigned char *commanddata);

static  int write_safe(WEATHERSTATION ws2300, int address, int number,
			   unsigned char encode_constant, unsigned char *writedata,
			   unsigned char *commanddata);



extern  double  nws_2001_windchill (double oTempF, double oWindSpeedMPH);
extern  double  check_iTemp (double iTemp);
extern  double  check_iHumidity (double iVal);
double  check_oTemp (double iVal);
double  check_oHumidity (double iVal);
double  check_oWindSpeed (double iVal);


/* Platform dependent functions */
static  int read_device(WEATHERSTATION serdevice, unsigned char *buffer, int size);
static  int write_device(WEATHERSTATION serdevice, unsigned char *buffer, int size);
static  void sleep_short(int milliseconds);
static  void sleep_long(int seconds);
//int http_request_url(char *urlline);
//int citizen_weather_send(struct config_type *config, char *datastring);


/* PMC Additions */
extern  char *read_date (WEATHERSTATION ws2300, char *date);
extern  char *read_time (WEATHERSTATION ws2300, char *date);



#ifdef	__cplusplus
}
#endif

#endif	/* _WS2300_H */

