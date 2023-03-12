#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/database.o \
	${OBJECTDIR}/inifile.o \
	${OBJECTDIR}/logger.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/mqtt.o \
	${OBJECTDIR}/stats.o \
	${OBJECTDIR}/ws2300.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L\"${`mysql_config\ --libs`}\" -lm /usr/local/lib/libinifiler.a -lmariadbclient /usr/local/lib/libmovingaverage_lib.a /usr/lib/x86_64-linux-gnu/libmosquitto.so

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation: /usr/local/lib/libinifiler.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation: /usr/local/lib/libmovingaverage_lib.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation: /usr/lib/x86_64-linux-gnu/libmosquitto.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/database.o: database.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/database.o database.c

${OBJECTDIR}/inifile.o: inifile.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/inifile.o inifile.c

${OBJECTDIR}/logger.o: logger.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/logger.o logger.c

${OBJECTDIR}/main.o: main.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.c

${OBJECTDIR}/mqtt.o: mqtt.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/mqtt.o mqtt.c

${OBJECTDIR}/stats.o: stats.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/stats.o stats.c

${OBJECTDIR}/ws2300.o: ws2300.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -D_POSIX_SOURCE -D__USE_XOPEN -std=c99 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ws2300.o ws2300.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} -r ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libmosquitto.so
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ws2308_weatherstation

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
