#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-CANPanel-18F25k80.mk)" "nbproject/Makefile-local-CANPanel-18F25k80.mk"
include nbproject/Makefile-local-CANPanel-18F25k80.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=CANPanel-18F25k80
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=canpanel.c io-canpanel.c max6951.c buttonscan.c hwsettings.c panelFLiM.c paneltest.c panelEvents.c ../cbuslib/can18.c ../cbuslib/cbus.c ../cbuslib/FliM.c ../cbuslib/events.c ../cbuslib/StatusLeds.c ../cbuslib/Bootloader.asm ../cbuslib/ticktime.c ../cbuslib/romops.c ../cbuslib/c018.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/canpanel.o ${OBJECTDIR}/io-canpanel.o ${OBJECTDIR}/max6951.o ${OBJECTDIR}/buttonscan.o ${OBJECTDIR}/hwsettings.o ${OBJECTDIR}/panelFLiM.o ${OBJECTDIR}/paneltest.o ${OBJECTDIR}/panelEvents.o ${OBJECTDIR}/_ext/1094713127/can18.o ${OBJECTDIR}/_ext/1094713127/cbus.o ${OBJECTDIR}/_ext/1094713127/FliM.o ${OBJECTDIR}/_ext/1094713127/events.o ${OBJECTDIR}/_ext/1094713127/StatusLeds.o ${OBJECTDIR}/_ext/1094713127/Bootloader.o ${OBJECTDIR}/_ext/1094713127/ticktime.o ${OBJECTDIR}/_ext/1094713127/romops.o ${OBJECTDIR}/_ext/1094713127/c018.o
POSSIBLE_DEPFILES=${OBJECTDIR}/canpanel.o.d ${OBJECTDIR}/io-canpanel.o.d ${OBJECTDIR}/max6951.o.d ${OBJECTDIR}/buttonscan.o.d ${OBJECTDIR}/hwsettings.o.d ${OBJECTDIR}/panelFLiM.o.d ${OBJECTDIR}/paneltest.o.d ${OBJECTDIR}/panelEvents.o.d ${OBJECTDIR}/_ext/1094713127/can18.o.d ${OBJECTDIR}/_ext/1094713127/cbus.o.d ${OBJECTDIR}/_ext/1094713127/FliM.o.d ${OBJECTDIR}/_ext/1094713127/events.o.d ${OBJECTDIR}/_ext/1094713127/StatusLeds.o.d ${OBJECTDIR}/_ext/1094713127/Bootloader.o.d ${OBJECTDIR}/_ext/1094713127/ticktime.o.d ${OBJECTDIR}/_ext/1094713127/romops.o.d ${OBJECTDIR}/_ext/1094713127/c018.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/canpanel.o ${OBJECTDIR}/io-canpanel.o ${OBJECTDIR}/max6951.o ${OBJECTDIR}/buttonscan.o ${OBJECTDIR}/hwsettings.o ${OBJECTDIR}/panelFLiM.o ${OBJECTDIR}/paneltest.o ${OBJECTDIR}/panelEvents.o ${OBJECTDIR}/_ext/1094713127/can18.o ${OBJECTDIR}/_ext/1094713127/cbus.o ${OBJECTDIR}/_ext/1094713127/FliM.o ${OBJECTDIR}/_ext/1094713127/events.o ${OBJECTDIR}/_ext/1094713127/StatusLeds.o ${OBJECTDIR}/_ext/1094713127/Bootloader.o ${OBJECTDIR}/_ext/1094713127/ticktime.o ${OBJECTDIR}/_ext/1094713127/romops.o ${OBJECTDIR}/_ext/1094713127/c018.o

# Source Files
SOURCEFILES=canpanel.c io-canpanel.c max6951.c buttonscan.c hwsettings.c panelFLiM.c paneltest.c panelEvents.c ../cbuslib/can18.c ../cbuslib/cbus.c ../cbuslib/FliM.c ../cbuslib/events.c ../cbuslib/StatusLeds.c ../cbuslib/Bootloader.asm ../cbuslib/ticktime.c ../cbuslib/romops.c ../cbuslib/c018.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-CANPanel-18F25k80.mk dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F25K80
MP_PROCESSOR_OPTION_LD=18f25k80
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1094713127/Bootloader.o: ../cbuslib/Bootloader.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/Bootloader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/Bootloader.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/_ext/1094713127/Bootloader.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG -d__MPLAB_DEBUGGER_PK3=1 -q -p$(MP_PROCESSOR_OPTION)  -l\"${OBJECTDIR}/_ext/1094713127/Bootloader.lst\" -e\"${OBJECTDIR}/_ext/1094713127/Bootloader.err\" $(ASM_OPTIONS)  -o\"${OBJECTDIR}/_ext/1094713127/Bootloader.o\" \"../cbuslib/Bootloader.asm\"
	@${DEP_GEN} -d "${OBJECTDIR}/_ext/1094713127/Bootloader.o"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/Bootloader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/_ext/1094713127/Bootloader.o: ../cbuslib/Bootloader.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/Bootloader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/Bootloader.o 
	@${FIXDEPS} dummy.d -e "${OBJECTDIR}/_ext/1094713127/Bootloader.err" $(SILENT) -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION)  -l\"${OBJECTDIR}/_ext/1094713127/Bootloader.lst\" -e\"${OBJECTDIR}/_ext/1094713127/Bootloader.err\" $(ASM_OPTIONS)  -o\"${OBJECTDIR}/_ext/1094713127/Bootloader.o\" \"../cbuslib/Bootloader.asm\"
	@${DEP_GEN} -d "${OBJECTDIR}/_ext/1094713127/Bootloader.o"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/Bootloader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/canpanel.o: canpanel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/canpanel.o.d 
	@${RM} ${OBJECTDIR}/canpanel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/canpanel.o   canpanel.c 
	@${DEP_GEN} -d ${OBJECTDIR}/canpanel.o 
	@${FIXDEPS} "${OBJECTDIR}/canpanel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/io-canpanel.o: io-canpanel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/io-canpanel.o.d 
	@${RM} ${OBJECTDIR}/io-canpanel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/io-canpanel.o   io-canpanel.c 
	@${DEP_GEN} -d ${OBJECTDIR}/io-canpanel.o 
	@${FIXDEPS} "${OBJECTDIR}/io-canpanel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/max6951.o: max6951.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/max6951.o.d 
	@${RM} ${OBJECTDIR}/max6951.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/max6951.o   max6951.c 
	@${DEP_GEN} -d ${OBJECTDIR}/max6951.o 
	@${FIXDEPS} "${OBJECTDIR}/max6951.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/buttonscan.o: buttonscan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/buttonscan.o.d 
	@${RM} ${OBJECTDIR}/buttonscan.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/buttonscan.o   buttonscan.c 
	@${DEP_GEN} -d ${OBJECTDIR}/buttonscan.o 
	@${FIXDEPS} "${OBJECTDIR}/buttonscan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/hwsettings.o: hwsettings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hwsettings.o.d 
	@${RM} ${OBJECTDIR}/hwsettings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hwsettings.o   hwsettings.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hwsettings.o 
	@${FIXDEPS} "${OBJECTDIR}/hwsettings.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/panelFLiM.o: panelFLiM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/panelFLiM.o.d 
	@${RM} ${OBJECTDIR}/panelFLiM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/panelFLiM.o   panelFLiM.c 
	@${DEP_GEN} -d ${OBJECTDIR}/panelFLiM.o 
	@${FIXDEPS} "${OBJECTDIR}/panelFLiM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/paneltest.o: paneltest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/paneltest.o.d 
	@${RM} ${OBJECTDIR}/paneltest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/paneltest.o   paneltest.c 
	@${DEP_GEN} -d ${OBJECTDIR}/paneltest.o 
	@${FIXDEPS} "${OBJECTDIR}/paneltest.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/panelEvents.o: panelEvents.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/panelEvents.o.d 
	@${RM} ${OBJECTDIR}/panelEvents.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/panelEvents.o   panelEvents.c 
	@${DEP_GEN} -d ${OBJECTDIR}/panelEvents.o 
	@${FIXDEPS} "${OBJECTDIR}/panelEvents.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/can18.o: ../cbuslib/can18.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/can18.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/can18.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/can18.o   ../cbuslib/can18.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/can18.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/can18.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/cbus.o: ../cbuslib/cbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/cbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/cbus.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/cbus.o   ../cbuslib/cbus.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/cbus.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/cbus.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/FliM.o: ../cbuslib/FliM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/FliM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/FliM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/FliM.o   ../cbuslib/FliM.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/FliM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/FliM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/events.o: ../cbuslib/events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/events.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/events.o   ../cbuslib/events.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/StatusLeds.o: ../cbuslib/StatusLeds.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/StatusLeds.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/StatusLeds.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/StatusLeds.o   ../cbuslib/StatusLeds.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/StatusLeds.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/StatusLeds.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/ticktime.o: ../cbuslib/ticktime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/ticktime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/ticktime.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/ticktime.o   ../cbuslib/ticktime.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/ticktime.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/ticktime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/romops.o: ../cbuslib/romops.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/romops.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/romops.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/romops.o   ../cbuslib/romops.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/romops.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/romops.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/c018.o: ../cbuslib/c018.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/c018.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/c018.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/c018.o   ../cbuslib/c018.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/c018.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/c018.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/canpanel.o: canpanel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/canpanel.o.d 
	@${RM} ${OBJECTDIR}/canpanel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/canpanel.o   canpanel.c 
	@${DEP_GEN} -d ${OBJECTDIR}/canpanel.o 
	@${FIXDEPS} "${OBJECTDIR}/canpanel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/io-canpanel.o: io-canpanel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/io-canpanel.o.d 
	@${RM} ${OBJECTDIR}/io-canpanel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/io-canpanel.o   io-canpanel.c 
	@${DEP_GEN} -d ${OBJECTDIR}/io-canpanel.o 
	@${FIXDEPS} "${OBJECTDIR}/io-canpanel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/max6951.o: max6951.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/max6951.o.d 
	@${RM} ${OBJECTDIR}/max6951.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/max6951.o   max6951.c 
	@${DEP_GEN} -d ${OBJECTDIR}/max6951.o 
	@${FIXDEPS} "${OBJECTDIR}/max6951.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/buttonscan.o: buttonscan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/buttonscan.o.d 
	@${RM} ${OBJECTDIR}/buttonscan.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/buttonscan.o   buttonscan.c 
	@${DEP_GEN} -d ${OBJECTDIR}/buttonscan.o 
	@${FIXDEPS} "${OBJECTDIR}/buttonscan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/hwsettings.o: hwsettings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hwsettings.o.d 
	@${RM} ${OBJECTDIR}/hwsettings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hwsettings.o   hwsettings.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hwsettings.o 
	@${FIXDEPS} "${OBJECTDIR}/hwsettings.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/panelFLiM.o: panelFLiM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/panelFLiM.o.d 
	@${RM} ${OBJECTDIR}/panelFLiM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/panelFLiM.o   panelFLiM.c 
	@${DEP_GEN} -d ${OBJECTDIR}/panelFLiM.o 
	@${FIXDEPS} "${OBJECTDIR}/panelFLiM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/paneltest.o: paneltest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/paneltest.o.d 
	@${RM} ${OBJECTDIR}/paneltest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/paneltest.o   paneltest.c 
	@${DEP_GEN} -d ${OBJECTDIR}/paneltest.o 
	@${FIXDEPS} "${OBJECTDIR}/paneltest.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/panelEvents.o: panelEvents.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/panelEvents.o.d 
	@${RM} ${OBJECTDIR}/panelEvents.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/panelEvents.o   panelEvents.c 
	@${DEP_GEN} -d ${OBJECTDIR}/panelEvents.o 
	@${FIXDEPS} "${OBJECTDIR}/panelEvents.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/can18.o: ../cbuslib/can18.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/can18.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/can18.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/can18.o   ../cbuslib/can18.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/can18.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/can18.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/cbus.o: ../cbuslib/cbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/cbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/cbus.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/cbus.o   ../cbuslib/cbus.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/cbus.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/cbus.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/FliM.o: ../cbuslib/FliM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/FliM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/FliM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/FliM.o   ../cbuslib/FliM.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/FliM.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/FliM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/events.o: ../cbuslib/events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/events.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/events.o   ../cbuslib/events.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/StatusLeds.o: ../cbuslib/StatusLeds.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/StatusLeds.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/StatusLeds.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/StatusLeds.o   ../cbuslib/StatusLeds.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/StatusLeds.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/StatusLeds.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/ticktime.o: ../cbuslib/ticktime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/ticktime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/ticktime.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/ticktime.o   ../cbuslib/ticktime.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/ticktime.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/ticktime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/romops.o: ../cbuslib/romops.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/romops.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/romops.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/romops.o   ../cbuslib/romops.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/romops.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/romops.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/1094713127/c018.o: ../cbuslib/c018.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1094713127" 
	@${RM} ${OBJECTDIR}/_ext/1094713127/c018.o.d 
	@${RM} ${OBJECTDIR}/_ext/1094713127/c018.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DCANPanel -I"../cbuslib" -I"../canpanel" -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1094713127/c018.o   ../cbuslib/c018.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1094713127/c018.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1094713127/c018.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    canpanel25k80.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "canpanel25k80.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   canpanel25k80.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "canpanel25k80.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w  -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/canpanel.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/CANPanel-18F25k80
	${RM} -r dist/CANPanel-18F25k80

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
