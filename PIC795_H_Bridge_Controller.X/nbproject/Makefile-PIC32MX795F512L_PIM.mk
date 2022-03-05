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
ifeq "$(wildcard nbproject/Makefile-local-PIC32MX795F512L_PIM.mk)" "nbproject/Makefile-local-PIC32MX795F512L_PIM.mk"
include nbproject/Makefile-local-PIC32MX795F512L_PIM.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC32MX795F512L_PIM
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../main.c ../DelayPIC32.c ../I2C_4BUS_EEPROM_PIC32.c "../PCA9685 _PIC32.c" ../CRCmodbus.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" ${OBJECTDIR}/_ext/1472/CRCmodbus.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o ${OBJECTDIR}/_ext/1472/CRCmodbus.o

# Source Files
SOURCEFILES=../main.c ../DelayPIC32.c ../I2C_4BUS_EEPROM_PIC32.c ../PCA9685 _PIC32.c ../CRCmodbus.c



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
	${MAKE}  -f nbproject/Makefile-PIC32MX795F512L_PIM.mk dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/84fefaf19645c90e34eea1405d1957918250143.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/6786f6c08cb62147f1aea328f4a35eb922b02dde.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/befa1634b2b637fded189246dc2e49f219e4eb46.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o: ../PCA9685\ _PIC32.c  .generated_files/481cccf3eed2869df843ec9e93cab58c3f5a28bb.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o".d 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" -o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" "../PCA9685 _PIC32.c"    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/fc624029517d6eec74e66e3891b27972424eb575.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
else
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/6c72392f444811e68af79d5db84b430901ddc3f.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/8d496aaf6d90167be25c1dc026712a6d863dd47d.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/fb90b1507784757931f4518dab5ca602d858d31d.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/PCA9685\ _PIC32.o: ../PCA9685\ _PIC32.c  .generated_files/3091f1133cbf3bff58e6d082cdcee1e41a16fc0b.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o".d 
	@${RM} "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o.d" -o "${OBJECTDIR}/_ext/1472/PCA9685 _PIC32.o" "../PCA9685 _PIC32.c"    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/978c4dd157117029ca20bab73cf6c54ffd5373fb.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../Microchip/Include" -DPIC32MX795F512L_PIM -I".." -I"../../Microchip/Include" -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC32MX795F512L_PIM=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PIC795_H_Bridge_Controller.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PIC32MX795F512L_PIM
	${RM} -r dist/PIC32MX795F512L_PIM

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
