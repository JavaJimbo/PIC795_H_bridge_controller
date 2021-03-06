/**********************************************************************************
 * PROJECT: PIC795 H BRIDGE CONTROLLER * 
 * Works with MIDI_ProTools
 * 
 * Previously named PIC795_MD13S_CONTROLLER
 * main.c
 * 
 * Compiled for PIC32MX795 XC32 compiler version 1.30 
 * For Robotnik MD13S Controller Board Rev 1.0
 * Adapted from Robotnik_Brain_Board
 * 
 * 8-13-20:     Set up IO for new board
 * 9-13-20:     Created ADC10_ManualInit()
 * 9-14-20:     Fixed ADC bug. Polling works great now.
 * 9-15-20:     Got all five PWMs working, also four SW inputs with interrupt on change,
 *              also four encoder counters and encoder direction inputs.
 *              Created USE_PID mode and tested PID with one motor.
 *              Fixed a few PID bugs - works better, and doesn't overrun and go wild.
 *              Added previousPosition, quadCurrent, quadPrevious to PIDtype struct.
 *              Enabled PID for all five motors.
 * 9-16-20:     Got DMA working with RS485 Rx at 981600 baud.
 * 9-20-20:     Got RC Feather servo board working.
 * 9-21-20: 
 * 9-22-20:     Got encoders working nicely in Destination mode - enter positions by text.
 * 9-23-20:     Eliminated Velocity mode for now.
 * 9-29-20:     Testing RC servo motors with Feather board. Baudrate: 115200
 * 10-2-20:     Four RC servos working with pots controlling Feather board.
 * 10-3-20:     Added Velocity variable to limit speed.
 * 10-17-20:    Got CRC working. 
 * 10-18-20:    Constant velocity mode is working nicely using Encoder inputs on ServoCity 26:1 motors.
 *              Got forward/backward/right/left working with joystick on XBEE input.
 *              Re-enabled POT mode for servo #0.
 * 10-19-20:    Made adjustments to joystick to work with large motors 
 *              and loads for complementary wheelchair wheels.
 * 10-20-20:    Added Watchdog timer. Cleaned up POT control mode, fixed bug. Motor can now go well beyond 360 degrees.
 * 11-14-20: 
 * 12-6-20:     Modified to receive MIDI device data on XBEE at 57600 baud and control first servo with it.
 *              So now it works with MIDI_Device recording and playing back one servo motor.
 * 12-16-20:    Works now with multiple servos recording and playing back on ProTools.
 * 12-17-20:    Cleaned up PID for POT mode. Was very glitchy for AM 218 motors.
 * 12-18-20:    Both RC and PID servos work great.
 *              Use REMOTE at startup. Enabled Feather board RC servo control.
 *              RCservoPos[i] = (short)(ADresult[i+6]/4) + 22;
 * 12-20-20:    XBEE baud rate didn't work well at 115200. Set back to 57600.
 * 12-27-20:    Added errorCode to detect transmission errors on incoming data.
 * 12-31-20:    No big changes at this point. Works beautifully with MIDI_Device
 *              running in either RS232 or USB mode.
 * 05-18-21;    
 * 06-12-21:    Removed RC servo, DMA, RS485, everything except POT control. Fixed SPI1 bug.
 *              Added USE_BRAIN_BOARD feature to make code compatible with existing REV 2 Brain Board.
 *              Tested POT mode with MIDI and XBEE - Works great.
 * 06-13-21:    Added Encoder test to JOG feature.
 * 06-20-21:    Added #define REV2  for new REV 2 ROBOTNIK MD13S CONTROLLER board
 * 10-26-21:    Verified with MIDI_ProTools. Number of servo motors = 2. 
 *              Added Featherboard RC servos also. 
 *              Enabled LOCAL mode pots to work with both types of servos
 *              Cleaned up, got rid of extraneous files.
 *              Verified it works with Brain Board.
 * 2-21-22:     Modified menu
 * 2-25-22:     Cytron wheel diameter: 5"
 *              Circumference: 15.7" = inches per revolution
 *              400 counts per rev >> 0.03927" per count
 *              Max speed: 87 RPM >> 114 feet per minute or 22.8 inches per second or 1.45 RPS or 581 counts per second
 *              Interrupts: every 4 ms or 250 per second
 *              Max counts per interrupt: 2.327 counts per interrupt
 *              Acceleration: 0.001 counts per interrupt
 *              Max Cytron velocity: AD counts / 1023 * 2.327 counts per interrupt
 *              
 * 2-26-22:     ServoCity 26:1 motor
 *              Max counts per minute:  57732 @ 12V
 *              Counts per revolution:   182
 *              Max RPM:  317 Max RPS: 5.29 Max counts per interrupt: 3.8488
 * 
 *              CONTINUOUS and DESTINATION motor modes working great.
 * 2-27-22:     Cleaned up and improved CONT and DEST modes.
 *              Derivative correction is now same as for POT mode.
 *              K constants could probably use tweaking.
 * 3-01-22:     Got CONTINUOUS and DESTINATION working with REMOTE commands.
 * 3-02-22:     Added braking to DESTINATION mode so destination can be continuously updated.
 *              This version has NOT been tested with the BRAIN BOARD
 * 3-03-22:     Further tweaks to DESTINATION mode.
 * 3-04-22:     Created separate DestinationEncoderControl()) then imported ContinuousEncoderControl()
 * 3-05-22:     Debugged new routines in REMOTE mode. 
 *              ServoControl() works with both analog and digital rotary position sensors.
 *              Changed project name to PIC795 H BRIDGE CONTYROLLER
 * 3-8-22:      Added SW5->D13(CN19), SW6->D4(CN13), MOTOR 2 DIR moved to D12
 *              SW3->B3, SW4->B4
 * 3-12-22:     Recompiled.
 * 3-19-22:     Added TxNumber - Works well with new RFM69HCW Feather M0 boards.
 * 4-02-22:     Changes made for REV 3 BOARD. TODO: Pushbutton input on Brain Board
 * 4-03-22:     
 * 4-04-22:     Added PID[ServoID].EncoderPolarityFlipped
 * 
 *              EEPROM MAPPING FOR 24LC256
 *              32768 bytes max memory
 *              Store everything in 64 byte blocks
 *              Block #0 is device config:
 *              0. Number of motors
 *              1, Number of RC servos
 * 
 *              0. Servo alias: 0-255
 *              1. OP MODE
 *              2. MOTOR POLARITY
 *              3. ENCODER POLARITY
 *              4. 
 * 4-06-22:     Memory map dump works nicely 
 *              ProcessMemoryMap()
 *              1. Using strtok(), pull CR terminated strings and attach to arrMemoryString[]
 *              2. Using strtok(), get each comma terminated substring
 *                  For each substring, do:
 *                  a) Step through spaces to first non space character
 *                  b) Using ispunct(), check for non characters
 *                      if one is encountered:
 *                          i. Compare against list
 *                          2. Process item
 *                          3. Convert numeric values
 *                          4. Modify PID record accordingly
 * 4-07-22:     
 * 4-08-22:    
 * 4-10-22:     Lots of debugging. Storing and fetching PID from EEprom works well now.
 * 4-11-22:     Added Upload feature. Minor cleanup. 
 *              Xbee mode works on robot with steering and drive motors.
 ***********************************************************************************/
#define HALT_COMMAND 0xC0
#define RUN_COMMAND 0xD0

#define BRAKE_DISTANCE 90
#define DESTINATION_MULTIPLIER 1
#define DESTINATION_THRESHOLD (DESTINATION_MULTIPLIER * 20)
#define MINIMUM_VELOCITY 10
#define MAX_CYTRON_VELOCITY 2.327
#define MAX_26_VELOCITY 3.8488
#define INTERRUPTS_PER_SECOND 250.0

#include <xc.h>
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "Compiler.h"
#include "I2C_4BUS_EEPROM_PIC32.h"
#include "Defs.h"

#define _SUPPRESS_PLIB_WARNING

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF            // Watchdog Timer Enabled
#pragma config WDTPS =    PS1024        // watchdog timeout 1.024 seconds
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#define false FALSE
#define true TRUE

#define EEPROM_DUMP_ADDRESS_OFFSET 1024
#define TEST_OUT LATCbits.LATC1
#define HOST_UART_TIMEOUT 200000

#define FILTERSIZE 16

struct PIDtype
{
    char* alias;
    long  error[FILTERSIZE];
    short errIndex;
    long  sumError;    
    float kP;
    float kI;
    float kD;
    long PWMoffset;
    long PWMvalue;                
    float CommandPos;       
    long CurrentPos;
    unsigned short PreviousQuad;
    long  RampUpStartPosition;
    long  RampDownStartPosition;    
    float Velocity;
    float TargetVelocity;        
    float MaxVelocity;
    float MinVelocity;    
    long PIDCommand;    
    long Destination;
    //long BrakeDistance;    
    BYTE saturation;    
    BYTE reset;    
    BYTE OpMode;    
    BYTE Halted;
    BYTE RemoteDataValid;
    BYTE RampState;
    BYTE Direction;
    BYTE EncoderPolarityFlipped;
    BYTE MotorPolarityFlipped;
    // BYTE DestinationState;
};


typedef union
{
	unsigned char b[2];
	unsigned short integer;
} MConvertType;

// #define USE_BRAIN_BOARD  1 // COMMENT OUT to use CYTRON CONTROLLER BOARD
// #define REV2
#define REV3

enum {
    NO_COMMAND = 0,
    CTRL_A,
    CTRL_B,
    CTRL_C,
    CTRL_D,
    CTRL_E,
    CTRL_F,
    CTRL_G,
    CTRL_H,
    CTRL_I,
    CTRL_J,
    CTRL_K,
    CTRL_L,
    CTRL_M,
    CTRL_N,
    CTRL_O,
    CTRL_P,
    CTRL_Q,
    CTRL_R,
    CTRL_S,
    CTRL_T,
    CTRL_U,
    CTRL_V,
    CTRL_W,
    CTRL_X,
    CTRL_Y,
    CTRL_Z
}; 

enum
{
    FORWARD = 0,
    REVERSE
};

// OP MODES
enum 
{
    NOT_USED = 0,
    SERVO_POT_MODE,     // 1
    SERVO_ENCODER_MODE, // 2
    DESTINATION_MODE,  //3
    CONTINUOUS_MODE,    //4
    MAXMODE
};

// RAMP STATES
enum
{
    RAMP_START = 0,
    RAMP_UP,
    RAMP_RUN,
    RAMP_DOWN,
    RAMP_HALT
};

// RUN STATES
enum 
{
    HALTED=0,
    RUN
};

// COMMAND MODES
enum 
{   
    NONE = 0,
    LOCAL,
    JOG,
    REMOTE    
};

enum{
    NO_ERROR = 0,
    CRC_ERROR,
    STX_ERROR,
    ETX_ERROR,
    OVERRUN_ERROR
};

#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023

#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86

#define USE_PID
 
#define PWM_MAX 4000

#define SUCCESS 0
#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board 


#define	STX '>'
#define	DLE '/'
#define	ETX '\r'


#define NUM_LOCAL_POTS 4

#ifdef USE_BRAIN_BOARD
    #define RS485_ENABLE LATBbits.LATB0
    #define PWM_DISABLE LATFbits.LATF1
    #define PWM_SPI_CS LATDbits.LATD7
#endif


// #define USE_FEATHER_BOARD


#ifdef REV1
    #define MAXPOTS 10
#else
    #define MAXPOTS 8
#endif

// For 26:1 motors
#define PWM_OFFSET 100
#define KP 8.0
#define KI 0.01 
#define KD 10.0 

#define DEST_PWM_OFFSET 200
#define DEST_KP 200
#define DEST_KI 10
#define DEST_KD 0

#define CONST_PWM_OFFSET 100
#define CONST_KP 8.0
#define CONST_KI 0.04
#define CONST_KD 0

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define MAXSERVOS 64  // Was 256

#define NUMMOTORS 4

#ifdef REV3
    #define SD_CS LATEbits.LATE4
    #define SD_DETECT PORTGbits.RG9
    #define XBEE_SLEEP LATAbits.LATA3
#endif

#ifdef USE_BRAIN_BOARD
    #define FORWARD 1
    #define REVERSE 0
#else
    #define FORWARD 0
    #define REVERSE 1
#endif

#ifdef USE_BRAIN_BOARD
    #define PWM1 OC2RS
    #define PWM2 OC3RS
    #define PWM3 OC4RS
    #define PWM4 OC5RS
#else
    #define PWM1 OC1RS
    #define PWM2 OC2RS
    #define PWM3 OC3RS
    #define PWM4 OC4RS
    #ifdef REV1
       #define PWM5 OC5RS 
    #endif
#endif

#define EncoderOne TMR1
#define EncoderTwo TMR5
#define EncoderThree TMR3
#define EncoderFour TMR4

#define EncoderOneDir PORTEbits.RE9
#define EncoderTwoDir PORTEbits.RE5
#define EncoderThreeDir PORTEbits.RE7
#define EncoderFourDir PORTEbits.RE8

#define MOTOR_DIR1 LATGbits.LATG12
#define MOTOR_DIR2 LATDbits.LATD12 // $$$$ Change to RD12
#define MOTOR_DIR3 LATDbits.LATD11
#define MOTOR_DIR4 LATAbits.LATA5

#ifndef USE_BRAIN_BOARD
    #ifndef REV2
        #define MOTOR_DIR5 LATDbits.LATD10
    #endif
#endif

#ifdef USE_BRAIN_BOARD
    #define LED LATEbits.LATE6
#endif

#ifdef REV3
    #define LED  LATAbits.LATA0
#endif

#ifdef REV1
    #define LED LATDbits.LATD9
#endif

#ifdef REV2
    #define LED LATDbits.LATD9
#endif

#ifdef REV3
#define SWA PORTBbits.RB0
#define SWB PORTBbits.RB1
#define SWC PORTBbits.RB2
#define SWD PORTBbits.RB4
#define SWE PORTDbits.RD13
#define SWF PORTDbits.RD4
#endif

#ifdef REV2
#define SW1 PORTBbits.RB0
#define SW2 PORTBbits.RB1
#define SW3 PORTBbits.RB2
#define SW4 PORTCbits.RC13
#endif

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR


#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66
#define MAXNUM 16

#define MAXBUFFER (EEBLOCKSIZE * 128)

#define CONSTANT_FORWARD 9999
#define CONSTANT_REVERSE -999



short MemRxIndex = 0;
unsigned long milliseconds;
BYTE flagRemoteTimeout = false;
unsigned short offSet;
BYTE NUMbuffer[MAXNUM + 1];
BYTE HOSTRxBuffer[MAXBUFFER+1];
BYTE MemRxBuffer[MAXBUFFER+1];
BYTE MemMapBuffer[MAXBUFFER+1];
BYTE HOSTRxBufferFull = false;
BYTE MemBlockFlag = false;
short HostRxIndex = 0;
BYTE HOSTTxBuffer[MAXBUFFER+1]; 

BYTE PIDDisplayMode = false;
BYTE XBEEDisplayMode = false;

BYTE XBEERxBuffer[MAXBUFFER+1];
BYTE XBEEPacket[MAXBUFFER+1];
long XBEEData[MAXBUFFER+1];
float ForwardReverse = 0, RightLeft = 0;
short XBEEPacketLength;
int timeout = 0;
extern BYTE CheckCRC (BYTE *ptrPacketData, short packetLength);
int ADC10_ManualInit(void);
static void InitializeSystem(void);
extern BYTE CheckCRC (BYTE *ptrRxModbus, short RxModbusLength);
unsigned short decodePacket(BYTE *ptrInPacket, BYTE *ptrData, BYTE *errorCode);
void ResetPID(struct PIDtype *PID, BYTE resetPositionFlag, BYTE setDefaults);
void ResetServoPID (short ServoID, struct PIDtype *PID, BYTE resetPositionFlag, BYTE setDefaults);
void InitPID(struct PIDtype *PID);
int ServoControl(short ServoID, struct PIDtype *PID);
int DestinationEncoderControl(short ServoID, struct PIDtype *PID);
int ContinuousEncoderControl(short ServoID, struct PIDtype *PID);
void GetEncoderPosition(short ServoID, struct PIDtype *PID);
BYTE processPacketData(BYTE *ptrPacket, short *numData, long *ptrData, BYTE *TxNumber, BYTE *command, BYTE *subCommand, BYTE *errorCode);
int ConfigurePID (struct PIDtype *PID, BYTE **ptrConfigString, int numStrings, BYTE DiagnosticsEnabled);
int ProcessCommandString (struct PIDtype *PID, BYTE *ptrString, BYTE DiagnosticsEnabled, BYTE *CommandMode, BYTE *RunState, BYTE UseTestCommands);
int GenerateMemMap(struct PIDtype *PID, BYTE *ptrMapBuffer);
int FetchMemMapFromEEprom (unsigned char *MemMapBuffer);
int FillPIDConfigList(BYTE *ptrBuffer, BYTE **ptrMemMap, int MaxStrings, BYTE displayList);
int StoreMapInEEprom (unsigned char *ptrMemBuffer, BYTE enableDiagnostics);
void DisplayServoConfig(struct PIDtype *PID, int ServoID);
void ConfigPIDFromEEprom (BYTE *ptrMemMapBuffer, BYTE **ptrPIDConfigList, struct PIDtype *PID);

unsigned short ADresult[MAXPOTS];
BYTE intFlag = false;
BYTE startPacket = false;
long XBEEtimeout = 0, HOSTtimeout = 0;
long ActualXBEEBaudRate, ActualHOSTBaudRate;
BYTE SWAstste = 1, SWBstste = 1, SWCstste = 1, SWDstste = 1, SWEstste = 1, SWFstste = 1;
BYTE SWChangeFlag = false;
BYTE MemMapRxMode = false;

int NumBytesRead = 0;

#define MAX_MEMORY_STRINGS 32
BYTE *PIDConfigList[MAX_MEMORY_STRINGS];
BYTE ControlCommand = 0;

short ServoSelect = 0;

int main(void) 
{
    struct PIDtype PID[NUMMOTORS];
    short i = 0;
    long PWMvalue = 0;            
    BYTE temp, remoteCommand, subCommand;
    short numDataIntegers;    
    float tempCommand;
    long  ServoCommandPosition;   
    BYTE CommandMode = REMOTE;    
    BYTE runState = RUN;
    BYTE errorCode = NO_ERROR;
    BYTE TxNumber = 0;
    short LEDcounter = 0;
    short displayCounter = 0;    
    short EEBlockCounter = 0;
    short EEaddress = 0;
    int numBytesStored = 0, numStrings = 0;
                
#ifdef USE_FEATHER_BOARD    
    #define NUM_RC_SERVOS 3
    short RCservoPos[NUM_RC_SERVOS];
    short PreviousRCservoPos[NUM_RC_SERVOS];    
    short RCServoID = 0;   
    unsigned char FeatherEnabled = FALSE;        
    
    for (i = 0; i < NUM_RC_SERVOS; i++) RCservoPos[i] = PreviousRCservoPos[i] = 0;
#endif                       
    
    DelayMs(400);           
    InitializeSystem();
    initI2C(EEBUS);
    
#ifdef USE_BRAIN_BOARD                
    PWM1 = PWM2 = PWM3 = PWM4 = 0;
#else
    #ifdef REV1
        PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
    #else 
        PWM1 = PWM2 = PWM3 = PWM4 = 0;
    #endif
#endif   
       
        
#ifdef USE_BRAIN_BOARD    
    printf("\r\rSTART BRAIN BOARD #0");
#else
    printf("\r\rSTART CYTRON BOARD #1");    
#endif
    printf("\rHEAP: 10000 BYTES, BUFFER: %d BYTES, OFFSET: %d", MAXBUFFER, EEPROM_DUMP_ADDRESS_OFFSET);    
    
    InitPID(PID);
    
    
    for (i = 0; i < MAXBUFFER; i++)MemMapBuffer[i] = '\0';
    
    printf("\rLOADING EEPROM PID CONFIG..."); 
    
    ConfigPIDFromEEprom (MemMapBuffer, PIDConfigList, PID);
    
#ifdef USE_FEATHER_BOARD
    printf("\rInitializing Feather Servo Board at address 0x80: ");
    if (initializePCA9685(PCA9685_ADDRESS))
    {
        printf("\rSUCCESS - Number of RC Servos: %d", NUM_RC_SERVOS);
        FeatherEnabled = true;
    }
    else
    {
        printf("I2C ERROR - RC SERVOS NOT ENABLED");    
        FeatherEnabled = false;
    }
#else
    printf("\rRC SERVOS DISABLED");
#endif    
    
    printf("\rHOST Baudrate: %ld", ActualHOSTBaudRate);        
    printf("\rXBEE Baudrate: %ld", ActualXBEEBaudRate);
    
    if (CommandMode==LOCAL) printf("\rLOCAL MODE:");
    else if (CommandMode==REMOTE) printf("\rREMOTE MODE:");
    else
    {
        printf("\rERROR Invalid Command Mode");    
        runState = HALTED;
    }
    
    if (runState == HALTED) printf(" SERVOS HALTED");
    else printf(" SERVOSS ON");  
    printf("\r\r");
    
    EEBlockCounter = 0;
    EEaddress = 0;
    

    
    while(1) 
    {   
        ClrWdt(); // CLEAR_WATCHDOG        
        
        if (XBEEPacketLength)   
        {         
            if ( processPacketData(XBEEPacket, &numDataIntegers, XBEEData, &TxNumber, &remoteCommand, &subCommand, &errorCode))
            {
                timeout = 5000;                  
                temp = remoteCommand & 0xF0;
                if (temp == HALT_COMMAND)
                {
                    runState = HALTED;
                    PWM1 = PWM2 = PWM3 = PWM4 = 0x0000;
                    if (XBEEDisplayMode) printf("\rHALT COMMAND");
                }
                else if (temp == RUN_COMMAND)
                {
                    runState = RUN;
                    ResetPID(PID, false, false);
                    if (XBEEDisplayMode) printf("\rRUN COMMAND");
                }                
                else if (temp == 0xB0 && subCommand >= 0)
                {                    
                    if (subCommand < NUMMOTORS)
                    {
                        if (PID[subCommand].OpMode == SERVO_POT_MODE || PID[subCommand].OpMode == SERVO_ENCODER_MODE)
                        {
                            tempCommand = ((float)XBEEData[0]) / 1023;
                            tempCommand = (tempCommand * (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS)) + MIN_COMMAND_COUNTS;
                            ServoCommandPosition = (long)tempCommand;                        
                            if (ServoCommandPosition < MIN_COMMAND_COUNTS) ServoCommandPosition = MIN_COMMAND_COUNTS;
                            if (ServoCommandPosition > MAX_COMMAND_COUNTS) ServoCommandPosition = MAX_COMMAND_COUNTS;                        
                            
                        }
                        else if (PID[subCommand].OpMode == CONTINUOUS_MODE)
                            ServoCommandPosition = (long)(XBEEData[0] - 512);
                        else  if (PID[subCommand].OpMode == DESTINATION_MODE) 
                            ServoCommandPosition = (long)((XBEEData[0] - 512) * DESTINATION_MULTIPLIER);
                        else printf("\eERROR Bad subCommand: %d", subCommand);
                        PID[subCommand].PIDCommand = ServoCommandPosition;
                        PID[subCommand].RemoteDataValid = true;
                        if (XBEEDisplayMode) printf("\r#%d COM %02X: SUB #%d: XBEE: %d, SERVO: %d, PID COM: %d, VAL: %d", TxNumber, remoteCommand, subCommand, XBEEData[0], ServoCommandPosition, PID[subCommand].PIDCommand,  PID[subCommand].RemoteDataValid);   
                    }
#ifdef USE_FEATHER_BOARD                     
                    else if (FeatherEnabled)
                    {
                        RCServoID = subCommand - NUMMOTORS;
                        if (RCServoID < NUM_RC_SERVOS && RCServoID >= 0)
                        {
                            ServoCommandPosition = (XBEEData[0] / 4) + 22;
                            if (ServoCommandPosition < 0) ServoCommandPosition = 0;
                            else if (ServoCommandPosition > 277) ServoCommandPosition = 277;
                            RCservoPos[RCServoID] = (short)ServoCommandPosition;
                        }                        
                    }
#endif                
                }
            }
            else if (XBEEDisplayMode) printf("\rPACKET ERROR CODE: %d", errorCode);            
            XBEEPacketLength = 0;
        }        
                    
        if (intFlag)
        {
            intFlag = false;     
            
            IFS1bits.AD1IF = 0;
            while(!IFS1bits.AD1IF);        
            AD1CON1bits.ASAM = 0;        // Pause sampling. 
            for (i = 0; i < MAXPOTS; i++)            
                ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer            
            AD1CON1bits.ASAM = 1;        // Restart sampling.
            
            if (runState)
            {        
                if (LEDcounter) LEDcounter--;
                
                if (!LEDcounter)
                {
                    if (LED) LED = 0;
                    else LED = 1;                    
                    LEDcounter = 40;
                }      
#ifdef USE_FEATHER_BOARD
                if (CommandMode == LOCAL && FeatherEnabled)
                {
                    for (j = 0; j < NUM_RC_SERVOS; j++) 
                    {
                        if (j < (NUMMOTORS + NUM_LOCAL_POTS))
                            RCservoPos[j] = (short)(ADresult[j+NUMMOTORS+6]/4) + 22;
                        else RCservoPos[j] = PreviousRCservoPos[j] = 0;
                    }
                }
                
                for (j = 0; j < NUM_RC_SERVOS; j++)
                {
                    if (RCservoPos[j] != PreviousRCservoPos[j])
                    {
                        if (!setPCA9685outputs (PCA9685_ADDRESS, j, 0, RCservoPos[j])) printf("\rFEATHER #%d ERROR", j);   
                        PreviousRCservoPos[j] = RCservoPos[j];
                    }
                }
                
#endif                            
                for (i = 0; i < NUMMOTORS; i++)
                {    
                    if (!PID[i].Halted && PID[i].OpMode)
                    {   
                        if (CommandMode == JOG && i == ServoSelect)
                        {
                            if (!displayCounter)
                            {
                                displayCounter = 40;                                
                                if (PID[i].OpMode == SERVO_POT_MODE) printf("\rPOT Servo %d PWM: %d, ", ServoSelect, PID[i].PWMvalue);
                                else if (PID[i].OpMode == SERVO_ENCODER_MODE) printf("\rENCODER Servo %d PWM: %d, ", ServoSelect, PID[i].PWMvalue);
                                else if (PID[i].OpMode == DESTINATION_MODE) printf("\rDESTINATION Servo %d PWM: %d, ", ServoSelect, PID[i].PWMvalue);
                                else if (PID[i].OpMode == CONTINUOUS_MODE) printf("\rCONTINUOUS Servo %d PWM: %d, ", ServoSelect, PID[i].PWMvalue);
                                else printf("\rERROR - BAD SERVO OP MODE, ");                                
                                if (PID[i].OpMode == SERVO_POT_MODE) printf("AD: %d", ADresult[i]);
                                else
                                {
                                    GetEncoderPosition(i, PID);
                                    printf("ENCODER: %d", PID[i].CurrentPos);
                                }
                            }
                            else displayCounter--;
                        }
                        else if (CommandMode == LOCAL) 
                        {
#ifdef USE_BRAIN_BOARD                                    
                            tempCommand = (float) ADresult[i];  
#else                  
    #ifdef REV1
                            tempCommand = (float) ADresult[i+6];                                                         
    #else            
                            tempCommand = (float) ADresult[i+4];       
    #endif                                    
#endif                       
                            // SERVO MODES
                            if (PID[i].OpMode == SERVO_POT_MODE || PID[i].OpMode == SERVO_ENCODER_MODE) 
                            {
                                tempCommand = tempCommand / 1023;
                                tempCommand = (tempCommand * (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS)) + MIN_COMMAND_COUNTS;                                    
                                if (tempCommand < MIN_COMMAND_COUNTS) tempCommand = MIN_COMMAND_COUNTS;
                                if (tempCommand > MAX_COMMAND_COUNTS) tempCommand = MAX_COMMAND_COUNTS;                                    
                                PID[i].PIDCommand = (long) tempCommand;                                    
                                ServoControl(i, PID);                                                                                                    
                            }
                            // ENCODER MODES:
                            else
                            {
                                if (PID[i].OpMode == DESTINATION_MODE)
                                {
                                    PID[i].PIDCommand = (((long)tempCommand) - 512) * DESTINATION_MULTIPLIER;
                                    DestinationEncoderControl(i, PID); 
                                }
                                else
                                {
                                    PID[i].PIDCommand = ((long)tempCommand) - 512;
                                    ContinuousEncoderControl(i, PID);
                                }
                            }
                        }
                        else // REMOTE MODE
                        {
                            if (PID[i].RemoteDataValid)
                            {
                                if (PID[i].OpMode == SERVO_POT_MODE || PID[i].OpMode == SERVO_ENCODER_MODE)
                                    ServoControl(i, PID);                                
                                else if (PID[i].OpMode == DESTINATION_MODE) 
                                    DestinationEncoderControl(i, PID);     
                                else if (PID[i].OpMode == CONTINUOUS_MODE)
                                    ContinuousEncoderControl(i, PID);     
                                else printf("\r#%d: BAD REMOTE OP MODE: %d", PID[i].OpMode);
                            }                                                         
                        }
                        PWMvalue = PID[i].PWMvalue;                                                
                    } // end if not halted
                    else PWMvalue = 0;
                                
                    if (i == 0) 
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR1 = REVERSE;                              
                            PWMvalue = 0 - PWMvalue;                            
                        }
                        else MOTOR_DIR1 = FORWARD;    
                        if (PID[i].MotorPolarityFlipped) MOTOR_DIR1 = !MOTOR_DIR1;
                        PWM1 = (unsigned short)PWMvalue;                        
                    }                        
                    else if (i == 1)
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR2 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR2 = FORWARD;    
                        if (PID[i].MotorPolarityFlipped) MOTOR_DIR2 = !MOTOR_DIR2;
                        PWM2 = (unsigned short)PWMvalue;
                    }                                                
                    else if (i == 2) 
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR3 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR3 = FORWARD;    
                        if (PID[i].MotorPolarityFlipped) MOTOR_DIR3 = !MOTOR_DIR3;
                        PWM3 = (unsigned short)PWMvalue;
                    }
                    else if (i == 3)  // Motor #4
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR4 = REVERSE;  
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR4 = FORWARD;  
                        if (PID[i].MotorPolarityFlipped) MOTOR_DIR4 = !MOTOR_DIR4;
                        PWM4 = (unsigned short)PWMvalue;  
                    }
#ifndef USE_BRAIN_BOARD     
    #ifdef REV1
                    else
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR5 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR5 = FORWARD;   
                        if (PID[i].MotorPolarityFlipped) MOTOR_DIR5 = !MOTOR_DIR5;
                        PWM5 = (unsigned short)PWMvalue;
                    }  
    #endif
#endif                    
                }
            }
            else            
            {
                for (i = 0; i < NUMMOTORS; i++) PID[i].sumError = 0;
                
#ifdef USE_BRAIN_BOARD                
                PWM1 = PWM2 = PWM3 = PWM4 = 0;
#else
    #ifdef REV1
                PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
    #else
                PWM1 = PWM2 = PWM3 = PWM4 = 0;
    #endif
#endif
                LED = 0;
            }                                
        } // End if intFlag
        
      
        
        if (ControlCommand)
        {
            switch(ControlCommand)
            {
                case CTRL_A:
                    printf("\r\rALL SERVOS:");
                    for (i = 0; i < NUMMOTORS; i++)
                        DisplayServoConfig(PID, i);
                    break;
                case CTRL_D:
                    if (PIDDisplayMode)
                    {
                        PIDDisplayMode = false;
                        printf("\rPID Display OFF");
                    }
                    else 
                    {
                        PIDDisplayMode = true;
                        printf("\rPID Display ON");
                    }   
                    break;                  
                    
                case CTRL_S:
                    GenerateMemMap(PID, MemMapBuffer);                    
                    numBytesStored = StoreMapInEEprom (MemMapBuffer, FALSE);
                    printf("\rSTORED: %d bytes written to EEprom\r\r", numBytesStored); 
                    break;
                    
                case CTRL_U:
                    if (GenerateMemMap(PID, MemMapBuffer) > 0)
                        printf("%s", MemMapBuffer);
                    break;                    
                
                case CTRL_R:
                    ConfigPIDFromEEprom (MemMapBuffer, PIDConfigList, PID);
                    break;
                    
                case ' ':
                    if (CommandMode != JOG) ResetPID(PID, false, false);
                    if (runState) 
                    {                                  
                        runState = HALTED; 
                        printf("\rHALT");
                    }
                    else
                    {
                        runState = RUN;
                        printf("\rRUN ");
                        if (CommandMode==LOCAL) printf("Command Mode = LOCAL");
                        else if (CommandMode==REMOTE) printf("Command Mode = REMOTE");
                        else if (CommandMode==JOG) printf("Command Mode = JOG");
                        else printf("ERROR Command Mode = %d", CommandMode);
                    }                         
                    break;   
                default:
                    printf("\rCommand: #%d => CTRL-%c", ControlCommand, (ControlCommand - 1 + 'A'));
                    break;
            }
            ControlCommand = 0;
        }
        else if (MemBlockFlag)
        {    
            runState = HALTED;
            numStrings = FillPIDConfigList(MemRxBuffer, PIDConfigList, MAX_MEMORY_STRINGS, TRUE);  
            
            int errors = ConfigurePID (PID, PIDConfigList, numStrings, TRUE);
            if (errors) printf("\rInitializePID error: %d", errors);
            else printf("\rPID Servo configs initialized");
            
            MemBlockFlag = false;
            HOSTRxBufferFull = false;
            HostRxIndex = 0;
            MemRxIndex = 0;
        }
        else if (HOSTRxBufferFull) 
        {
            HOSTRxBufferFull = false;             
            ProcessCommandString (PID, HOSTRxBuffer, TRUE, &CommandMode, &runState, TRUE);             
        }
    } // End while(1))
} // End main())



BYTE processPacketData(BYTE *ptrPacket, short *numData, long *ptrData, BYTE *TxNumber, BYTE *command, BYTE *subCommand, BYTE *errorCode)
{
    MConvertType dataValue;    
    short j, i = 0;  
    BYTE packetDataLength, packetBytes[MAXBUFFER];
    
    *errorCode = NO_ERROR; // Reset error flag to start with.
    
    packetDataLength = decodePacket(ptrPacket, packetBytes, errorCode);    
    if (packetDataLength == 0) return false;    
    
    i = 0;
    *TxNumber = packetBytes[i++]; 
    *command = packetBytes[i++];
    *subCommand = packetBytes[i++];
    *numData = packetBytes[i++];    
        
    j = 0;
    while(j < *numData)
    {
        dataValue.b[0] = packetBytes[i++];
        dataValue.b[1] = packetBytes[i++];
        ptrData[j++] = (long)dataValue.integer;
    }    
    
// printf("\rCRC1: %d, CRC2: %d", packetBytes[i], packetBytes[i+1]);    
    
    if (!CheckCRC(packetBytes, i)) 
    {
        *errorCode = CRC_ERROR;
        return false; 
    }
    return true;
}

unsigned short decodePacket(BYTE *ptrInPacket, BYTE *ptrData, BYTE *errorCode)
{
    unsigned short i, j;
    BYTE escapeFlag = FALSE;
    BYTE startFlag = false;
    BYTE ch;

    j = 0;
    for (i = 0; i < MAXBUFFER; i++) 
    {
        ch = ptrInPacket[i];
        // Escape flag not active
        if (!escapeFlag) 
        {
            if (ch == STX) 
            {
                if (!startFlag) 
                {
                    startFlag = true;
                    j = 0;
                }
                else
                {
                    *errorCode = STX_ERROR;
                    return (0);
                }
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else
            {
                *errorCode = OVERRUN_ERROR;
                return (0);
            }
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else
            {
                *errorCode = OVERRUN_ERROR;
                return(0);
            }
        }
    }
    
    if (i >= MAXBUFFER) *errorCode = OVERRUN_ERROR;
    else if (!startFlag) *errorCode = STX_ERROR;
    else *errorCode = ETX_ERROR;
    return (0);
}

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{        
    static int intCounter = 0;   
    static int millisecondCounter = 20;
    
    mT2ClearIntFlag(); // clear the interrupt flag        
    
    if (millisecondCounter) millisecondCounter--;
    if (millisecondCounter <= 0)
    {
        millisecondCounter = 20;
        milliseconds++;
        if (TEST_OUT) TEST_OUT = 0;
        else TEST_OUT = 1;
    }
    
    if (HOSTtimeout)
    {
        HOSTtimeout--;
        if (HOSTtimeout==0)
        {
            MemRxIndex = 0;
            MemMapRxMode = false;
            printf("\rMEM MAP TIMEOUT ERROR");
        }
    }
    
    if (XBEEtimeout)
    {
        XBEEtimeout--;
        if (!XBEEtimeout)
            startPacket = false;
    }
    
    intCounter++;
    if (intCounter >= 80)// -80) // 2000
    {
        intCounter = 0;
        intFlag = true;
    }
    
    if (timeout)
    {
        timeout--;
        if (!timeout) flagRemoteTimeout = true;
    }
}






// XBEE UART interrupt handler it is set at priority level 2
void __ISR(XBEE_VECTOR, ipl2) IntXBEEUartHandler(void) 
{
    BYTE ch;
    static short XBEERxIndex = 0;    
    short i;
    
    if (XBEEbits.OERR || XBEEbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
        XBEERxIndex = 0;
    } 
    else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart)))
    {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));
        if (UARTReceivedDataIsAvailable(XBEEuart)) 
        {
            XBEEtimeout = 10;
            ch = UARTGetDataByte(XBEEuart);
            if (!startPacket) 
            {
                if (ch == STX)
                {
                    startPacket = true;
                    XBEERxIndex = 0;
                    XBEERxBuffer[XBEERxIndex++] = STX;
                }
            }
            else if (XBEERxIndex < MAXBUFFER) 
                    XBEERxBuffer[XBEERxIndex++] = ch;
            if (ch == ETX) 
            {
                XBEEPacketLength = XBEERxIndex;
                XBEERxIndex = 0;
                for (i = 0; i < XBEEPacketLength; i++)
                    XBEEPacket[i] = XBEERxBuffer[i];
                startPacket = false;
                XBEEtimeout = 0;
            }                
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
}

void InitializeSystem(void) 
{	
    SYSTEMConfigPerformance(SYS_FREQ);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Configure PIC ADC for ten AD input channels
    ADC10_ManualInit();    
    
    // Set up UART Rx DMA interrupts
    // SetupDMA_Rx();
    
    // I/O Ports:
#ifdef REV3
        PORTSetPinsDigitalOut(IOPORT_A, BIT_0 | BIT_3 | BIT_5 | BIT_14 | BIT_15);  // LED, XBEE SLEEP, MOTOR_DIR4     
        PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_3 | BIT_4);  // SWA,SWB,SWC,SWD,
        PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
        // PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
        PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT        
        PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_11 | BIT_12);  // EE_WR, LED, MOTOR DIR #3, #2
        EEPROM_WP = 1;

        PORTSetPinsDigitalOut(IOPORT_E, BIT_4);
        
        PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8 | BIT_13 | BIT_4);  // ENCODER DIRECTION INPUTS 1-4, SWE,SWF
        PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1
        PORTSetPinsDigitalIn(IOPORT_G, BIT_9);  // SD DETECT
        
        mCNOpen(CN_ON, CN2_ENABLE | CN3_ENABLE | CN5_ENABLE | CN6_ENABLE | CN13_ENABLE | CN19_ENABLE, 0);
        ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);         
        
        XBEE_SLEEP = 0; // Put XBEE in active mode
        SD_CS = 1; // Disable SD card
        
#elif (USE_BRAIN_BOARD != 0)
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);  // RS485_ENABLE
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #3, #2
    PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8 | BIT_6);  // ENCODER DIRECTION INPUTS 1-4 
    PORTSetPinsDigitalOut(IOPORT_E, BIT_6);  // LED
    PORTSetPinsDigitalOut(IOPORT_F, BIT_1);  // PWM DISABLE
    PWM_DISABLE = 0;
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1           
#else    
    #ifndef REV2   
        PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
        PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
        PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
        PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
        PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT        
        PORTSetPinsDigitalIn(IOPORT_D, BIT_7);  // SW5
        PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #5, #3, #2
        PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
        PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1       
    #else    
        PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
        PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
        PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
        PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
        PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT        
        PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #3, #2
        PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
        PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1          
    #endif    
    mCNOpen(CN_ON, CN1_ENABLE | CN2_ENABLE | CN3_ENABLE | CN4_ENABLE, CN1_PULLUP_ENABLE | CN2_PULLUP_ENABLE | CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);    
#endif        
    
    
    TEST_OUT = 1;
    
    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip     

    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip     
    
    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 4000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);                    
   
    // Set up PWM OC1
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;  

    // Set up PWM OC2
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;    

    // Set up PWM OC5
    #ifdef REV1
        OC5CON = 0x00;
        OC5CONbits.OC32 = 0; // 16 bit PWM
        OC5CONbits.ON = 1; // Turn on PWM
        OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
        OC5CONbits.OCM2 = 1; // PWM enabled, no fault pin
        OC5CONbits.OCM1 = 1;
        OC5CONbits.OCM0 = 0;
        OC5RS = 0;         
    #endif
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    // UARTConfigure(HOSTuart, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualHOSTBaudRate = UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
    
 
    // Set up XBEE UART at 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);    
    // UARTConfigure(XBEEuart, UART_ENABLE_PINS_TX_RX_ONLY);    
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualXBEEBaudRate = UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_ENABLED); 
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);        
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();   
}// END InitializeSystem() 


#ifdef USE_BRAIN_BOARD
// When using Brain Board: Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
int ADC10_ManualInit(void)
{
    int i, dummy;
    
    AD1CON1bits.ON = 0;
    mAD1IntEnable(INT_DISABLED);   
    mAD1ClearIntFlag();
    
    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CHS  = 0;
    AD1CSSL = 0;
    
    // Set each Port B pin for digital or analog
    // Analog = 0, digital = 1
    AD1PCFGbits.PCFG0 = 1; 
    
    AD1PCFGbits.PCFG1 = 0; 
    AD1PCFGbits.PCFG2 = 0; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    
    AD1PCFGbits.PCFG8 = 1; 
    AD1PCFGbits.PCFG9 = 1; 
    AD1PCFGbits.PCFG10 = 1; 
    AD1PCFGbits.PCFG11 = 1;
    
    AD1PCFGbits.PCFG1 = 0; 
    AD1PCFGbits.PCFG2 = 0; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    
    AD1PCFGbits.PCFG8 = 1; 
    AD1PCFGbits.PCFG9 = 1; 
    AD1PCFGbits.PCFG10 = 1; 
    AD1PCFGbits.PCFG11 = 1;
    
    
    AD1PCFGbits.PCFG12 = 0; 
    AD1PCFGbits.PCFG13 = 0; 
    AD1PCFGbits.PCFG14 = 0; 
    AD1PCFGbits.PCFG15 = 0;     
    
    AD1CON1bits.FORM = 000;        // 16 bit integer format.
    AD1CON1bits.SSRC = 7;        // Auto Convert
    AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
    AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
    
    AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
    AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
    AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
    AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
    AD1CON2bits.BUFM = 0;        // One 16 word buffer
    AD1CON2bits.ALTS = 0;        // Use only Mux A
    AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
    AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
    AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
    AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3

    // Set conversion clock and set sampling time.
    AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
    AD1CON3bits.SAMC = 0b11111;        // Sample time max
    AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

    // Select channels to scan. Scan channels = 1, Skip channels = 0
    AD1CSSLbits.CSSL0 = 0;
    
    AD1CSSLbits.CSSL1 = 1;
    AD1CSSLbits.CSSL2 = 1;
    AD1CSSLbits.CSSL3 = 1;    
    AD1CSSLbits.CSSL4 = 1;
    
    AD1CSSLbits.CSSL5 = 0;
    AD1CSSLbits.CSSL6 = 0;
    AD1CSSLbits.CSSL7 = 0;
    AD1CSSLbits.CSSL8 = 0;
    AD1CSSLbits.CSSL9 = 0;
    AD1CSSLbits.CSSL10 = 0;
    AD1CSSLbits.CSSL11 = 0;
    
    AD1CSSLbits.CSSL12 = 1;
    AD1CSSLbits.CSSL13 = 1;
    AD1CSSLbits.CSSL14 = 1;
    AD1CSSLbits.CSSL15 = 1;
    
    // Make sure all buffers have been Emptied. 
    for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
    
    AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
    AD1CON1bits.ON = 1;            // Turn on ADC.
    return (1);
}

#else
    #ifdef REV1
    // When using REV 1 MD13S Grove controller board:
    // Analog input Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
        mAD1IntEnable(INT_DISABLED);   
        mAD1ClearIntFlag();
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        // Set each Port B pin for digital or analog
        // Analog = 0, digital = 1
        AD1PCFGbits.PCFG0 = 1; 
        AD1PCFGbits.PCFG1 = 1; 
        AD1PCFGbits.PCFG2 = 1; 
        AD1PCFGbits.PCFG3 = 0; 
        AD1PCFGbits.PCFG4 = 0; 
        AD1PCFGbits.PCFG5 = 1; 
        AD1PCFGbits.PCFG6 = 1; 
        AD1PCFGbits.PCFG7 = 1; 
        AD1PCFGbits.PCFG8 = 0; 
        AD1PCFGbits.PCFG9 = 0; 
        AD1PCFGbits.PCFG10 = 0; 
        AD1PCFGbits.PCFG11 = 0; 
        AD1PCFGbits.PCFG12 = 0; 
        AD1PCFGbits.PCFG13 = 0; 
        AD1PCFGbits.PCFG14 = 0; 
        AD1PCFGbits.PCFG15 = 0;     
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 0;
        AD1CSSLbits.CSSL1 = 0;
        AD1CSSLbits.CSSL2 = 0;
        AD1CSSLbits.CSSL3 = 1;
        AD1CSSLbits.CSSL4 = 1;
        AD1CSSLbits.CSSL5 = 0;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 1;
        AD1CSSLbits.CSSL9 = 1;
        AD1CSSLbits.CSSL10 = 1;
        AD1CSSLbits.CSSL11 = 1;
        AD1CSSLbits.CSSL12 = 1;
        AD1CSSLbits.CSSL13 = 1;
        AD1CSSLbits.CSSL14 = 1;
        AD1CSSLbits.CSSL15 = 1;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
        
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return (1);
    }
    #else
    // When using REV 2 or REV 3 MD13S board:
    // Analog input Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
        mAD1IntEnable(INT_DISABLED);   
        mAD1ClearIntFlag();
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        // Set each Port B pin for digital or analog
        // Analog = 0, digital = 1
        AD1PCFGbits.PCFG0 = 1; 
        AD1PCFGbits.PCFG1 = 1; 
        AD1PCFGbits.PCFG2 = 1; 
        AD1PCFGbits.PCFG3 = 1; 
        AD1PCFGbits.PCFG4 = 1; 
        AD1PCFGbits.PCFG5 = 1; 
        AD1PCFGbits.PCFG6 = 1; 
        AD1PCFGbits.PCFG7 = 1; 
        AD1PCFGbits.PCFG8 = 0; 
        AD1PCFGbits.PCFG9 = 0; 
        AD1PCFGbits.PCFG10 = 0; 
        AD1PCFGbits.PCFG11 = 0;         
        AD1PCFGbits.PCFG12 = 0; 
        AD1PCFGbits.PCFG13 = 0; 
        AD1PCFGbits.PCFG14 = 0; 
        AD1PCFGbits.PCFG15 = 0;     
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 0;
        AD1CSSLbits.CSSL1 = 0;
        AD1CSSLbits.CSSL2 = 0;
        AD1CSSLbits.CSSL3 = 0;
        AD1CSSLbits.CSSL4 = 0;
        AD1CSSLbits.CSSL5 = 0;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 1;
        AD1CSSLbits.CSSL9 = 1;
        AD1CSSLbits.CSSL10 = 1;
        AD1CSSLbits.CSSL11 = 1;
        AD1CSSLbits.CSSL12 = 1;
        AD1CSSLbits.CSSL13 = 1;
        AD1CSSLbits.CSSL14 = 1;
        AD1CSSLbits.CSSL15 = 1;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
        
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return (1);
    }
    #endif
#endif    


int DestinationEncoderControl(short ServoID, struct PIDtype *PID)
{
    long Error;                  
    long derError;       
    float PCorr = 0, ICorr = 0, DCorr = 0, PIDcorrection = 0;    
    static short displayCounter = 0;        
    float Acceleration, Deacceleration;          
    static enableDisplay = true;
    long RampLength = 0;
    
    if (PID[ServoID].OpMode != DESTINATION_MODE)
    {
        PID[ServoID].Halted = true;
        PID[ServoID].PWMvalue = 0;
        return 0;
    }
    
    if (PID[ServoID].Halted)
    {
        PID[ServoID].PWMvalue = 0;
        return 0;
    }
    
    GetEncoderPosition(ServoID, PID);
    
    Acceleration = ((PID[ServoID].MaxVelocity / INTERRUPTS_PER_SECOND)) * 8;
    Deacceleration = Acceleration / 2;
    

    if (ServoID < 0)
    {
        PID[ServoID].Halted = true;
        PID[ServoID].PWMvalue = 0;
        return 0;
    }
    
    PID[ServoID].Destination =   PID[ServoID].PIDCommand;
    if (PID[ServoID].RampState == RAMP_START)
    {
        PID[ServoID].RampUpStartPosition = PID[ServoID].CurrentPos;
        PID[ServoID].RampDownStartPosition = PID[ServoID].Destination; // Temporary
        if (PID[ServoID].Destination > PID[ServoID].CurrentPos) 
        {
            PID[ServoID].Direction = FORWARD;
            PID[ServoID].TargetVelocity = PID[ServoID].MaxVelocity;
        }
        else
        {
            PID[ServoID].Direction = REVERSE;
            PID[ServoID].TargetVelocity = 0 - PID[ServoID].MaxVelocity;
        }
        PID[ServoID].RampState++;
    }     
        
    if (PID[ServoID].RampState == RAMP_UP)
    {            
        if (PID[ServoID].Direction == FORWARD)
        {
            PID[ServoID].Velocity = PID[ServoID].Velocity + Acceleration;
            if(PID[ServoID].Velocity >= PID[ServoID].TargetVelocity)
            {
                PID[ServoID].Velocity = PID[ServoID].TargetVelocity;
                RampLength = (PID[ServoID].CurrentPos - PID[ServoID].RampUpStartPosition) * 2;
                PID[ServoID].RampDownStartPosition = PID[ServoID].Destination - RampLength;
                PID[ServoID].RampState++;       
            }
        }
        else
        {
            PID[ServoID].Velocity = PID[ServoID].Velocity - Acceleration;
            if(PID[ServoID].Velocity <= PID[ServoID].TargetVelocity)
            {
                PID[ServoID].Velocity = PID[ServoID].TargetVelocity;
                RampLength = (PID[ServoID].RampUpStartPosition - PID[ServoID].CurrentPos) * 2;
                PID[ServoID].RampDownStartPosition = PID[ServoID].Destination + RampLength;
                PID[ServoID].RampState++;       
            }                
        }                
    }   
    else if (PID[ServoID].RampState == RAMP_RUN)
    {
        if (PID[ServoID].Direction == FORWARD)
        {
            if (PID[ServoID].CurrentPos >= PID[ServoID].RampDownStartPosition)
                   PID[ServoID].RampState++;
        }
        else
        {
            if (PID[ServoID].CurrentPos <= PID[ServoID].RampDownStartPosition)
                PID[ServoID].RampState++;                
        }
    }
    else if (PID[ServoID].RampState == RAMP_DOWN)
    {
        if (PID[ServoID].Direction == FORWARD)
        {
            PID[ServoID].Velocity = PID[ServoID].Velocity - Deacceleration;
            if (PID[ServoID].Velocity < PID[ServoID].MinVelocity) PID[ServoID].Velocity = PID[ServoID].MinVelocity;
        }
        else
        {
            PID[ServoID].Velocity = PID[ServoID].Velocity + Deacceleration;
            if (PID[ServoID].Velocity > PID[ServoID].MinVelocity) PID[ServoID].Velocity = PID[ServoID].MinVelocity;
        }
        if (PID[ServoID].Velocity == PID[ServoID].MinVelocity) PID[ServoID].RampState++;
    }             
    
    PID[ServoID].CommandPos = PID[ServoID].CommandPos + PID[ServoID].Velocity;   
    
    Error = PID[ServoID].CurrentPos - (long) PID[ServoID].CommandPos;    
    derError = Error - PID[ServoID].error[PID[ServoID].errIndex];
    
    PID[ServoID].error[PID[ServoID].errIndex] = Error;
    PID[ServoID].errIndex++; 
    if (PID[ServoID].errIndex >= FILTERSIZE) PID[ServoID].errIndex = 0;                

    if (!PID[ServoID].saturation) PID[ServoID].sumError = PID[ServoID].sumError + (long)Error;
    
    PCorr = ((float)Error) * -PID[ServoID].kP;    
    ICorr = ((float)PID[ServoID].sumError) * -PID[ServoID].kI;
    DCorr = ((float)derError) * -PID[ServoID].kD;
    if (PID[ServoID].OpMode == DESTINATION_MODE && PID[ServoID].RampState != RAMP_DOWN)
        DCorr = 0;
    

    PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection < 0) PID[ServoID].PWMvalue = (long) (PIDcorrection - PID[ServoID].PWMoffset);            
    else PID[ServoID].PWMvalue = (long) (PIDcorrection + PID[ServoID].PWMoffset);                    
           
        
    if (PID[ServoID].PWMvalue >= PWM_MAX) 
    {
        PID[ServoID].PWMvalue = PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else if (PID[ServoID].PWMvalue <= -PWM_MAX) 
    {
        PID[ServoID].PWMvalue = -PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else PID[ServoID].saturation = false;       
    
    if (PID[ServoID].RampState == RAMP_HALT)
    {
        PID[ServoID].PWMvalue = 0;
        PID[ServoID].Halted = true;        
    }    
    
    if (ServoID == ServoSelect)
    {                 
        displayCounter++;
        if (PIDDisplayMode)
        {
            if (displayCounter >= 10 || PID[ServoID].Halted)
            {                
                displayCounter = 0;      
                if (enableDisplay)
                {
                    if (PID[ServoID].Halted) printf("\rHALT: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[ServoID].Destination, PID[ServoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[ServoID].PWMvalue);                                                
                    else if (PID[ServoID].RampState == RAMP_UP) printf("\rRAMP: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[ServoID].Destination, PID[ServoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[ServoID].PWMvalue);
                    else if (PID[ServoID].RampState == RAMP_RUN) printf("\rRUN: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[ServoID].Destination, PID[ServoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[ServoID].PWMvalue);                        
                    else if (PID[ServoID].RampState == RAMP_DOWN) printf("\rBRAKE: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[ServoID].Destination, PID[ServoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[ServoID].PWMvalue);
                    else printf("\rHALT: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[ServoID].Destination, PID[ServoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[ServoID].PWMvalue);
                 }
            }
        }
    }       



    return 1;
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{    
    unsigned short PORTB_Read = 0, PORTD_Read = 0;
    static unsigned short PreviousPORTB_Read = 0, PreviousPORTD_Read = 0;
    
    // Step #1 - always clear the mismatch condition first
    PORTB_Read = PORTB & 0b0000000000011011;
    if (PORTB_Read & 0x0001) SWAstste = 1; else SWAstste = 0;
    if (PORTB_Read & 0x0002) SWBstste = 1; else SWBstste = 0;
    if (PORTB_Read & 0x0008) SWCstste = 1; else SWCstste = 0;
    if (PORTB_Read & 0x0010) SWDstste = 1; else SWDstste = 0;
    
    PORTD_Read = PORTD & 0b0010000000010000;
    if (PORTD_Read & 0x0010) SWFstste = 1; else SWFstste = 0;
    if (PORTD_Read & 0x2000) SWEstste = 1; else SWEstste = 0;
    
    if (PORTB_Read != PreviousPORTB_Read) SWChangeFlag = true;
    if (PORTD_Read != PreviousPORTD_Read) SWChangeFlag = true;
    
    PreviousPORTB_Read = PORTB_Read;
    PreviousPORTD_Read = PORTD_Read;
    
    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
}


#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023
#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86

int ServoControl(short ServoID, struct PIDtype *PID)
{
    long Error;              
    long totalDerError = 0;
    long derError;    
    long ADvalue = 0;
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    short i;
    static short displayCounter = 0;    
    unsigned short QuadReading = 0;
    
    if (PID[ServoID].OpMode == SERVO_POT_MODE)
    {    
#ifdef USE_BRAIN_BOARD 
        PID[ServoID].CurrentPos = (long)(ADresult[ServoID+4]);
        
#elif REV2
        PID[ServoID].CurrentPos = (long)(ADresult[ServoID+1]);
#else        
        PID[ServoID].CurrentPos = (long)(ADresult[ServoID]);
#endif
        ADvalue = PID[ServoID].CurrentPos;
        if (PID[ServoID].CurrentPos < QUAD_ONE) QuadReading = QUAD_ONE;
        else if (PID[ServoID].CurrentPos < QUAD_TWO) QuadReading = QUAD_TWO;
        else QuadReading = QUAD_THREE;
    
        if (PID[ServoID].PreviousQuad == NO_QUAD) 
            PID[ServoID].PreviousQuad = QuadReading;
        else if (QuadReading == QUAD_TWO)
            PID[ServoID].PreviousQuad = QUAD_TWO;
        else if (PID[ServoID].PreviousQuad == QUAD_TWO)
            PID[ServoID].PreviousQuad = QuadReading;
        else if (PID[ServoID].PreviousQuad == QUAD_ONE)
        {
            if (QuadReading == QUAD_THREE) 
                PID[ServoID].CurrentPos = PID[ServoID].CurrentPos - (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
        }
        else if (PID[ServoID].PreviousQuad == QUAD_THREE)
        {
            if (QuadReading == QUAD_ONE) 
                PID[ServoID].CurrentPos = PID[ServoID].CurrentPos + (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
        }
    }
    else if (PID[ServoID].OpMode == SERVO_ENCODER_MODE)
        GetEncoderPosition(ServoID, PID);
    else
    {
        PID[ServoID].PWMvalue = 0;
        return 0;
    }
    Error = PID[ServoID].CurrentPos - PID[ServoID].PIDCommand;            
    PID[ServoID].error[PID[ServoID].errIndex] = Error;
    PID[ServoID].errIndex++; 
    if (PID[ServoID].errIndex >= FILTERSIZE) PID[ServoID].errIndex = 0;   
    if (!PID[ServoID].saturation) PID[ServoID].sumError = PID[ServoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[ServoID].error[i];
    derError = totalDerError / FILTERSIZE;    
    
    PCorr = ((float)Error) * -PID[ServoID].kP;    
    ICorr = ((float)PID[ServoID].sumError)  * -PID[ServoID].kI;
    DCorr = ((float)derError) * -PID[ServoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[ServoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[ServoID].PWMvalue = (long) (PIDcorrection - PID[ServoID].PWMoffset);            
    else PID[ServoID].PWMvalue = (long) (PIDcorrection + PID[ServoID].PWMoffset);                    
         
    if (PID[ServoID].PWMvalue > PWM_MAX) 
    {
        PID[ServoID].PWMvalue = PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else if (PID[ServoID].PWMvalue < -PWM_MAX) 
    {
        PID[ServoID].PWMvalue = -PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else PID[ServoID].saturation = false;        
        
        
        displayCounter++; 
        if (ServoID == ServoSelect)
        {            
            if (displayCounter >= 40 && PIDDisplayMode)
            {   
                if (PID[ServoID].PreviousQuad == QUAD_ONE) printf("\rQI ");
                else if (PID[ServoID].PreviousQuad == QUAD_TWO) printf("\rQII ");
                else printf("\rQIII ");
                printf("COM: %d, POS: %d AD: %d, ERR: %d P: %0.1f I: %0.1f PWM: %d ", PID[ServoID].PIDCommand, PID[ServoID].CurrentPos, ADvalue, Error, PCorr, ICorr, PID[ServoID].PWMvalue);
                displayCounter = 0;
            }
        }
        
    return 1;
}


void GetEncoderPosition(short ServoID, struct PIDtype *PID)
{
    BYTE EncoderDirection;   
    
    if (ServoID == 0)
    {
        EncoderDirection = EncoderOneDir;    
        if (PID[ServoID].EncoderPolarityFlipped) EncoderDirection = !EncoderDirection;
        if (EncoderDirection) 
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos + (long)EncoderOne;
        else
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos - (long)EncoderOne;
        EncoderOne = 0;
    }    
    else if (ServoID == 1)
    {
        EncoderDirection = EncoderTwoDir; 
        if (PID[ServoID].EncoderPolarityFlipped) EncoderDirection = !EncoderDirection;
        if (EncoderDirection) 
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos + (long)EncoderTwo;
        else
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos - (long)EncoderTwo;
        EncoderTwo = 0;
    }    
    else if (ServoID == 2)
    {
        EncoderDirection = EncoderThreeDir;
        if (PID[ServoID].EncoderPolarityFlipped) EncoderDirection = !EncoderDirection;
        if (EncoderDirection) 
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos + (long)EncoderThree;
        else
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos - (long)EncoderThree;
        EncoderThree = 0;
    }    
    else
    {
        EncoderDirection = EncoderFourDir;
        if (PID[ServoID].EncoderPolarityFlipped) EncoderDirection = !EncoderDirection;
        if (EncoderDirection) 
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos + (long)EncoderFour;
        else
            PID[ServoID].CurrentPos = PID[ServoID].CurrentPos - (long)EncoderFour;
        EncoderFour = 0;
    }       
}



#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
BYTE ch, inByte;
int i;
    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = UARTGetDataByte(HOSTuart);
            ch = toupper(inByte);
            
            if (ch == '>') 
            {                
                MemMapRxMode = true;  
                MemBlockFlag = false;
                MemRxIndex = 0;
                HostRxIndex = 0;
            }

            if (MemMapRxMode) 
            {
                HOSTtimeout = HOST_UART_TIMEOUT;
                if (inByte != '\n') 
                {
                    if (MemRxIndex < MAXBUFFER) 
                        MemRxBuffer[MemRxIndex++] = inByte;
                    else
                    {
                        MemMapRxMode = false;                        
                        HOSTtimeout = 0;
                        MemRxIndex = 0;
                        printf("\r\nOVERRUN ERROR: %d bytes", MemRxIndex);                        
                    }                    
                }      
                if (inByte == '<')
                {
                    HOSTtimeout = 0;
                    MemBlockFlag = true;
                    MemMapRxMode = false;      
                    MemRxBuffer[MemRxIndex++] = '\0';
                }                
            }
            else if (ch != 0 && ch != '\n')            
            {            
                if (' ' == ch && HostRxIndex == 0)
                    ControlCommand = ' ';
                else if ('\r' == ch) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[HostRxIndex++] = ch;
                    HOSTRxBuffer[HostRxIndex] = '\0';
                    HostRxIndex = 0;
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\r');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\n');
                }     
                else if (ch < ' ')
                {
                    ControlCommand = ch;
                    HostRxIndex = 0;
                }
                else if (ch == BACKSPACE) 
                {
                    if (HostRxIndex != 0) HostRxIndex--;
                    HOSTRxBuffer[HostRxIndex] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (HostRxIndex < MAXBUFFER) 
                {
                    HOSTRxBuffer[HostRxIndex] = ch;
                    HostRxIndex++;
                }            
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

void ClearStrings(BYTE **ptrString, short NumStrings)
{
    int i;
    for (i = 0; i < NumStrings; i++) ptrString[i] = NULL;    
}

void ClearBuffer(BYTE *ptrBuffer, short NumBytes)
{
    int i;
    for (i = 0; i < NumBytes; i++) ptrBuffer[i] = '\0';
}

BYTE ProcessMemoryMap(BYTE *ptrPacket, BYTE **ptrString)
{
int length;

    BYTE arrPacketCommand[256] = "This is as normal as I get";

    printf("\rTESTING MALLOC #0:");
    length = strlen(arrPacketCommand);
    ptrString[0] = malloc(length * sizeof(*ptrString[0]));
    strcpy(ptrString[0], arrPacketCommand);
    
    return 0;
}

int FillPIDConfigList(BYTE *ptrBuffer, BYTE **ptrMemMap, int MaxStrings, BYTE displayList)
{
    int stringIndex, j, size;
    char *ptrChar, *ptrEND, *ptrString;
    char Delimiters[] = "\r";
    
    for (stringIndex = 0; stringIndex < MaxStrings; stringIndex++) ptrMemMap[stringIndex] = NULL;  

    ptrChar = strchr(ptrBuffer, '>');
    if (ptrChar == NULL) 
    {
        printf("\rERROR FillPIDConfigList Missing >");
        return 0;    
    }
    
    ptrChar = strchr(ptrBuffer, '<');
    if (ptrChar == NULL) 
    {
        printf("\rERROR FillPIDConfigList Missing <");        
        return 0;
    }
    
    ptrChar = strtok (ptrBuffer, Delimiters); 
    stringIndex = 0;    
    while (ptrChar != NULL && stringIndex < MaxStrings)
    {
        ptrEND = strchr(ptrChar, '<');
        if (ptrEND) ptrEND[1] = '\0';
        else
        {
            ptrEND = strstr(ptrChar, "//");  // comments
            if (ptrEND) ptrEND[1] = '\0';
        }
        int length = strlen(ptrChar);
        ptrMemMap[stringIndex] = NULL;
        size = sizeof(*ptrMemMap[stringIndex]) * (length + 1);
        ptrMemMap[stringIndex] = malloc(size);        
        memcpy(ptrMemMap[stringIndex], ptrChar, length);
        ptrMemMap[stringIndex][length] = '\0';
        stringIndex++;
        ptrChar = strtok(NULL, Delimiters);
    }
    if (stringIndex >= MaxStrings) 
    {
        printf("\rOVERRUN ERROR FillPIDConfigList()");
        return 0;
    }
    else if (displayList)
    {
        printf ("\rList has %d lines", stringIndex);
        for (j = 0; j < stringIndex; j++)
            printf("\r%s", ptrMemMap[j]);
    }
    return stringIndex;
}    

int FetchMemMapFromEEprom (unsigned char *MemMapBuffer)
{
    int i = 0, j = 0, k = 0;
    unsigned char dataByte;
    unsigned char EEReadMemoryBlock[EEBLOCKSIZE+2];
    
    BYTE error = 0;
    
    i = 0;
    do
    {                        
        error = ReadEEpromBlock (EEBUS, EEPROM_ID, (k * EEBLOCKSIZE) + EEPROM_DUMP_ADDRESS_OFFSET, EEReadMemoryBlock, EEBLOCKSIZE);
        EEReadMemoryBlock[EEBLOCKSIZE] = '\0';
                

        if (error) 
        {
            printf("\r\rREAD ERROR: %d", error);
            return 0;
        }   
        
        else if((k == 0) && (NULL == strchr(EEReadMemoryBlock, '>')) )
        {
            printf("\rFetchMemMapFromEEprom ERROR - NO START CHAR >");
            return 0;
        }
        
        for (j = 0; j < EEBLOCKSIZE; j++)
        {                                
            dataByte = EEReadMemoryBlock[j];
            if (i < MAXBUFFER) MemMapBuffer[i++] = dataByte;
            if (dataByte == '<') 
            {
                MemMapBuffer[i] = '\0';
                break;
            }
        }                            
        k++;
    } while (dataByte != '<' && i < MAXBUFFER);
    if (i >= MAXBUFFER)
    {
        printf("\rFetchMemMapFromEEprom ERROR - NO END CHAR <");
        return 0;
    }
    return (i);
}   

int StoreMapInEEprom (unsigned char *ptrMemBuffer, BYTE enableDiagnostics)
{
    BYTE ch, EEWriteMemoryBlock[EEBLOCKSIZE+2];
    int error = 0, i = 0, j = 0, EEBlockCounter = 0;                
                
    do {
        ch = ptrMemBuffer[i++]; 
        if (ch!='\0')
        {
            EEWriteMemoryBlock[j++] = ch;
            if (j == EEBLOCKSIZE || ch == '<')
            {
                error = WriteEEpromBlock (EEBUS, EEPROM_ID, (EEBlockCounter * EEBLOCKSIZE) + EEPROM_DUMP_ADDRESS_OFFSET, EEWriteMemoryBlock, j);
                EEWriteMemoryBlock[j++] = '\0';                    
                if (error) printf("\rBLOCK #%d: WRITE ERROR: %d", EEBlockCounter, error);
                else if (enableDiagnostics) printf("\rBLOCK WRITE #%d: %s", EEBlockCounter, EEWriteMemoryBlock);
                EEBlockCounter++;
                j = 0;
            }
        }
        else break;
    } while (i < MAXBUFFER && ch != '<');
    return i;
}


int ConfigurePID (struct PIDtype *PID, BYTE **ptrConfigString, int numStrings, BYTE DiagnosticsEnabled)
{
    unsigned char *ptrString, *ptrChar, *ptrVal, *ptrAlias, *ptrColon, ch;
    int i, j, length;
        
    if (numStrings > MAX_MEMORY_STRINGS) return 1;        
    i = 0;
    while (i < numStrings)
    {
        ProcessCommandString (PID, ptrConfigString[i], DiagnosticsEnabled, NULL, NULL, FALSE);
        i++;
    }
    return 0;
}


void ResetPID(struct PIDtype *PID, BYTE resetPositionFlag, BYTE setDefaults)
{
    int i = 0;
    for (i = 0; i < NUMMOTORS; i++) ResetServoPID (i, PID, resetPositionFlag, setDefaults);
}


void InitPID (struct PIDtype *PID)
{
    int i, length;
    BYTE strAlias[256];
    
    
    ResetPID(PID, true, true);    
    
    for (i = 0; i < NUMMOTORS; i++)
    {                   
        length = sprintf (strAlias, "$Servo%d", i);
        PID[i].alias = malloc(sizeof (*PID[i].alias) * length);
        strcpy(PID[i].alias, strAlias);
        
        PID[i].kP = KP;
        PID[i].kI = KI;
        PID[i].kD = KD;
        PID[i].PWMoffset = PWM_OFFSET;        

        /*
        if (PID[i].OpMode == SERVO_POT_MODE || PID[i].OpMode == SERVO_ENCODER_MODE)     
        {
            PID[i].kP = KP;
            PID[i].kI = KI;
            PID[i].kD = KD;
            PID[i].PWMoffset = PWM_OFFSET;
        }
        else if (PID[i].OpMode == DESTINATION_MODE)
        {
            PID[i].kP = DEST_KP;
            PID[i].kI = DEST_KI;
            PID[i].kD = DEST_KD;            
            PID[i].PWMoffset = DEST_PWM_OFFSET;
        }    
        else
        {
            PID[i].kP = CONST_KP;
            PID[i].kI = CONST_KI;
            PID[i].kD = CONST_KD;            
            PID[i].PWMoffset = CONST_PWM_OFFSET;
        }    
        */    
    }    
}

void ResetServoPID (short ServoID, struct PIDtype *PID, BYTE resetPositionFlag, BYTE setDefaults)
{
        int j;

        EncoderOne = 0;
        EncoderTwo = 0;
        EncoderThree = 0;
        EncoderFour = 0;

        PID[ServoID].sumError = 0; // For 53:1 ratio Servo City motor
        PID[ServoID].errIndex = 0;
        PID[ServoID].PWMvalue = 0;        
        PID[ServoID].PIDCommand = 0;        
        PID[ServoID].saturation = false;
        PID[ServoID].Velocity = 0;        
        PID[ServoID].Halted = false;
        PID[ServoID].RemoteDataValid = false;   
        
        if (setDefaults)
        {
            PID[ServoID].EncoderPolarityFlipped = false;
            PID[ServoID].MotorPolarityFlipped = false;        
            if (resetPositionFlag)
            {
                PID[ServoID].CurrentPos = 0;
                PID[ServoID].CommandPos = 0;            
            }                
            PID[ServoID].OpMode = 0;            
            PID[ServoID].MaxVelocity = MAX_26_VELOCITY / 2;                
        }        
        
        PID[ServoID].Velocity = 0;
        PID[ServoID].TargetVelocity = 0;        
        PID[ServoID].MinVelocity = 0;         
        PID[ServoID].Destination = 0;
        PID[ServoID].RampUpStartPosition = 0;
        PID[ServoID].RampDownStartPosition = 0;
        PID[ServoID].PreviousQuad = NO_QUAD;
        PID[ServoID].Velocity = 0;       
        PID[ServoID].RampState = RAMP_START;
        PID[ServoID].Direction = FORWARD;
        for (j = 0; j < FILTERSIZE; j++) PID[ServoID].error[j] = 0;
}


#define MAXNAME 256
int ProcessCommandString (struct PIDtype *PID, BYTE *ptrString, BYTE DiagnosticsEnabled, BYTE *CommandMode, BYTE *RunState, BYTE UseTestCommands)
{
    char *ptrChar, *ptrVal, ch;
    int i, length, temp;    
    static int TestPWM = 0;
    
    
    if (UseTestCommands)
    {
        ptrChar = strstr (ptrString, "SER");        
        if (ptrChar)
        {
            ptrVal = strchr(ptrChar, ' ');
            if (ptrVal)
            {
                *RunState = HALTED;
                ptrVal[0] = ' ';
                ServoSelect = atoi(ptrVal); 
                if (ServoSelect < 0) ServoSelect = 0;
                else if (ServoSelect >= NUMMOTORS) ServoSelect = NUMMOTORS - 1;
                for (i = 0; i < NUMMOTORS; i++)
                {
                    if (i == ServoSelect) PID[i].PWMvalue = TestPWM;
                    else PID[i].PWMvalue = 0;                    
                }
                if (DiagnosticsEnabled) printf("\rSERVO ID #%d", ServoSelect);            
            }
        }
        
        if (ServoSelect < 0)
        {
            printf("\rSelect servo");
            return 1;
        }
        
        ptrChar = strstr (ptrString, "LOC");        
        if (ptrChar)
        {
            *CommandMode = LOCAL;  
            if (DiagnosticsEnabled) printf("\rLOCAL MODE");
        }    
        ptrChar = strstr (ptrString, "REM");        
        if (ptrChar)
        {
            *CommandMode = REMOTE;            
            if (DiagnosticsEnabled) printf("\rREMOTE MODE");
        }    
        ptrChar = strstr (ptrString, "JOG");                
        if (ptrChar)
        {
            ptrVal = strchr(ptrChar, ' ');                
            if (ptrVal)
            {
                *RunState = HALTED;
                ptrVal[0] = ' ';
                temp = atoi(ptrVal);                     
                if (temp >=0 && temp < NUMMOTORS)
                {
                    ServoSelect = temp;
                    *CommandMode = JOG;            
                    if (DiagnosticsEnabled) printf("\rJOG MODE SERVO #%d", ServoSelect);
                }
                else printf("\rERROR INVALID SERVO #%d", temp);
            }
            else printf("\rERROR INVALID COMMAND");
        } 
        
        ptrChar = strstr (ptrString, "FOR");                
        if (ptrChar)
        {
            printf("\rOK");
            return 1;
            *RunState = HALTED;
            if (ServoSelect >= 0 && ServoSelect < NUMMOTORS)
            {
                if (DiagnosticsEnabled) printf("\rSERVO FORWARD");
                PID[ServoSelect].PWMvalue = abs(PID[ServoSelect].PWMvalue);
                *RunState = RUN;
            }
            
        } 
        
        ptrChar = strstr (ptrString, "REV"); 
        if (ptrChar)
        {
            *RunState = HALTED;
            if (ServoSelect >= 0 && ServoSelect < NUMMOTORS)
            {
                if (DiagnosticsEnabled) printf("\rSERVO REVERSE");
                PID[ServoSelect].PWMvalue = 0 - abs(PID[ServoSelect].PWMvalue);
                *RunState = RUN;
            }
        } 
        
        ptrChar = strstr (ptrString, "PWM");        
        if (ptrChar)
        {
            *CommandMode = JOG;
            ptrVal = strchr(ptrChar, ' ');
            if (ptrVal)
            {
                ptrVal[0] = ' ';
                TestPWM = atoi(ptrVal); 
                if (TestPWM > PWM_MAX) TestPWM = PWM_MAX;
                else if (TestPWM < -PWM_MAX) TestPWM = -PWM_MAX;
                for (i = 0; i < NUMMOTORS; i++)
                {
                    if (i == ServoSelect) PID[i].PWMvalue = TestPWM;
                    else PID[i].PWMvalue = 0;                    
                }
                if (DiagnosticsEnabled) printf("\rJOG MODE SERVO #%d, PWM = %d", ServoSelect, PID[ServoSelect].PWMvalue);
            }
        }
    }
    
    ptrChar = strchr(ptrString, '#');        
    if (ptrChar)
    {
            length = strlen(ptrChar);
            ptrChar[0] = ' ';
            ServoSelect = atoi(ptrChar);            
    }
    else if (!UseTestCommands) return 1;
    
    if (ServoSelect >= 0 && ServoSelect < NUMMOTORS)
    {             
        if (strstr(ptrString, "POT")) PID[ServoSelect].OpMode = SERVO_POT_MODE;
        else if (strstr(ptrString, "CONT")) PID[ServoSelect].OpMode = CONTINUOUS_MODE;
        else if (strstr(ptrString, "ENC")) PID[ServoSelect].OpMode = SERVO_ENCODER_MODE;
        else if (strstr(ptrString, "DEST")) PID[ServoSelect].OpMode = DESTINATION_MODE;
                
        ptrChar = strstr(ptrString, "MOT-");
        if (ptrChar) PID[ServoSelect].MotorPolarityFlipped = true;
        
        ptrChar = strstr(ptrString, "MOT+");
        if (ptrChar) PID[ServoSelect].MotorPolarityFlipped = false;
        
        ptrChar = strstr(ptrString, "ENC-");
        if (ptrChar) PID[ServoSelect].EncoderPolarityFlipped = true;
        
        ptrChar = strstr(ptrString, "ENC+");
        if (ptrChar) PID[ServoSelect].EncoderPolarityFlipped = false;
                
        ptrChar = strstr(ptrString, "MAX_VEL");
        if (ptrChar) 
        {
            ptrVal = strchr(ptrChar, ':');
            if (ptrVal)
            {
                ptrVal[0] = ' ';
                PID[ServoSelect].MaxVelocity = atof(ptrVal);
            }
        }                
        
        ptrChar = strstr(ptrString, "KP");
        if (ptrChar) 
        {
            ptrVal = strchr(ptrChar, ':');
            if (ptrVal)
            {
                ptrVal[0] = ' ';
                PID[ServoSelect].kP = atof(ptrVal);
            }
        }
                
        ptrChar = strstr(ptrString, "KI");
        if (ptrChar) 
        {
            ptrVal = strchr(ptrChar, ':');
            if (ptrVal)
            {
                ptrVal[0] = ' ';
                PID[ServoSelect].kI = atof(ptrVal);
            }
        }
                
        ptrChar = strstr(ptrString, "KD");
        if (ptrChar) 
        {
            ptrVal = strchr(ptrChar, ':');
            if (ptrVal)
            {
                ptrVal[0] = ' ';
                PID[ServoSelect].kD = atof(ptrVal);
            }
        }
                    
        ptrChar = strstr(ptrString, "OFF");
        if (ptrChar) 
        {
             ptrVal = strchr(ptrChar, ':');
             if (ptrVal)
             {
                ptrVal[0] = ' ';
                PID[ServoSelect].PWMoffset = atoi(ptrVal);
             }
        }
                      
        // if (DiagnosticsEnabled) printf("\rKP: %0.4f, KI: %0.4f, KD: %0.4f, OFF: %d", PID[ServoSelect].kP, PID[ServoSelect].kI, PID[ServoSelect].kD, PID[ServoSelect].PWMoffset);                
                
        ptrChar = strchr(ptrString, '$');                
        if (ptrChar)
        {
            length = strlen(ptrChar);
            for (i = 1; i < length; i++)
            {
                ch = ptrChar[i];
                if (!isalpha(ch) && !isdigit(ch) && ch != '_') break;
            }

            if (i < length && i > 0)
            {
                ptrChar[i] = '\0';
                length = strlen(ptrChar);
                PID[ServoSelect].alias = NULL;
                PID[ServoSelect].alias = malloc(sizeof(*PID[ServoSelect].alias) * length);
                strcpy(PID[ServoSelect].alias, ptrChar);                 
            }
        }             
        if (DiagnosticsEnabled) DisplayServoConfig(PID, ServoSelect);
    } // END if (ServoSelect >= 0
    return 0;
}

void DisplayServoConfig(struct PIDtype *PID, int ServoID)
{    
    if (ServoID >=0 && ServoID < NUMMOTORS)
    {
        if (!PID[ServoID].OpMode)
            printf("Servo #%d not used", ServoID);
        else
        {
            if (PID[ServoID].alias != NULL) printf("\r#%d %s: ", ServoID, PID[ServoID].alias);
            else printf("\r#%d: ", ServoID);
            
            if (PID[ServoID].OpMode == SERVO_POT_MODE) printf("POT SERVO ");
            else if (PID[ServoID].OpMode == SERVO_ENCODER_MODE) printf("ENC SERVO ");
            else if (PID[ServoID].OpMode == DESTINATION_MODE) printf("DEST ");
            else if (PID[ServoID].OpMode == CONTINUOUS_MODE) printf("CONT ");
            else printf("ERROR - INVALID CONTROL MODE");
            if (PID[ServoID].MotorPolarityFlipped) printf("MOT- ");
            else printf("MOT+ ");
            if (PID[ServoID].EncoderPolarityFlipped) printf("ENC- ");
            else printf("ENC+ ");     
            
            printf("\rKP: %0.4f, KI: %0.4f, KD: %0.4f", PID[ServoID].kP, PID[ServoID].kI, PID[ServoID].kD);
            printf("\rPWM OFFSET: %d, MAX VELOCITY: %0.3f\r", PID[ServoID].PWMoffset, PID[ServoID].MaxVelocity);
        }
    }
}

int GenerateMemMap(struct PIDtype *PID, BYTE *ptrMapBuffer)
{
    int i, length;
    BYTE tempString[256];
    
    ptrMapBuffer[0] = '\0';
    strcpy(ptrMapBuffer, ">EEPROM MEMORY MAP:\r");
    
    for (i = 0; i < NUMMOTORS; i++)
    {
        if (PID[i].OpMode > 0 && PID[i].OpMode < MAXMODE)
        {
            sprintf(tempString, "#%d: ", i);
            strcat (ptrMapBuffer, tempString);
            if (PID[i].alias != NULL && strlen(PID[i].alias))
            {
                strcat(ptrMapBuffer, PID[i].alias);
                strcat(ptrMapBuffer, ", ");
            }
            if (PID[i].OpMode == 1) strcat(ptrMapBuffer, "SERVO_POT_MODE, ");
            else if (PID[i].OpMode == 2) strcat(ptrMapBuffer, "SERVO_ENCODER_MODE, ");
            else if (PID[i].OpMode == 3) strcat(ptrMapBuffer, "DESTINATION_MODE, ");
            else if (PID[i].OpMode == 4) strcat(ptrMapBuffer, "CONTINUOUS_MODE, ");
            
            if (PID[i].MotorPolarityFlipped) strcat(ptrMapBuffer, "MOT-, ");
            else strcat(ptrMapBuffer, "MOT+, ");
            if (PID[i].EncoderPolarityFlipped) strcat(ptrMapBuffer, "ENC-, ");
            else strcat(ptrMapBuffer, "ENC+, ");
            
            sprintf(tempString, "KP: %0.3f, KI: %0.3f, KD: %0.3f, MAX_VEL: %0.3f, OFFSET: %d\r", PID[i].kP, PID[i].kI, PID[i].kD, PID[i].MaxVelocity, PID[i].PWMoffset);
            strcat(ptrMapBuffer, tempString);
        }        
    }
    strcat(ptrMapBuffer, "END<");
    length = strlen(ptrMapBuffer);
    return length;
}


void ConfigPIDFromEEprom (BYTE *ptrMemMapBuffer, BYTE **ptrPIDConfigList, struct PIDtype *PID)
{
    int NumBytesRead, numStrings, errors;
    
    NumBytesRead = FetchMemMapFromEEprom (ptrMemMapBuffer);
    if (NumBytesRead == 0) printf("\rERROR FetchMemMapFromEEprom"); 
    else
    {
        printf("\r\rMAP SIZE %d BYTES TOTAL:", NumBytesRead);
        numStrings = FillPIDConfigList(MemMapBuffer, PIDConfigList, MAX_MEMORY_STRINGS, FALSE);
        if (!numStrings) printf("\rERROR: NO STRINGS LOADED");
        else
        {                                
            errors = ConfigurePID (PID, PIDConfigList, numStrings, TRUE);
            if (errors) printf("\rInitializePID error: %d", errors);
            else printf("\rPID Servo configs initialized\r");                                
        }
    }
}


int ContinuousEncoderControl(short ServoID, struct PIDtype *PID)
{
    long Error;              
    long derError;       
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    static short displayCounter = 0;
    float MotorAcceleration;     
    
    GetEncoderPosition(ServoID, PID);
    
    MotorAcceleration = (PID[ServoID].MaxVelocity / INTERRUPTS_PER_SECOND) * 8;
    
    if (PID[ServoID].Halted)
    {
        PID[ServoID].PWMvalue = 0;
        return 0;
    }

    if (ServoID < 0)
    {
        PID[ServoID].Halted = true;
        PID[ServoID].PWMvalue = 0;
        return 0;
    }
    
    PID[ServoID].TargetVelocity =  (((float) PID[ServoID].PIDCommand) / 512) * PID[ServoID].MaxVelocity;
    if (PID[ServoID].TargetVelocity > PID[ServoID].MaxVelocity) 
        PID[ServoID].TargetVelocity = PID[ServoID].MaxVelocity;
    else if (PID[ServoID].TargetVelocity < 0 - PID[ServoID].MaxVelocity) 
        PID[ServoID].TargetVelocity = 0 - PID[ServoID].MaxVelocity;
    if (abs(PID[ServoID].TargetVelocity) < MINIMUM_VELOCITY) PID[ServoID].sumError = 0;
    
    if (PID[ServoID].Velocity < PID[ServoID].TargetVelocity) 
    {
        PID[ServoID].Velocity = PID[ServoID].Velocity + MotorAcceleration;
        if (PID[ServoID].Velocity > PID[ServoID].TargetVelocity)
            PID[ServoID].Velocity = PID[ServoID].TargetVelocity;
    }
    else if (PID[ServoID].Velocity > PID[ServoID].TargetVelocity)
    {
        PID[ServoID].Velocity = PID[ServoID].Velocity - MotorAcceleration;
        if (PID[ServoID].Velocity < PID[ServoID].TargetVelocity)
            PID[ServoID].Velocity = PID[ServoID].TargetVelocity;        
    }               
                              
    PID[ServoID].CommandPos = PID[ServoID].CommandPos + PID[ServoID].Velocity;    
    Error = PID[ServoID].CurrentPos - (long) PID[ServoID].CommandPos;
    
    derError = Error - PID[ServoID].error[PID[ServoID].errIndex];
    PID[ServoID].error[PID[ServoID].errIndex] = Error;
    PID[ServoID].errIndex++; 
    if (PID[ServoID].errIndex >= FILTERSIZE) PID[ServoID].errIndex = 0;                

    if (!PID[ServoID].saturation) PID[ServoID].sumError = PID[ServoID].sumError + (long)Error;
        
    
    PCorr = ((float)Error) * -PID[ServoID].kP;    
    ICorr = ((float)PID[ServoID].sumError) * -PID[ServoID].kI;
    DCorr = ((float)derError) * -PID[ServoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[ServoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[ServoID].PWMvalue = (long) (PIDcorrection - PID[ServoID].PWMoffset);            
    else PID[ServoID].PWMvalue = (long) (PIDcorrection + PID[ServoID].PWMoffset);                    
           
        
    if (PID[ServoID].PWMvalue >= PWM_MAX) 
    {
        PID[ServoID].PWMvalue = PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else if (PID[ServoID].PWMvalue <= -PWM_MAX) 
    {
        PID[ServoID].PWMvalue = -PWM_MAX;
        PID[ServoID].saturation = true;
    }
    else PID[ServoID].saturation = false;       
   
    
    if (ServoID == ServoSelect)
    {                 
        displayCounter++;
        if (PIDDisplayMode)
        {
            if (displayCounter >= 20)
            {                
                displayCounter = 0;      
                // printf("\r>#%d COM: %d, ERR: %d, P: %0.2f, I: %0.2f, D: %0.2f, PWM: %d ", ServoID, PID[ServoID].PIDCommand, Error, PCorr, ICorr, DCorr, PID[ServoID].PWMvalue);
                printf("\r>#%d POT: %d, COM: %0.1f, POS: %d, ERR: %d, PWM: %d ", ServoID, PID[ServoID].PIDCommand, PID[ServoID].CommandPos, PID[ServoID].CurrentPos, Error, PID[ServoID].PWMvalue);
            }
        }
    }       
    return 1;
}
