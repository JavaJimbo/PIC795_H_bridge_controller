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
 ***********************************************************************************/
#define BRAKE_DISTANCE 90
#define DESTINATION_MULTIPLIER 1
#define DESTINATION_THRESHOLD (DESTINATION_MULTIPLIER * 20)
#define PID_ADJUST 1
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

#include "Delay.h"
#define _SUPPRESS_PLIB_WARNING

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = ON           // Watchdog Timer Enabled
#pragma config WDTPS =    PS8192        // Watchdog Timer Postscaler (1:8192) For 31,250 clock divided by 8192 = 262 mS timeout
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

#define TEST_OUT LATCbits.LATC1

typedef union
{
	unsigned char b[2];
	unsigned short integer;
} MConvertType;

// #define USE_BRAIN_BOARD  // COMMENT OUT to use CYTRON CONTROLLER BOARD
// #define REV2

enum
{
    FORWARD = 0,
    REVERSE
};

// MOTOR MODES
enum 
{
    SERVO_POT_MODE = 0,
    SERVO_ENCODER_MODE,
    DESTINATION_MODE,
    CONTINUOUS_MODE    
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
    LOCAL = 0,
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
#define FILTERSIZE 16

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'


#define NUM_LOCAL_POTS 4

#ifdef USE_BRAIN_BOARD
    #define MAXPOTS 8
    #define RS485_ENABLE LATBbits.LATB0
    #define PWM_DISABLE LATFbits.LATF1
    #define PWM_SPI_CS LATDbits.LATD7
#else
#define USE_FEATHER_BOARD
#ifdef REV2
    #define MAXPOTS 8
#else
    #define MAXPOTS 10
#endif

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

#ifdef USE_BRAIN_BOARD
    #define NUMMOTORS 4
#else 
    #ifdef REV2
        #define NUMMOTORS 4
    #else        
        #define NUMMOTORS 5
    #endif
#endif

#ifdef USE_BRAIN_BOARD
    #define FORWARD 1
    #define REVERSE 0
#else
    #define FORWARD 0
    #define REVERSE 1
#endif

// #define MAXSUM 500000


 


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
    #ifndef REV2
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
#define MOTOR_DIR2 LATDbits.LATD13
#define MOTOR_DIR3 LATDbits.LATD11
#define MOTOR_DIR4 LATAbits.LATA5

#ifndef USE_BRAIN_BOARD
    #ifndef REV2
        #define MOTOR_DIR5 LATDbits.LATD10
    #endif
#endif

#define EEPROM_WRITE_PROTECT LATDbits.LATD8


#ifdef USE_BRAIN_BOARD
    #define LED LATEbits.LATE6
#else
    #define LED LATDbits.LATD9
#endif


#ifndef USE_BRAIN_BOARD
#define SW1 PORTBbits.RB0
#define SW2 PORTBbits.RB1
#define SW3 PORTBbits.RB2
#define SW4 PORTCbits.RC13
    #ifndef REV2
        #define SW5 PORTDbits.RD7
    #endif
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
#define MAXBUFFER 1024

#define CONSTANT_FORWARD 9999
#define CONSTANT_REVERSE -999

struct PIDtype
{
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
    // BYTE DestinationState;
};




BYTE flagRemoteTimeout = false;
unsigned short offSet;
BYTE NUMbuffer[MAXNUM + 1];
BYTE HOSTRxBuffer[MAXBUFFER+1];
BYTE HOSTRxBufferFull = false;
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
void ResetPID(struct PIDtype *PID);
void ResetServoPID (short servoID, struct PIDtype *PID);
void InitPID(struct PIDtype *PID);
int ServoControl(short servoID, struct PIDtype *PID);
int DestinationEncoderControl(short servoID, struct PIDtype *PID);
int ContinuousEncoderControl(short servoID, struct PIDtype *PID);
void GetEncoderPosition(short servoID, struct PIDtype *PID);
BYTE processPacketData(short packetLength, BYTE *ptrPacket, short *numData, long *ptrData, BYTE *command, BYTE *subCommand, BYTE *errorCode);

unsigned short ADresult[MAXPOTS];
#ifndef USE_BRAIN_BOARD
    unsigned short SWRead = 0;
    BYTE SWChangeFlag = false;
#endif    
BYTE intFlag = false;
BYTE startPacket = false;
short XBEEtimeout = 0;

long ActualXBEEBaudRate, ActualHOSTBaudRate;

int main(void) 
{
    struct PIDtype PID[NUMMOTORS];
    short i = 0, j = 0, p = 0, q = 0;        
    long PWMvalue = 0;            
    float floValue;            
    unsigned ADdisplay = true;
    BYTE ch, temp, localCommand, remoteCommand, subCommand;
    short numDataIntegers;
    short packetCounter = 0;
    float tempCommand;
    long  ServoCommandPosition;    
    BYTE runState = HALTED;
    BYTE CommandMode = LOCAL;    
    BYTE errorCode = NO_ERROR;
    short LEDcounter = 0;
    short testCounter = 0;
    short anotherTestCounter = 0;
        
#ifdef USE_FEATHER_BOARD    
    #define NUM_RC_SERVOS 3
    short RCservoPos[NUM_RC_SERVOS];
    short PreviousRCservoPos[NUM_RC_SERVOS];    
    short RCServoID = 0;   
    unsigned char FeatherEnabled = FALSE;    
    
    
    for (i = 0; i < NUM_RC_SERVOS; i++) RCservoPos[i] = PreviousRCservoPos[i] = 0;
#endif    
         
    DelayMs(10);           
    InitializeSystem();

#ifdef USE_BRAIN_BOARD                
    PWM1 = PWM2 = PWM3 = PWM4 = 0;
#else
    #ifndef REV2                
        PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
    #else 
        PWM1 = PWM2 = PWM3 = PWM4 = 0;
    #endif
#endif   
       
        
#ifdef USE_BRAIN_BOARD    
    printf("\r\rSTART BRAIN BOARD VERSION #0 NO RC SERVOS");
#else
    printf("\r\rPIC795 H BRIDGE CONTROLLER COMPLETE #1");
#endif
    
    InitPID(PID);
    
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
#endif    
    
    printf("\rHOST Baudrate: %ld", ActualHOSTBaudRate);        
    printf("\rXBEE Baudrate: %ld", ActualXBEEBaudRate);
    
    if (CommandMode==LOCAL) printf("\rCommand Mode = LOCAL");
    else if (CommandMode==REMOTE) printf("\rCommand Mode = REMOTE");
    else printf("\rCommand Mode = Error");    
    
    if (runState == HALTED) printf("\rRUN STATE: HALTED");
    else printf("\rRUN STATE: ON");

    while(1) 
    {   
        ClrWdt(); // CLEAR_WATCHDOG
        
        if (XBEEPacketLength)   
        {         
            if ( processPacketData(XBEEPacketLength, XBEEPacket, &numDataIntegers, XBEEData, &remoteCommand, &subCommand, &errorCode))
            {
                timeout = 5000;                  
                temp = remoteCommand & 0xF0;
                if (temp == 0xB0 && subCommand >= 0)
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
                        if (XBEEDisplayMode) printf("\rREMOTE #%d %02X: SUB #%d: XBEE: %d, SERVO: %d, PID COM: %d, VAL: %d", packetCounter++, remoteCommand, subCommand, XBEEData[0], ServoCommandPosition, PID[subCommand].PIDCommand,  PID[subCommand].RemoteDataValid);                        
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
                    if (!PID[i].Halted)
                    {                        
                        if (CommandMode == LOCAL) 
                        {
#ifdef USE_BRAIN_BOARD                                    
                            tempCommand = (float) ADresult[i];  
#else                  
    #ifndef REV2             
                            tempCommand = (float) ADresult[i+6];                                                         
    #else                                                                                                   
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
                        PWM3 = (unsigned short)PWMvalue;
                    }
                    else if (i == 3)  // Motor #4
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR4 = REVERSE;  
                            PWMvalue = 0 - PWMvalue;
                        }
                        else
                        {
                            MOTOR_DIR4 = FORWARD;
                        }
                        PWM4 = (unsigned short)PWMvalue;  
                    }
#ifndef USE_BRAIN_BOARD     
    #ifndef REV2                     
                    else
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR5 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR5 = FORWARD;                            
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
    #ifndef REV2                 
                PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
    #else
                PWM1 = PWM2 = PWM3 = PWM4 = 0;
    #endif
#endif
            }                                
        } // End if intFlag
    
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;             
            q = 0;
            localCommand = 0;            
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (ch < ' ' && ch != '\r')
                {
                    localCommand = ch;
                    break;
                }
                if (isalpha(ch) || (ch==' ' && p==0)) localCommand = ch;                
                putchar(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( (isdigit(ch) || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof(NUMbuffer);
            }
            if (localCommand) 
            {
                switch (localCommand) 
                {
                    case 'P':
                        if (q) PID[PID_ADJUST].kP = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[PID_ADJUST].kP, PID[PID_ADJUST].kI, PID[PID_ADJUST].kD, PID[PID_ADJUST].PWMoffset);
                        break;
                    case 'I':
                        if (q) PID[PID_ADJUST].kI = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[PID_ADJUST].kP, PID[PID_ADJUST].kI, PID[PID_ADJUST].kD, PID[PID_ADJUST].PWMoffset);
                        break;
                    case 'D':
                        if (q) PID[PID_ADJUST].kD = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[PID_ADJUST].kP, PID[PID_ADJUST].kI, PID[PID_ADJUST].kD, PID[PID_ADJUST].PWMoffset);
                        break;
                    case 'O':
                        if (q) PID[PID_ADJUST].PWMoffset = (long) floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[PID_ADJUST].kP, PID[PID_ADJUST].kI, PID[PID_ADJUST].kD, PID[PID_ADJUST].PWMoffset);
                        break;
                    case 'R':
                        CommandMode = REMOTE;
                        runState = HALTED;
                        printf("REMOTE COMMAND MODE");
                        break;
                    case 'L':
                        CommandMode = LOCAL;
                        runState = HALTED;
                        printf("LOCAL COMMAND MODE");
                        break;       
                    case ' ':
                        ResetPID(PID);
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
                            else printf("ERROR Command Mode = %d", CommandMode);
                        }                         
                        break;                        
                    case 'M':
                        if (PIDDisplayMode)
                        {
                            PIDDisplayMode = false;
                            printf("\rPID Display OFF");
                        }
                        else {
                            PIDDisplayMode = true;
                            printf("\rPID Display ON");
                        }   
                        break;                        

                    case 'X':
                        if (XBEEDisplayMode)
                        {
                            XBEEDisplayMode = false;
                            printf("\rXBEE Display OFF");
                        }
                        else {
                            XBEEDisplayMode = true;
                            printf("\rXBEE Display ON");
                        }   
                        break;                        
                        
                    case 'Q':
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[PID_ADJUST].kP, PID[PID_ADJUST].kI, PID[PID_ADJUST].kD, PID[PID_ADJUST].PWMoffset);
                        break;                        
                    case 'T':
                        if (ADdisplay)
                        {
                            ADdisplay = false;
                            printf("\rPot display OFF");
                        }
                        else
                        {
                            ADdisplay = true;
                            printf("\rPot display ON");
                        }
                        break;                        
                    case 'V':
                        if (q) PID[PID_ADJUST].Velocity = (float)floValue;
                        printf("Velocity: %0.3f", PID[3].Velocity);                        
                        break;                        
                    case 'Z':
                        if (q) PID[PID_ADJUST].Destination = (long)floValue;                        
                        printf("Destination: %ld, Velocity: %0.3f", PID[PID_ADJUST].Destination, PID[PID_ADJUST].Velocity);
                        break;                        
                    default:                        
                        if (localCommand < ' ') printf("\rCommand: #%d => %c", localCommand, (localCommand - 1 + 'A'));
                        else printf("\rCommand: %c", localCommand);
                        break;
                } // end switch localCommand                
                
                CONTINUE1: continue;
                localCommand = 0;
            } // End if (localCommand)
        } // End if HOSTRxBufferFull        
    } // End while(1))
} // End main())


#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
BYTE ch, inByte;
static unsigned long i = 0;

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
            if (ch != 0 && ch != '\n') 
            {            
                if ('\r'==ch || (' '==ch && i==0)) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[i++] = ch;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\r');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\n');
                }                
                else if (ch < ' ')
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[0] = ch;
                    HOSTRxBuffer[1] = '\0';
                    i = 0;
                }
                else if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

BYTE processPacketData(short packetLength, BYTE *ptrPacket, short *numData, long *ptrData, BYTE *command, BYTE *subCommand, BYTE *errorCode)
{
    MConvertType dataValue;    
    short j, i = 0;  
    BYTE packetDataLength, packetBytes[MAXBUFFER];
    
    *errorCode = NO_ERROR; // Reset error flag to start with.
    
    packetDataLength = decodePacket(ptrPacket, packetBytes, errorCode);    
    if (packetDataLength == 0) return false;    
    
    i = 0;
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
    
    mT2ClearIntFlag(); // clear the interrupt flag        
    
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
    SYSTEMConfigPerformance(80000000);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Configure PIC ADC for ten AD input channels
    ADC10_ManualInit();    
    
    // Set up UART Rx DMA interrupts
    // SetupDMA_Rx();
    
    // I/O Ports:
#ifdef USE_BRAIN_BOARD
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

#ifndef USE_BRAIN_BOARD     
    // Set up PWM OC5
    #ifndef REV2
        OC5CON = 0x00;
        OC5CONbits.OC32 = 0; // 16 bit PWM
        OC5CONbits.ON = 1; // Turn on PWM
        OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
        OC5CONbits.OCM2 = 1; // PWM enabled, no fault pin
        OC5CONbits.OCM1 = 1;
        OC5CONbits.OCM0 = 0;
        OC5RS = 0;         
    #endif
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

void ResetPID(struct PIDtype *PID)
{
    int i = 0;
    for (i = 0; i < NUMMOTORS; i++) ResetServoPID (i, PID);
}






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
    #ifndef REV2
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
    // When using REV 2 MD13S Grove controller board:
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
    
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{    
    // Step #1 - always clear the mismatch condition first
    SWRead = 0x0000;
    SWRead = PORTB & 0b0000000000000111; // Read RB0, RB1, RB2, mask off the rest of Port B
    if (PORTC & 0b0010000000000000) // Read RC13, mask off the rest of Port C
        SWRead = SWRead | 0b1000;
    SWChangeFlag = true;
    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
}
#endif




void GetEncoderPosition(short servoID, struct PIDtype *PID)
{
    BYTE EncoderDirection;
    
        if (servoID == 1)
    {
        EncoderDirection = EncoderOneDir;    
        
        if (EncoderDirection) 
            PID[servoID].CurrentPos = PID[servoID].CurrentPos + (long)EncoderOne;
        else
            PID[servoID].CurrentPos = PID[servoID].CurrentPos - (long)EncoderOne;
        EncoderOne = 0;
    }    
    else if (servoID == 2)
    {
        EncoderDirection = EncoderTwoDir;
        if (EncoderDirection) 
            PID[servoID].CurrentPos = PID[servoID].CurrentPos + (long)EncoderTwo;
        else
            PID[servoID].CurrentPos = PID[servoID].CurrentPos - (long)EncoderTwo;
        EncoderTwo = 0;
    }    
    else if (servoID == 3)
    {
        EncoderDirection = EncoderThreeDir;
        if (EncoderDirection) 
            PID[servoID].CurrentPos = PID[servoID].CurrentPos + (long)EncoderThree;
        else
            PID[servoID].CurrentPos = PID[servoID].CurrentPos - (long)EncoderThree;
        EncoderThree = 0;
    }    
    else
    {
        EncoderDirection = EncoderFourDir;
        if (EncoderDirection) 
            PID[servoID].CurrentPos = PID[servoID].CurrentPos + (long)EncoderFour;
        else
            PID[servoID].CurrentPos = PID[servoID].CurrentPos - (long)EncoderFour;
        EncoderFour = 0;
    }       
}


#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023
#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86

int ServoControl(short servoID, struct PIDtype *PID)
{
    short Error;              
    long totalDerError = 0;
    long derError;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    short i;
    static short displayCounter = 0;    
    unsigned short QuadReading = 0;
    
    if (PID[servoID].OpMode == SERVO_POT_MODE)
    {    
 #ifdef USE_BRAIN_BOARD 
        PID[servoID].CurrentPos = (short)(ADresult[servoID+4]);
#else
        PID[servoID].CurrentPos = (short)(ADresult[servoID+1]);
#endif               
        if (PID[servoID].CurrentPos < QUAD_ONE) QuadReading = QUAD_ONE;
        else if (PID[servoID].CurrentPos < QUAD_TWO) QuadReading = QUAD_TWO;
        else QuadReading = QUAD_THREE;
    
        if (PID[servoID].PreviousQuad == NO_QUAD) 
            PID[servoID].PreviousQuad = QuadReading;
        else if (QuadReading == QUAD_TWO)
            PID[servoID].PreviousQuad = QUAD_TWO;
        else if (PID[servoID].PreviousQuad == QUAD_TWO)
            PID[servoID].PreviousQuad = QuadReading;
        else if (PID[servoID].PreviousQuad == QUAD_ONE)
        {
            if (QuadReading == QUAD_THREE) 
                PID[servoID].CurrentPos = PID[servoID].CurrentPos - (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
        }
        else if (PID[servoID].PreviousQuad == QUAD_THREE)
        {
            if (QuadReading == QUAD_ONE) 
                PID[servoID].CurrentPos = PID[servoID].CurrentPos + (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
        }
    }
    else if (PID[servoID].OpMode == SERVO_ENCODER_MODE)
        GetEncoderPosition(servoID, PID);
    else
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    Error = PID[servoID].CurrentPos - PID[servoID].PIDCommand;            
    PID[servoID].error[PID[servoID].errIndex] = Error;
    PID[servoID].errIndex++; 
    if (PID[servoID].errIndex >= FILTERSIZE) PID[servoID].errIndex = 0;   
    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[servoID].error[i];
    derError = totalDerError / FILTERSIZE;    
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
         
    if (PID[servoID].PWMvalue > PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;        
        
        
        displayCounter++; 
        if (servoID == PID_ADJUST)
        {            
            if (displayCounter >= 40 && PIDDisplayMode)
            {   
                if (PID[servoID].PreviousQuad == QUAD_ONE) printf("\rQI ");
                else if (PID[servoID].PreviousQuad == QUAD_TWO) printf("\rQII ");
                else printf("\rQIII ");
                printf("COM: %d, ROT: %d, ACT: %d ERR: %d P: %0.1f I: %0.1f PWM: %d ", PID[servoID].PIDCommand, PID[servoID].CurrentPos, PID[servoID].CurrentPos, Error, PCorr, ICorr, PID[servoID].PWMvalue);
                displayCounter = 0;
            }
        }
        
    return 1;
}

int DestinationEncoderControl(short servoID, struct PIDtype *PID)
{
    long Error;                  
    long derError;       
    float PCorr = 0, ICorr = 0, DCorr = 0, PIDcorrection = 0;    
    static short displayCounter = 0;        
    float Acceleration, Deacceleration;          
    static enableDisplay = true;
    long RampLength = 0;
    
    if (PID[servoID].OpMode != DESTINATION_MODE)
    {
        PID[servoID].Halted = true;
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    
    if (PID[servoID].Halted)
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    
    GetEncoderPosition(servoID, PID);
    
    Acceleration = ((PID[servoID].MaxVelocity / INTERRUPTS_PER_SECOND)) * 8;
    Deacceleration = Acceleration / 2;
    

    if (servoID < 0)
    {
        PID[servoID].Halted = true;
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    
    PID[servoID].Destination =   PID[servoID].PIDCommand;
    if (PID[servoID].RampState == RAMP_START)
    {
        PID[servoID].RampUpStartPosition = PID[servoID].CurrentPos;
        PID[servoID].RampDownStartPosition = PID[servoID].Destination; // Temporary
        if (PID[servoID].Destination > PID[servoID].CurrentPos) 
        {
            PID[servoID].Direction = FORWARD;
            PID[servoID].TargetVelocity = PID[servoID].MaxVelocity;
        }
        else
        {
            PID[servoID].Direction = REVERSE;
            PID[servoID].TargetVelocity = 0 - PID[servoID].MaxVelocity;
        }
        PID[servoID].RampState++;
    }     
        
    if (PID[servoID].RampState == RAMP_UP)
    {            
        if (PID[servoID].Direction == FORWARD)
        {
            PID[servoID].Velocity = PID[servoID].Velocity + Acceleration;
            if(PID[servoID].Velocity >= PID[servoID].TargetVelocity)
            {
                PID[servoID].Velocity = PID[servoID].TargetVelocity;
                RampLength = (PID[servoID].CurrentPos - PID[servoID].RampUpStartPosition) * 2;
                PID[servoID].RampDownStartPosition = PID[servoID].Destination - RampLength;
                PID[servoID].RampState++;       
            }
        }
        else
        {
            PID[servoID].Velocity = PID[servoID].Velocity - Acceleration;
            if(PID[servoID].Velocity <= PID[servoID].TargetVelocity)
            {
                PID[servoID].Velocity = PID[servoID].TargetVelocity;
                RampLength = (PID[servoID].RampUpStartPosition - PID[servoID].CurrentPos) * 2;
                PID[servoID].RampDownStartPosition = PID[servoID].Destination + RampLength;
                PID[servoID].RampState++;       
            }                
        }                
    }   
    else if (PID[servoID].RampState == RAMP_RUN)
    {
        if (PID[servoID].Direction == FORWARD)
        {
            if (PID[servoID].CurrentPos >= PID[servoID].RampDownStartPosition)
                   PID[servoID].RampState++;
        }
        else
        {
            if (PID[servoID].CurrentPos <= PID[servoID].RampDownStartPosition)
                PID[servoID].RampState++;                
        }
    }
    else if (PID[servoID].RampState == RAMP_DOWN)
    {
        if (PID[servoID].Direction == FORWARD)
        {
            PID[servoID].Velocity = PID[servoID].Velocity - Deacceleration;
            if (PID[servoID].Velocity < PID[servoID].MinVelocity) PID[servoID].Velocity = PID[servoID].MinVelocity;
        }
        else
        {
            PID[servoID].Velocity = PID[servoID].Velocity + Deacceleration;
            if (PID[servoID].Velocity > PID[servoID].MinVelocity) PID[servoID].Velocity = PID[servoID].MinVelocity;
        }
        if (PID[servoID].Velocity == PID[servoID].MinVelocity) PID[servoID].RampState++;
    }             
    
    PID[servoID].CommandPos = PID[servoID].CommandPos + PID[servoID].Velocity;   
    
    Error = PID[servoID].CurrentPos - (long) PID[servoID].CommandPos;    
    derError = Error - PID[servoID].error[PID[servoID].errIndex];
    
    PID[servoID].error[PID[servoID].errIndex] = Error;
    PID[servoID].errIndex++; 
    if (PID[servoID].errIndex >= FILTERSIZE) PID[servoID].errIndex = 0;                

    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error;
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError) * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;
    if (PID[servoID].OpMode == DESTINATION_MODE && PID[servoID].RampState != RAMP_DOWN)
        DCorr = 0;
    

    PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
        
    if (PID[servoID].PWMvalue >= PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue <= -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;       
    
    if (PID[servoID].RampState == RAMP_HALT)
    {
        PID[servoID].PWMvalue = 0;
        PID[servoID].Halted = true;        
    }    
    
    if (servoID == PID_ADJUST)
    {                 
        displayCounter++;
        if (PIDDisplayMode)
        {
            if (displayCounter >= 10 || PID[servoID].Halted)
            {                
                displayCounter = 0;      
                if (enableDisplay)
                {
                    if (PID[servoID].Halted) printf("\rHALT: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[servoID].Destination, PID[servoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[servoID].PWMvalue);                                                
                    else if (PID[servoID].RampState == RAMP_UP) printf("\rRAMP: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[servoID].Destination, PID[servoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[servoID].PWMvalue);
                    else if (PID[servoID].RampState == RAMP_RUN) printf("\rRUN: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[servoID].Destination, PID[servoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[servoID].PWMvalue);                        
                    else if (PID[servoID].RampState == RAMP_DOWN) printf("\rBRAKE: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[servoID].Destination, PID[servoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[servoID].PWMvalue);
                    else printf("\rHALT: %d, POS: %d, ERR: %d, P: %d, I: %d, D: %d, PWM: %d ", PID[servoID].Destination, PID[servoID].CurrentPos, Error, (long)PCorr, (long)ICorr, (long)DCorr, PID[servoID].PWMvalue);
                 }
            }
        }
    }       

    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;

    return 1;
}

void ResetServoPID (short servoID, struct PIDtype *PID)
{
    int j;
    
    // printf("\rRESET %d", servoID);
    EncoderOne = 0;
    EncoderTwo = 0;
    EncoderThree = 0;
    EncoderFour = 0;

        PID[servoID].sumError = 0; // For 53:1 ratio Servo City motor
        PID[servoID].errIndex = 0;
        PID[servoID].PWMvalue = 0;        
        PID[servoID].PIDCommand = 0;        
        PID[servoID].saturation = false;
        PID[servoID].Velocity = 0;
        PID[servoID].CurrentPos = 0;
        PID[servoID].CommandPos = 0;
        PID[servoID].Halted = false;
        PID[servoID].RemoteDataValid = false;        
        if (servoID == 1) 
        {
                PID[servoID].OpMode = CONTINUOUS_MODE;
                PID[servoID].MaxVelocity = MAX_CYTRON_VELOCITY;
        }                
        else if (servoID == 3) 
        {
            PID[servoID].OpMode = SERVO_ENCODER_MODE;
            PID[servoID].MaxVelocity = MAX_26_VELOCITY / 2;                                    
        }                
        else PID[servoID].OpMode = SERVO_POT_MODE;
        
        PID[servoID].Velocity = 0;
        PID[servoID].TargetVelocity = 0;
        PID[servoID].MinVelocity = 0; // PID[servoID].MaxVelocity / 4;        
        PID[servoID].Destination = 0;
        PID[servoID].RampUpStartPosition = 0;
        PID[servoID].RampDownStartPosition = 0;
        PID[servoID].PreviousQuad = NO_QUAD;
        PID[servoID].Velocity = 0;       
        PID[servoID].RampState = RAMP_START;
        PID[servoID].Direction = FORWARD;
        for (j = 0; j < FILTERSIZE; j++) PID[servoID].error[j] = 0;
}

void InitPID(struct PIDtype *PID)
{
    ResetPID(PID);
    int i;
    for (i = 0; i < NUMMOTORS; i++)
    {                       
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
               
    }    
}

int ContinuousEncoderControl(short servoID, struct PIDtype *PID)
{
    long Error;              
    long derError;       
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    static short displayCounter = 0;
    float MotorAcceleration;     
    
    GetEncoderPosition(servoID, PID);
    
    MotorAcceleration = (PID[servoID].MaxVelocity / INTERRUPTS_PER_SECOND) * 8;
    
    if (PID[servoID].Halted)
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }

    if (servoID < 0)
    {
        PID[servoID].Halted = true;
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    
    PID[servoID].TargetVelocity =  (((float) PID[servoID].PIDCommand) / 512) * PID[servoID].MaxVelocity;
    if (PID[servoID].TargetVelocity > PID[servoID].MaxVelocity) 
        PID[servoID].TargetVelocity = PID[servoID].MaxVelocity;
    else if (PID[servoID].TargetVelocity < 0 - PID[servoID].MaxVelocity) 
        PID[servoID].TargetVelocity = 0 - PID[servoID].MaxVelocity;
    if (abs(PID[servoID].TargetVelocity) < MINIMUM_VELOCITY) PID[servoID].sumError = 0;
    
    if (PID[servoID].Velocity < PID[servoID].TargetVelocity) 
    {
        PID[servoID].Velocity = PID[servoID].Velocity + MotorAcceleration;
        if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;
    }
    else if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
    {
        PID[servoID].Velocity = PID[servoID].Velocity - MotorAcceleration;
        if (PID[servoID].Velocity < PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;        
    }               
                              
    PID[servoID].CommandPos = PID[servoID].CommandPos + PID[servoID].Velocity;    
    Error = PID[servoID].CurrentPos - (long) PID[servoID].CommandPos;
    
    derError = Error - PID[servoID].error[PID[servoID].errIndex];
    PID[servoID].error[PID[servoID].errIndex] = Error;
    PID[servoID].errIndex++; 
    if (PID[servoID].errIndex >= FILTERSIZE) PID[servoID].errIndex = 0;                

    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error;
        
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError) * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
        
    if (PID[servoID].PWMvalue >= PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue <= -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;       
   
    
    if (servoID == PID_ADJUST)
    {                 
        displayCounter++;
        if (PIDDisplayMode)
        {
            if (displayCounter >= 20)
            {                
                displayCounter = 0;      
                printf("\r>#%d: PID COM: %d, ERR: %d, P: %0.2f, I: %0.2f, D: %0.2f, PWM: %d ", servoID, PID[servoID].PIDCommand, Error, PCorr, ICorr, DCorr, PID[servoID].PWMvalue);
            }
        }
    }       
    
    
    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;

    return 1;
}
