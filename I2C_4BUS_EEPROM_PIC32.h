/* 
 * File:   I2C_4BUS_EEPROM_PIC32.h
 * Author: Jim
 *
 * Created on August 14, 2019, 8:52 AM
 */

#include "GenericTypeDefs.h"
#define SYS_FREQ 80000000
#define GetPeripheralClock() SYS_FREQ
#ifndef I2C_4BUS_EEPROM_PIC32_H
#define	I2C_4BUS_EEPROM_PIC32_H

// #define TEST_OUT LATEbits.LATE3
#define EEBLOCKSIZE 64
#define EEPROM_ID 0xA0
#define EEPROM_WP PORTDbits.RD8

#define UINT32 unsigned long
#define BOOL unsigned char
#define UINT8 unsigned char

unsigned char WriteEEpromBlock (unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes);
unsigned char ReadEEpromBlock (unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes);
unsigned char WriteEEpromByte (unsigned char busID, unsigned char device, unsigned short address, unsigned char data);
unsigned char ReadEEpromByte (unsigned char busID, unsigned char device, unsigned short address, unsigned char *ptrData);
unsigned char I2CReceiveByte(unsigned char busID, unsigned char NACKflag);

void initI2C(unsigned char busID);
BOOL StartTransfer(unsigned char busID, BOOL restart);
BOOL TransmitOneByte(unsigned char busID, unsigned char data);
void StopTransfer(unsigned char busID);
void print_status(unsigned char busID);
unsigned char I2CGetACK(unsigned char busID);

#define EEBUS I2C1

#endif	/* I2C_4BUS_EEPROM_PIC32_H */

