#ifndef __AT25M__
#define __AT25M__

#define RDSREG				(0x05)
#define LPWPOL				(0x08)
#define SETWEL				(0x06)
#define RSTWRL				(0x04)
#define WRSREG				(0x01)
#define RDDATA				(0x03)
#define WRDATA				(0x02)

#define EEPROM_CAPACITY		(131072u)	// 1Mbit
#define EEPROM_PAGESIZE		(256)
#endif
