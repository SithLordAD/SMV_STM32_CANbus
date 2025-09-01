#ifndef __SMV_BOARD_ENUMS_H
#define __SMV_BOARD_ENUMS_H

// enumerators for SMV boards.
// pick the appropriate one for your board
// every time you send a message, pick the appropriate data type for that message.

enum devices {
	HS1,
	HS2,
	HS3
};

enum types {
	PRESSURE,
	RPM
};

#endif
