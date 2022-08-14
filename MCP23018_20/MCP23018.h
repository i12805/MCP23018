#ifndef MCP23018_h
#define MCP23018_h

/*       reg    bank=0   bank=1 */
#define IODIRA	(0x00) //(0x00) 
#define IODIRB	(0x01) //(0x10) 
#define IPOLA	(0x02) //(0x01) 
#define IPOLB	(0x03) //(0x11) 
#define GPINTENA	(0x04) //(0x02) 
#define GPINTENB	(0x05) //(0x12) 
#define DEFVALA	(0x06) //(0x03) 
#define DEFVALB	(0x07) //(0x13) 
#define INTCONA	(0x08) //(0x04) 
#define INTCONB	(0x09) //(0x14) 
#define IOCON	(0x0A) //(0x05) 
//#define IOCON	(0x0B) //(0x15) reg is unique for the device 
#define GPPUA	(0x0C) //(0x06) 
#define GPPUB	(0x0D) //(0x16) 
#define INTFA	(0x0E) //(0x07) 
#define INTFB	(0x0F) //(0x17) 
#define INTCAPA	(0x10) //(0x08) 
#define INTCAPB	(0x11) //(0x18) 
#define GPIOA	(0x12) //(0x09) 
#define GPIOB	(0x13) //(0x19) 
#define OLATA	(0x14) //(0x0A) 
#define OLATB	(0x15) //(0x1A) 


#endif
