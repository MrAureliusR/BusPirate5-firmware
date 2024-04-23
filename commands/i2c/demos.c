#include <stdio.h>
#include "pico/stdlib.h"
#include <stdint.h>
#include "pirate.h"
#include "pirate/hwi2c_pio.h"
#include "ui/ui_term.h"
#include "system_config.h"
#include "opt_args.h"
#include "bytecode.h"
#include "mode/hwi2c.h"
#include "ui/ui_help.h"
#include "ui/ui_cmdln.h"
#include "lib/ms5611/ms5611.h"
#include "lib/tsl2561/driver_tsl2561.h" 

void demo_tsl2561(struct command_result *res){
	//select register [0b01110010 0b11100000]
	// start device [0b01110010 3]
	//confirm start [0b01110011 r]
	// select ID register [0b01110010 0b11101010]
	// read ID register [0b01110011 r] 7:4 0101 = TSL2561T 3:0 0 = revision
	// select ADC register [0b01110010 0b11101100]
	//0b11011100
	uint16_t chan0, chan1;
	char data[4];
	printf("%s\r\n",t[T_HELP_I2C_TSL2561]);
	//select register [0b01110010 0b11100000]
	data[0]=0b11100000;
	if(pio_i2c_write_blocking_timeout( 0b01110010, data, 1, 0xffff)) return;
	// start device [0b01110010 3]
	data[0]=3;
	if(pio_i2c_write_blocking_timeout( 0b01110010, data, 1, 0xffff)) return;
	busy_wait_ms(500);
	// select ID register [0b01110010 0b11101010]
	// read ID register [0b01110011 r] 7:4 0101 = TSL2561T 3:0 0 = revision
	data[0]=0b11101010;
	if(pio_i2c_transaction_blocking_timeout( 0b01110010, data, 1, data, 1, 0xffff)) return;	
	printf("ID: %d REV: %d\r\n", data[0]>>4, data[0]&0b1111);
	// select ADC register [0b01110010 0b11101100]	
	data[0]=0b11101100;
	if(pio_i2c_transaction_blocking_timeout( 0b01110010, data, 1, data, 4, 0xffff)) return;	
	chan0=data[1]<<8|data[0];
	chan1=data[3]<<8|data[2];

	uint32_t lux1=a_tsl2561_calculate_lux(0, 2,chan0, chan1);

	printf("Chan0: %d Chan1: %d LUX: %d\r\n", chan0, chan1, lux1);
}

void demo_ms5611(struct command_result *res){
	//PS high, CSB low
	//reset [0b11101110 0b00011110]
	//PROM read [0b11101110 0b10100110] [0b11101111 r:2]
	//start conversion [0b11101110 0b01001000]
	//ADC read [0b11101110 0] [0b11101111 r:3]
	float temperature;
	float pressure;
	printf("%s\r\n",t[T_HELP_I2C_MS5611]);
	if(ms5611_read_temperature_and_pressure_simple( &temperature, &pressure)){
		res->error=1;
		return;
	}
	printf("Temperature: %f\r\nPressure: %f\r\n", temperature, pressure);
}

void demo_si7021(struct command_result *res){
	uint8_t data[8];
	printf("%s\r\n",t[T_HELP_I2C_SI7021]);
	// humidity
	data[0]=0xf5;
	if(pio_i2c_write_blocking_timeout( 0x80, data, 1, 0xffff)) return;
	busy_wait_ms(23); //delay for max conversion time
	if(pio_i2c_read_blocking_timeout( 0x81, data, 2, 0xffff)) return;

	float f=(float)((float)(125*(data[0]<<8 | data[1]))/65536)-6;
	printf("Humidity: %.2f%% (%#04x %#04x) ", f, data[0], data[1]);

	// temperature [0x80 0xe0] [0x81 r:2]	
	data[0]=0xf3;
	if(pio_i2c_write_blocking_timeout( 0x80, data, 1, 0xffff)) return;
	busy_wait_ms(100); //delay for max conversion time
	if(pio_i2c_read_blocking_timeout( 0x81, data, 2, 0xffff)) return;

	f=(float)((float)(175.72*(data[0]<<8 | data[1]))/65536)-46.85;
	printf("Temperature: %.2fC (%#04x %#04x)\r\n", f, data[0], data[1]);

	//SN
	data[0]=0xfa;
	data[1]=0xf0;
	uint8_t sn[8];
	if(pio_i2c_transaction_blocking_timeout( 0x80, data, 2, data, 8, 0xffff)) return;
	sn[2]=data[6];
	sn[3]=data[4];
	sn[4]=data[2];
	sn[5]=data[0];

	data[0]=0xfc;
	data[1]=0xc9;
	if(pio_i2c_transaction_blocking_timeout( 0x80, data, 2, data, 6, 0xffff)) return;
	sn[0]=data[1];
	sn[1]=data[0];
	sn[6]=data[4];
	sn[7]=data[3]; 	
	printf("Serial Number: 0x%02x%02x%02x%02x%02x%02x%02x%02x\r\n", sn[7],sn[6],sn[5],sn[4],sn[3],sn[2],sn[1],sn[0]);
}

void demo_ssd1306(struct *command_result res) {
	uint8_t data[25];
	const uint8_t dp_logo = { 0x00 }; // Dangerous Prototypes logo will be here
	const uint8_t command_prefix = 0x00;
	const uint8_t data_prefix = 0x40;
	const uint8_t ssd1306_addr = 0x3C; // 7-bit address
	// go through initial setup
	data[0] = 0xAE; // display off
	data[1] = 0xD5; // divide ratio & osc freq to..
	data[2] = 0x80; // defaults
	data[3] = 0xA8; // multiplex ratio to..
	data[4] = 0x1F; // 32 MUX
	data[5] = 0xD3; // vertical shift to...
	data[6] = 0x00; // none
	data[7] = 0x40; // display RAM start line is 0
	data[8] = 0x8D;
	data[9] = 0x14;
	data[10] = 0x20; // addressing mode to...
	data[11] = 0x00; // horizontal addressing
	data[12] = 0xA1; // segment remap
	data[13] = 0xC8; // reverse scan direction
	data[14] = 0xDA; // set COM mapping to...
	data[15] = 0x02; // regular mapping
	data[16] = 0x81; // set contrast to...
	data[17] = 0x2F; // about 25% (really brightness, not contrast)
	data[18] = 0xD9; // set precharge value to...
	data[19] = 0xF1; // the recommended value
	data[20] = 0xD8; // set VCOMH deselect leve to...
	data[21] = 0x40; // the recommended value
	data[22] = 0xA4; // do not fill the display with white 
	data[23] = 0xA6; // non-inverted display (normal display mode)
	data[24] = 0xAF; // enable display

	printf("Initializing display...\r\n");
	uint8_t builtCommand[2] = { command_prefix, 0x00 };
	// Send the entire initialization sequence
	for(int i = 0; i < 25; i++) {
		builtCommand[1] = data[i];
		if(pio_i2c_write_blocking_timeout(ssd1306_addr, builtCommand, 2, 0xffff)) return;
	}
	printf("Sending example bitmap...\r\n");
	builtCommand[0] = data_prefix;
	for(int i = 0; i < 512; i++) {
		// send the DP logo
	}
	
}
