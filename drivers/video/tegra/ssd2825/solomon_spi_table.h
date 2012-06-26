/*
 * Xilinx SPI device driver API and platform data header file
 *
 * Copyright (c) 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _SPI_SOLOMON_TABLE_H_
#define _SPI_SOLOMON_TABLE_H_

#define MIPI_NON_BURST_MODE  //default(2/27)
//#define HITACHI_CABC_ENABLE
//#define CURRENT_BIAS_MIPI_OUTPUT  //default : no use(2/27)
#define HITACHI_GAMMA_S_CURVE

#include "./ssd2825_bridge.h"

typedef struct spi_cmd_data16{
	unsigned short delay;
	unsigned short value;
};

struct spi_cmd_data16 solomon_init_sequence_set[] = {
		/*also need to change solomon_reg_read_set*/
		// 0xB1 : 0x04, 0x04
		{0, 0x00B1},
		{0, 0x0104},
		{0, 0x0101},
		// 0xB2 : 0x3A, 0x0E
#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B2},
		{0, 0x0142}, // default Non-burst mode:{0, 0x013E},
		{0, 0x0104}, // default Non-burst mode:{0, 0x0103},
#else
		{0, 0x00B2},
		{0, 0x0142}, // burst mode to reduce power consumption:{0, 0x011E+4(sync_width)},
		{0, 0x0104}, // burst mode to reduce power consumption:{0, 0x0103+1(sync_width)},
#endif
		// 0xB3 : 0x08, 0x08
		{0, 0x00B3},
		{0, 0x015C}, //{0, 0x0188}
		{0, 0x0106}, //{0, 0x0104},

		// 0xB4 : 0xD0, 0x02
		{0, 0x00B4},
		{0, 0x01D0},
		{0, 0x0102},

		{0, 0x00B5},
		{0, 0x0100},
		{0, 0x0105},
#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B6},
		{0, 0x0107}, // default Non-burst mode:{0, 0x0103},
		{0, 0x0100}, // default Non-burst mode:{0, 0x0100},
#else
		{0, 0x00B6},
		{0, 0x010B}, // burst mode for power consumption:{0, 0x010B},
		{0, 0x0100}, // burst mode for power consumption:{0, 0x0100},
#endif
		{0, 0x00DE},
		{0, 0x0103},
		{0, 0x0100},

		{0, 0x00D6},
		{0, 0x0104},
		{0, 0x0100},

#ifdef CURRENT_BIAS_MIPI_OUTPUT
		{0, 0x00D8},
		{0, 0x011C},
		//{0, 0x0112}, // Default
		//{0, 0x011E}, // Max
		{0, 0x0102}, // Min
#endif
		{0, 0x00B9},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00C9},
		{0, 0x0103},
		{0, 0x0121},

		{0, 0x00BA},
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		//{0, 0x0138},    // 38h (448Mbps)
		//{0, 0x0183},    // 83h (448Mbps)
		{0, 0x01AB},    // 513Mbps 0xC8AB
		{0, 0x01C8},    // 513Mbps 0xC8AB

		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B7},
		{0, 0x0102},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		/*set_address_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0136},{0, 0x0100},

		/*set_pixel_format*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x013A},{0, 0x0170},

		/*SSD2825_leave_sleep_sequence*/
		{0, 0x00B7},{0, 0x0142},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*exit_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0111},{80, 0x0100},

#if defined(HITACHI_GAMMA_S_CURVE)
		//change by Generic, manu facturer command access protect off
		{0, 0x00B7},{0, 0x0102},{0, 0x0103}, // change by generic
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},

		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

		/*Power Setting *** [TANTATIVE]*/
		{0, 0x00BC},{0, 0x0107},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01D1}, {0, 0x0114}, {0, 0x011D}, {0, 0x0121},
		{0, 0x0167}, {0, 0x0111}, {0, 0x019A},

#ifdef HITACHI_CABC_ENABLE
		/*Hitachi CABC *** backlight control*/
		{0, 0x00BC},{0, 0x0106},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B8}, {0, 0x0101}, {0, 0x011A}, {0, 0x011A},
		{0, 0x0102}, {0, 0x0140},

		/*CABC***B9*** -10% ~ -30% */
		{0, 0x00BC},{0, 0x0108},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B9}, {0, 0x0118}, {0, 0x0100}, {0, 0x0118},
		{0, 0x0118}, {0, 0x019F}, {0, 0x011F}, {0, 0x010F},

		/*CABC***BA*** -10% *//*New 1st Option*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BA},{0, 0x0100},{0, 0x0100},{0, 0x010C},
		{0, 0x010A},{0, 0x016C},{0, 0x010A},{0, 0x01AC},
		{0, 0x010A},{0, 0x010C},{0, 0x010A},{0, 0x0100},
		{0, 0x01DA},{0, 0x016D},{0, 0x0103},{0, 0x01FF},
		{0, 0x01FF},{0, 0x0110},{0, 0x01D9},{0, 0x01E4},
		{0, 0x01EE},{0, 0x01F7},{0, 0x01FF},{0, 0x019F},
		{0, 0x0100},

		/*CABC***BA*** -20% *//*New 2nd Option*/
		/*{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},{0, 0x01BA},
		{0, 0x0100},{0, 0x0100},{0, 0x010C},{0, 0x010B},
		{0, 0x016C},{0, 0x010B},{0, 0x01AC},{0, 0x010B},
		{0, 0x010C},{0, 0x010B},{0, 0x0100},{0, 0x01DA},
		{0, 0x016D},{0, 0x0103},{0, 0x01FF},{0, 0x01FF},
		{0, 0x0110},{0, 0x01B3},{0, 0x01C9},{0, 0x01DC},
		{0, 0x01EE},{0, 0x01FF},{0, 0x019F},{0, 0x0100},*/

		/*CABC***BA*** -30% *//*New 3rd Option*/
		/*{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},{0, 0x01BA},
		{0, 0x0100},{0, 0x0100},{0, 0x010C},{0, 0x010D},
		{0, 0x016C},{0, 0x010D},{0, 0x01AC},{0, 0x010D},
		{0, 0x010C},{0, 0x010D},{0, 0x0100},{0, 0x01DA},
		{0, 0x016D},{0, 0x0103},{0, 0x01FF},{0, 0x01FF},
		{0, 0x0110},{0, 0x018C},{0, 0x01AA},{0, 0x01C7},
		{0, 0x01E3},{0, 0x01FF},{0, 0x019F},{0, 0x0100},*/
#endif

		/* DV 1st *** S-curve final */
		/*gamma setting A*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*gamma setting B*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C9},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*gamma setting C*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01CA},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},

		//change by DCS, display on
		{0, 0x00B7},{0, 0x0142},{0, 0x0103}, // change by generic
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
#endif
		/*Display on*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0129},{50, 0x0100},

		// ssd2825_video_sequence
		{0, 0x00BA},
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		//{0, 0x0138},    // 38h (448Mbps)
		//{0, 0x0183},    // 83h (448Mbps)
		{0, 0x01AB},    // 513Mbps 0xC8AB
		{0, 0x01C8},    // 513Mbps 0xC8AB

		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00B7},
		{0, 0x0109},
		{0, 0x0103},
};

static struct spi_cmd_data16 solomon_power_off_set[] = {
		{0, 0x00B7}, {0, 0x014B}, {0, 0x0103},
		{0, 0x00BC}, {0, 0x0102}, {0, 0x0100},
		{0, 0x00BF}, {0, 0x0128}, {20, 0x0100},
		{0, 0x00BF}, {0, 0x0110}, {60, 0x0100},
		{0, 0x00B7}, {0, 0x0100}, {0, 0x0103},
		{0, 0x00B9}, {0, 0x0100}, {0, 0x0100},
};

struct spi_cmd_data16 solomon_reg_read_set[] = {
		{0, 0x00C0}, //SW reset
		{0, 0x0100},
		{0, 0x0101},

		{0, 0x00B1},
		{0, 0x0104},
		{0, 0x0101},
#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B2},
		{0, 0x0142}, // default Non-burst mode:{0, 0x013E},
		{0, 0x0104}, // default Non-burst mode:{0, 0x0103},
#else
		{0, 0x00B2},
		{0, 0x0142}, // burst mode to reduce power consumption:{0, 0x011E+4(sync_width)},
		{0, 0x0104}, // burst mode to reduce power consumption:{0, 0x0103+1(sync_width)},
#endif
		// 0xB3 : 0x08, 0x08
		{0, 0x00B3},
		{0, 0x015C}, //{0, 0x0188}
		{0, 0x0106}, //{0, 0x0104},

		// 0xB4 : 0xD0, 0x02
		{0, 0x00B4},
		{0, 0x01D0},
		{0, 0x0102},

		{0, 0x00B5},
		{0, 0x0100},
		{0, 0x0105},

#ifdef MIPI_NON_BURST_MODE
		{0, 0x00B6},
		{0, 0x0107}, // default Non-burst mode:{0, 0x0103},
		{0, 0x0100}, // default Non-burst mode:{0, 0x0100},
#else
		{0, 0x00B6},
		{0, 0x010B}, // burst mode for power consumption:{0, 0x010B},
		{0, 0x0100}, // burst mode for power consumption:{0, 0x0100},
#endif

		{0, 0x00DE},
		{0, 0x0103},
		{0, 0x0100},

		{0, 0x00D6},
		{0, 0x0104},
		{0, 0x0100},

#ifdef CURRENT_BIAS_MIPI_OUTPUT
		{0, 0x00D8},
		{0, 0x011C},
		//{0, 0x0112}, // Default
		//{0, 0x011E}, // Max
		{0, 0x0102}, // Min
#endif

		{0, 0x00B9},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00C9},
		{0, 0x0103},
		{0, 0x0121},

		{0, 0x00BA},
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		//{0, 0x0138},    // 38h (448Mbps) //448Mbps 0x8338
		//{0, 0x0183},    // 83h (448Mbps)
		{0, 0x01AB},    // 513Mbps 0xC8AB
		{0, 0x01C8},    // 513Mbps 0xC8AB

		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		{0, 0x00B9},
		{0, 0x0101},
		{0, 0x0100},

		{0, 0x00B7},
		{0, 0x0102},
		{0, 0x0103},

		{0, 0x00B8},
		{0, 0x0100},
		{0, 0x0100},

		/*set_address_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0136},{0, 0x0100},

		/*set_pixel_format*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x013A},{0, 0x0170},

		/*Solomon_leave_sleep_sequence*/
		{0, 0x00B7},{0, 0x0142},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*exit_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0111},{500, 0x0100},

#if defined(HITACHI_GAMMA_S_CURVE)
		//change by Generic, manu facturer command access protect off
		{0, 0x00B7},{0, 0x0102},{0, 0x0103}, // change by generic
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},

		/*Manufacturer Command Access Protect Off*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0104},

		/*Power Setting *** [TANTATIVE]*/
		{0, 0x00BC},{0, 0x0107},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01D1}, {0, 0x0114}, {0, 0x011D}, {0, 0x0121},
		{0, 0x0167}, {0, 0x0111}, {0, 0x019A},

#ifdef HITACHI_CABC_ENABLE
		/*Hitachi CABC *** backlight control*/
		{0, 0x00BC},{0, 0x0106},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B8}, {0, 0x0101}, {0, 0x011A}, {0, 0x011A},
		{0, 0x0102}, {0, 0x0140},

		/*CABC***B9*** -10% ~ -30% */
		{0, 0x00BC},{0, 0x0108},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B9}, {0, 0x0118}, {0, 0x0100}, {0, 0x0118},
		{0, 0x0118}, {0, 0x019F}, {0, 0x011F}, {0, 0x010F},

		/*CABC***BA*** -10% *//*New 1st Option*/
		{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BA},{0, 0x0100},{0, 0x0100},{0, 0x010C},
		{0, 0x010A},{0, 0x016C},{0, 0x010A},{0, 0x01AC},
		{0, 0x010A},{0, 0x010C},{0, 0x010A},{0, 0x0100},
		{0, 0x01DA},{0, 0x016D},{0, 0x0103},{0, 0x01FF},
		{0, 0x01FF},{0, 0x0110},{0, 0x01D9},{0, 0x01E4},
		{0, 0x01EE},{0, 0x01F7},{0, 0x01FF},{0, 0x019F},
		{0, 0x0100},

		/*CABC***BA*** -20% *//*New 2nd Option*/
		/*{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},{0, 0x01BA},
		{0, 0x0100},{0, 0x0100},{0, 0x010C},{0, 0x010B},
		{0, 0x016C},{0, 0x010B},{0, 0x01AC},{0, 0x010B},
		{0, 0x010C},{0, 0x010B},{0, 0x0100},{0, 0x01DA},
		{0, 0x016D},{0, 0x0103},{0, 0x01FF},{0, 0x01FF},
		{0, 0x0110},{0, 0x01B3},{0, 0x01C9},{0, 0x01DC},
		{0, 0x01EE},{0, 0x01FF},{0, 0x019F},{0, 0x0100},*/

		/*CABC***BA*** -30% *//*New 3rd Option*/
		/*{0, 0x00BC},{0, 0x0119},{0, 0x0100},
		{0, 0x00BF},{0, 0x01BA},
		{0, 0x0100},{0, 0x0100},{0, 0x010C},{0, 0x010D},
		{0, 0x016C},{0, 0x010D},{0, 0x01AC},{0, 0x010D},
		{0, 0x010C},{0, 0x010D},{0, 0x0100},{0, 0x01DA},
		{0, 0x016D},{0, 0x0103},{0, 0x01FF},{0, 0x01FF},
		{0, 0x0110},{0, 0x018C},{0, 0x01AA},{0, 0x01C7},
		{0, 0x01E3},{0, 0x01FF},{0, 0x019F},{0, 0x0100},*/
#endif

		/* DV 1st *** S-curve final */
		/*gamma setting A*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C8},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*gamma setting B*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01C9},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*gamma setting C*/
		{0, 0x00BC},{0, 0x011D},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01CA},{0, 0x0100},{0, 0x011A},
		{0, 0x0120},{0, 0x0128},{0, 0x0125},
		{0, 0x0124},{0, 0x0126},{0, 0x0115},
		{0, 0x0113},{0, 0x0111},{0, 0x0118},
		{0, 0x011E},{0, 0x011C},{0, 0x0100},
		{0, 0x0100},{0, 0x011A},{0, 0x0120},
		{0, 0x0128},{0, 0x0125},{0, 0x0124},
		{0, 0x0126},{0, 0x0115},{0, 0x0113},
		{0, 0x0111},{0, 0x0118},{0, 0x011E},
		{0, 0x011C},{0, 0x0100},

		/*Manufacturer Command Access Protect On*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01B0},{0, 0x0103},

		//change by DCS, display on
		{0, 0x00B7},{0, 0x0142},{0, 0x0103}, // change by generic
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
#endif

		/*Display on*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},

		{0, 0x00BF},
		{0, 0x0129},{500, 0x0100},

		// ssd2825_video_sequence
		//{0, 0x0112},  // 12h (432Mbps)
		//{0, 0x0180},  // 80h (432Mbps)
		{0, 0x0138},    // 38h (448Mbps)
		{0, 0x0183},    // 83h (448Mbps)
		{0, 0x00BB},
		{0, 0x0109}, //{0, 0x0108}, //change for Hitach ESD
		{0, 0x0100},
		{0, 0x00B9},{0, 0x0101},{0, 0x0100},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00B7},{0, 0x0109},{0, 0x0103},
};

struct spi_cmd_data16 solomon_reg_read_set2[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00FF},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set3[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C6},
		//{0, 0x00FA},
};

/*DCS packets in LP mode*/
struct spi_cmd_data16 solomon_reg_read_set4[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x01C2}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x010C}, //Hitachi LCD get_pixel_format: 0Ch => 07h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*DCS packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set5[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x01C3}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x010A}, //Hitachi LCD get_power_mode: 0Ah => 1Ch
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*Generic Packets in LP mode*/
struct spi_cmd_data16 solomon_reg_read_set6[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0104},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x0182}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BF}, //Hitachi LCD Device Code Read(BFh) => 01h,22h,93h,28h,01h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

/*Generic Packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set7[] = {
		/*Solomon_enter_sleep_sequence*/
		{0, 0x00B7},{0, 0x014B},{0, 0x0103},
		{0, 0x00B8},{0, 0x0100},{0, 0x0100},
		{0, 0x00BC},{0, 0x0100},{0, 0x0100},

		/*Display off*/
		{0, 0x00BF}, {0, 0x0128}, {0, 0x0100},

		/*enter_sleep_mode*/
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},
		{0, 0x0110},{500, 0x0100},

		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0104},

		{0, 0x00C4},
		{0, 0x0101},
		{0, 0x0100},
		{0, 0x00C1},
		{0, 0x0120}, //{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		{0, 0x0183}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x0103},
		/*********/

		{0, 0x00BC},
		{0, 0x0101}, //0x0100 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
		{0, 0x01BF},//Hitachi LCD Device Code Read(BFh) => 01h,22h,93h,28h,01h
		{0, 0x0100},

		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00C2},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set8[] = {
		{0, 0x00D4},
		{0, 0x01FA},
		{0, 0x0100},
		{0, 0x00B0},
		//{0, 0x00FA},
};

struct spi_cmd_data16 solomon_reg_read_set9[] = {
		{0, 0x00B7},{0, 0x0102},{0, 0x0103},
		{0, 0x00BC},{0, 0x0102},{0, 0x0100},
		{0, 0x00BF},{0, 0x01B0},{0, 0x0103},
};

const char ssd2825_reg_set[]={
0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xDD,0xB7,0xB8,
0xB9,0xBA,0xD5,0xBB,0xBC,0xBD,0xBE,0xBF,0xC0,0xC1,
0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xCB,
0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD6,
0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDE,0xDF,0xE0,0xE1,
0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xEB,
0xFF};
/*DCS packets in HS mode*/
struct spi_cmd_data16 solomon_reg_read_set4_1[] = {
		{0, 0x00C1},
		{0, 0x010A}, //maximum return packet size
		{0, 0x0100},
		{0, 0x00B7},
		//{0, 0x01C2}, //0x01c2:DCS in LP, 0x0182: Generic in LP, 0x0183: Generic in HS ,0x01c3:DCS in HS
		{0, 0x01C9}, // keeping video mode on
		{0, 0x0103},
		/*********/
		{0, 0x00BC},
		{0, 0x0101}, //0x0101 : DCS //0x0101 : generic
		{0, 0x0100},
		{0, 0x00BF},
};
#endif // _SPI_SOLOMON_TABLE_H_
