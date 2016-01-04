/* Driver for I2C Master Controller inside TI Calypso */

/* (C) 2010 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
//这个i2c接口只支持主模式 The Master I2C Interface Module supports I2C Master Only mode with
/*
1. 7 bits address DEVICE	7位设备地址
2. 8 bits sub address		8位设备寄存器地址
3. Master write to slave receiver in single or multiple mode (data loop) 写入多个从设备
16 bytes deep transmit FIFO
4. Master simple Read to slave receiver		读入数据
5. Read Combined cycle
6. 3-bit programmable pre-scale internal clock divider and 7-bit programmable SCL clock divider to 
support wide clock frequency range of module input clock signal. The I2C SCL clock
frequency are: I2C Standard Mode: 100 kHz
I2C Fast Mode: 400 kHz
两个时钟模式，100KHZ与400KHZ	
7. 3-bit programmable spike filter to provide I2C bus input signal noise filtering ability.
8. Asynchronous Rhea bus access with Zero Wait State Insertion
Except for the synchronization of Rhea read access to STATUS_ACTIVITY.
9. Error Handling Capability during I2C bus access
*/
#include <stdint.h>
#include <stdio.h>

#include <debug.h>
#include <memory.h>
#include <i2c.h>

#define BASE_ADDR_I2C	0xfffe2800	//i2c内存地址
#define I2C_REG(x)	(BASE_ADDR_I2C+(x))	//注册函数？

enum i2c_reg {	//枚举注册值I2C register mapping
	DEVICE_REG	= 0,//DEVICE_REG FFFE:2800 7 bits R/W 1000 0000 
	ADDRESS_REG,//ADDRESS_REG FFFE:2801 8 bits R/W 0000 0000  
	DATA_WR_REG,//DATA_WR_REG FFFE:2802 8 bits R/W 0000 0000		fifo数据注册
	DATA_RD_REG,//DATA_RD_REG FFFE:2803 8 bits R 0000 0000
	CMD_REG,//CMD_REG FFFE:2804 6 bits R/W 1101 0001
	CONF_FIFO_REG,//CONF_FIFO_REG FFFE:2805 4 bits R/W 1111 1111		fifo数据的长度注册
	CONF_CLK_REG,//CONF_CLK_REG FFFE:2806 6 bits R/W 1100 0000
	CONF_CLK_FUNC_REF,//CONF_CLK_FUNC_REF FFFE:2807 7 bits R/W 1000 1010
	STATUS_FIFO_REG,//STATUS_FIFO_REG FFFE:2808 6 bits R 1100 0010
	STATUS_ACTIVITY_REG,//STATUS_ACTIVITY_REG FFFE:2809 4 bits R 1111 0000	i2c激活状态
};

#define I2C_CMD_SOFT_RESET	(1 << 0)//软件复位
#define I2C_CMD_EN_CLK		(1 << 1)//时钟使能
#define I2C_CMD_START		(1 << 2)//开始命令
#define I2C_CMD_RW_READ		(1 << 3)//读写控制读取
#define I2C_CMD_COMP_READ	(1 << 4)//comp控制读取
#define I2C_CMD_IRQ_ENABLE	(1 << 5)//中断打开

#define I2C_STATUS_ERROR_DATA	(1 << 0)//数据状态错误
#define I2C_STATUS_ERROR_DEV	(1 << 1)//驱动状态错
#define I2C_STATUS_IDLE		(1 << 2) // idle状态值1: not idle, 0: idle i2c是否处于忙的状态
#define I2C_STATUS_INTERRUPT	(1 << 3)//

int i2c_write(uint8_t chip, uint32_t addr, int alen, const uint8_t *buffer, int len)
/*int i2c_write(u_int8_t chip, //芯片的i2c地址，不包含读写位
u_int32_t addr, //芯片内的读写地址，比如寄存器地址
int alen, //这个要看代码才知道是地址的长度。比如有的flash比较大就有16位地址。
              //uboot支持32位地址，不过要看驱动支不支持。0=8bit,1=16bit,2=32bit地址长度
u_int8_t *buf, //数据
int len)            //数据长度
*/
{
	uint8_t cmd;

	/* 如果地址长度大于16位返回错误	Calypso I2C controller doesn't support fancy addressing */
	if (alen > 1)
		return -1;

	/*数据长度大于16位也返回错误 FIXME: implement writes longer than fifo size */
	if (len > 16)
		return -1;
	//打印从设备地址寄存器
	printd("i2c_write(chip=0x%02u, addr=0x%02u): ", chip, addr);
	//注册设备地址值
	writeb(chip & 0x7f, I2C_REG(DEVICE_REG));
	writeb(addr & 0xff, I2C_REG(ADDRESS_REG));
	
	/*写入数据长度注册 we have to tell the controller how many bits we'll put into the fifo ?!? */
	writeb(len-1, I2C_REG(CONF_FIFO_REG));

	/* fifo数据注册写入从设备fifo数据	fill the FIFO */
	while (len--) {
		uint8_t byte = *buffer++;
		writeb(byte, I2C_REG(DATA_WR_REG));
		printd("%02X ", byte);
	}
	dputchar('\n');

	/*开始写入 start the transfer */
	cmd = readb(I2C_REG(CMD_REG));	//读取注册命令枚举
	cmd |= I2C_CMD_START;	//开始i2c
	writeb(cmd, I2C_REG(CMD_REG));//写入注册命令

	/* 判断是否传输完成wait until transfer completes */
	while (1) {
		uint8_t reg = readb(I2C_REG(STATUS_ACTIVITY_REG));//读取i2c激活状态
		printd("I2C Status: 0x%02x\n", reg & 0xf);
		if (!(reg & I2C_STATUS_IDLE)) // 0: idle 1: not idle如果状态是空闲返回，忙继续循环while
			break;
	}
	dputs("I2C transfer completed\n");
	//传输完成函数结束
	return 0;
}
int i2c_read(uint8_t chip, uint32_t addr, int alen, const uint8_t *buffer, int len)
{
	uint8_t cmd;

	/* Calypso I2C controller doesn't support fancy addressing */
	if (alen > 1)
	return -1;

	/* FIXME: implement writes longer than fifo size */
	if (len > 16)
	return -1;

	printd("i2c_write(chip=0x%02u, addr=0x%02u): ", chip, addr);

	writeb(chip & 0x7f, I2C_REG(DEVICE_REG));
	writeb(addr & 0xff, I2C_REG(ADDRESS_REG));
	
	/* we have to tell the controller how many bits we'll put into the fifo ?!? */
	writeb(len-1, I2C_REG(CONF_FIFO_REG));

	/* fill the FIFO */
	while (len--) {
	uint8_t byte = *buffer++;
	writeb(byte, I2C_REG(DATA_RD_REG));
	printd("%02X ", byte);
	}
	dputchar('\n');

	/* start the transfer */
	cmd = readb(I2C_REG(CMD_REG));
	cmd |= I2C_CMD_START;
	writeb(cmd, I2C_REG(CMD_REG));

	/* wait until transfer completes */
	while (1) {
	uint8_t reg = readb(I2C_REG(STATUS_ACTIVITY_REG));
	printd("I2C Status: 0x%02x\n", reg & 0xf);
	if (!(reg & I2C_STATUS_IDLE)) // 0: idle 1: not idle
	break;
	}
	dputs("I2C transfer completed\n");

	return 0;
}

void i2c_init(int speed, int slaveadd)
{
	/* scl_out = clk_func_ref / 3,
	   clk_func_ref = master_clock_freq / (divisor_2 + 1)
	   master_clock_freq = ext_clock_freq / divisor_1 */
	/* clk_func_ref = scl_out * 3,
	   divisor_2 = (master_clock_freq / clk_func_ref) - 1
	   divisor_1 = ext_clock_freq / master_clock_freq */
	/* for a target freq of 200kHz:
		ext_clock_freq = 13MHz
		clk_func_ref = 3 * 300kHZ = 600kHz
		divisor_1 = 1 => master_clock_freq = ext_clock_freq = 13MHz
		divisor_2 = 21 => clk_func_ref = 13MHz / (21+2) = 590.91 kHz
		scl_out = clk_func_ref / 3 = 509.91 kHz / 3 = 196.97kHz */
	writeb(I2C_CMD_SOFT_RESET, I2C_REG(CMD_REG));

	writeb(0x00, I2C_REG(CONF_CLK_REG));
	writeb(21, I2C_REG(CONF_CLK_FUNC_REF));

	writeb(I2C_CMD_EN_CLK, I2C_REG(CMD_REG));
}
