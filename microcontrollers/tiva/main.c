/*
This is the code for the Tiva TM4C
Please don't look at this...
it is bad and doesn't work
please don't judge me for how sloppy it is
I was trying to get this code to work on sleepless nights
it is bad
like real bad
please don't look at it
*/
#define PART_TM4C123GH6PM
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "inc/tm4c123gh6pm.h"
#include "bme68x.h"

#define BME_ADDR 0x76 // default I2C address for BME

void delay_ms(uint32_t ms);
void uart_init(void);
void lora_send_str(const char *s);
void lora_send_cmd(const char *cmd);
int lora_read(char *buf, int maxlen, uint32_t timeout_ms);
bool lora_init(void);
bool lora_send_text(uint16_t dest, const char *text);
bool lora_rcv(char *buf, int maxlen);
void handle_rcv(const char *line);
void uart0_print(const char *str);
void uart0_println(const char *str);
void i2c_init(void);
int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
int8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme_delay(uint32_t period, void *intf_ptr);
int8_t bme680_init(void);
int8_t bme_read_once(struct bme68x_data *data);
void bme680_print_reading(void);
void uart0_print_int(int value);
void uart0_print_uint(unsigned int value);
void uart0_print_hex8(uint8_t value);
int8_t bme_force_measurement(void);
void bme_dump_raw_regs(void);
int8_t bme_read_tph_raw(uint32_t *pres_adc, uint32_t *temp_adc, uint16_t *hum_adc);
int16_t bme_comp_temp(uint32_t temp_adc);
uint32_t bme_comp_press(uint32_t pres_adc);
uint32_t bme_comp_hum(uint16_t hum_adc);
void bme680_print_gas_after_warmup(void);

int8_t bme680_init_gas(void);
int8_t bme_read_gas_raw(uint16_t *gas_adc, uint8_t *gas_range, uint8_t *gas_valid, uint8_t *heat_stab);
uint32_t bme_calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range);
void bme680_print_gas_reading(void);

static struct bme68x_dev bme;
static uint8_t bme_addr = BME_ADDR;

static uint32_t gas_warmup_ms = 0;
static bool gas_ready = false;

bool bme_get_raw_tph(uint32_t *pres_adc, uint32_t *temp_adc, uint16_t *hum_adc);
bool bme_get_raw_gas(uint16_t *gas_adc, uint8_t *gas_range, bool *gas_valid_out, bool *heat_stab_out);
bool bme_build_raw_packet(char *msg, size_t msg_size,
                          uint32_t pres_adc, uint32_t temp_adc, uint16_t hum_adc,
                          uint16_t gas_adc, uint8_t gas_range,
                          bool gas_valid, bool heat_stab);

static void uint_to_str(uint32_t val, char *buf);

bool i2c_wait_idle(void)
{
    uint32_t timeout = 100000;

    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--timeout == 0)
        {
            uart0_println("I2C timeout");
            return false;
        }
    }

    return true;
}

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); // 40 Mhz clk
	
	uart_init();
	i2c_init();
	uart0_println("Starting BME680...");
	
	int8_t result;
	char rcv[128];
	

	
	delay_ms(100);
	
	result = bme680_init();
	uart0_print("init rslt=");
	uart0_print_int(result);
	uart0_println("");

	uart0_println("=== BME test ===");
	bme680_print_reading();
	uint32_t tick_ms = 0;
	uint32_t tph_tick_ms = 0;
	uint32_t gas_tick_ms = 0;

	
	while(!lora_init())
	{
		uart0_println("LoRa init failed, retrying...");
    delay_ms(500);
	}
	
	uart0_println("=== BUILD A1 ===");
while (1)
{
    
		if (lora_rcv(rcv, sizeof(rcv)))
    {
        uart0_println(rcv);

        if (strcmp(rcv, "+OK") == 0)
        {
            uart0_println("Send confirmed");
        }
        else if (strncmp(rcv, "+RCV=", 5) == 0)
        {
            handle_rcv(rcv);
        }
    }
		
if (tick_ms >= 500)
{
    uint32_t pres_adc;
    uint32_t temp_adc;
    uint16_t hum_adc;

    uint16_t gas_adc = 0;
    uint8_t gas_range = 0;
    bool gas_valid = false;
    bool heat_stab = false;

    char msg[128];

    tick_ms = 0;

    if (bme_get_raw_tph(&pres_adc, &temp_adc, &hum_adc))
    {
        if (!bme_get_raw_gas(&gas_adc, &gas_range, &gas_valid, &heat_stab))
        {
            gas_adc = 0;
            gas_range = 0;
            gas_valid = false;
            heat_stab = false;
        }

        if (bme_build_raw_packet(msg, sizeof(msg),
                                 pres_adc, temp_adc, hum_adc,
                                 gas_adc, gas_range,
                                 gas_valid, heat_stab))
        {
            if (lora_send_text(2, msg))
            {
                uart0_print("Sent: ");
                uart0_println(msg);
            }
            else
            {
                uart0_println("LoRa send failed");
            }
        }
        else
        {
            uart0_println("Raw packet build failed");
        }
    }
    else
    {
        uart0_println("Raw TPH read failed");
    }
}


delay_ms(10);
tick_ms += 10;
gas_tick_ms += 10;
}
}

void delay_ms(uint32_t ms)
{
	SysCtlDelay((SysCtlClockGet() / 3 / 1000) * ms);
}

void uart_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// uart1 on portb will be used for LoRa
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)) {}
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
	// PB0 = RX,  PB1 = TX	
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	
	// tell portb pin 0 and 1 are for UART
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// will use 115200 for LoRA baudrate
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
	UARTEnable(UART1_BASE);
		
		
	// UART0 for debug on pc
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // TM4C123 Mapped to 
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure Pins for UART
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTDisable(UART0_BASE); // Turn off UART for configuration
			// Configure for 115200 baud, 8-bit data, no parity, 1 stop bit
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTEnable(UART0_BASE); // Turn on UART post configuration
		
}

void lora_send_str(const char *s)
{
	while(*s)
	{
		UARTCharPut(UART1_BASE, *s++);
	}
}

void lora_send_cmd(const char *cmd)
{
	lora_send_str(cmd);
	lora_send_str("\r\n");
}

int lora_read(char *buf, int maxlen, uint32_t timeout_ms)
{
	int i = 0;
	uint32_t elapsed = 0;
	
	while(elapsed < timeout_ms)
	{
		while(UARTCharsAvail(UART1_BASE))
		{
			char c = (char)UARTCharGet(UART1_BASE);
			if(c == '\r')
				continue;
			if(c == '\n')
			{
				if(i > 0)
				{
					buf[i] = '\0';
					return i;
				}
			}else if(i < maxlen - 1)
			{
				buf[i++] = c;
			}
		}
		delay_ms(1);
		elapsed++;
	}
	if(i > 0)
	{
		buf[i] = '\0';
		return i;
	}
	
	return 0;
}

// this checks for the +OK message from LoRa
/*bool lora_ok(uint32_t timeout_ms)
{
    char line[128];
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms)
    {
        if (lora_rcv(line, sizeof(line)))
        {
            if (strcmp(line, "+OK") == 0)
            {
                return true;
            }

            if (strncmp(line, "+RCV=", 5) == 0)
            {
                handle_rcv(line);  // don't lose it!
            }
            else
            {
                uart0_print("Other: ");
                uart0_println(line);
            }
        }

        delay_ms(10);
        elapsed += 10;
    }

    return false;
}
*/

bool lora_init(void)
{
	// check for +OK after every initialization
	lora_send_cmd("AT");
	delay_ms(100);
	lora_send_cmd("AT+ADDRESS=1");
	delay_ms(100);
	lora_send_cmd("AT+NETWORKID=6");
	delay_ms(100);
	lora_send_cmd("AT+BAND=915000000");
	delay_ms(100);
	lora_send_cmd("AT+PARAMETER=9,7,1,12");
	delay_ms(100);
	uart0_println("LoRa init");
	return true;
}

// send command needs destination address, message length, and message
bool lora_send_text(uint16_t dest, const char *text)
{
	char len_str[8];
  char dest_str[8];
	size_t len = strlen(text);
	
	if(len > 240)
		return false;
	
	sprintf(dest_str, "%u", dest);
  sprintf(len_str, "%u", (unsigned)len);

  lora_send_str("AT+SEND=");
  lora_send_str(dest_str);
  lora_send_str(",");
  lora_send_str(len_str);
  lora_send_str(",");
  lora_send_str(text);
  lora_send_str("\r\n");
	
	return true;
}

bool lora_rcv(char *buf, int maxlen)
{
	static int idx = 0;
	while(UARTCharsAvail(UART1_BASE))
	{
		char c = (char)UARTCharGet(UART1_BASE);
		
		if(c == '\r') continue;
		
		if(c == '\n')
		{
			if(idx > 0)
			{
				buf[idx] = '\0';
				idx = 0;
				return true;
			}
		}else if(idx < maxlen - 1)
		{
			buf[idx++] = c;
		}
	}
	return false;
}

void handle_rcv(const char *line)
{
    uart0_print("RECEIVED FROM PI: ");
    uart0_println(line);
}

void uart0_print(const char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str++);
    }
}

void uart0_println(const char *str) {
    uart0_print(str);
    uart0_print("\r\n");
}


void uart0_print_int(int value)
{
    char buf[16];
    int i = 0;
    bool neg = false;

    if (value == 0)
    {
        uart0_print("0");
        return;
    }

    if (value < 0)
    {
        neg = true;
        value = -value;
    }

    while (value > 0 && i < (int)(sizeof(buf) - 1))
    {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    if (neg)
    {
        buf[i++] = '-';
    }

    while (i > 0)
    {
        UARTCharPut(UART0_BASE, buf[--i]);
    }
}

void uart0_print_uint(unsigned int value)
{
    char buf[16];
    int i = 0;

    if (value == 0)
    {
        uart0_print("0");
        return;
    }

    while (value > 0 && i < (int)(sizeof(buf) - 1))
    {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0)
    {
        UARTCharPut(UART0_BASE, buf[--i]);
    }
}

void uart0_print_hex8(uint8_t value)
{
    const char hex[] = "0123456789ABCDEF";

    UARTCharPut(UART0_BASE, hex[(value >> 4) & 0x0F]);
    UARTCharPut(UART0_BASE, hex[value & 0x0F]);
}

void i2c_init(void)
{
	
	// gpiob is already enabled in uart_init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
	{
	}
	
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}


/*
int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
	uint32_t err;
	
	I2CMasterSlaveAddrSet(I2C0_BASE, addr, false);
	I2CMasterDataPut(I2C0_BASE, reg);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	err = I2CMasterErr(I2C0_BASE);
	if(I2CMasterErr(I2C0_BASE))
	{
		char msg[48];
        snprintf(msg, sizeof(msg), "i2c_write err after START: 0x%02X\r\n", err);
        uart0_print(msg);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            while(I2CMasterBusy(I2C0_BASE)) {}
							
		return -1;
	}
	
	for(uint16_t i = 0; i < len; i++)
	{
		I2CMasterDataPut(I2C0_BASE, data[i]);
		if(i == (len - 1))
		{
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		}
		else
		{
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		}
		while(I2CMasterBusy(I2C0_BASE))
		{
		}
		if(I2CMasterErr(I2C0_BASE))
		{
			char msg[48];
        snprintf(msg, sizeof(msg), "i2c_write err after START: 0x%02X\r\n", err);
        uart0_print(msg);
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
      while(I2CMasterBusy(I2C0_BASE)) {}
			return -1;
		}
	}
	
	return 0;
}

int8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
	I2CMasterSlaveAddrSet(I2C0_BASE, addr, false);
	I2CMasterDataPut(I2C0_BASE, reg);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	
	if(I2CMasterErr(I2C0_BASE))
		return -1;
	
	I2CMasterSlaveAddrSet(I2C0_BASE, addr, true);
	
	if(len == 1)
	{
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		while(I2CMasterBusy(I2C0_BASE))
		{
		}
		
		if(I2CMasterErr(I2C0_BASE))
			return -1;
		
		data[0] = I2CMasterDataGet(I2C0_BASE);
		return 0;
	}
	
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	if(I2CMasterErr(I2C0_BASE))
		return -1;
	
	data[0] = I2CMasterDataGet(I2C0_BASE);
	
	for(uint16_t i = 1; i < len - 1; i++)
	{
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		while(I2CMasterBusy(I2C0_BASE))
		{
		}
		if(I2CMasterErr(I2C0_BASE))
			return -1;
		data[i] = I2CMasterDataGet(I2C0_BASE);
	}
	
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_BASE))
	{
	}
	if(I2CMasterErr(I2C0_BASE))
		return -1;
	
	data[len-1] = I2CMasterDataGet(I2C0_BASE);
	
	return 0;
}


*/
int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint32_t err;
    char msg[64];

    I2CMasterSlaveAddrSet(I2C0_BASE, addr, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    if (!i2c_wait_idle())
    {
        uart0_println("write: timeout after start");
        return -1;
    }

    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE)
    {
        snprintf(msg, sizeof(msg), "write: start err=0x%02lX\r\n", (unsigned long)err);
        uart0_print(msg);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
        i2c_wait_idle();
        return -1;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        I2CMasterDataPut(I2C0_BASE, data[i]);

        if (i == (len - 1))
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        else
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

        if (!i2c_wait_idle())
        {
            uart0_println("write: timeout during data");
            return -1;
        }

        err = I2CMasterErr(I2C0_BASE);
        if (err != I2C_MASTER_ERR_NONE)
        {
            snprintf(msg, sizeof(msg), "write: data err=0x%02lX\r\n", (unsigned long)err);
            uart0_print(msg);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            i2c_wait_idle();
            return -1;
        }
    }

    return 0;
}

int8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint32_t err;
    char msg[64];

    I2CMasterSlaveAddrSet(I2C0_BASE, addr, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (!i2c_wait_idle())
    {
        uart0_println("read: timeout sending reg");
        return -1;
    }

    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE)
    {
        snprintf(msg, sizeof(msg), "read: addr err=0x%02lX\r\n", (unsigned long)err);
        uart0_print(msg);
        return -1;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, addr, true);

    if (len == 1)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        if (!i2c_wait_idle())
        {
            uart0_println("read: timeout single receive");
            return -1;
        }

        err = I2CMasterErr(I2C0_BASE);
        if (err != I2C_MASTER_ERR_NONE)
        {
            snprintf(msg, sizeof(msg), "read: single err=0x%02lX\r\n", (unsigned long)err);
            uart0_print(msg);
            return -1;
        }

        data[0] = I2CMasterDataGet(I2C0_BASE);
        return 0;
    }

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    if (!i2c_wait_idle())
    {
        uart0_println("read: timeout burst start");
        return -1;
    }

    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE)
    {
        snprintf(msg, sizeof(msg), "read: burst start err=0x%02lX\r\n", (unsigned long)err);
        uart0_print(msg);
        return -1;
    }

    data[0] = I2CMasterDataGet(I2C0_BASE);

    for (uint16_t i = 1; i < len - 1; i++)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        if (!i2c_wait_idle())
        {
            uart0_println("read: timeout burst cont");
            return -1;
        }

        err = I2CMasterErr(I2C0_BASE);
        if (err != I2C_MASTER_ERR_NONE)
        {
            snprintf(msg, sizeof(msg), "read: burst cont err=0x%02lX\r\n", (unsigned long)err);
            uart0_print(msg);
            return -1;
        }

        data[i] = I2CMasterDataGet(I2C0_BASE);
    }

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    if (!i2c_wait_idle())
    {
        uart0_println("read: timeout burst finish");
        return -1;
    }

    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE)
    {
        snprintf(msg, sizeof(msg), "read: burst finish err=0x%02lX\r\n", (unsigned long)err);
        uart0_print(msg);
        return -1;
    }

    data[len - 1] = I2CMasterDataGet(I2C0_BASE);
    return 0;
}

int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t addr = *(uint8_t *)intf_ptr;
	return i2c_read_reg(addr, reg_addr, reg_data, (uint16_t)len);
}

int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;

    if ((reg_data == NULL) || (len == 0))
        return -1;

    return i2c_write_reg(addr, reg_addr, reg_data, (uint16_t)len);
}
void bme_delay(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr;

  uint32_t ticks = (SysCtlClockGet() / 3000000U) * period;
  SysCtlDelay(ticks);
}

int8_t bme680_init(void)
{
    int8_t rslt;
    struct bme68x_conf conf = {0};
    struct bme68x_heatr_conf heatr_conf = {0};

    uart0_println("bme init: start");

    memset(&bme, 0, sizeof(bme));

    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = &bme_addr;
    bme.read = bme_read;
    bme.write = bme_write;
    bme.delay_us = bme_delay;
    bme.amb_temp = 25;

    rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK)
        return rslt;

    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;

    rslt = bme68x_set_conf(&conf, &bme);
    if (rslt != BME68X_OK)
        return rslt;

    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur  = 100;

    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    if (rslt != BME68X_OK)
        return rslt;

    uart0_println("bme init: done");
    return BME68X_OK;
}

int8_t bme_force_measurement(void)
{
    uint8_t ctrl_meas;
    int8_t rslt;

    rslt = i2c_read_reg(BME_ADDR, 0x74, &ctrl_meas, 1);
    if (rslt != 0)
    {
        return -1;
    }

    ctrl_meas = (ctrl_meas & ~0x03) | 0x01;   // forced mode

    rslt = i2c_write_reg(BME_ADDR, 0x74, &ctrl_meas, 1);
    if (rslt != 0)
    {
        return -1;
    }

    return 0;
}

void bme_dump_raw_regs(void)
{
    uint8_t buf[8];
    int8_t rslt;

    rslt = i2c_read_reg(BME_ADDR, 0x1F, buf, 8);
    if (rslt != 0)
    {
        uart0_println("raw read failed");
        return;
    }

    uart0_print("raw: ");
    for (int i = 0; i < 8; i++)
    {
        uart0_print_hex8(buf[i]);
        uart0_print(" ");
    }
    uart0_println("");
}

int8_t bme_read_once(struct bme68x_data *data)
{
    int8_t rslt;
    uint8_t n_fields = 0;
    uint32_t del_period;
    struct bme68x_conf conf = {0};

    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (rslt != BME68X_OK)
        return rslt;

    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + 5000U;
    bme.delay_us(del_period, bme.intf_ptr);

    rslt = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &bme);
    if (rslt != BME68X_OK)
        return rslt;

    if (n_fields == 0)
        return BME68X_W_NO_NEW_DATA;

    return BME68X_OK;
}


/*void bme680_print_reading(void)
{
    struct bme68x_data data = {0};
    int8_t rslt;

    rslt = bme_read_once(&data);

    if (rslt == BME68X_W_NO_NEW_DATA)
    {
        uart0_println("BME680: no new data");
        return;
    }

    if (rslt != BME68X_OK)
    {
        uart0_print("bme_read_once failed: ");
        uart0_print_int(rslt);
        uart0_println("");
        return;
    }

    uart0_print("Temp: ");
    uart0_print_int((int)(data.temperature / 100));
    uart0_println(" C");

    uart0_print("Press: ");
    uart0_print_uint((unsigned int)data.pressure);
    uart0_println(" Pa");

    uart0_print("Hum: ");
    uart0_print_uint((unsigned int)(data.humidity / 1000));
    uart0_println(" %RH");

    uart0_print("Status: 0x");
    uart0_print_hex8(data.status);
    uart0_println("");
}*/

void bme680_print_reading(void)
{
    uint32_t pres_adc;
    uint32_t temp_adc;
    uint16_t hum_adc;

    int16_t temp_c_x100;
    uint32_t press_pa;
    uint32_t hum_rh_x1000;

    int8_t rslt;

    rslt = bme_read_tph_raw(&pres_adc, &temp_adc, &hum_adc);
    if (rslt != 0)
    {
        uart0_print("bme_read_tph_raw failed: ");
        uart0_print_int(rslt);
        uart0_println("");
        return;
    }

    temp_c_x100 = bme_comp_temp(temp_adc);
    press_pa = bme_comp_press(pres_adc);
    hum_rh_x1000 = bme_comp_hum(hum_adc);

    uart0_print("Temp: ");
    uart0_print_int(temp_c_x100 / 100);
    uart0_print(".");
    uart0_print_uint((unsigned int)(abs(temp_c_x100 % 100)));
    uart0_println(" C");

    uart0_print("Press: ");
    uart0_print_uint((unsigned int)press_pa);
    uart0_println(" Pa");

    uart0_print("Hum: ");
    uart0_print_uint((unsigned int)(hum_rh_x1000 / 1000));
    uart0_print(".");
    uart0_print_uint((unsigned int)(hum_rh_x1000 % 1000));
    uart0_println(" %RH");
}

int8_t bme_read_tph_raw(uint32_t *pres_adc, uint32_t *temp_adc, uint16_t *hum_adc)
{
    uint8_t buf[8];
    int8_t rslt;

    rslt = bme_force_measurement();
    if (rslt != 0)
    {
        return rslt;
    }

    delay_ms(50);

    rslt = i2c_read_reg(BME_ADDR, 0x1F, buf, 8);
    if (rslt != 0)
    {
        return rslt;
    }

    *pres_adc = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)buf[2] >> 4);
    *temp_adc = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4);
    *hum_adc  = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];

    return 0;

}

int16_t bme_comp_temp(uint32_t temp_adc)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int16_t calc_temp;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)bme.calib.par_t1 << 1);
    var2 = (var1 * (int32_t)bme.calib.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = (var3 * ((int32_t)bme.calib.par_t3 << 4)) >> 14;

    bme.calib.t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)((((int16_t)bme.calib.t_fine * 5) + 128) >> 8);

    return calc_temp;   // 0.01 deg C
}

uint32_t bme_comp_hum(uint16_t hum_adc)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t calc_hum;

    temp_scaled = (((int32_t)bme.calib.t_fine * 5) + 128) >> 8;

    var1 = (int32_t)(hum_adc - ((int32_t)bme.calib.par_h1 * 16)) -
           (((temp_scaled * (int32_t)bme.calib.par_h3) / 100) >> 1);

    var2 = ((int32_t)bme.calib.par_h2 *
           (((temp_scaled * (int32_t)bme.calib.par_h4) / 100) +
           ((((temp_scaled * ((temp_scaled * (int32_t)bme.calib.par_h5) / 100)) >> 6) / 100)) +
           (int32_t)(1 << 14))) >> 10;

    var3 = var1 * var2;
    var4 = ((int32_t)bme.calib.par_h6 << 7);
    var4 = (var4 + ((temp_scaled * (int32_t)bme.calib.par_h7) / 100)) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;

    calc_hum = (((var3 + var6) >> 10) * 1000) >> 12;

    if (calc_hum > 100000)
    {
        calc_hum = 100000;
    }
    else if (calc_hum < 0)
    {
        calc_hum = 0;
    }

    return (uint32_t)calc_hum;   // 0.001 %RH
}

uint32_t bme_comp_press(uint32_t pres_adc)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t pressure_comp;
    const int32_t pres_ovf_check = INT32_C(0x40000000);

    var1 = (((int32_t)bme.calib.t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)bme.calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)bme.calib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)bme.calib.par_p4 << 16);

    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)bme.calib.par_p3 << 5)) >> 3) +
           (((int32_t)bme.calib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)bme.calib.par_p1) >> 15;

    pressure_comp = 1048576 - (int32_t)pres_adc;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));

    if (pressure_comp >= pres_ovf_check)
    {
        pressure_comp = ((pressure_comp / var1) << 1);
    }
    else
    {
        pressure_comp = ((pressure_comp << 1) / var1);
    }

    var1 = ((int32_t)bme.calib.par_p9 *
           (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)bme.calib.par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
           (int32_t)(pressure_comp >> 8) * (int32_t)bme.calib.par_p10) >> 17;

    pressure_comp = pressure_comp +
                   ((var1 + var2 + var3 + ((int32_t)bme.calib.par_p7 << 7)) >> 4);

    return (uint32_t)pressure_comp;   // Pa
}



int8_t bme_read_gas_raw(uint16_t *gas_adc, uint8_t *gas_range, uint8_t *gas_valid, uint8_t *heat_stab)
{
    uint8_t ctrl_gas_1;
    uint8_t ctrl_meas;
    uint8_t buf[2];
    int8_t rslt;

    rslt = i2c_read_reg(BME_ADDR, 0x71, &ctrl_gas_1, 1);
    if (rslt != 0)
        return rslt;

    ctrl_gas_1 |= 0x10;   /* run_gas = 1 */

    rslt = i2c_write_reg(BME_ADDR, 0x71, &ctrl_gas_1, 1);
    if (rslt != 0)
        return rslt;

    rslt = i2c_read_reg(BME_ADDR, 0x74, &ctrl_meas, 1);
    if (rslt != 0)
        return rslt;

    ctrl_meas = (ctrl_meas & ~0x03) | 0x01;   /* forced mode */

    rslt = i2c_write_reg(BME_ADDR, 0x74, &ctrl_meas, 1);
    if (rslt != 0)
        return rslt;

    /* TPH + heater duration */
    delay_ms(150);

    rslt = i2c_read_reg(BME_ADDR, 0x2A, buf, 2);
    if (rslt != 0)
        return rslt;

    *gas_adc   = ((uint16_t)buf[0] << 2) | (buf[1] >> 6);
    *gas_range = buf[1] & 0x0F;
    *gas_valid = (buf[1] >> 5) & 0x01;
    *heat_stab = (buf[1] >> 4) & 0x01;

    return 0;
}

uint32_t bme_calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range)
{
    static const uint32_t lookup_k1_range[16] = {
        2147483647U, 2147483647U, 2147483647U, 2147483647U,
        2147483647U, 2126008810U, 2147483647U, 2130303777U,
        2147483647U, 2147483647U, 2143188679U, 2136746228U,
        2147483647U, 2126008810U, 2147483647U, 2147483647U
    };

    static const uint32_t lookup_k2_range[16] = {
        4096000000U, 2048000000U, 1024000000U, 512000000U,
        255744255U, 127110228U, 64000000U, 32258064U,
        16016016U, 8000000U, 4000000U, 2000000U,
        1000000U, 500000U, 250000U, 125000U
    };

    uint64_t var1, var2, calc;

    if (gas_range >= 16)
        return 0;

    var1 = ((uint64_t)(1340 + (5 * bme.calib.range_sw_err)) * lookup_k1_range[gas_range]) >> 16;
    var2 = (((uint64_t)gas_res_adc << 15) - 16777216ULL) + var1;

    if (var2 == 0)
        return 0;

    calc = ((uint64_t)lookup_k2_range[gas_range] * var1) / var2;

    if (calc > 0xFFFFFFFFULL)
        calc = 0xFFFFFFFFULL;

    return (uint32_t)calc;
}


void bme680_print_gas_after_warmup(void)
{
    uint16_t gas_adc;
    uint8_t gas_range;
    uint8_t gas_valid;
    uint8_t heat_stab;
    uint32_t gas_ohms;
    int8_t rslt;

    rslt = bme_read_gas_raw(&gas_adc, &gas_range, &gas_valid, &heat_stab);
    if (rslt != 0)
    {
        uart0_print("bme_read_gas_raw failed: ");
        uart0_print_int(rslt);
        uart0_println("");
        return;
    }

    if (!gas_valid || !heat_stab)
    {
        uart0_println("Gas warming / not valid yet");
        return;
    }

    if (!gas_ready)
    {
        uart0_print("Gas warm-up: ");
        uart0_print_uint(gas_warmup_ms / 1000);
        uart0_println(" s");
        return;
    }

    gas_ohms = bme_calc_gas_resistance(gas_adc, gas_range);

    uart0_print("Gas: ");
    uart0_print_uint(gas_ohms);
    uart0_println(" ohms");
}

static void uint_to_str(uint32_t val, char *buf)
{
    char tmp[16];
    int i = 0;
    int j = 0;

    if (val == 0)
    {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }

    while (val > 0)
    {
        tmp[i++] = (char)('0' + (val % 10));
        val /= 10;
    }

    while (i > 0)
    {
        buf[j++] = tmp[--i];
    }

    buf[j] = '\0';
}

bool bme_get_raw_tph(uint32_t *pres_adc, uint32_t *temp_adc, uint16_t *hum_adc)
{
    int8_t rslt;

    if (pres_adc == NULL || temp_adc == NULL || hum_adc == NULL)
        return false;

    rslt = bme_read_tph_raw(pres_adc, temp_adc, hum_adc);
    if (rslt != 0)
        return false;

    return true;
}

bool bme_get_raw_gas(uint16_t *gas_adc, uint8_t *gas_range, bool *gas_valid_out, bool *heat_stab_out)
{
    uint8_t gas_valid;
    uint8_t heat_stab;
    int8_t rslt;

    if (gas_adc == NULL || gas_range == NULL || gas_valid_out == NULL || heat_stab_out == NULL)
        return false;

    rslt = bme_read_gas_raw(gas_adc, gas_range, &gas_valid, &heat_stab);
    if (rslt != 0)
        return false;

    *gas_valid_out = (gas_valid != 0);
    *heat_stab_out = (heat_stab != 0);

    return true;
}

bool bme_build_raw_packet(char *msg, size_t msg_size,
                          uint32_t pres_adc, uint32_t temp_adc, uint16_t hum_adc,
                          uint16_t gas_adc, uint8_t gas_range,
                          bool gas_valid, bool heat_stab)
{
    char tmp[20];

    if (msg == NULL || msg_size < 64)
        return false;

    msg[0] = '\0';

    strcat(msg, "P=");
    uint_to_str(pres_adc, tmp);
    strcat(msg, tmp);

    strcat(msg, ",T=");
    uint_to_str(temp_adc, tmp);
    strcat(msg, tmp);

    strcat(msg, ",H=");
    uint_to_str(hum_adc, tmp);
    strcat(msg, tmp);

    strcat(msg, ",G=");
    if (gas_valid && heat_stab)
    {
        uint_to_str(gas_adc, tmp);
        strcat(msg, tmp);
    }
    else
    {
        strcat(msg, "0");
    }

    strcat(msg, ",R=");
    uint_to_str(gas_range, tmp);
    strcat(msg, tmp);

    return true;
}
