/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @defgroup temperature_example_main main.c
* @{
* @ingroup temperature_example
* @brief Temperature Example Application main file.
* @details
* This file contains the source code for a sample application using the temperature sensor.
* This contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31. PAN 43 is not covered.
*  - PAN_028 rev2.0A anomaly 28 - TEMP: Negative measured values are not represented correctly
*  - PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register.   
*  - PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs.
*  - PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module
*  - PAN_028 rev2.0A anomaly 43 - TEMP: Using PPI between DATARDY event and START task is not functional. 
*
*/

#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_temp.h"
#include "app_uart.h"
#include "app_error.h"
#include "spi_master.h"

#include "lis3dh.h"

#define UART_TX_BUF_SIZE 256                                                          /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                            /**< UART RX buffer size. */

#define RX_PIN_NUMBER 8
#define TX_PIN_NUMBER 9
#define RTS_PIN_NUMBER 0xFF
#define CTS_PIN_NUMBER 0xFF
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
/** @brief Function for writing a single register to LIS3DH.
 */
static uint8_t mosiBuf [2];
static uint8_t misoBuf [2];
void writeLisReg(uint8_t subAddr, uint8_t value)
{
   uint32_t err_code;
	 while(spi_master_get_state(SPI_MASTER_0) != SPI_MASTER_STATE_IDLE);
	 mosiBuf[0] = subAddr & ~0xc0;
	 mosiBuf[1] = value;
	 err_code = spi_master_send_recv(SPI_MASTER_0, mosiBuf, 2, misoBuf,2);
	 APP_ERROR_CHECK(err_code);
}

uint8_t readLisReg(uint8_t subAddr)
{
   uint32_t err_code;
	 while(spi_master_get_state(SPI_MASTER_0) != SPI_MASTER_STATE_IDLE);
	 mosiBuf[0] = subAddr | 0x80;
	 mosiBuf[1] = 0;
	misoBuf[0] = 0x55;
	misoBuf[1] = 0xAA;
	 err_code = spi_master_send_recv(SPI_MASTER_0, mosiBuf, 2, misoBuf,2);
	 APP_ERROR_CHECK(err_code);
	 while(spi_master_get_state(SPI_MASTER_0) != SPI_MASTER_STATE_IDLE);
	 return misoBuf[1];
}

void init_lis3dh() {
	writeLisReg(LIS3DH_CTRL_REG1, 0x57); //All axes, normal, 100Hz
	writeLisReg(LIS3DH_CTRL_REG2, 0x00); // No HighPass filter
	writeLisReg(LIS3DH_CTRL_REG3, 0x00); // No interrupts
	writeLisReg(LIS3DH_CTRL_REG4, 0x0); // all defaults
	writeLisReg(LIS3DH_CTRL_REG5, 0x0); // all defaults
	writeLisReg(LIS3DH_CTRL_REG6, 0x0); // all defaults
}

void readLisData()
{
	int16_t x,y,z;
	double dx, dy, dz;
	while(readLisReg(LIS3DH_STATUS_REG) & 0x8 == 0 );
	x = readLisReg(LIS3DH_OUT_X_H) * 256 +  readLisReg(LIS3DH_OUT_X_L);
	y = readLisReg(LIS3DH_OUT_Y_H) * 256 +  readLisReg(LIS3DH_OUT_Y_L);
	z = readLisReg(LIS3DH_OUT_Z_H) * 256 +  readLisReg(LIS3DH_OUT_Z_L);
	dx = x/16384.0;
	dy = y/16384.0;
	dz = z/16384.0;
	printf("X: %f; Y: %f; Z: %f; SUM: %f\n\r", dx , dy, dz, sqrt(dx*dx + dy*dy + dz*dz));
}
/** @brief Function for main application entry.
 */
int main(void)
{
    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t volatile temp;

    nrf_temp_init();
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
     {
         RX_PIN_NUMBER,
         TX_PIN_NUMBER,
         RTS_PIN_NUMBER,
         CTS_PIN_NUMBER,
         APP_UART_FLOW_CONTROL_DISABLED,
         false,
         UART_BAUDRATE_BAUDRATE_Baud230400
     };
		 
		 const spi_master_config_t spi_param =
		{
			SPI_FREQUENCY_FREQUENCY_M1, /**< SPI master frequency */
			0,												  /**< SCK pin number. */
			1,													/**< MISO pin number. */
			2,													/**< MOSI pin number .*/
			3,													/**< Slave select pin number. */
			APP_IRQ_PRIORITY_LOW,				/**< SPI master interrupt priority. */
			SPI_CONFIG_ORDER_MsbFirst,	/**< Bytes order LSB or MSB shifted out first. */
			SPI_CONFIG_CPOL_ActiveLow,	/**< Serial clock polarity ACTIVEHIGH or ACTIVELOW. */
			SPI_CONFIG_CPHA_Trailing,    /**< Serial clock phase LEADING or TRAILING. */
			1  													/**< Disable all IRQs in critical section. */
		};

    APP_UART_FIFO_INIT(&comm_params,
                    UART_RX_BUF_SIZE,
                    UART_TX_BUF_SIZE,
                    uart_error_handle,
                    APP_IRQ_PRIORITY_LOW,
                    err_code);

    APP_ERROR_CHECK(err_code);
		err_code = spi_master_open(SPI_MASTER_0, &spi_param);
    APP_ERROR_CHECK(err_code);
		nrf_delay_ms(500);
		init_lis3dh();
		printf("WHO_AM_I: %x\n", readLisReg(LIS3DH_WHO_AM_I));
		
    while (true)
    {
        NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        while (NRF_TEMP->EVENTS_DATARDY == 0)
        {
            // Do nothing.
        }
        NRF_TEMP->EVENTS_DATARDY = 0;

        /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */
        temp = (nrf_temp_read() / 4);

        /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
        NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

        printf("Actual temperature: %d\n\r", (int)temp);
				readLisData();
//        nrf_delay_ms(500);
    }
}


/** @} */
