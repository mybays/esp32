/* Uart Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

/**
 * This is a example exaple which echos any data it receives on UART1 back to the sender, with hardware flow control
 * turned on. It does not use UART driver event queue.
 *
 * - port: UART1
 * - rx buffer: on
 * - tx buffer: off
 * - flow control: on
 * - event queue: off
 * - pin assignment: txd(io4), rxd(io5), rts(18), cts(19)
 */

#define ECHO_TEST_TXD  (4)
#define ECHO_TEST_RXD  (5)
#define ECHO_TEST_RTS  (18)
#define ECHO_TEST_CTS  (19)

#define BUF_SIZE (1024)


bool check_sum(char *data,int len)
{
    //保证起始符1=0x42,起始符2=0x4d
    if((data[0] != 0x42) || (data[1] != 0x4d))
    {
        printf("header error:%x %x\r\n",data[0],data[1]);
        return false;
    }

    //保证后面的数据长度是28
    int length = (data[2]<<8) + data[3];
    if((28 != length)&& (36 != length))
    {
        printf("length error:%d\r\n",length);
        return false;
    }

    // 保证checksum检验ok
    int sum=0;
    for(int i=0;i<len-2;i++)
    {
        sum += data[i];
    }
    int check=(data[len-2]<<8)+data[len-1];

    if(sum == check)
    {
        return true;
    }
    printf("checksum error:%x  %x\r\n",sum,check);
    return false;
}

void phrase(char *data,int len)
{
    /*
    uint16_t *pvalue;
    for(int i=0;i<16;i++)
    {
        pvalue=(uint16_t *)data+i;
        printf("%x ",*pvalue);
    }
    */
    printf("\r\n");

    int value = (data[4]<<8) + data[5];
    printf("PM1.0(CF=1):%d\r\n",value);

    value = (data[6]<<8) + data[7];
    printf("PM2.5(CF=1):%d\r\n",value);

    value = (data[8]<<8) + data[9];
    printf("PM10 (CF=1):%d\r\n",value);

    value = (data[10]<<8) + data[11];
    printf("PM1.0 (STD):%d\r\n",value);

    value = (data[12]<<8) + data[13];
    printf("PM2.5 (STD):%d\r\n",value);

    value = (data[14]<<8) + data[15];
    printf("PM10 (STD):%d\r\n",value);

    value = (data[16]<<8) + data[17];
    printf(">0.3um     :%d\r\n",value);

    value = (data[18]<<8) + data[19];
    printf(">0.5um     :%d\r\n",value);

    value = (data[20]<<8) + data[21];
    printf(">1.0um     :%d\r\n",value);

    value = (data[22]<<8) + data[23];
    printf(">2.5um     :%d\r\n",value);

    value = (data[24]<<8) + data[25];
    printf(">5.0um     :%d\r\n",value);

    value = (data[26]<<8) + data[27];
    printf(">10um     :%d\r\n",value);

    value = (data[28]<<8) + data[29];
    printf(">jaquan   :%d\r\n",value);

    if(40 == len)
    {
        value = (data[30]<<8) + data[31];
        printf(">temperature   :%d\r\n",value);

        value = (data[32]<<8) + data[33];
        printf(">humidity   :%d\r\n",value);
    }
    
}


//an example of echo test with hardware flow control on UART1
static void echo_task()
{
    const int uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART1 parameters
    uart_param_config(uart_num, &uart_config);
    //Set UART1 pins(TX: IO4, RX: I05, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while(1)
    {
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        //Write data back to UART
        if(0 != len)
        {
            printf("len=%d\r\n",len);
            if((32 == len)||(40 == len))
            {
                bool isok = check_sum((char*)data,len);
                if(isok)
                {
                    phrase((char*)data,len);
                }
                else
                {
                    printf("data error\r\n");
                }
            }
        }
    }
}

void app_main()
{
    //A uart read/write example without event queue;
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
