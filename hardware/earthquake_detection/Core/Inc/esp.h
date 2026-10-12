#ifndef __ESP_H__
#define __ESP_H__

#include "stm32f4xx_hal.h"

#define MAX_ESP_RX_BUFFER      1024
#define MAX_ESP_COMMAND_LEN    64

#define SSID "kcci603"
#define PASS "@kcci603!"
#define LOGID "CSH_ARD"
#define PASSWD "PASSWD"
#define DST_IP "10.10.16.74"
#define DST_PORT 5000

typedef struct _cb_data_t
{
    uint8_t buf[MAX_ESP_RX_BUFFER];
    uint16_t length;
}cb_data_t;

int drv_esp_init(void);
void AiotClient_Init(void);
int esp_client_conn(void);
void esp_send_data(char *data);
int esp_get_status(void);

/* ESP 응답 제한과 서보 명령 버퍼 크기 */
#define MAX_UART_RX_BUFFER      512
#define MAX_UART_COMMAND_LEN    64

#endif
