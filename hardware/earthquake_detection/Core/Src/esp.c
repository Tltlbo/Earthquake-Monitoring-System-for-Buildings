#include <stdio.h>
#include <string.h>

#include "esp.h"

static char ip_addr[16];
static char response[MAX_ESP_RX_BUFFER];
//==================uart2=========================
extern UART_HandleTypeDef huart2;

//==================uart6=========================
static uint8_t data;
cb_data_t cb_data;
/* Optional hook overridden by main.c; startup behavior remains unchanged. */
__weak void esp_background_service(void) {}
extern UART_HandleTypeDef huart6;
static int esp_at_command(uint8_t *cmd, uint8_t *resp, uint16_t *length, int16_t time_out)
{
    *length = 0;
    memset(resp, 0x00, MAX_UART_RX_BUFFER);
    if(strstr((char *)cb_data.buf, "+IPD") == NULL)
    {
        memset(&cb_data, 0x00, sizeof(cb_data_t));
    }
    if(HAL_UART_Transmit(&huart6, cmd, strlen((char *)cmd), 100) != HAL_OK)
    {
        printf("[ESP Send Fail]\r\n");
        return -1;
    }

    uint32_t wait_start = HAL_GetTick();
    while((uint32_t)(HAL_GetTick() - wait_start) < (uint32_t)time_out)
    {
        if(cb_data.length >= MAX_UART_RX_BUFFER)
        {
            printf("[ESP RX Overflow]\r\n");
            return -2;
        }
        else if(strstr((char *)cb_data.buf, "ALREADY CONNECTED") != NULL)
        {
            memcpy(resp, cb_data.buf, cb_data.length);
            *length = cb_data.length;
            return 0;
        }
        else if(strstr((char *)cb_data.buf, "ERROR") != NULL)
        {
            printf("[ESP Received ERROR: %s]\r\n", cb_data.buf);
            return -3;
        }
        else if(strstr((char *)cb_data.buf, "OK") != NULL)
        {
            memcpy(resp, cb_data.buf, cb_data.length);
            *length = cb_data.length;
            return 0;
        }
        esp_background_service();
        HAL_Delay(1);
    }
    printf("[ESP Timeout] Cmd: %s -> Rx %d bytes: [%s]\r\n", (char *)cmd, cb_data.length, (char *)cb_data.buf);
    return -4;
}

static int request_ip_addr(uint8_t is_debug)
{
    uint16_t length = 0;

    if(esp_at_command((uint8_t *)"AT+CIFSR\r\n", (uint8_t *)response, &length, 1000) != 0)
        printf("request ip_addr command fail\r\n");
    else
    {
        char *line = strtok(response, "\r\n");

        if(is_debug)
        {
            for(int i = 0 ; i < length ; i++)
                printf("%c", response[i]);
        }

        while(line != NULL)
        {
            if(strstr(line, "CIFSR:STAIP") != NULL)
            {
                char *ip;

                strtok(line, "\"");
                ip = strtok(NULL, "\"");
                if(strcmp(ip, "0.0.0.0") != 0)
                {
                    memset(ip_addr, 0x00, sizeof(ip_addr));
                    memcpy(ip_addr, ip, strlen(ip));
                    return 0;
                }
            }
            line = strtok(NULL, "\r\n");
        }
    }
    return -1;
}
int esp_client_conn()
{
	char at_cmd[MAX_ESP_COMMAND_LEN] = {0, };
    uint16_t length = 0;

    // 단일 연결 모드 설정 시도; 반환값 검증은 별도 과제
    esp_at_command((uint8_t *)"AT+CIPMUX=0\r\n", (uint8_t *)response, &length, 1000);

	sprintf(at_cmd,"AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",DST_IP,DST_PORT);
    printf("[TCP] Connecting to Server %s:%d...\r\n", DST_IP, DST_PORT);
	int ret = esp_at_command((uint8_t *)at_cmd,(uint8_t *)response, &length, 4000);

    if(ret == 0 || strstr((char *)response, "CONNECT") != NULL || strstr((char *)response, "ALREADY") != NULL)
    {
        printf("[TCP] Server Connected Successfully!\r\n");
        HAL_Delay(200);
        esp_send_data("["LOGID":"PASSWD"]");
        return 0;
    }
    else
    {
        printf("[TCP ERROR] Failed to connect to %s:%d\r\n", DST_IP, DST_PORT);
        return -1;
    }
}
int esp_get_status()
{
	uint16_t length = 0;
	esp_at_command((uint8_t *)"AT+CIPSTATUS\r\n",(uint8_t *)response, &length, 1000);					//CONNECT

    if(strstr((char *)response, "STATUS:3") != NULL)  //STATUS:3 The ESP8266 Station has created a TCP or UDP transmission
    {
    	return 0;
    }
	return -1;
}
int drv_esp_init(void)
{
    uint16_t length = 0;
    uint32_t bauds[] = {115200, 38400, 9600};
    memset(ip_addr, 0x00, sizeof(ip_addr));

    printf("==================================================\r\n");
    printf("[ESP] Starting Auto Baud Rate Scan...\r\n");

    for(int b = 0; b < 3; b++)
    {
        uint32_t baud = bauds[b];
        printf("[ESP] Trying BaudRate: %lu bps...\r\n", baud);

        HAL_UART_AbortReceive(&huart6);
        huart6.Init.BaudRate = baud;
        if(HAL_UART_Init(&huart6) != HAL_OK)
        {
            printf("[ESP] HAL_UART_Init failed for %lu bps\r\n", baud);
            continue;
        }
        HAL_UART_Receive_IT(&huart6, &data, 1);

        for(int retry = 0; retry < 2; retry++)
        {
            if(esp_at_command((uint8_t *)"AT\r\n", (uint8_t *)response, &length, 800) == 0)
            {
                printf("==================================================\r\n");
                printf(">>> [ESP SUCCESS] Connected at %lu bps! <<<\r\n", baud);
                printf("==================================================\r\n");
                return 0;
            }
            HAL_Delay(200);
        }
    }

    printf("==================================================\r\n");
    printf("[ESP FAILED] No response at 115200, 38400, or 9600 bps!\r\n");
    printf("Please check physical wiring:\r\n");
    printf("  1. STM32 PC6 (CN10 Pin 4) -> ESP RXD\r\n");
    printf("  2. STM32 PC7 (CN9 D9)     -> ESP TXD\r\n");
    printf("  3. ESP CH_PD (EN)         -> 3.3V (REQUIRED!)\r\n");
    printf("  4. ESP VCC -> 3.3V, GND   -> GND\r\n");
    printf("==================================================\r\n");
    return -1;
}
void ap_conn_func(char *ssid, char *passwd)
{
  uint16_t length = 0;
  char at_cmd[MAX_ESP_COMMAND_LEN] = {0, };
  if(ssid == NULL || passwd == NULL)
  {
      printf("invalid command : ap_conn <ssid> <passwd>\r\n");
      return;
  }
  printf("[WiFi] Connecting to AP '%s'...\r\n", ssid);
  if(esp_at_command((uint8_t *)"AT+CWMODE=1\r\n", (uint8_t *)response, &length, 1000) != 0)
      printf("Station mode fail\r\n");
  sprintf(at_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid,passwd);
  if(esp_at_command((uint8_t *)at_cmd, (uint8_t *)response, &length, 12000) != 0)
      printf("[WiFi ERROR] AP connection failed : %s\r\n", at_cmd);
  else
      printf("[WiFi] AP connected successfully!\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart->Instance == USART6)
    {
        if(cb_data.length < MAX_ESP_RX_BUFFER - 1)
        {
            cb_data.buf[cb_data.length++] = data;
            cb_data.buf[cb_data.length] = '\0';
        }

        HAL_UART_Receive_IT(huart, &data, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UART_Receive_IT(huart, &data, 1);
    }
}

void AiotClient_Init()
{
    uint16_t length = 0;
    printf("==================================================\r\n");
    printf("[AiotClient] Initializing WiFi & TCP Connection...\r\n");

    // 단일 연결 모드(CIPMUX=0) 설정 시도
    esp_at_command((uint8_t *)"AT+CIPMUX=0\r\n", (uint8_t *)response, &length, 1000);

	ap_conn_func(SSID, PASS);
	request_ip_addr(1);
	esp_client_conn();
	esp_get_status();
    printf("==================================================\r\n");
}

void esp_send_data(char *data)
{
	char at_cmd[MAX_ESP_COMMAND_LEN] = {0, };
	uint16_t length = 0;
	sprintf(at_cmd,"AT+CIPSEND=%d\r\n",strlen(data));
	if(esp_at_command((uint8_t *)at_cmd,(uint8_t *)response, &length, 1000) == 0)
	{
		esp_at_command((uint8_t *)data,(uint8_t *)response, &length, 1000);
	}
}

int __io_putchar(int ch)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10) == HAL_OK)
        return ch;
    return -1;
}

