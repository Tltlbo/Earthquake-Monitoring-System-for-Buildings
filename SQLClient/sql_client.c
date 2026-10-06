#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>

#define BUF_SIZE 200
#define NAME_SIZE 20
#define ARR_CNT 15

void * send_msg(void * arg);
void * recv_msg(void * arg);
void error_handling(char * msg);
void finish_with_error(MYSQL *con);

char name[NAME_SIZE]="[Default]";
char msg[BUF_SIZE];

int main(int argc, char *argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread;
	void * thread_return;

	if(argc != 4) {
		printf("Usage : %s <IP> <port> <name>\n",argv[0]);
		exit(1);
	}

	sprintf(name, "%s",argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if(sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family=AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg,"[%s:PASSWD]",name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void *)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void *)&sock);

	pthread_join(snd_thread, &thread_return);
	//	pthread_join(rcv_thread, &thread_return);

	close(sock);
	return 0;
}

void * send_msg(void * arg)
{
	int *sock = (int *)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE+2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n",stdout);
	while(1) {
		memset(msg,0,sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if(FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if(!strncmp(msg,"quit\n",5)) {
				*sock = -1;
				return NULL;
			}
			else if(msg[0] != '[')
			{
				strcat(name_msg,"[ALLMSG]");
				strcat(name_msg,msg);
			}
			else
				strcpy(name_msg,msg);
			if(write(*sock, name_msg, strlen(name_msg))<=0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if(ret == 0) 
		{
			if(*sock == -1) 
				return NULL;
		}
	}
}

void * recv_msg(void * arg)
{
	int * sock = (int *)arg;	
	int i;
	char *pToken;
	char *pArray[ARR_CNT]={0};
	MYSQL *con;
        int res;
//        int illu;
//        char name[10];
//        float temp;
//        float humi;
        char sql_cmd[200];

	char name_msg[NAME_SIZE + BUF_SIZE +1];
	int str_len;
	while(1) {
		memset(name_msg,0x0,sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE );
		if(str_len <= 0) 
		{
			*sock = -1;
			return NULL;
		}
		name_msg[str_len] = 0;
		fputs(name_msg, stdout);
		name_msg[strcspn(name_msg,"\r\n")]='\0';

		pToken = strtok(name_msg,"[@]");
		i = 0;
		while(pToken != NULL)
		{
			pArray[i] =  pToken;
			if(i++ >= ARR_CNT)
				break;
			pToken = strtok(NULL,"[@]");
		}

//		printf("id:%s, msg:%s,%s,%s,%s\n",pArray[0],pArray[1],pArray[2],pArray[3],pArray[4]);

		con = mysql_init(NULL);
		if (con == NULL)
		{
			fprintf(stderr, "mysql_init() failed\n");
			exit(1);
		}

		if (mysql_real_connect(con, "127.0.0.1", "iot", "pwiot", "sensor_data", 0, NULL, 0) == NULL)
		{
			finish_with_error(con);
		}

		if(!strcmp(pArray[1],"SERVOS"))
		{
			// 수신 데이터 형식: [ID]SERVOS@motor1@motor2@motor3@motor4@gyroX@gyroY@gyroZ@vibration@status
			// pArray[0]=ID, [1]=SENSOR, [2~10]=Data
			sprintf(sql_cmd,
				"INSERT INTO sensor_logs(record_date, motor1, motor2, motor3, motor4, gyro_x, gyro_y, gyro_z, vibration, jetson_status) "
				"VALUES(now(), %s, %s, %s, %s, %s, %s, %s, %s, '%s')",
				pArray[2], pArray[3], pArray[4], pArray[5], 
				pArray[6], pArray[7], pArray[8], pArray[9], pArray[10]);

			res = mysql_query(con, sql_cmd);
			if(!res)
				printf("inserted %lu rows\n",(unsigned long)mysql_affected_rows(con));
			else
				finish_with_error(con);
		}
		//req:[PJS_ARD]GETDB@LAMP
		//res:[{PJS_ARD]GETDB@LAMP@ON
		else if(!strcmp(pArray[1],"GETDB"))
		{
			sprintf(sql_cmd,"SELECT value FROM device WHERE name='%s'",pArray[2]);
			if(mysql_query(con, sql_cmd))
				finish_with_error(con);

			MYSQL_RES *result = mysql_store_result(con);
			if (result == NULL)
				finish_with_error(con);
			MYSQL_ROW row = mysql_fetch_row(result);
			sprintf(sql_cmd,"[%s]%s@%s@%s\n",pArray[0],pArray[1],pArray[2],row[0]);
			write(*sock, sql_cmd, strlen(sql_cmd));
		}
		//req:[PJS_ARD]SETDB@LAMP@ON
		//res:[PJS_ARD]GETDB@LAMP@ON
		else if(!strcmp(pArray[1],"SETDB"))
		{
			sprintf(sql_cmd,"UPDATE device SET value='%s', date=now(), time=now() WHERE name='%s'", pArray[3], pArray[2]);
			if(mysql_query(con, sql_cmd))
				finish_with_error(con);

			if (i == 4) {
				sprintf(sql_cmd,"[%s]%s@%s@%s\n",pArray[0],pArray[1],pArray[2],pArray[3]);
			}
			else if (i == 5) {
				sprintf(sql_cmd, "[%s]%s@%s\n", pArray[4], pArray[2], pArray[3]);
			}
			write(*sock, sql_cmd, strlen(sql_cmd));
		}
		mysql_close(con);
	}
}

void error_handling(char * msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}

void finish_with_error(MYSQL *con)
{
        fprintf(stderr, "%s\n", mysql_error(con));
        mysql_close(con);
        exit(1);
}
