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
#include <time.h>
#include <math.h>

#define BUF_SIZE 300
#define NAME_SIZE 20
#define ARR_CNT 20

/* =========================================================
 * 센서 클라이언트 ID
 * ========================================================= */
#define SENSOR_CLIENT_ID "PJS_STM"


/* =========================================================
 * 데이터 유효 시간
 * ========================================================= */
#define SENSOR_TIMEOUT_SEC 1.0
#define JETSON_TIMEOUT_SEC 3.0


/* =========================================================
 * 지진 판정 임계값
 *
 * 주의:
 * 아래 값은 예시값.
 * 실제 센서 측정값을 보고 보정해야 함.
 * ========================================================= */
#define GYRO_CAUTION_THRESHOLD  2.0f
#define GYRO_WARNING_THRESHOLD  5.0f

#define VIBRATION_CAUTION_THRESHOLD 1
#define VIBRATION_WARNING_THRESHOLD 2

#define MOTOR_CAUTION_THRESHOLD 20.0f
#define MOTOR_WARNING_THRESHOLD 40.0f


/* =========================================================
 * 지진 상태
 * ========================================================= */
typedef enum
{
    EARTHQUAKE_SAFETY = 0,
    EARTHQUAKE_CAUTION,
    EARTHQUAKE_WARNING

} EarthquakeLevel;


/* =========================================================
 * Sensor Data
 * ========================================================= */
typedef struct
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;

    float motor5;
    float motor6;
    float motor7;
    float motor8;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    int vibration;

    double timestamp;

    unsigned long sequence;

    int valid;

} SensorData;


/* =========================================================
 * Jetson Data
 * ========================================================= */
typedef struct
{
    char status[20];

    double timestamp;

    int valid;

} JetsonData;


/* =========================================================
 * 함수 선언
 * ========================================================= */
void *send_msg(void *arg);
void *recv_msg(void *arg);

void error_handling(char *msg);
void finish_with_error(MYSQL *con);

void socket_send(
    int sock,
    const char *data
);

double get_time_sec(void);

void process_packet(
    int sock,
    MYSQL *con,
    char *packet
);

void process_sensor(
    int sock,
    MYSQL *con,
    char **pArray,
    int count
);

void process_status(
    int sock,
    MYSQL *con,
    char **pArray,
    int count
);

void try_process_integrated_data(
    int sock,
    MYSQL *con
);

EarthquakeLevel detect_earthquake(
    SensorData *sensor,
    JetsonData *jetson
);

void control_servos(
    int sock,
    EarthquakeLevel level
);

void send_servo_command(
    int sock,
    int angle
);

void save_database(
    MYSQL *con,
    SensorData *sensor,
    JetsonData *jetson,
    EarthquakeLevel level
);

const char *earthquake_level_string(
    EarthquakeLevel level
);


/* =========================================================
 * Global
 * ========================================================= */
char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

SensorData latest_sensor = {0};
JetsonData latest_jetson = {0};

/*
 * SENSOR 하나가 중복 처리되는 것을 방지
 */
unsigned long last_processed_sensor_sequence = 0;


/*
 * 이전 지진 상태
 *
 * 상태가 바뀌었을 때만 Servo 명령 전송
 */
EarthquakeLevel previous_level = -1;


/*
 * send thread / recv thread가
 * 동시에 socket write 하는 것을 방지
 */
pthread_mutex_t socket_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* =========================================================
 * MAIN
 * ========================================================= */
int main(int argc, char *argv[])
{
    int sock;

    struct sockaddr_in serv_addr;

    pthread_t snd_thread;
    pthread_t rcv_thread;

    void *thread_return;


    if(argc != 4)
    {
        printf(
            "Usage : %s <IP> <port> <name>\n",
            argv[0]
        );

        exit(1);
    }


    snprintf(
        name,
        sizeof(name),
        "%s",
        argv[3]
    );


    /* =====================================================
     * Socket 생성
     * ===================================================== */
    sock = socket(
        PF_INET,
        SOCK_STREAM,
        0
    );


    if(sock == -1)
    {
        error_handling(
            "socket() error"
        );
    }


    memset(
        &serv_addr,
        0,
        sizeof(serv_addr)
    );


    serv_addr.sin_family =
        AF_INET;


    serv_addr.sin_addr.s_addr =
        inet_addr(argv[1]);


    serv_addr.sin_port =
        htons(
            atoi(argv[2])
        );


    /* =====================================================
     * 서버 연결
     * ===================================================== */
    if(
        connect(
            sock,
            (struct sockaddr *)&serv_addr,
            sizeof(serv_addr)
        ) == -1
    )
    {
        error_handling(
            "connect() error"
        );
    }


    /* =====================================================
     * 로그인
     * ===================================================== */
    snprintf(
        msg,
        sizeof(msg),
        "[%s:PASSWD]",
        name
    );


    socket_send(
        sock,
        msg
    );


    /* =====================================================
     * Thread 시작
     * ===================================================== */
    pthread_create(
        &rcv_thread,
        NULL,
        recv_msg,
        (void *)&sock
    );


    pthread_create(
        &snd_thread,
        NULL,
        send_msg,
        (void *)&sock
    );


    pthread_join(
        snd_thread,
        &thread_return
    );


    close(sock);


    return 0;
}


/* =========================================================
 * Socket Send
 * ========================================================= */
void socket_send(
    int sock,
    const char *data
)
{
    pthread_mutex_lock(
        &socket_mutex
    );


    write(
        sock,
        data,
        strlen(data)
    );


    pthread_mutex_unlock(
        &socket_mutex
    );
}


/* =========================================================
 * Keyboard Send Thread
 * ========================================================= */
void *send_msg(void *arg)
{
    int *sock =
        (int *)arg;

    int ret;

    fd_set initset;
    fd_set newset;

    struct timeval tv;

    char name_msg[
        NAME_SIZE + BUF_SIZE + 2
    ];


    FD_ZERO(
        &initset
    );


    FD_SET(
        STDIN_FILENO,
        &initset
    );


    fputs(
        "Input a message! [ID]msg (Default ID:ALLMSG)\n",
        stdout
    );


    while(1)
    {
        memset(
            msg,
            0,
            sizeof(msg)
        );


        memset(
            name_msg,
            0,
            sizeof(name_msg)
        );


        tv.tv_sec = 1;
        tv.tv_usec = 0;


        newset =
            initset;


        ret =
            select(
                STDIN_FILENO + 1,
                &newset,
                NULL,
                NULL,
                &tv
            );


        if(
            FD_ISSET(
                STDIN_FILENO,
                &newset
            )
        )
        {
            fgets(
                msg,
                BUF_SIZE,
                stdin
            );


            if(
                !strncmp(
                    msg,
                    "quit\n",
                    5
                )
            )
            {
                *sock = -1;

                return NULL;
            }


            else if(msg[0] != '[')
            {
                strcat(
                    name_msg,
                    "[ALLMSG]"
                );


                strcat(
                    name_msg,
                    msg
                );
            }


            else
            {
                strcpy(
                    name_msg,
                    msg
                );
            }


            socket_send(
                *sock,
                name_msg
            );
        }


        if(ret == 0)
        {
            if(*sock == -1)
            {
                return NULL;
            }
        }
    }
}


/* =========================================================
 * Receive Thread
 *
 * TCP는 read() 1번 = packet 1개가 아님.
 *
 * 따라서 반드시 송신 데이터 끝에 \n을 붙이는 방식 권장.
 *
 * 예:
 *
 * [PJS_SQL]SENSOR@...\n
 * [PJS_SQL]STATUS@WARNING\n
 * ========================================================= */
void *recv_msg(void *arg)
{
    int *sock =
        (int *)arg;


    MYSQL *con;


    /* =====================================================
     * DB 연결
     *
     * 매 패킷마다 connect하지 않고
     * 한번 연결 후 계속 사용
     * ===================================================== */
    con =
        mysql_init(NULL);


    if(con == NULL)
    {
        fprintf(
            stderr,
            "mysql_init() failed\n"
        );

        return NULL;
    }


    if(
        mysql_real_connect(
            con,

            "127.0.0.1",
            "iot",
            "pwiot",
            "sensor_data",

            0,
            NULL,
            0
        ) == NULL
    )
    {
        finish_with_error(
            con
        );
    }


    printf(
        "MySQL Connected\n"
    );


    char recv_buffer[
        BUF_SIZE + 1
    ];


    char packet_buffer[
        BUF_SIZE * 5
    ];


    int packet_len = 0;


    while(1)
    {
        int str_len;


        memset(
            recv_buffer,
            0,
            sizeof(recv_buffer)
        );


        str_len =
            read(
                *sock,
                recv_buffer,
                BUF_SIZE
            );


        if(str_len <= 0)
        {
            *sock = -1;


            mysql_close(
                con
            );


            return NULL;
        }


        /* =================================================
         * TCP 누적
         * ================================================= */
        for(
            int j = 0;
            j < str_len;
            j++
        )
        {
            char c =
                recv_buffer[j];


            /*
             * 한 packet 종료
             */
            if(c == '\n')
            {
                packet_buffer[
                    packet_len
                ] = '\0';


                /*
                 * \r 제거
                 */
                if(
                    packet_len > 0 &&
                    packet_buffer[
                        packet_len - 1
                    ] == '\r'
                )
                {
                    packet_buffer[
                        packet_len - 1
                    ] = '\0';
                }


                if(packet_len > 0)
                {
                    process_packet(
                        *sock,
                        con,
                        packet_buffer
                    );
                }


                packet_len = 0;
            }


            else
            {
                if(
                    packet_len <
                    (int)sizeof(packet_buffer) - 1
                )
                {
                    packet_buffer[
                        packet_len++
                    ] = c;
                }


                else
                {
                    printf(
                        "Packet Buffer Overflow\n"
                    );


                    packet_len = 0;
                }
            }
        }
    }
}


/* =========================================================
 * Packet Parsing
 *
 * SENSOR
 *
 * [PJS_STM]SENSOR@
 * M1@M2@M3@M4@
 * M5@M6@M7@M8@
 * GX@GY@GZ@
 * vibration
 *
 *
 * Jetson
 *
 * [PJS_JET]STATUS@WARNING
 *
 * ========================================================= */
void process_packet(
    int sock,
    MYSQL *con,
    char *packet
)
{
    char packet_copy[
        BUF_SIZE * 5
    ];


    char *pArray[
        ARR_CNT
    ] = {0};


    char *pToken;


    int count = 0;


    snprintf(
        packet_copy,
        sizeof(packet_copy),
        "%s",
        packet
    );


    printf(
        "\nRX : %s\n",
        packet_copy
    );


    /*
     * [
     * ]
     * @
     *
     * delimiter
     */
    pToken =
        strtok(
            packet_copy,
            "[@]"
        );


    while(
        pToken != NULL &&
        count < ARR_CNT
    )
    {
        pArray[count++] =
            pToken;


        pToken =
            strtok(
                NULL,
                "[@]"
            );
    }


    if(count < 2)
    {
        printf(
            "Invalid Packet\n"
        );

        return;
    }


    printf(
        "ID  : %s\n",
        pArray[0]
    );


    printf(
        "CMD : %s\n",
        pArray[1]
    );


    /* =====================================================
     * SENSOR
     * ===================================================== */
    if(
        !strcmp(
            pArray[1],
            "SENSOR"
        )
    )
    {
        process_sensor(
            sock,
            con,
            pArray,
            count
        );
    }


    /* =====================================================
     * Jetson Status
     * ===================================================== */
    else if(
        !strcmp(
            pArray[1],
            "STATUS"
        )
    )
    {
        process_status(
            sock,
            con,
            pArray,
            count
        );
    }


    /* =====================================================
     * 기존 GETDB
     * ===================================================== */
    else if(
        !strcmp(
            pArray[1],
            "GETDB"
        )
    )
    {
        if(count < 3)
            return;


        char sql_cmd[
            200
        ];


        snprintf(
            sql_cmd,
            sizeof(sql_cmd),

            "SELECT value "
            "FROM device "
            "WHERE name='%s'",

            pArray[2]
        );


        if(
            mysql_query(
                con,
                sql_cmd
            )
        )
        {
            fprintf(
                stderr,
                "GETDB ERROR : %s\n",
                mysql_error(con)
            );

            return;
        }


        MYSQL_RES *result =
            mysql_store_result(
                con
            );


        if(result == NULL)
        {
            fprintf(
                stderr,
                "mysql_store_result ERROR : %s\n",
                mysql_error(con)
            );

            return;
        }


        MYSQL_ROW row =
            mysql_fetch_row(
                result
            );


        if(row != NULL)
        {
            snprintf(
                sql_cmd,
                sizeof(sql_cmd),

                "[%s]%s@%s@%s\n",

                pArray[0],
                pArray[1],
                pArray[2],
                row[0]
            );


            socket_send(
                sock,
                sql_cmd
            );
        }


        mysql_free_result(
            result
        );
    }


    /* =====================================================
     * 기존 SETDB
     * ===================================================== */
    else if(
        !strcmp(
            pArray[1],
            "SETDB"
        )
    )
    {
        if(count < 4)
            return;


        char sql_cmd[
            200
        ];


        snprintf(
            sql_cmd,
            sizeof(sql_cmd),

            "UPDATE device "
            "SET value='%s', "
            "date=now(), "
            "time=now() "
            "WHERE name='%s'",

            pArray[3],
            pArray[2]
        );


        if(
            mysql_query(
                con,
                sql_cmd
            )
        )
        {
            fprintf(
                stderr,
                "SETDB ERROR : %s\n",
                mysql_error(con)
            );

            return;
        }


        snprintf(
            sql_cmd,
            sizeof(sql_cmd),

            "[%s]SETDB@%s@%s\n",

            pArray[0],
            pArray[2],
            pArray[3]
        );


        socket_send(
            sock,
            sql_cmd
        );
    }
}


/* =========================================================
 * SENSOR 처리
 *
 * pArray
 *
 * 0  = ID
 * 1  = SENSOR
 *
 * 2  = motor1
 * 3  = motor2
 * 4  = motor3
 * 5  = motor4
 *
 * 6  = motor5
 * 7  = motor6
 * 8  = motor7
 * 9  = motor8
 *
 * 10 = gyro_x
 * 11 = gyro_y
 * 12 = gyro_z
 *
 * 13 = vibration
 *
 * ========================================================= */
void process_sensor(
    int sock,
    MYSQL *con,
    char **pArray,
    int count
)
{
    if(count < 14)
    {
        printf(
            "SENSOR Packet Error : count=%d\n",
            count
        );

        return;
    }


    latest_sensor.motor1 =
        atof(
            pArray[2]
        );


    latest_sensor.motor2 =
        atof(
            pArray[3]
        );


    latest_sensor.motor3 =
        atof(
            pArray[4]
        );


    latest_sensor.motor4 =
        atof(
            pArray[5]
        );


    latest_sensor.motor5 =
        atof(
            pArray[6]
        );


    latest_sensor.motor6 =
        atof(
            pArray[7]
        );


    latest_sensor.motor7 =
        atof(
            pArray[8]
        );


    latest_sensor.motor8 =
        atof(
            pArray[9]
        );


    latest_sensor.gyro_x =
        atof(
            pArray[10]
        );


    latest_sensor.gyro_y =
        atof(
            pArray[11]
        );


    latest_sensor.gyro_z =
        atof(
            pArray[12]
        );


    latest_sensor.vibration =
        atoi(
            pArray[13]
        );


    latest_sensor.timestamp =
        get_time_sec();


    latest_sensor.sequence++;


    latest_sensor.valid = 1;


    printf(
        "\n===== SENSOR UPDATE =====\n"
    );


    printf(
        "MOTOR : "
        "%.2f %.2f %.2f %.2f "
        "%.2f %.2f %.2f %.2f\n",

        latest_sensor.motor1,
        latest_sensor.motor2,
        latest_sensor.motor3,
        latest_sensor.motor4,

        latest_sensor.motor5,
        latest_sensor.motor6,
        latest_sensor.motor7,
        latest_sensor.motor8
    );


    printf(
        "GYRO  : %.3f %.3f %.3f\n",

        latest_sensor.gyro_x,
        latest_sensor.gyro_y,
        latest_sensor.gyro_z
    );


    printf(
        "VIB   : %d\n",
        latest_sensor.vibration
    );


    /*
     * Jetson 데이터와 통합 가능한지 확인
     */
    try_process_integrated_data(
        sock,
        con
    );
}


/* =========================================================
 * Jetson STATUS 처리
 *
 * [PJS_JET]STATUS@SAFETY
 * [PJS_JET]STATUS@CAUTION
 * [PJS_JET]STATUS@WARNING
 * ========================================================= */
void process_status(
    int sock,
    MYSQL *con,
    char **pArray,
    int count
)
{
    if(count < 3)
    {
        printf(
            "STATUS Packet Error\n"
        );

        return;
    }


    snprintf(
        latest_jetson.status,
        sizeof(latest_jetson.status),
        "%s",
        pArray[2]
    );


    latest_jetson.timestamp =
        get_time_sec();


    latest_jetson.valid = 1;


    printf(
        "\n===== JETSON UPDATE =====\n"
    );


    printf(
        "STATUS : %s\n",
        latest_jetson.status
    );


    /*
     * SENSOR가 먼저 와있을 수도 있으므로
     * STATUS 수신 시에도 통합 검사
     */
    try_process_integrated_data(
        sock,
        con
    );
}


/* =========================================================
 * SENSOR + JETSON 통합
 * ========================================================= */
void try_process_integrated_data(
    int sock,
    MYSQL *con
)
{
    /*
     * 둘 다 한번 이상 수신해야 함
     */
    if(
        !latest_sensor.valid ||
        !latest_jetson.valid
    )
    {
        return;
    }


    /*
     * 이미 처리한 SENSOR 샘플
     */
    if(
        latest_sensor.sequence ==
        last_processed_sensor_sequence
    )
    {
        return;
    }


    double now =
        get_time_sec();


    double sensor_age =
        now -
        latest_sensor.timestamp;


    double jetson_age =
        now -
        latest_jetson.timestamp;


    /*
     * 오래된 센서 데이터
     */
    if(
        sensor_age >
        SENSOR_TIMEOUT_SEC
    )
    {
        printf(
            "Sensor Data Stale : %.2f sec\n",
            sensor_age
        );

        return;
    }


    /*
     * 오래된 Jetson 상태
     */
    if(
        jetson_age >
        JETSON_TIMEOUT_SEC
    )
    {
        printf(
            "Jetson Data Stale : %.2f sec\n",
            jetson_age
        );

        return;
    }


    /* =====================================================
     * 지진 판단
     * ===================================================== */
    EarthquakeLevel level =
        detect_earthquake(
            &latest_sensor,
            &latest_jetson
        );


    printf(
        "\n"
        "========================================\n"
        "       EARTHQUAKE INTEGRATED DATA\n"
        "========================================\n"
    );


    printf(
        "Motor : "
        "%.2f %.2f %.2f %.2f "
        "%.2f %.2f %.2f %.2f\n",

        latest_sensor.motor1,
        latest_sensor.motor2,
        latest_sensor.motor3,
        latest_sensor.motor4,

        latest_sensor.motor5,
        latest_sensor.motor6,
        latest_sensor.motor7,
        latest_sensor.motor8
    );


    printf(
        "Gyro      : %.3f %.3f %.3f\n",

        latest_sensor.gyro_x,
        latest_sensor.gyro_y,
        latest_sensor.gyro_z
    );


    printf(
        "Vibration : %d\n",
        latest_sensor.vibration
    );


    printf(
        "Jetson    : %s\n",
        latest_jetson.status
    );


    printf(
        "Result    : %s\n",
        earthquake_level_string(
            level
        )
    );


    printf(
        "========================================\n\n"
    );


    /* =====================================================
     * DB 저장
     * ===================================================== */
    save_database(
        con,
        &latest_sensor,
        &latest_jetson,
        level
    );


    /*
     * 이 SENSOR는 처리 완료
     */
    last_processed_sensor_sequence =
        latest_sensor.sequence;


    /* =====================================================
     * SERVO 제어
     *
     * 상태가 바뀌었을 때만 명령 전송
     * ===================================================== */
    if(
        level !=
        previous_level
    )
    {
        control_servos(
            sock,
            level
        );


        previous_level =
            level;
    }
}


/* =========================================================
 * 지진 판단
 *
 * 여러 센서를 score 방식으로 통합
 *
 * 예시:
 *
 * Jetson WARNING  +4
 * Jetson CAUTION  +2
 *
 * Vibration 위험 +4
 * Vibration 주의 +2
 *
 * Gyro 위험      +4
 * Gyro 주의      +2
 *
 * Motor 편차 위험 +3
 * Motor 편차 주의 +1
 *
 * 총점
 *
 * 0~2  : SAFETY
 * 3~5  : CAUTION
 * >=6  : WARNING
 * ========================================================= */
EarthquakeLevel detect_earthquake(
    SensorData *sensor,
    JetsonData *jetson
)
{
    int score = 0;


    /* =====================================================
     * Jetson
     * ===================================================== */
    if(
        !strcmp(
            jetson->status,
            "WARNING"
        )
    )
    {
        score += 4;
    }


    else if(
        !strcmp(
            jetson->status,
            "CAUTION"
        )
    )
    {
        score += 2;
    }


    /* =====================================================
     * Vibration
     * ===================================================== */
    if(
        sensor->vibration >=
        VIBRATION_WARNING_THRESHOLD
    )
    {
        score += 4;
    }


    else if(
        sensor->vibration >=
        VIBRATION_CAUTION_THRESHOLD
    )
    {
        score += 2;
    }


    /* =====================================================
     * Gyro
     * ===================================================== */
    float gyro_magnitude =
        sqrtf(
            sensor->gyro_x *
            sensor->gyro_x

            +

            sensor->gyro_y *
            sensor->gyro_y

            +

            sensor->gyro_z *
            sensor->gyro_z
        );


    if(
        gyro_magnitude >=
        GYRO_WARNING_THRESHOLD
    )
    {
        score += 4;
    }


    else if(
        gyro_magnitude >=
        GYRO_CAUTION_THRESHOLD
    )
    {
        score += 2;
    }


    /* =====================================================
     * Motor 8개 편차
     * ===================================================== */
    float motors[8] =
    {
        sensor->motor1,
        sensor->motor2,
        sensor->motor3,
        sensor->motor4,

        sensor->motor5,
        sensor->motor6,
        sensor->motor7,
        sensor->motor8
    };


    float motor_min =
        motors[0];


    float motor_max =
        motors[0];


    for(
        int i = 1;
        i < 8;
        i++
    )
    {
        if(
            motors[i] <
            motor_min
        )
        {
            motor_min =
                motors[i];
        }


        if(
            motors[i] >
            motor_max
        )
        {
            motor_max =
                motors[i];
        }
    }


    float motor_diff =
        motor_max -
        motor_min;


    if(
        motor_diff >=
        MOTOR_WARNING_THRESHOLD
    )
    {
        score += 3;
    }


    else if(
        motor_diff >=
        MOTOR_CAUTION_THRESHOLD
    )
    {
        score += 1;
    }


    /* =====================================================
     * Debug
     * ===================================================== */
    printf(
        "\n===== EARTHQUAKE CHECK =====\n"
    );


    printf(
        "Jetson Status : %s\n",
        jetson->status
    );


    printf(
        "Vibration     : %d\n",
        sensor->vibration
    );


    printf(
        "Gyro Mag      : %.3f\n",
        gyro_magnitude
    );


    printf(
        "Motor Diff    : %.3f\n",
        motor_diff
    );


    printf(
        "Score         : %d\n",
        score
    );


    /* =====================================================
     * 최종 판단
     * ===================================================== */
    if(score >= 6)
    {
        printf(
            "Decision      : WARNING\n"
        );


        return
            EARTHQUAKE_WARNING;
    }


    else if(score >= 3)
    {
        printf(
            "Decision      : CAUTION\n"
        );


        return
            EARTHQUAKE_CAUTION;
    }


    printf(
        "Decision      : SAFETY\n"
    );


    return
        EARTHQUAKE_SAFETY;
}


/* =========================================================
 * Servo Control
 *
 * 현재 요구사항:
 *
 * 정상 → 0도
 * 지진 감지 → 180도
 *
 * 여기서는 WARNING만 실제 지진으로 판단.
 *
 * CAUTION은 아직 0도 유지.
 * ========================================================= */
void control_servos(
    int sock,
    EarthquakeLevel level
)
{
    int angle;


    if(
        level ==
        EARTHQUAKE_WARNING
    )
    {
        angle = 180;


        printf(
            "\n!!! EARTHQUAKE DETECTED !!!\n"
        );
    }


    else
    {
        angle = 0;
    }


    send_servo_command(
        sock,
        angle
    );
}


/* =========================================================
 * Servo Command Send
 *
 * 정상:
 *
 * [PJS_STM]SERVOS@0@0@0@0@0@0@0@0
 *
 *
 * 지진:
 *
 * [PJS_STM]SERVOS@180@180@180@180@180@180@180@180
 * ========================================================= */
void send_servo_command(
    int sock,
    int angle
)
{
    char send_buf[
        BUF_SIZE
    ];


    snprintf(
        send_buf,
        sizeof(send_buf),

        "[%s]"
        "SERVOS@"
        "%d@%d@%d@%d@"
        "%d@%d@%d@%d\n",

        SENSOR_CLIENT_ID,

        angle,
        angle,
        angle,
        angle,

        angle,
        angle,
        angle,
        angle
    );


    printf(
        "\nSERVO TX : %s",
        send_buf
    );


    socket_send(
        sock,
        send_buf
    );
}


/* =========================================================
 * Database Save
 *
 * Table:
 *
 * sensor_logs
 *
 * record_date
 *
 * motor1 ~ motor8
 *
 * gyro_x
 * gyro_y
 * gyro_z
 *
 * vibration
 *
 * jetson_status
 *
 * ========================================================= */
void save_database(
    MYSQL *con,
    SensorData *sensor,
    JetsonData *jetson,
    EarthquakeLevel level
)
{
    char sql_cmd[
        700
    ];


    snprintf(
        sql_cmd,
        sizeof(sql_cmd),

        "INSERT INTO sensor_logs("

        "record_date, "

        "motor1, motor2, motor3, motor4, "
        "motor5, motor6, motor7, motor8, "

        "gyro_x, gyro_y, gyro_z, "

        "vibration, "

        "jetson_status"

        ") "

        "VALUES("

        "NOW(), "

        "%.2f, %.2f, %.2f, %.2f, "
        "%.2f, %.2f, %.2f, %.2f, "

        "%.4f, %.4f, %.4f, "

        "%d, "

        "'%s'"

        ")",


        sensor->motor1,
        sensor->motor2,
        sensor->motor3,
        sensor->motor4,

        sensor->motor5,
        sensor->motor6,
        sensor->motor7,
        sensor->motor8,

        sensor->gyro_x,
        sensor->gyro_y,
        sensor->gyro_z,

        sensor->vibration,

        jetson->status
    );


    printf(
        "\nSQL : %s\n",
        sql_cmd
    );


    int res =
        mysql_query(
            con,
            sql_cmd
        );


    if(!res)
    {
        printf(
            "DB INSERT SUCCESS : %lu row [%s]\n",

            (unsigned long)
            mysql_affected_rows(
                con
            ),

            earthquake_level_string(
                level
            )
        );
    }


    else
    {
        fprintf(
            stderr,

            "DB INSERT ERROR : %s\n",

            mysql_error(
                con
            )
        );
    }
}


/* =========================================================
 * 현재 시간
 * ========================================================= */
double get_time_sec(void)
{
    struct timespec ts;


    clock_gettime(
        CLOCK_MONOTONIC,
        &ts
    );


    return
        (double)ts.tv_sec
        +
        (double)ts.tv_nsec /
        1000000000.0;
}


/* =========================================================
 * 상태 문자열
 * ========================================================= */
const char *earthquake_level_string(
    EarthquakeLevel level
)
{
    switch(level)
    {
        case EARTHQUAKE_SAFETY:

            return
                "SAFETY";


        case EARTHQUAKE_CAUTION:

            return
                "CAUTION";


        case EARTHQUAKE_WARNING:

            return
                "WARNING";
    }


    return
        "UNKNOWN";
}


/* =========================================================
 * Error
 * ========================================================= */
void error_handling(
    char *msg
)
{
    fputs(
        msg,
        stderr
    );


    fputc(
        '\n',
        stderr
    );


    exit(1);
}


/* =========================================================
 * MySQL Error
 * ========================================================= */
void finish_with_error(
    MYSQL *con
)
{
    fprintf(
        stderr,
        "%s\n",
        mysql_error(
            con
        )
    );


    mysql_close(
        con
    );


    exit(1);
}