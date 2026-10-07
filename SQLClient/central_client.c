#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
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
 * STM32 Client ID
 * ========================================================= */
#define SENSOR_CLIENT_ID "PJS_STM"

/* =========================================================
 * 데이터 유효 시간
 * ========================================================= */
#define SENSOR_TIMEOUT_SEC 1.0
#define JETSON_TIMEOUT_SEC 3.0

/* =========================================================
 * MPU6050
 *
 * 현재 STM32에서는 ACCEL_CONFIG를 별도로 변경하지 않으므로
 * 기본 ±2g 범위 기준으로 사용
 *
 * ±2g -> 16384 LSB/g
 * ========================================================= */
#define MPU6050_1G_RAW 16384.0f

/* =========================================================
 * 가속도 판정 임계값
 *
 * 실제 측정 후 반드시 튜닝 필요
 * ========================================================= */
#define ACCEL_CAUTION_THRESHOLD 2000.0f
#define ACCEL_WARNING_THRESHOLD 5000.0f

/* =========================================================
 * Servo
 * ========================================================= */
#define SERVO_LOCK_ANGLE       0
#define SERVO_RELEASE_ANGLE  180

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
 *
 * STM32 Packet
 *
 * [STM32]SENSOR@AX@AY@AZ@VIBRATION
 * ========================================================= */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

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
    const char *command
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
 * 현재 서보 상태
 *
 * LOCK    = 0
 * RELEASE = 180
 *
 * DB의 motor1 ~ motor8에 이 값을 저장한다.
 */
int current_servo_angle = SERVO_LOCK_ANGLE;

/*
 * SENSOR 중복 처리 방지
 */
unsigned long last_processed_sensor_sequence = 0;

/*
 * 이전 지진 상태
 *
 * 지진 상태가 바뀌었을 때만
 * STM32로 서보 제어 명령 전송
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

        if(ret < 0)
        {
            if(*sock == -1)
            {
                return NULL;
            }

            continue;
        }

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
 * TCP는 read() 1번 = packet 1개가 아니므로
 * '\n' 기준으로 packet을 조립한다.
 *
 * 예:
 *
 * [STM32]SENSOR@100@200@16000@1\n
 * [PJS_JET]STATUS@WARNING\n
 * ========================================================= */
void *recv_msg(void *arg)
{
    int *sock =
        (int *)arg;

    MYSQL *con;

    /* =====================================================
     * DB 연결
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
         * TCP packet 누적
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
             * '\n' = packet 종료
             */
            if(c == '\n')
            {
                packet_buffer[
                    packet_len
                ] = '\0';

                /*
                 * CRLF 대응
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
 * STM32:
 *
 * [STM32]SENSOR@AX@AY@AZ@VIBRATION
 *
 * Jetson:
 *
 * [PJS_JET]STATUS@SAFETY
 * [PJS_JET]STATUS@CAUTION
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
     * delimiter:
     *
     * [
     * ]
     * @
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
     * STM32 SENSOR
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
        {
            return;
        }

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
        {
            return;
        }

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
 * STM32 SENSOR 처리
 *
 * Packet:
 *
 * [STM32]SENSOR@AX@AY@AZ@VIBRATION
 *
 * pArray
 *
 * 0 = STM32
 * 1 = SENSOR
 * 2 = accel_x
 * 3 = accel_y
 * 4 = accel_z
 * 5 = vibration
 *
 * ========================================================= */
void process_sensor(
    int sock,
    MYSQL *con,
    char **pArray,
    int count
)
{
    if(count < 6)
    {
        printf(
            "SENSOR Packet Error : count=%d\n",
            count
        );

        return;
    }

    latest_sensor.accel_x =
        (int16_t)atoi(
            pArray[2]
        );

    latest_sensor.accel_y =
        (int16_t)atoi(
            pArray[3]
        );

    latest_sensor.accel_z =
        (int16_t)atoi(
            pArray[4]
        );

    latest_sensor.vibration =
        atoi(
            pArray[5]
        );

    latest_sensor.timestamp =
        get_time_sec();

    latest_sensor.sequence++;

    latest_sensor.valid = 1;

    printf(
        "\n===== SENSOR UPDATE =====\n"
    );

    printf(
        "ACCEL : %d %d %d\n",
        latest_sensor.accel_x,
        latest_sensor.accel_y,
        latest_sensor.accel_z
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

    /*
     * 허용된 STATUS만 처리
     */
    if(
        strcmp(pArray[2], "SAFETY") != 0 &&
        strcmp(pArray[2], "CAUTION") != 0 &&
        strcmp(pArray[2], "WARNING") != 0
    )
    {
        printf(
            "Unknown Jetson STATUS : %s\n",
            pArray[2]
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
     * SENSOR와 Jetson STATUS가 모두
     * 한번 이상 들어와야 판단 가능
     */
    if(
        !latest_sensor.valid ||
        !latest_jetson.valid
    )
    {
        return;
    }

    /*
     * 이미 처리한 SENSOR 샘플이면 종료
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
     * 오래된 SENSOR 데이터
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
     * 오래된 Jetson 데이터
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
        "Accel     : %d %d %d\n",
        latest_sensor.accel_x,
        latest_sensor.accel_y,
        latest_sensor.accel_z
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

    /*
     * 중요:
     *
     * 상태가 바뀌었을 때 먼저 서보 상태를 결정한다.
     *
     * 그래야 아래 DB 저장에서
     * 현재 판단 결과에 해당하는 0 / 180이 저장된다.
     */
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

    printf(
        "Servo     : %d deg\n",
        current_servo_angle
    );

    printf(
        "========================================\n\n"
    );

    /* =====================================================
     * DB 저장
     *
     * 기존 DB 구조 유지
     *
     * motor1 ~ motor8
     *      -> 현재 servo angle
     *
     * gyro_x/y/z
     *      -> 실제 accel_x/y/z
     * ===================================================== */
    save_database(
        con,
        &latest_sensor,
        &latest_jetson,
        level
    );

    /*
     * SENSOR 처리 완료
     */
    last_processed_sensor_sequence =
        latest_sensor.sequence;
}

/* =========================================================
 * 지진 판단
 *
 * 현재 사용 데이터
 *
 * 1. Jetson Detection
 * 2. Vibration Sensor
 * 3. MPU6050 Accelerometer
 *
 * 점수:
 *
 * Jetson
 * WARNING : +4
 * CAUTION : +2
 *
 * Vibration
 * 감지     : +2
 *
 * Acceleration
 * WARNING : +4
 * CAUTION : +2
 *
 * 총점
 *
 * 0 ~ 2 : SAFETY
 * 3 ~ 5 : CAUTION
 * >= 6  : WARNING
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
     *
     * 현재 STM32 SHOCK 값:
     * 0 / 1
     *
     * 현재는 1 = 감지로 가정
     * ===================================================== */
    if(
        sensor->vibration == 1
    )
    {
        score += 2;
    }

    /* =====================================================
     * Accelerometer
     *
     * MPU6050 ±2g 기본 범위:
     *
     * 1g ≈ 16384 raw
     *
     * 기울기에 영향을 덜 받게 하기 위해
     * XYZ magnitude를 계산한 후
     * 1g에서 벗어난 정도를 사용
     * ===================================================== */
    float ax =
        (float)sensor->accel_x;

    float ay =
        (float)sensor->accel_y;

    float az =
        (float)sensor->accel_z;

    float accel_magnitude =
        sqrtf(
            ax * ax +
            ay * ay +
            az * az
        );

    float accel_delta =
        fabsf(
            accel_magnitude -
            MPU6050_1G_RAW
        );

    if(
        accel_delta >=
        ACCEL_WARNING_THRESHOLD
    )
    {
        score += 4;
    }

    else if(
        accel_delta >=
        ACCEL_CAUTION_THRESHOLD
    )
    {
        score += 2;
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
        "Accel XYZ     : %d %d %d\n",
        sensor->accel_x,
        sensor->accel_y,
        sensor->accel_z
    );

    printf(
        "Accel Mag     : %.3f\n",
        accel_magnitude
    );

    printf(
        "Accel Delta   : %.3f\n",
        accel_delta
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
 * SAFETY
 *    -> LOCK
 *    -> 0 degree
 *
 * CAUTION
 *    -> LOCK
 *    -> 0 degree
 *
 * WARNING
 *    -> RELEASE
 *    -> 180 degree
 * ========================================================= */
void control_servos(
    int sock,
    EarthquakeLevel level
)
{
    if(
        level ==
        EARTHQUAKE_WARNING
    )
    {
        printf(
            "\n!!! EARTHQUAKE DETECTED !!!\n"
        );

        /*
         * 먼저 프로그램 내부 상태 변경
         */
        current_servo_angle =
            SERVO_RELEASE_ANGLE;

        /*
         * STM32 명령
         */
        send_servo_command(
            sock,
            "RELEASE"
        );
    }

    else
    {
        current_servo_angle =
            SERVO_LOCK_ANGLE;

        send_servo_command(
            sock,
            "LOCK"
        );
    }
}

/* =========================================================
 * Servo Command Send
 *
 * 정상:
 *
 * [PJS_STM]SERVOS@LOCK
 *
 * 지진:
 *
 * [PJS_STM]SERVOS@RELEASE
 * ========================================================= */
void send_servo_command(
    int sock,
    const char *command
)
{
    char send_buf[
        BUF_SIZE
    ];

    snprintf(
        send_buf,
        sizeof(send_buf),

        "[%s]SERVOS@%s\n",

        SENSOR_CLIENT_ID,
        command
    );

    printf(
        "SERVO TX : %s",
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
 * ★ DB TABLE 구조는 기존 그대로 사용
 *
 * 기존 컬럼:
 *
 * record_date
 *
 * motor1
 * motor2
 * motor3
 * motor4
 * motor5
 * motor6
 * motor7
 * motor8
 *
 * gyro_x
 * gyro_y
 * gyro_z
 *
 * vibration
 * jetson_status
 *
 *
 * 실제 저장 의미:
 *
 * motor1 ~ motor8
 *      = 현재 Servo 명령 각도
 *      = LOCK    -> 0
 *      = RELEASE -> 180
 *
 * gyro_x
 *      = 실제 accel_x
 *
 * gyro_y
 *      = 실제 accel_y
 *
 * gyro_z
 *      = 실제 accel_z
 *
 * vibration
 *      = STM32 SHOCK
 *
 * jetson_status
 *      = Jetson detection status
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

        "%d, %d, %d, %d, "
        "%d, %d, %d, %d, "

        "%d, %d, %d, "

        "%d, "

        "'%s'"

        ")",

        /*
         * motor1 ~ motor8
         *
         * 현재 서보 명령 상태
         */
        current_servo_angle,
        current_servo_angle,
        current_servo_angle,
        current_servo_angle,

        current_servo_angle,
        current_servo_angle,
        current_servo_angle,
        current_servo_angle,

        /*
         * DB 컬럼 이름은 gyro지만
         * 실제 데이터는 accelerometer
         */
        sensor->accel_x,
        sensor->accel_y,
        sensor->accel_z,

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