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

#define SENSOR_CLIENT_ID "PJS_STM"

#define SENSOR_TIMEOUT_SEC 1.0
#define JETSON_TIMEOUT_SEC 3.0

#define GYRO_CAUTION_THRESHOLD 2.0f
#define GYRO_WARNING_THRESHOLD 5.0f

#define VIBRATION_CAUTION_THRESHOLD 1
#define VIBRATION_WARNING_THRESHOLD 2


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
    float servo[8]
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

void socket_send(
    int sock,
    const char *data
);


/* =========================================================
 * Global
 * ========================================================= */

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

SensorData latest_sensor = {0};
JetsonData latest_jetson = {0};

unsigned long last_processed_sensor_sequence = 0;

EarthquakeLevel previous_level = -1;

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
        htons(atoi(argv[2]));


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


        ret = select(
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
 * ========================================================= */

void *recv_msg(void *arg)
{
    int *sock =
        (int *)arg;

    MYSQL *con;


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


        for(
            int j = 0;
            j < str_len;
            j++
        )
        {
            char c =
                recv_buffer[j];


            if(c == '\n')
            {
                packet_buffer[
                    packet_len
                ] = '\0';


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
                    sizeof(packet_buffer) - 1
                )
                {
                    packet_buffer[
                        packet_len++
                    ] = c;
                }

                else
                {
                    printf(
                        "packet buffer overflow\n"
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
 * SENSOR:
 *
 * [PJS_STM]SENSOR@
 * M1@M2@M3@M4@
 * M5@M6@M7@M8@
 * GX@GY@GZ@
 * VIB
 *
 *
 * STATUS:
 *
 * [PJS_JET]STATUS@WARNING
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
            "Invalid packet\n"
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
                "GETDB error : %s\n",
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
                "store_result error : %s\n",
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
                "SETDB error : %s\n",
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
 * SENSOR
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
 * 총 14개 token
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
            "SENSOR packet error : %d\n",
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
        "Sensor updated\n"
    );


    printf(
        "Motor : "
        "%.1f %.1f %.1f %.1f "
        "%.1f %.1f %.1f %.1f\n",

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
        "Gyro : %.3f %.3f %.3f\n",

        latest_sensor.gyro_x,
        latest_sensor.gyro_y,
        latest_sensor.gyro_z
    );


    printf(
        "Vibration : %d\n",
        latest_sensor.vibration
    );


    try_process_integrated_data(
        sock,
        con
    );
}


/* =========================================================
 * STATUS
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
            "STATUS packet error\n"
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
        "Jetson Status updated : %s\n",
        latest_jetson.status
    );


    try_process_integrated_data(
        sock,
        con
    );
}


/* =========================================================
 * Sensor + Jetson Integration
 * ========================================================= */

void try_process_integrated_data(
    int sock,
    MYSQL *con
)
{
    if(
        !latest_sensor.valid ||
        !latest_jetson.valid
    )
    {
        return;
    }


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


    if(
        sensor_age >
        SENSOR_TIMEOUT_SEC
    )
    {
        printf(
            "Sensor data stale : %.2f sec\n",
            sensor_age
        );

        return;
    }


    if(
        jetson_age >
        JETSON_TIMEOUT_SEC
    )
    {
        printf(
            "Jetson data stale : %.2f sec\n",
            jetson_age
        );

        return;
    }


    EarthquakeLevel level =
        detect_earthquake(
            &latest_sensor,
            &latest_jetson
        );


    printf(
        "\n"
        "=====================================\n"
        "      INTEGRATED EARTHQUAKE DATA\n"
        "=====================================\n"
    );


    printf(
        "Motor : "
        "%.1f %.1f %.1f %.1f "
        "%.1f %.1f %.1f %.1f\n",

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
        "Decision  : %s\n",
        earthquake_level_string(
            level
        )
    );


    printf(
        "=====================================\n\n"
    );


    save_database(
        con,
        &latest_sensor,
        &latest_jetson,
        level
    );


    last_processed_sensor_sequence =
        latest_sensor.sequence;


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
 * Earthquake Detection
 * ========================================================= */

EarthquakeLevel detect_earthquake(
    SensorData *sensor,
    JetsonData *jetson
)
{
    EarthquakeLevel level =
        EARTHQUAKE_SAFETY;


    if(
        !strcmp(
            jetson->status,
            "WARNING"
        )
    )
    {
        level =
            EARTHQUAKE_WARNING;
    }


    else if(
        !strcmp(
            jetson->status,
            "CAUTION"
        )
    )
    {
        level =
            EARTHQUAKE_CAUTION;
    }


    if(
        sensor->vibration >=
        VIBRATION_WARNING_THRESHOLD
    )
    {
        level =
            EARTHQUAKE_WARNING;
    }


    else if(
        sensor->vibration >=
        VIBRATION_CAUTION_THRESHOLD
    )
    {
        if(
            level <
            EARTHQUAKE_CAUTION
        )
        {
            level =
                EARTHQUAKE_CAUTION;
        }
    }


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


    printf(
        "Gyro magnitude : %.3f\n",
        gyro_magnitude
    );


    if(
        gyro_magnitude >=
        GYRO_WARNING_THRESHOLD
    )
    {
        level =
            EARTHQUAKE_WARNING;
    }


    else if(
        gyro_magnitude >=
        GYRO_CAUTION_THRESHOLD
    )
    {
        if(
            level <
            EARTHQUAKE_CAUTION
        )
        {
            level =
                EARTHQUAKE_CAUTION;
        }
    }


    return level;
}


/* =========================================================
 * Servo Control
 * ========================================================= */

void control_servos(
    int sock,
    EarthquakeLevel level
)
{
    float servo[8];


    switch(level)
    {
        case EARTHQUAKE_SAFETY:

            for(
                int i = 0;
                i < 8;
                i++
            )
            {
                servo[i] =
                    10.0f;
            }

            break;


        case EARTHQUAKE_CAUTION:

            for(
                int i = 0;
                i < 8;
                i++
            )
            {
                servo[i] =
                    20.0f;
            }

            break;


        case EARTHQUAKE_WARNING:

            for(
                int i = 0;
                i < 8;
                i++
            )
            {
                servo[i] =
                    30.0f;
            }

            break;
    }


    send_servo_command(
        sock,
        servo
    );
}


/* =========================================================
 * Servo Command
 *
 * [PJS_ARD]SERVOS@
 * 10.0@10.0@10.0@10.0@
 * 10.0@10.0@10.0@10.0
 * ========================================================= */

void send_servo_command(
    int sock,
    float servo[8]
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
        "%.1f@%.1f@%.1f@%.1f@"
        "%.1f@%.1f@%.1f@%.1f\n",

        SENSOR_CLIENT_ID,

        servo[0],
        servo[1],
        servo[2],
        servo[3],

        servo[4],
        servo[5],
        servo[6],
        servo[7]
    );


    printf(
        "TX : %s",
        send_buf
    );


    socket_send(
        sock,
        send_buf
    );
}


/* =========================================================
 * Database Save
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
        "SQL : %s\n",
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
            "inserted %lu rows [%s]\n",

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
            mysql_error(con)
        );
    }
}


/* =========================================================
 * Time
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
 * Level String
 * ========================================================= */

const char *earthquake_level_string(
    EarthquakeLevel level
)
{
    switch(level)
    {
        case EARTHQUAKE_SAFETY:
            return "SAFETY";


        case EARTHQUAKE_CAUTION:
            return "CAUTION";


        case EARTHQUAKE_WARNING:
            return "WARNING";
    }


    return "UNKNOWN";
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


void finish_with_error(
    MYSQL *con
)
{
    fprintf(
        stderr,
        "%s\n",
        mysql_error(con)
    );


    mysql_close(
        con
    );


    exit(1);
}