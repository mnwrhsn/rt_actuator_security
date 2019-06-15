/*
 * Header finels for RT_IO_SEC PTA
 */

#ifndef __PTA_RT_IO_SEC_H
#define __PTA_RT_IO_SEC_H

#define RT_IO_SEC_PTA_NAME "rtiosec.pta"


/* This UUID is generated with uuidgen
   the ITU-T UUID generator at http://www.itu.int/ITU-T/asn1/uuid.html */
#define RT_IO_SEC_PTA_UUID { 0xd7cf3a38, 0x6626, 0x11e9, \
        { 0xa9, 0x23, 0x16, 0x81, 0xbe, 0x66, 0x3d, 0x3e} }

/* max number of peripheral per task can access */
#define MAX_PERIPHERAL 10

/* max number of actuation commands per peripheral (per job) */
#define MAX_COMMAND_NUM 50


/* Buffer size for go pi go */
#define GPG_WRITE_BUF_SIZE 5
#define GPG_READ_BUF_SIZE 32
#define GPG_N_LF_SENSOR 5 // number of LF sensors

/* for getting info from NW */
typedef struct rt_task_tee_info {
	int task_id;  // id
	int p_access_list[MAX_PERIPHERAL]; // which peripheral can access
    int n_peripheral; // number of peripheral it can access
    /* this array is to save all the actuation commands */
    int actuation_cmd_list[MAX_PERIPHERAL][MAX_COMMAND_NUM];
    /* saves number of actuation commands per peripheral */
    int n_commands[MAX_PERIPHERAL];
} RT_TSK_TEE_INFO;


/* for checking info From SW */
typedef struct rt_task_rov_inv_check {
	int task_id;  // id
    int periph; // name of pheripheral
    char gpg_cmd_nw[GPG_WRITE_BUF_SIZE]; // rover command send from NW
    char gpg_cmd_predict_sw[GPG_WRITE_BUF_SIZE]; // command predict from SW
    char gpg_lf_sensor_val[GPG_N_LF_SENSOR]; // lf sensor values
    bool is_access_ok;  //this flag ensures if the actuation command is permitted
} RT_TSK_ROV_INV_CHK;


/* Macro for RT Acess */

#define RT_ACCESS_OK 0xDE44
#define RT_ACCESS_DENIED 0xDE00

/* an invalid number */
#define RT_INVALID_NUMBER -1

/* maximum number of commands a job can send (for rate limit) */
#define MAX_ROV_ACT_PER_JOB 7

#define ROV_SAFE_SPEED 80 /* Rover speed: 0-255 */


/**
 * struct rt_task_access_info describes access info of the RT task
 */
struct rt_task_access_info {
    int task_id;  // id
	int p_access_list[MAX_PERIPHERAL]; // which peripheral can access
    int n_peripheral; // number of peripheral it can access
    /* saves actuation indices per peripheral */
    int last_act_counter[MAX_PERIPHERAL];

	SLIST_ENTRY(rt_task_access_info) link;
};

/* this is for robotic arm or PWM (Adafruit servo controller)
* may need to vary based on the CPS platform
*/
enum __peripheral_list {
    S_HAT_SERVO_0=0xAB0,
    S_HAT_SERVO_1=0xAB1,
    S_HAT_SERVO_2=0xAB2,
    S_HAT_SERVO_3=0xAB3,
    S_HAT_SERVO_4=0xAB4,
    S_HAT_SERVO_5=0xAB5
};


/**
* Commands to perform SW operations
*/
#define PTA_CMD_ADD_RT_TASK_INFO 0x4
#define PTA_CMD_CHECK_RT_INV 0x5
#define PTA_CMD_REM_RT_TASK_INFO 0x6
#define PTA_CMD_CHECK_ROVER_RT_TASK_INFO 0x7
#define PTA_CMD_RT_JOB_DONE 0x8
#define PTA_CMD_CHECK_ROVER_INFO_W_LF 0x9



/* All GoPiGo Defines */
#define ispd_cmd             116        //Increase the speed by 10
#define dspd_cmd             103        //Decrease the speed by 10
#define set_left_speed_cmd   70            //Set the speed of the right motor
#define set_right_speed_cmd  71            //Set the speed of the left motor
#define fwd_cmd              119        //Move forward with PID
#define motor_fwd_cmd        105        //Move forward without PID
#define bwd_cmd              115        //Move back with PID
#define motor_bwd_cmd        107        //Move back without PID
#define left_cmd             97            //Turn Left by turning off one motor
#define left_rot_cmd         98            //Rotate left by running both motors is opposite direction
#define right_cmd            100        //Turn Right by turning off one motor


#endif /* __PTA_RT_IO_SEC_H */
