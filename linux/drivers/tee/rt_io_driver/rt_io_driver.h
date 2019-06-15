#ifndef RT_IO_DRIVER_H
#define RT_IO_DRIVER_H


#include <linux/list.h>

/* define the platform */
// #define __RT_PLAT_SERVO_ROBOT
#define __RT_PLAT_GPG_ROVER


/* Marcro to decide whether we have our system enabled */
#define __RT_IO_PROT_ENABLED 1


#define RT_IO_DRIVER_DEV_NAME "RT_IO_DRIVER "

#define RT_IO_PROCFS_NAME "rtiodev"

/* Add a mutex */
static DEFINE_MUTEX(__rt_io_mutex);

/* Some macro to define size for add RT tasks */
#define RT_IO_COMMAND_LEN 20 /* length of the commands */
#define RT_IO_PHP_ACCESS_LEN 40
#define RT_IO_PHP_ID_LEN 10  /* length of the periheral id */
#define RT_IO_ACTU_CMD_LEN 80 /* length of the actuation cmd sequence */

#define RT_IO_PROC_BUF_SIZE 160

#define RT_INVALID_NUMBER -1


/* Commands for each RT job/tasks  */
#define __COMMAND_RT_TASK_REGISTER "__rt_task_reg"
#define __COMMAND_RT_TASK_DONE "__rt_task_done"
#define __COMMAND_RT_JOB_DONE "__rt_job_done"
#define __COMMAND_RT_ADD_ACTU "__rt_task_act"
#define __COMMAND_RT_TX_SW "__rt_task_store_sw"


/* max number of peripheral per task can access */
#define MAX_PERIPHERAL 10

/* max number of actuation commands per peripheral (per job) */
#define MAX_COMMAND_NUM 50

/* number of IOCTL call per PWM (Servo Driver) */
#define PCA9685_N_CALL_PER_IOCTL 4

#define RT_ACCESS_OK 0xDE44
#define RT_ACCESS_DENIED 0xDE00


/**
* Commands to perform SW operations
*/
#define PTA_CMD_ADD_RT_TASK_INFO 0x4
#define PTA_CMD_CHECK_RT_INV 0x5
#define PTA_CMD_REM_RT_TASK_INFO 0x6
#define PTA_CMD_CHECK_ROVER_RT_TASK_INFO 0x7
#define PTA_CMD_RT_JOB_DONE 0x8
#define PTA_CMD_CHECK_ROVER_INFO_W_LF 0x9



/* this is for robotic arm or PWM (Adafruit servo controller)
* may need to vary based on the CPS platform
*/
enum __peripheral_list {
    S_HAT_SERVO_0=0xAB0,
    S_HAT_SERVO_1=0xAB1,
    S_HAT_SERVO_2=0xAB2,
    S_HAT_SERVO_3=0xAB3,
    S_HAT_SERVO_4=0xAB4,
    S_HAT_SERVO_5=0xAB5,
    GPG_ROV_NAV=0xAB6
};

/* Buffer size for go pi go */
#define GPG_WRITE_BUF_SIZE 5
#define GPG_READ_BUF_SIZE 32

/* All GPG Commands */
#define GPG_LF_REG_SIZE     10
#define GPG_N_LF_SENSOR     5
#define GPG_LINE_READ_CMD   3       //analogRead() command format header
#define GPG_ENC_READ_CMD    53      //Read encoder values
#define GPG_VOLT_CMD        118     //Read the voltage of the batteries



/*
* This is the structure as used to get the IOCTL data and channel
* Registers from the RT tasks
* pulse_data: actual data
* channel_reg: register number (there are four for each call)
*/
typedef struct rt_task_ioctl_param {
	int pulse_data;  // actual data
	int channel_reg;  // register
} RT_TSK_IOCTL;



/*
* This is the structure as used to save information about the RT tasks
* and their peripheral access matrix
*/
typedef struct rt_task_access_info {
	int task_id;  // id
	int p_access_list[MAX_PERIPHERAL]; // which peripheral can access
    int n_peripheral; // number of peripheral it can access
    unsigned long job_count; // job count (mainly for experiments)
    /* this array is to save all the actuation commands */
    int actuation_cmd_list[MAX_PERIPHERAL][MAX_COMMAND_NUM];
    /* saves number of actuation commands per peripheral */
    int n_commands[MAX_PERIPHERAL];
    struct list_head list;  // list
} RT_TSK_AI;

/* for passing info to SW */
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
typedef struct rt_task_inv_check {
	int task_id;  // id
    int periph; // name of pheripheral
    int pulse_val; // value send for write
    int is_access_ok; // whether it can access peripheral
    int predict_val_sw; // predicted value from SW
} RT_TSK_INV_CHK;


/* for checking info From SW */
typedef struct rt_task_rov_inv_check {
	int task_id;  // id
    int periph; // name of pheripheral
    char gpg_cmd_nw[GPG_WRITE_BUF_SIZE]; // rover command send from NW
    char gpg_cmd_predict_sw[GPG_WRITE_BUF_SIZE]; // command predict from SW
    char gpg_lf_sensor_val[GPG_N_LF_SENSOR]; // lf sensor values
    bool is_access_ok;  //this flag ensures if the actuation command is permitted
} RT_TSK_ROV_INV_CHK;

/*
* Returns true if the input pid is found in the global RT_TSK_AI list
*/
bool __rt_io_get_pid(int pid);

/* Returns the channel regsiter, pulse value from RT task
* (in the rtiop struct)
* if failed return value is negative
*/
int get_ioctl_value_from_rt_task(RT_TSK_IOCTL *rtiop,
                                unsigned long arg);

/* Returns the PWM Channel number from the input register
* Currently supports only PCA9685 16-channel driver
* if failed return value is negative
*/
int get_pwm_channel_from_reg(int reg);

/* This function actually called from NW to check invariants in SW
    if failed returns negative
*/

int __check_invariant_from_sw(RT_TSK_INV_CHK *rt_inv_chk);

/* Checks rover invariants from SW
* if failed returns negative
*/
int __check_rover_invariant_from_sw(RT_TSK_ROV_INV_CHK *rt_inv_chk,
                                                        int pta_cmd);


/* this is to redirect our ioctl call so that we can send
* our precomputed data
*/

long __rt_io_i2cdev_ioctl(unsigned int fd, unsigned int cmd,
                        unsigned long arg, __u8 precomputed_data);



#endif /* _RT_IO_DRIVER_H */
