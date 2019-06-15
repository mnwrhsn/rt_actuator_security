

/*
 * This pseudo TA is used by NW Linux TEE client to implement IO access controls
 */

#include <stdio.h>
#include <trace.h>
#include <kernel/pseudo_ta.h>
#include <string.h>
#include <tee/uuid.h>
#include <pta_rt_io_sec.h>


//
// int my_global = 10;

static bool __rt_task_init_success = false;

static SLIST_HEAD(, rt_task_access_info) __rtai_list =
                                SLIST_HEAD_INITIALIZER(__rtai_list);


/* Store RT task info to the global data structure */
static TEE_Result add_rt_task_info(uint32_t types,
			      TEE_Param params[TEE_NUM_PARAMS])
{

    struct rt_task_access_info *rtai;  // an iterator
    RT_TSK_TEE_INFO *rt_task_info = NULL; // info from NW
    TEE_Result res = TEE_SUCCESS;
    int i;

    /* some error checking */
    if (types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_INOUT,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;


    rt_task_info = (RT_TSK_TEE_INFO *)params[0].memref.buffer;
    rtai = malloc(sizeof(*rtai));

    if (rtai == NULL)
		return TEE_ERROR_OUT_OF_MEMORY;

    rtai->task_id = rt_task_info->task_id;
    rtai->n_peripheral = rt_task_info->n_peripheral;

    for(i=0;i<MAX_PERIPHERAL;i++) {
        rtai->p_access_list[i] = rt_task_info->p_access_list[i];
        rtai->last_act_counter[i] = 0;
    }

    /* Add to list */
    SLIST_INSERT_HEAD(&__rtai_list, rtai, link);

    /* change the flag */
    __rt_task_init_success = true;
    // DMSG("SW PID:%d, N_PHP:%d, Init done!", rtai->task_id,
    //             rtai->n_peripheral);
    return res;

}


/* Store RT task info to the global data structure */
static TEE_Result remove_rt_task_info(uint32_t types,
			      TEE_Param params[TEE_NUM_PARAMS])
{


    TEE_Result res = TEE_SUCCESS;
    struct rt_task_access_info *rtai = NULL;  // an iterator
    bool is_found = false;
    int pid;


    /* some error checking */
    if (types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

    if (!__rt_task_init_success) {
        DMSG("ERROR: RT Task Data structure is not initiated properly!");
        return TEE_ERROR_NO_DATA;
    }



    pid = params[0].value.a;

    SLIST_FOREACH(rtai, &__rtai_list, link) {
        if (rtai->task_id == pid) {
            is_found = true;
            break;
        }

    }

    /* now remove the element */
    if (is_found)
        SLIST_REMOVE(&__rtai_list, rtai, rt_task_access_info, link);

    return res;

}

/* update actuation counter when job done */
static TEE_Result update_job_done_info(uint32_t types,
			      TEE_Param params[TEE_NUM_PARAMS])
{


    TEE_Result res = TEE_SUCCESS;
    struct rt_task_access_info *rtai = NULL;  // an iterator

    int pid;
    int i;


    /* some error checking */
    if (types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

    if (!__rt_task_init_success) {
        DMSG("ERROR: RT Task Data structure is not initiated properly!");
        return TEE_ERROR_NO_DATA;
    }



    pid = params[0].value.a;

    SLIST_FOREACH(rtai, &__rtai_list, link) {
        if (rtai->task_id == pid) {
            for (i=0;i<rtai->n_peripheral;i++) {
                rtai->last_act_counter[i] = 0;  /* reset counter */
            }
            break;
        }

    }

    return res;

}

/* check the task is allowed to acess the peripheral */
static bool is_access_allowed(int pid, int periph)
{
    struct rt_task_access_info *rtai = NULL;  // an iterator
    int i;

	SLIST_FOREACH(rtai, &__rtai_list, link) {
        if (rtai->task_id == pid) {
            for (i=0;i<rtai->n_peripheral;i++) {
                if (rtai->p_access_list[i] == periph) {
                    return true;
                }

            }
        }

    }

    return false;
}

/* check the task is allowed to acess the peripheral */
/* Note this will be called only if access OK */
static void update_rover_rate_inv(RT_TSK_ROV_INV_CHK *rt_task_info)
{
    struct rt_task_access_info *rtai = NULL;  // an iterator
    int i;

	SLIST_FOREACH(rtai, &__rtai_list, link) {
        if (rtai->task_id == rt_task_info->task_id) {
            for (i=0;i<rtai->n_peripheral;i++) {
                if (rtai->p_access_list[i] == rt_task_info->periph) {
                    rtai->last_act_counter[i] += 1;

                    if (rtai->last_act_counter[i] <= MAX_ROV_ACT_PER_JOB)
                        rt_task_info->is_access_ok = true;

                    // DMSG("ACT Counter: %d", rtai->last_act_counter[i]);

                    else if (rtai->last_act_counter[i] > MAX_ROV_ACT_PER_JOB) {

                        if (rtai->last_act_counter[i] == MAX_ROV_ACT_PER_JOB + 1) {

                            if (rt_task_info->gpg_cmd_nw[1] == set_left_speed_cmd ||
                                rt_task_info->gpg_cmd_nw[1] == set_right_speed_cmd) {
                                    /* set predefined speed */
                                    rt_task_info->gpg_cmd_predict_sw[2] =
                                                            ROV_SAFE_SPEED;
                                    rt_task_info->is_access_ok = true;
                                }
                            else
                                rt_task_info->is_access_ok = false;
                    }
                    else
                          rt_task_info->is_access_ok = false;
                }


                    // else {
                    //     /* ignore subesequent requests */
                    //     rt_task_info->is_access_ok = false;
                    // }
                    return;
                }


            }

        }


    }

    return;
}


static char get_predicted_movement(char *sensor_val)
{
    /* idea from: https://github.com/DexterInd/DI_Sensors/blob/master/Python/di_sensors/red_line_follower/line_follower/line_follow.py */

    int white[GPG_N_LF_SENSOR]={767,815,859,710,700}; //reading when all sensors are at white
    int black[GPG_N_LF_SENSOR]={1012,1013,1015,1003,1004}; //#reading when all sensors are black
    int range_col[GPG_N_LF_SENSOR]; //difference between black and white

    int multp[GPG_N_LF_SENSOR]={-100,-50,0,50,100}; //Add a multipler to each sensor


    int i;
    int curr_pos=0;

    // char cmd;

    int diff_val[GPG_N_LF_SENSOR];


    int percent_black_line[GPG_N_LF_SENSOR]={0};

    for(i=0;i<GPG_N_LF_SENSOR;i++) {
        range_col[i] = black[i] - white[i];
        diff_val[i] = (int) sensor_val[i] - white[i];
        percent_black_line[i]=diff_val[i]*100/range_col[i];
        curr_pos+=percent_black_line[i]*multp[i];
    }



    if (curr_pos <-2500) {
        return right_cmd;
    }

	else if (curr_pos >2500) {
        return fwd_cmd;
    }
    else {
        return left_cmd;
    }

}



/* check the task is allowed to acess the peripheral */
/* Note this will be called only if access OK */
static void update_rover_inv_lf(RT_TSK_ROV_INV_CHK *rt_task_info)
{
    struct rt_task_access_info *rtai = NULL;  // an iterator
    int i;
    char cmd;

	SLIST_FOREACH(rtai, &__rtai_list, link) {
        if (rtai->task_id == rt_task_info->task_id) {
            for (i=0;i<rtai->n_peripheral;i++) {
                if (rtai->p_access_list[i] == rt_task_info->periph) {
                    rtai->last_act_counter[i] += 1;

                    if (rtai->last_act_counter[i] <= MAX_ROV_ACT_PER_JOB) {
                        cmd = get_predicted_movement(rt_task_info->gpg_lf_sensor_val);
                        /* update command */
                        DMSG("input cmd %d, predict cmd: %d.",
                                                rt_task_info->gpg_cmd_nw[1], cmd);

                        rt_task_info->gpg_cmd_predict_sw[1] = cmd;
                        rt_task_info->is_access_ok = true;
                    }


                    // DMSG("ACT Counter: %d", rtai->last_act_counter[i]);

                    if (rtai->last_act_counter[i] > MAX_ROV_ACT_PER_JOB) {

                        if (rtai->last_act_counter[i] == MAX_ROV_ACT_PER_JOB + 1) {

                            if (rt_task_info->gpg_cmd_nw[1] == set_left_speed_cmd ||
                                rt_task_info->gpg_cmd_nw[1] == set_right_speed_cmd) {
                                    /* set predefined speed */
                                    rt_task_info->gpg_cmd_predict_sw[2] =
                                                            ROV_SAFE_SPEED;
                                    rt_task_info->is_access_ok = true;
                                }
                            else
                                rt_task_info->is_access_ok = false;
                    }
                    else
                          rt_task_info->is_access_ok = false;
                }
                    return;
                }


            }

        }


    }

    return;
}


/* Check the invariant and allow access */
static TEE_Result check_rover_rate_control(uint32_t types,
			      TEE_Param params[TEE_NUM_PARAMS])
{

    TEE_Result res = TEE_SUCCESS;
    RT_TSK_ROV_INV_CHK *rt_task_info = NULL; // info from NW
    rt_task_info = (RT_TSK_ROV_INV_CHK *)params[0].memref.buffer;
    bool is_allowed;



    /* some error checking */
    if (types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_INOUT,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

    if (!__rt_task_init_success) {
        DMSG("ERROR: RT Task Data structure is not initiated properly!");
        return TEE_ERROR_NO_DATA;
    }

    // DMSG("PID from NW: %d", rt_task_info->task_id);
    // rt_task_info->is_access_ok = true;

    is_allowed = is_access_allowed(rt_task_info->task_id, rt_task_info->periph);
    if (is_allowed) {
        // DMSG("Task access allowed!");
        // rt_task_info->is_access_ok = true;
        update_rover_rate_inv(rt_task_info);

    }
    else {
        rt_task_info->is_access_ok = false;
    }




    return res;

}


/* Check rover the invariant (with line followert) and allow access */
static TEE_Result check_rover_inv_w_lf(uint32_t types,
			      TEE_Param params[TEE_NUM_PARAMS])
{

    TEE_Result res = TEE_SUCCESS;
    RT_TSK_ROV_INV_CHK *rt_task_info = NULL; // info from NW
    rt_task_info = (RT_TSK_ROV_INV_CHK *)params[0].memref.buffer;
    bool is_allowed;



    /* some error checking */
    if (types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_INOUT,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE,
				     TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

    if (!__rt_task_init_success) {
        DMSG("ERROR: RT Task Data structure is not initiated properly!");
        return TEE_ERROR_NO_DATA;
    }

    // DMSG("PID from NW: %d", rt_task_info->task_id);
    // rt_task_info->is_access_ok = true;

    is_allowed = is_access_allowed(rt_task_info->task_id, rt_task_info->periph);
    if (is_allowed) {
        DMSG("Task access allowed -> Inv checking implementation ongoing!");
        // rt_task_info->is_access_ok = true;
        update_rover_inv_lf(rt_task_info);

    }
    else {
        rt_task_info->is_access_ok = false;
    }




    return res;

}



/*
 * Trusted Application Entry Points
 */

 static TEE_Result invoke_command(void *pSessionContext __unused,
 				 uint32_t nCommandID, uint32_t nParamTypes,
 				 TEE_Param pParams[TEE_NUM_PARAMS])
{

    // test_shm(pParams);

    switch (nCommandID) {
	case PTA_CMD_ADD_RT_TASK_INFO:
		return add_rt_task_info(nParamTypes, pParams);
    case PTA_CMD_CHECK_RT_INV:
        /* for rover this is removed */
        return TEE_ERROR_NOT_IMPLEMENTED;
    case PTA_CMD_REM_RT_TASK_INFO:
        return remove_rt_task_info(nParamTypes, pParams);
    case PTA_CMD_CHECK_ROVER_RT_TASK_INFO:
        return check_rover_rate_control(nParamTypes, pParams);
    case PTA_CMD_CHECK_ROVER_INFO_W_LF:
        return check_rover_inv_w_lf(nParamTypes, pParams);
    case PTA_CMD_RT_JOB_DONE:
        return update_job_done_info(nParamTypes, pParams);
	default:
		break;
	}


    return TEE_ERROR_NOT_IMPLEMENTED;
}

pseudo_ta_register(.uuid = RT_IO_SEC_PTA_UUID, .name = RT_IO_SEC_PTA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = invoke_command);
