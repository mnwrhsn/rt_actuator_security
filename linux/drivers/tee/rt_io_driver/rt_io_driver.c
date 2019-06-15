/*
 * Copyright (c) 2019, Monowar Hasan (SyNeRCyS -- CS@UIUC)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/types.h>

/* include for RT-IO-DRIVER and OPTEE */
#include "rt_io_driver.h"
// #include <linux/rt_io_driver.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>
#include "../optee/optee_private.h"

// for i2c-dev
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
// for file
#include <linux/file.h>
#include <linux/fs.h>



//
// #include <linux/cdev.h>
// #include <linux/fs.h>
// #include <linux/idr.h>
// #include <linux/module.h>
// #include <linux/slab.h>
// #include <linux/tee_drv.h>
// #include <linux/uaccess.h>
// #include "../tee_private.h"

// int call_to_secure_world(void);

/* entry for the procfs */
static struct proc_dir_entry *procfs_entry;

/* UUID of our TA */
const uuid_t pta_uuid =
    UUID_INIT(0xd7cf3a38, 0x6626, 0x11e9,
          0xa9, 0x23, 0x16, 0x81, 0xbe, 0x66, 0x3d, 0x3e);

/* Global variables for Opening Context with SW */
struct tee_ioctl_open_session_arg sess_arg;
struct tee_context *ctx = NULL;

/* Declare and init the head of the RT Task Access info linked list. */
LIST_HEAD(__rtai_list);

/* function prototype */
static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data);


static noinline int rt_io_sec_i2cdev_ioctl_smbus(struct i2c_client *client,
		u8 read_write, u8 command, u32 size,
		union i2c_smbus_data __user *data, __u8 precomputed_data)
{
	union i2c_smbus_data temp = {};
	int datasize, res;

	if ((size != I2C_SMBUS_BYTE) &&
	    (size != I2C_SMBUS_QUICK) &&
	    (size != I2C_SMBUS_BYTE_DATA) &&
	    (size != I2C_SMBUS_WORD_DATA) &&
	    (size != I2C_SMBUS_PROC_CALL) &&
	    (size != I2C_SMBUS_BLOCK_DATA) &&
	    (size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
	    (size != I2C_SMBUS_I2C_BLOCK_DATA) &&
	    (size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_dbg(&client->adapter->dev,
			"size out of range (%x) in ioctl I2C_SMBUS.\n",
			size);
		return -EINVAL;
	}
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((read_write != I2C_SMBUS_READ) &&
	    (read_write != I2C_SMBUS_WRITE)) {
		dev_dbg(&client->adapter->dev,
			"read_write out of range (%x) in ioctl I2C_SMBUS.\n",
			read_write);
		return -EINVAL;
	}

	/* Note that command values are always valid! */

	if ((size == I2C_SMBUS_QUICK) ||
	    ((size == I2C_SMBUS_BYTE) &&
	    (read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data */
		return i2c_smbus_xfer(client->adapter, client->addr,
				      client->flags, read_write,
				      command, size, NULL);

	if (data == NULL) {
		dev_dbg(&client->adapter->dev,
			"data is NULL pointer in ioctl I2C_SMBUS.\n");
		return -EINVAL;
	}

	if ((size == I2C_SMBUS_BYTE_DATA) ||
	    (size == I2C_SMBUS_BYTE))
		datasize = sizeof(data->byte);
	else if ((size == I2C_SMBUS_WORD_DATA) ||
		 (size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof(data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof(data->block);

	if ((size == I2C_SMBUS_PROC_CALL) ||
	    (size == I2C_SMBUS_BLOCK_PROC_CALL) ||
	    (size == I2C_SMBUS_I2C_BLOCK_DATA) ||
	    (read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user(&temp, data, datasize))
			return -EFAULT;
	}
	if (size == I2C_SMBUS_I2C_BLOCK_BROKEN) {
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}

    /* this is out modified code */
    if ((size == I2C_SMBUS_BYTE_DATA) ||
	    (size == I2C_SMBUS_BYTE))
        temp.byte = precomputed_data;

	res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
	      read_write, command, size, &temp);
	if (!res && ((size == I2C_SMBUS_PROC_CALL) ||
		     (size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		     (read_write == I2C_SMBUS_READ))) {
		if (copy_to_user(data, &temp, datasize))
			return -EFAULT;
	}
	return res;
}


long __rt_io_i2cdev_ioctl(unsigned int fd, unsigned int cmd, unsigned long arg, __u8 precomputed_data)
{
    struct fd f = fdget(fd);
    struct file *file = f.file;
    struct i2c_client *client = file->private_data;

    struct i2c_smbus_ioctl_data data_arg;
		if (copy_from_user(&data_arg,
				   (struct i2c_smbus_ioctl_data __user *) arg,
				   sizeof(struct i2c_smbus_ioctl_data)))
			return -EFAULT;
		return rt_io_sec_i2cdev_ioctl_smbus(client, data_arg.read_write,
					  data_arg.command,
					  data_arg.size,
					  data_arg.data,
                      precomputed_data);

}
EXPORT_SYMBOL_GPL(__rt_io_i2cdev_ioctl);


bool __rt_io_get_pid(int pid)
{
    RT_TSK_AI *ptr;

    list_for_each_entry(ptr, &__rtai_list, list) {
        if (ptr->task_id == pid)
            return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(__rt_io_get_pid);




/*
* initialze Context with SW
* There are two Global variable maintained: sess_arg and ctx
* that is reused by every caller
*/
int init_sw_context(void)
{
    int rc;

    memset(&sess_arg, 0, sizeof(sess_arg));

    /* Open context with OP-TEE driver */
    ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
    if (IS_ERR(ctx))
    	return -ENODEV;

    /* Open session with device enumeration pseudo TA */
    memcpy(sess_arg.uuid, pta_uuid.b, TEE_IOCTL_UUID_LEN);
    sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
    sess_arg.num_params = 0;

    rc = tee_client_open_session(ctx, &sess_arg, NULL);
    if ((rc < 0) || (sess_arg.ret != TEEC_SUCCESS)) {
    	/* RT-IO-SEC pseudo TA not found */
    	rc = 0;
    	printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "TA Not Found!\n");
    	tee_client_close_context(ctx);
    }
    // printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "Open session successfully!\n");
    return rc;
}

int get_ioctl_value_from_rt_task(struct rt_task_ioctl_param *rtiop,
                                unsigned long arg)
{
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp = {};
	int datasize;

	if (copy_from_user(&data_arg,
			   (struct i2c_smbus_ioctl_data __user *) arg,
			   sizeof(struct i2c_smbus_ioctl_data))) {

		printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "%s Failed to copy\n", __func__);
		return -EFAULT;

	}

	if ((data_arg.size == I2C_SMBUS_BYTE_DATA) ||
	    (data_arg.size == I2C_SMBUS_BYTE))
		datasize = sizeof(data_arg.data->byte);
	else if ((data_arg.size == I2C_SMBUS_WORD_DATA) ||
		 (data_arg.size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof(data_arg.data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof(data_arg.data->block);

	if ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		(data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		(data_arg.size == I2C_SMBUS_I2C_BLOCK_DATA) ||
		(data_arg.read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user(&temp, data_arg.data, datasize))
			return -EFAULT;
	}


	/* we do not care about unsigned char --> int is fine for boundary checking */

    rtiop->pulse_data = (int)temp.byte;  // actual data
    rtiop->channel_reg = (int)data_arg.command;  // register

    // on success returns zero
    return 0;
}
EXPORT_SYMBOL_GPL(get_ioctl_value_from_rt_task);




/* redefined from drivers/tee/optee/device.c */
static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}


/*
* Command to check invariants (currenly for pwm driver)
*/
static int invoke_command_check_invariant(struct tee_context *ctx, u32 session,
                                        RT_TSK_INV_CHK *rt_inv_chk)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke Check invariant function */
	inv_arg.func = PTA_CMD_CHECK_RT_INV;
	inv_arg.session = session;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
    param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
    param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

    param[0].u.value.a = rt_inv_chk->task_id;
    param[0].u.value.b = rt_inv_chk->periph;
    param[1].u.value.b = rt_inv_chk->pulse_val;
    param[1].u.value.a = rt_inv_chk->is_access_ok;

	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if (ret < 0) {
		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
		       inv_arg.ret);
		return -EINVAL;
	}

    /* if access ok, set it in the struct */
    if (param[1].u.value.a == RT_ACCESS_OK)
        rt_inv_chk->is_access_ok = RT_ACCESS_OK;

    rt_inv_chk->predict_val_sw = param[2].u.value.a;

	return 0;

}


/*
* Command to remove RT task info from SW
*/
static int invoke_command_rem_rt_task(struct tee_context *ctx, u32 session,
                                        int pid)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke Check invariant function */
	inv_arg.func = PTA_CMD_REM_RT_TASK_INFO;
	inv_arg.session = session;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
    param[0].u.value.a = pid;

	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if (ret < 0) {
		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
		       inv_arg.ret);
		return -EINVAL;
	}

	return 0;

}


/*
* Command to notify SW that job is done
*/
static int invoke_command_rt_job_done(struct tee_context *ctx, u32 session,
                                        int pid)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke Check invariant function */
	inv_arg.func = PTA_CMD_RT_JOB_DONE;
	inv_arg.session = session;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
    param[0].u.value.a = pid;

	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if (ret < 0) {
		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
		       inv_arg.ret);
		return -EINVAL;
	}

	return 0;

}



static int do_init_sw_shm(struct tee_context *ctx, u32 session,
		       struct tee_shm *tee_shm, int command_func, size_t memref_size)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke add task function */
	// inv_arg.func = PTA_CMD_ADD_RT_TASK_INFO;
    inv_arg.func = command_func;
	inv_arg.session = session;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[0].u.memref.shm = tee_shm;
	// param[0].u.memref.size = sizeof(RT_TSK_TEE_INFO);
    param[0].u.memref.size = memref_size;
	param[0].u.memref.shm_offs = 0;

	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if (ret < 0) {
		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
		       inv_arg.ret);
		return -EINVAL;
	}


	return 0;

}


static int get_rov_result_by_shm(struct tee_context *ctx, u32 session,
		       struct tee_shm *tee_shm, int command_func, size_t memref_size,
               RT_TSK_ROV_INV_CHK *tee_rtai_out)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
    RT_TSK_ROV_INV_CHK *tee_rtai_va = NULL;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke add task function */
    inv_arg.func = command_func;
	inv_arg.session = session;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[0].u.memref.shm = tee_shm;
    param[0].u.memref.size = memref_size;
	param[0].u.memref.shm_offs = 0;

	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if (ret < 0) {
		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
		       inv_arg.ret);
		return -EINVAL;
	}

    /* to get the changes from SW */
    tee_rtai_va = tee_shm_get_va(param[0].u.memref.shm, 0);

    /* copy elements */
    memcpy(tee_rtai_out, tee_rtai_va, sizeof(RT_TSK_ROV_INV_CHK));


	if (IS_ERR(tee_rtai_out)) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "tee_shm_get_va failed!\n");
        tee_shm_free(tee_shm);
        tee_client_close_session(ctx, sess_arg.session);
	}
    // printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Is access from SHM: %d. Pid: %d\n",
    //     tee_rtai_out->is_access_ok, tee_rtai_out->task_id);

	return 0;

}


int __check_invariant_from_sw(RT_TSK_INV_CHK *rt_inv_chk)
{
    int rc;

    /* initialize SW context */
    mutex_lock(&__rt_io_mutex);

    rc = init_sw_context();

    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Create Context!\n");
        return -EFAULT;
    }

    rc = invoke_command_check_invariant(ctx, sess_arg.session, rt_inv_chk);

	if (rc < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
            "%s : Invoke Command Failed!\n", __func__);
        tee_client_close_session(ctx, sess_arg.session);
	}

	/* Close the session */
    tee_client_close_session(ctx, sess_arg.session);

    mutex_unlock(&__rt_io_mutex);

	return rc;


}
EXPORT_SYMBOL_GPL(__check_invariant_from_sw);


/* this is used in DoS experiment */
int __check_rover_invariant_from_sw(RT_TSK_ROV_INV_CHK *tee_rtai, int pta_cmd)
{
    u32 shm_size = sizeof(RT_TSK_ROV_INV_CHK);
    struct tee_shm *tee_shm = NULL;
    RT_TSK_ROV_INV_CHK *tee_rtai_va = NULL;
    int rc;

    /* initialize SW context */
    mutex_lock(&__rt_io_mutex);

    rc = init_sw_context();

    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Create Context!\n");
        return -EFAULT;
    }


    tee_shm = tee_shm_alloc(ctx, shm_size, TEE_SHM_MAPPED);

    if (IS_ERR(tee_shm)) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "TEE SHM Alloc failed!\n");
		rc = PTR_ERR(tee_shm);
		tee_client_close_session(ctx, sess_arg.session);
        return rc;
	}

    tee_rtai_va = tee_shm_get_va(tee_shm, 0);
    if (IS_ERR(tee_rtai_va)) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "tee_shm_get_va failed!\n");
		rc = PTR_ERR(tee_rtai_va);
        tee_shm_free(tee_shm);
        tee_client_close_session(ctx, sess_arg.session);
        mutex_unlock(&__rt_io_mutex);
        return rc;

	}

    /* copy elements */
    memcpy(tee_rtai_va, tee_rtai, sizeof(RT_TSK_ROV_INV_CHK));

    /* Call SW function */
	rc = get_rov_result_by_shm(ctx, sess_arg.session, tee_shm,
                        pta_cmd,
                        sizeof(RT_TSK_ROV_INV_CHK),
                        tee_rtai);
	if (rc < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Invoke SHM Failed!\n");
        tee_shm_free(tee_shm);
        tee_client_close_session(ctx, sess_arg.session);
        mutex_unlock(&__rt_io_mutex);
        return rc;
	}

    // tee_rtai = tee_shm_get_va(tee_shm, 0);
    // printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Is access from caller: %d. Pid: %d.\n",
    //     tee_rtai->is_access_ok, tee_rtai->task_id);


    /* Close the session */
    tee_shm_free(tee_shm);
    tee_client_close_session(ctx, sess_arg.session);

    mutex_unlock(&__rt_io_mutex);

	return rc;


}
EXPORT_SYMBOL_GPL(__check_rover_invariant_from_sw);

int notify_job_done_to_nw(int pid) {

    int rc;

    /* initialize SW context */
    mutex_lock(&__rt_io_mutex);

    rc = init_sw_context();

    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Create Context!\n");
        return -EFAULT;
    }

    rc = invoke_command_rt_job_done(ctx, sess_arg.session, pid);

	if (rc < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Invoke Command Failed!\n");
		tee_client_close_session(ctx, sess_arg.session);
	}

    tee_client_close_session(ctx, sess_arg.session);

    mutex_unlock(&__rt_io_mutex);

    return rc;

}


int remove_rt_task_from_sw(int pid)
{

    int rc;

    /* initialize SW context */
    mutex_lock(&__rt_io_mutex);

    rc = init_sw_context();

    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Create Context!\n");
        return -EFAULT;
    }

    rc = invoke_command_rem_rt_task(ctx, sess_arg.session, pid);

	if (rc < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Invoke Command Failed!\n");
		tee_client_close_session(ctx, sess_arg.session);
	}

    tee_client_close_session(ctx, sess_arg.session);

    mutex_unlock(&__rt_io_mutex);

    return rc;

}


/*
* returns PWM channel value
* from RT Task (userspace) input
*/
int get_pwm_channel_from_reg(int reg)
{
    switch (reg) {
	case 6:
    case 7:
    case 8:
    case 9:
		return S_HAT_SERVO_0;
    case 10:
    case 11:
    case 12:
    case 13:
		return S_HAT_SERVO_1;
	default:
		break;
	}

    return -EINVAL;
}
EXPORT_SYMBOL_GPL(get_pwm_channel_from_reg);

/* returns the index from periheral id */
static int get_index_from_peripheral_id(int pid, int php_id)
{
    RT_TSK_AI *ptr;
    int i;

    list_for_each_entry(ptr, &__rtai_list, list) {
        if (ptr->task_id == pid) {
            for (i=0; i<ptr->n_peripheral; i++) {
                if (ptr->p_access_list[i] == php_id)
                    return i;
            }
        }
    }

    return -EINVAL;
}




int store_rt_task_param_to_sw(RT_TSK_TEE_INFO *tee_rtai)
{

    u32 shm_size = sizeof(RT_TSK_TEE_INFO);
    struct tee_shm *tee_shm = NULL;
    RT_TSK_TEE_INFO *tee_rtai_va = NULL;
    int rc;

    /* initialize SW context */
    mutex_lock(&__rt_io_mutex);

    rc = init_sw_context();

    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Create Context!\n");
        return -EFAULT;
    }


    tee_shm = tee_shm_alloc(ctx, shm_size, TEE_SHM_MAPPED);
    // int j;
    //
    // for (j=0; j<10; j++) {
    //     printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
    //     "%s cmd-list[0][0]:%d\n",__func__, tee_rtai->actuation_cmd_list[0][j]);
    // }


    if (IS_ERR(tee_shm)) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "TEE SHM Alloc failed!\n");
		rc = PTR_ERR(tee_shm);
		tee_client_close_session(ctx, sess_arg.session);
        return rc;
	}

    tee_rtai_va = tee_shm_get_va(tee_shm, 0);

    /* copy elements */
    memcpy(tee_rtai_va, tee_rtai, sizeof(RT_TSK_TEE_INFO));

    /* Call SW function */
	rc = do_init_sw_shm(ctx, sess_arg.session, tee_shm,
        PTA_CMD_ADD_RT_TASK_INFO, sizeof(RT_TSK_TEE_INFO));
	if (rc < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Invoke SHM Failed!\n");
        tee_shm_free(tee_shm);
        tee_client_close_session(ctx, sess_arg.session);
	}

    /* Close the session */
    tee_shm_free(tee_shm);
    tee_client_close_session(ctx, sess_arg.session);

    mutex_unlock(&__rt_io_mutex);

	return rc;


}



static ssize_t rt_io_write(struct file *file, const char __user *ubuf,
                        size_t count, loff_t *ppos)
{
    char *command, *proc_buf, *tmp_command;
    char *tmp_php;
    char *act_cmd_list; /* to save actuation commnad lists */
    char *access_list;
    int i=0;
    int j;
    int pid;
    int tval;
    int php_indx;

    int t_acl[MAX_COMMAND_NUM]; /* for saving temp actuation cmd */

    char tchar[RT_IO_ACTU_CMD_LEN]; /* a temporary buffer */

    /* for processing RT task input */
    char *tok, *end, *char_ptr;

    RT_TSK_AI *ptr, *next;
    RT_TSK_AI *rtai = NULL;

    // RT_TSK_TEE_INFO tee_rtai;
    RT_TSK_TEE_INFO *tee_rtai;

    if(*ppos > 0 || count > RT_IO_PROC_BUF_SIZE) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
            "BUFFER SIZE Over! Return ERROR..\n");
        return -EFAULT;
    }



    proc_buf=kmalloc(count, GFP_KERNEL);
    proc_buf[count] = '\0';
    command=kmalloc(RT_IO_COMMAND_LEN, GFP_KERNEL);
    tmp_command=kmalloc(RT_IO_COMMAND_LEN, GFP_KERNEL);
    access_list=kmalloc(RT_IO_PHP_ACCESS_LEN, GFP_KERNEL);

    if(copy_from_user(proc_buf, ubuf, count))
		return -EFAULT;

    sscanf(proc_buf, "%s %s", command, access_list);

    // printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "write handler\n");
    //
    // printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "%s -> command: %s\n",
    //                                 __func__, command);
    // printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "%s -> access_list: %s\n",
    //                                 __func__, access_list);


    if(strcmp(command, __COMMAND_RT_TASK_REGISTER)==0) {


        // RT_TSK_AI *rtai;
        rtai = kmalloc(sizeof(*rtai), GFP_KERNEL);

        /* initialize access list and actuaion command list */
        for (i=0; i<MAX_PERIPHERAL; i++) {
            rtai->p_access_list[i] = RT_INVALID_NUMBER;
            // tee_rtai.p_access_list[i] = RT_INVALID_NUMBER;
            for (j=0; j<MAX_COMMAND_NUM; j++) {
                rtai->actuation_cmd_list[i][j] = RT_INVALID_NUMBER;
            }
            rtai->n_commands[i] = RT_INVALID_NUMBER;
        }

        // rtai->task_id = RT_INVALID_NUMBER;
        // rtai->n_peripheral = RT_INVALID_NUMBER;


        /* pid of the userspace task */

        rtai->task_id = (int) task_pid_nr(current);
        // tee_rtai.task_id =(int) task_pid_nr(current);

        tok = access_list, end = access_list;
        i = 0;
        while (tok != NULL) {
            strsep(&end, ":");
            tval = simple_strtoul(tok, &char_ptr, 16);
            // rtai->p_access_list[i] = simple_strtoul(tok, &char_ptr, 16);
            rtai->p_access_list[i] = tval;
            // tee_rtai.p_access_list[i] = tval;
            tok = end;
            i++;
            if (i>=MAX_PERIPHERAL) {
                printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
                    "PERIPHERAL ACCESS OUT OF BOUND!!\n");
            }

        }

        rtai->n_peripheral = i;
        // tee_rtai.n_peripheral = i;

        rtai->job_count=0; /* initial job counter */

        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "RTTASK PID:%d\n", rtai->task_id);
        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //                         "RTTASK N_P:%d\n", rtai->n_peripheral);
        // for (i=0; i<MAX_PERIPHERAL; i++) {
        //     printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //         "PERI_INDX:%d, ACCESS:%d\n", i, rtai->p_access_list[i]);
        // }

        /* initialize list */
        INIT_LIST_HEAD(&rtai->list);
        /* Add this to the main list. */
        list_add_tail(&rtai->list, &__rtai_list);


        // /* iterate and print the list */
        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "Display the list:\n");
        // list_for_each_entry(ptr, &__rtai_list, list) {
        //     printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "id: %d, n_peripheral: %d",
        //            ptr->task_id,
        //            ptr->n_peripheral);
        // }
        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "Display done!\n");

        /* now save this info to SW */

        // if (__RT_IO_PROT_ENABLED)
        //     store_rt_task_param_to_sw(&tee_rtai);

        /* This print is just for timestamping from kernel trace */
        // printk(KERN_INFO RT_IO_DRIVER_DEV_NAME
        //         "Task: %d -> Init Done! PROT_FLAG: %d\n",
        //         rtai->task_id, __RT_IO_PROT_ENABLED);

    }

    if(strcmp(command, __COMMAND_RT_TASK_DONE)==0) {
        /* pid of the userspace task */
        // __RT_IO_MON_PID = -1;

        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //     "Reset PID from write:%d\n", __RT_IO_MON_PID);

        pid = (int) task_pid_nr(current);


        // RT_TSK_AI *ptr, *next;
        list_for_each_entry_safe(ptr, next, &__rtai_list, list) {
            if (ptr->task_id == pid) {
                printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
                    "Removing PID:%d...\n", pid);
                list_del(&ptr->list);
                kfree(ptr);
            }
        }


        // /* iterate and print the list after delete */
        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "Display the list after del:\n");
        // // RT_TSK_AI *ptr;
        // list_for_each_entry(ptr, &__rtai_list, list) {
        //     printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "id: %d, n_peripheral: %d",
        //            ptr->task_id,
        //            ptr->n_peripheral);
        // }
        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME "Display done del!\n");
        //

        /* remove the RT task from SW */
        if (__RT_IO_PROT_ENABLED)
            remove_rt_task_from_sw(pid);

    }

    if(strcmp(command, __COMMAND_RT_JOB_DONE)==0) {

        pid = (int) task_pid_nr(current);

        list_for_each_entry(ptr, &__rtai_list, list) {
            if (ptr->task_id == pid) {
                printk(KERN_INFO RT_IO_DRIVER_DEV_NAME
                    "PID:%d -> Job %lu Done\n", pid, ptr->job_count);

                /*increase count */
                ptr->job_count++;

                if (__RT_IO_PROT_ENABLED)
                    notify_job_done_to_nw(pid);
            }
        }

    }

    if(strcmp(command, __COMMAND_RT_ADD_ACTU)==0) {

        tmp_php=kmalloc(RT_IO_PHP_ID_LEN, GFP_KERNEL);
        act_cmd_list=kmalloc(RT_IO_ACTU_CMD_LEN, GFP_KERNEL);
        sscanf(proc_buf, "%s %s %s", tmp_command, tmp_php, act_cmd_list);

        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //     "tmp_php: %s, act_cmd_list %s\n", tmp_php, act_cmd_list);


        tok = tmp_php;
        char_ptr = tchar;
        tval = (int) simple_strtoul(tok, &char_ptr, 16);

        pid = (int) task_pid_nr(current);
        php_indx = get_index_from_peripheral_id(pid, tval);

        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //     "php_indx got: %d, pid_got %d, php: %x\n", php_indx, pid, tval);

        if (php_indx < 0) {
            printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
                "Unable to perse peripheral id! Return ERROR..\n");

            kfree(tmp_php), kfree(act_cmd_list), kfree(proc_buf);
            kfree(command), kfree(tmp_command), kfree(access_list);
            return -EFAULT;
        }

        /* save actuation values to a temorary array */

        for (j=0; j<MAX_COMMAND_NUM; j++)
            t_acl[j] = RT_INVALID_NUMBER;

        char_ptr = tchar;
        tok = act_cmd_list, end = act_cmd_list;
        i = 0;
        while (tok != NULL) {
            strsep(&end, ":");
            tval = (int) simple_strtoul(tok, &char_ptr, 10);
            t_acl[i] = tval;
            tok = end;
            i++;
            if (i>=MAX_COMMAND_NUM) {
               printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
                   "PERIPHERAL CMD LEN OUT OF BOUND!!\n");
               kfree(tmp_php), kfree(act_cmd_list), kfree(proc_buf);
               kfree(command), kfree(tmp_command), kfree(access_list);
               return -EFAULT;
           }

        }

        // printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "N_ACT %d\n", i);

        /* now save to our struct */
        mutex_lock(&__rt_io_mutex);
        list_for_each_entry(ptr, &__rtai_list, list) {
            if (ptr->task_id == pid) {
                ptr->n_commands[php_indx] = i;
                for (i=0; i<ptr->n_commands[php_indx]; i++) {
                    ptr->actuation_cmd_list[php_indx][i] = t_acl[i];
                    // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
                    //     "CMD  id %d -> %d\n",
                    //     i, ptr->actuation_cmd_list[php_indx][i]);
                }
                break;
            }

        }
        mutex_unlock(&__rt_io_mutex);

        kfree(tmp_php);
        kfree(act_cmd_list);

    }

    if(strcmp(command, __COMMAND_RT_TX_SW)==0) {

        /* initialize tee struct */
        pid = (int) task_pid_nr(current);
        tee_rtai = kmalloc(sizeof(*tee_rtai), GFP_KERNEL);

        if (!tee_rtai) {
            printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
                "Unable to get memory for tee_rtai. Aborting...!\n");
                kfree(proc_buf), kfree(command);
                kfree(tmp_command), kfree(access_list);
                return -EFAULT;

        }

        mutex_lock(&__rt_io_mutex);
        list_for_each_entry(ptr, &__rtai_list, list) {
            if (ptr->task_id == pid) {

                tee_rtai->task_id = pid;
                tee_rtai->n_peripheral = ptr->n_peripheral;

                for (i=0; i<MAX_PERIPHERAL; i++) {
                    tee_rtai->p_access_list[i] = ptr->p_access_list[i];
                    tee_rtai->n_commands[i] = ptr->n_commands[i];
                    for (j=0; j<MAX_COMMAND_NUM; j++) {
                        tee_rtai->actuation_cmd_list[i][j] =
                                        ptr->actuation_cmd_list[i][j];
                    }
                }
                break;
            }
        }
        mutex_unlock(&__rt_io_mutex);

        /* now save this info to SW */
        if (__RT_IO_PROT_ENABLED)
            store_rt_task_param_to_sw(tee_rtai);

        // printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
        //     "Saving RT Task info to SW. PID: %d\n", pid);
        printk(KERN_INFO RT_IO_DRIVER_DEV_NAME
                "Task: %d -> Init Done. Saving to SW. PROT_FLAG: %d\n",
                pid, __RT_IO_PROT_ENABLED);

        kfree(tee_rtai);

    }

    // free all the buffers
    kfree(proc_buf);
    kfree(command);
    kfree(tmp_command);
    kfree(access_list);

	return count;
}

static ssize_t rt_io_read(struct file *file, char __user *ubuf,
                        size_t count, loff_t *ppos)
{
	printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "RT-IO read is NOT permitted!\n");
	return EPERM;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = rt_io_read,
	.write = rt_io_write,
};



static int __init rt_io_module_init(void)
{
    printk(KERN_INFO RT_IO_DRIVER_DEV_NAME " %s: Init RT Driver!\n", __func__);

    procfs_entry=proc_create(RT_IO_PROCFS_NAME, 0660, NULL, &fops);

    return 0;
}

static void __exit rt_io_module_cleanup(void)
{
    RT_TSK_AI *ptr, *next;
    int rc;

    list_for_each_entry_safe(ptr, next, &__rtai_list, list) {
        list_del(&ptr->list);
        kfree(ptr);
    }

    /* initialize SW context */
    rc = init_sw_context();
    if (rc < 0) {
        printk(KERN_ERR RT_IO_DRIVER_DEV_NAME "Cannot Clean Context!\n");
        return;
    }
    tee_client_close_session(ctx, sess_arg.session);
    tee_client_close_context(ctx);

    printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "Module cleanup done.\n");


    return;
}



module_init(rt_io_module_init);
module_exit(rt_io_module_cleanup);

MODULE_AUTHOR("Monowar Hasan");
MODULE_DESCRIPTION("RT-IO-SECURITY driver");
MODULE_LICENSE("GPL");
