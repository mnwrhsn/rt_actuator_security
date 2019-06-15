/*
    i2c-dev.c - i2c-bus driver, char device interface

    Copyright (C) 1995-97 Simon G. Vogl
    Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
    Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

/* Note that this is a complete rewrite of Simon Vogl's i2c-dev module.
   But I have used so much of his original code and ideas that it seems
   only fair to recognize him as co-author -- Frodo */

/* The I2C_RDWR ioctl code is written by Kolja Waschk <waschk@telos.de> */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/delay.h> /* usleep_range */

/* include for RT-IO-DRIVER and OPTEE */
#include "../tee/rt_io_driver/rt_io_driver.h"
// #include <linux/tee_drv.h>
// #include <linux/uuid.h>
// #include "../tee/optee/optee_private.h"

// #include <linux/rt_io_driver.h>

// extern int call_to_secure_world(void);


/*
 * An i2c_dev represents an i2c_adapter ... an I2C or SMBus master, not a
 * slave (i2c_client) with which messages will be exchanged.  It's coupled
 * with a character special file which is accessed by user mode drivers.
 *
 * The list of i2c_dev structures is parallel to the i2c_adapter lists
 * maintained by the driver model, and is updated using bus notifications.
 */
struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
	struct cdev cdev;
};

#define I2C_MINORS	MINORMASK
static LIST_HEAD(i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static void put_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct i2c_dev *i2c_dev = i2c_dev_get_by_minor(MINOR(dev->devt));

	if (!i2c_dev)
		return -ENODEV;
	return sprintf(buf, "%s\n", i2c_dev->adap->name);
}
static DEVICE_ATTR_RO(name);

static struct attribute *i2c_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(i2c);

/* ------------------------------------------------------------------------- */

/*
 * After opening an instance of this character special file, a file
 * descriptor starts out associated only with an i2c_adapter (and bus).
 *
 * Using the I2C_RDWR ioctl(), you can then *immediately* issue i2c_msg
 * traffic to any devices on the bus used by that adapter.  That's because
 * the i2c_msg vectors embed all the addressing information they need, and
 * are submitted directly to an i2c_adapter.  However, SMBus-only adapters
 * don't support that interface.
 *
 * To use read()/write() system calls on that file descriptor, or to use
 * SMBus interfaces (and work with SMBus-only hosts!), you must first issue
 * an I2C_SLAVE (or I2C_SLAVE_FORCE) ioctl.  That configures an anonymous
 * (never registered) i2c_client so it holds the addressing information
 * needed by those system calls and by this SMBus interface.
 */

static ssize_t i2cdev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	char *tmp;
	int ret;

	struct i2c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	pr_debug("i2c-dev: i2c-%d reading %zu bytes.\n",
		iminor(file_inode(file)), count);

	ret = i2c_master_recv(client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	kfree(tmp);
	return ret;
}

static ssize_t vanilla_i2cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
	struct i2c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	pr_debug("i2c-dev: i2c-%d writing %zu bytes.\n",
		iminor(file_inode(file)), count);

	ret = i2c_master_send(client, tmp, count);
	kfree(tmp);
	return ret;
}

/* our wrapper to read sensor from kernel space */
static ssize_t rt_io_custtom_i2cdev_read(struct file *file, char *buf,
										size_t count, loff_t *offset)
{
	char *tmp;
	int ret;
	int i;

	struct i2c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	pr_debug("i2c-dev: i2c-%d reading %zu bytes.\n",
		iminor(file_inode(file)), count);

	ret = i2c_master_recv(client, tmp, count);

	// if (ret >= 0)
	// 	ret = memcpy(buf, tmp, count*sizeof(char)) ? -EFAULT : ret;

	if (ret >= 0) {
		for (i=0; i<count; i++) {
			buf[i] = tmp[i];
		}
	}

	// if (IS_ERR(buf)) {
	// 	printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
	// 		"is error buff true!");
	// }

	kfree(tmp);
	return ret;
}


/* this is our wrapper for i2c-dev write */
static ssize_t rt_io_custom_i2cdev_write(struct file *file, const char *buf,
		size_t count, loff_t *offset)
{
	// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
	// 	"custom i2c write called...");
	int ret;
	char *tmp;
	struct i2c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count*sizeof(char), GFP_KERNEL);
	memcpy(tmp, buf, count*sizeof(char));

	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	pr_debug("i2c-dev: i2c-%d writing %zu bytes.\n",
		iminor(file_inode(file)), count);

	ret = i2c_master_send(client, tmp, count);
	kfree(tmp);
	return ret;
}

void print_buff(const char __user *buf, size_t count) {
	int i;
	for (i=0; i< count; i++) {
		printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			"buf[%d]: %d", i, buf[i]);

	}
}

static ssize_t get_lf_sensor_reading(struct file *file, loff_t *offset,
									char* read_val)
{

	char w_buf[GPG_WRITE_BUF_SIZE];
	char r_buf[GPG_READ_BUF_SIZE]={0};

	int ret;
	int i;
	int flag=0;

	w_buf[0]=1;
    w_buf[1]=GPG_LINE_READ_CMD;
    w_buf[2]=0;
    w_buf[3]=0;
    w_buf[4]=0;

	ret = rt_io_custom_i2cdev_write(file, w_buf, GPG_WRITE_BUF_SIZE, offset);

	if (ret < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
			"LF Sensor Write Failed");
		return -EFAULT;
	}

	udelay(100);

	ret  = rt_io_custtom_i2cdev_read(file, r_buf, GPG_LF_REG_SIZE, offset);

	if (ret < 0) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
			"LF Sensor Read Failed. Return code: %d", ret);
			return -EFAULT;
	}

	/* obtained from GPG */

	for(i=0;i<GPG_LF_REG_SIZE;i=i+2) {
		// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
		// 	"buf[%d] is: %d.", i, r_buf[i]);                   // To convert the 10 bit analog reading of each sensor to decimal and store it in read_val[]
		read_val[i/2]=r_buf[i]*256+r_buf[i+1]; // Values less than 100 - White, Values greater than 800- Black
        if (read_val[i/2] > 65000) {
			flag=1;
		}             // Checking for junk values in the input

	}
    if (flag==1) {
		printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
			"Flag is ture!\n");
		for(i=0;i<GPG_N_LF_SENSOR;i++) {
			read_val[i] = 255;                    // Making junk input values to -1
		}
    }

	// for (i=0;i<GPG_N_LF_SENSOR;i++) {
	// 	printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
	// 		"LF Val[%d]: %d\n", i, read_val[i]);
	// }

	return ret;



}

static ssize_t __do_rt_io_rover_check_with_lf(struct file *file,
							   const char __user *buf,
							   size_t count, loff_t *offset)
{
	int res;
	int cur_pid = (int)current->pid;
	RT_TSK_ROV_INV_CHK rt_rov_data;
	char read_val[GPG_N_LF_SENSOR]={255};
	int i;

	if (__rt_io_get_pid(cur_pid)) {

		/* Check if our protection enabled */
		if (__RT_IO_PROT_ENABLED) {

			//TODO
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before NW check...");
			// print_buff(buf, count);

			if (count != GPG_WRITE_BUF_SIZE) {
				printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
					"Rover Buffer size missmatch");
				return -EBADF;
			}

			/* if this is reading some sensors ignore world switch */
			if (buf[1] == GPG_LINE_READ_CMD ||
				buf[1] == GPG_ENC_READ_CMD   ||
				buf[1] ==  GPG_VOLT_CMD)
				return vanilla_i2cdev_write(file, buf, count, offset);


			res = get_lf_sensor_reading(file, offset, read_val);

			rt_rov_data.task_id = cur_pid;
			rt_rov_data.periph = GPG_ROV_NAV;
			rt_rov_data.is_access_ok = false;

			for (i=0; i<GPG_N_LF_SENSOR; i++)
				rt_rov_data.gpg_lf_sensor_val[i] = read_val[i];

			for (i=0; i<count; i++) {
				rt_rov_data.gpg_cmd_nw[i] = buf[i];
				rt_rov_data.gpg_cmd_predict_sw[i] = buf[i];  /* copy same */

			}
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before inv check...");
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before NW -> is Access: %d...", rt_rov_data.is_access_ok);

			res = __check_rover_invariant_from_sw(&rt_rov_data,
									PTA_CMD_CHECK_ROVER_INFO_W_LF);


			if (rt_rov_data.is_access_ok) {
				/* write the command only if access is okay
				* and pass invariant checking
				*/
				return rt_io_custom_i2cdev_write(file,
					rt_rov_data.gpg_cmd_predict_sw,
					GPG_WRITE_BUF_SIZE, offset);
			}
			else
				return -EBADF;


			// return vanilla_i2cdev_write(file, buf, count, offset);
		}
		else {

			/* invariant checking is not enabled. do original call */
			return vanilla_i2cdev_write(file, buf, count, offset);
		}




	}
	else {
		/* if PID not a RT task, return the original call */
		return vanilla_i2cdev_write(file, buf, count, offset);
	}


}


static ssize_t __do_rt_io_rover_check(struct file *file, const char __user *buf,
							   size_t count, loff_t *offset)
{
	int res;
	int cur_pid = (int)current->pid;
	RT_TSK_ROV_INV_CHK rt_rov_data;
	int i;

	if (__rt_io_get_pid(cur_pid)) {

		/* Check if our protection enabled */
		if (__RT_IO_PROT_ENABLED) {

			//TODO
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before NW check...");
			// print_buff(buf, count);

			if (count != GPG_WRITE_BUF_SIZE) {
				printk(KERN_ERR RT_IO_DRIVER_DEV_NAME
					"Rover Buffer size missmatch");
				return -EBADF;
			}

			rt_rov_data.task_id = cur_pid;
			rt_rov_data.periph = GPG_ROV_NAV;
			rt_rov_data.is_access_ok = false;

			for (i=0; i<GPG_N_LF_SENSOR; i++)
				rt_rov_data.gpg_lf_sensor_val[i] = RT_INVALID_NUMBER;

			for (i=0; i<count; i++) {
				rt_rov_data.gpg_cmd_nw[i] = buf[i];
				rt_rov_data.gpg_cmd_predict_sw[i] = buf[i];  /* copy same */

			}
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before inv check...");
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"Before NW -> is Access: %d...", rt_rov_data.is_access_ok);

			res = __check_rover_invariant_from_sw(&rt_rov_data,
									PTA_CMD_CHECK_ROVER_RT_TASK_INFO);



			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"After NW check...");
			// print_buff(rt_rov_data.gpg_cmd_predict_sw, GPG_WRITE_BUF_SIZE);
			// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
			// 	"is Access: %d...", rt_rov_data.is_access_ok);

			if (rt_rov_data.is_access_ok) {
				/* write the command only if access is okay
				* and pass invariant checking
				*/
				// printk(KERN_DEBUG RT_IO_DRIVER_DEV_NAME
				// 	"Pass Inv Check...");
				return rt_io_custom_i2cdev_write(file,
					rt_rov_data.gpg_cmd_predict_sw,
					GPG_WRITE_BUF_SIZE, offset);
			}
			else
				return -EBADF;

			// return vanilla_i2cdev_write(file, buf, count, offset);
		}
		else {

			/* invariant checking is not enabled. do original call */
			return vanilla_i2cdev_write(file, buf, count, offset);
		}




	}
	else {
		/* if PID not a RT task, return the original call */
		return vanilla_i2cdev_write(file, buf, count, offset);
	}



}

static ssize_t i2cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	/* this is for Rover platform */
	#ifdef __RT_PLAT_GPG_ROVER
		/* Do Checking if only enabled */
		#ifdef __RT_IO_PROT_ENABLED

		/* this is for DoS rate limit */
		return __do_rt_io_rover_check(file, buf, count, offset);

		/* this is for LF invariant check */
		// return __do_rt_io_rover_check_with_lf(file, buf, count, offset);


		#else
			/* return the original call */
			return vanilla_i2cdev_write(file, buf, count, offset);
		#endif
	#else
		/* return the original call */
		return vanilla_i2cdev_write(file, buf, count, offset);
	#endif
}



static int i2cdev_check(struct device *dev, void *addrp)
{
	struct i2c_client *client = i2c_verify_client(dev);

	if (!client || client->addr != *(unsigned int *)addrp)
		return 0;

	return dev->driver ? -EBUSY : 0;
}

/* walk up mux tree */
static int i2cdev_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result;

	result = device_for_each_child(&adapter->dev, &addr, i2cdev_check);
	if (!result && parent)
		result = i2cdev_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */
static int i2cdev_check_mux_children(struct device *dev, void *addrp)
{
	int result;

	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
						i2cdev_check_mux_children);
	else
		result = i2cdev_check(dev, addrp);

	return result;
}

/* This address checking function differs from the one in i2c-core
   in that it considers an address with a registered device, but no
   driver bound to it, as NOT busy. */
static int i2cdev_check_addr(struct i2c_adapter *adapter, unsigned int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;

	if (parent)
		result = i2cdev_check_mux_parents(parent, addr);

	if (!result)
		result = device_for_each_child(&adapter->dev, &addr,
						i2cdev_check_mux_children);

	return result;
}

static noinline int i2cdev_ioctl_rdwr(struct i2c_client *client,
		unsigned long arg)
{
	// printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "in %s: PID:%d\n", __func__, (int)current->pid);
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, res;

	if (copy_from_user(&rdwr_arg,
			   (struct i2c_rdwr_ioctl_data __user *)arg,
			   sizeof(rdwr_arg)))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
	if (rdwr_arg.nmsgs > I2C_RDWR_IOCTL_MAX_MSGS)
		return -EINVAL;

	rdwr_pa = memdup_user(rdwr_arg.msgs,
			      rdwr_arg.nmsgs * sizeof(struct i2c_msg));
	if (IS_ERR(rdwr_pa))
		return PTR_ERR(rdwr_pa);

	data_ptrs = kmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(rdwr_pa);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount */
		if (rdwr_pa[i].len > 8192) {
			res = -EINVAL;
			break;
		}

		data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
		rdwr_pa[i].buf = memdup_user(data_ptrs[i], rdwr_pa[i].len);
		if (IS_ERR(rdwr_pa[i].buf)) {
			res = PTR_ERR(rdwr_pa[i].buf);
			break;
		}

		/*
		 * If the message length is received from the slave (similar
		 * to SMBus block read), we must ensure that the buffer will
		 * be large enough to cope with a message length of
		 * I2C_SMBUS_BLOCK_MAX as this is the maximum underlying bus
		 * drivers allow. The first byte in the buffer must be
		 * pre-filled with the number of extra bytes, which must be
		 * at least one to hold the message length, but can be
		 * greater (for example to account for a checksum byte at
		 * the end of the message.)
		 */
		if (rdwr_pa[i].flags & I2C_M_RECV_LEN) {
			if (!(rdwr_pa[i].flags & I2C_M_RD) ||
			    rdwr_pa[i].buf[0] < 1 ||
			    rdwr_pa[i].len < rdwr_pa[i].buf[0] +
					     I2C_SMBUS_BLOCK_MAX) {
				res = -EINVAL;
				break;
			}

			rdwr_pa[i].len = rdwr_pa[i].buf[0];
		}
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(rdwr_pa[j].buf);
		kfree(data_ptrs);
		kfree(rdwr_pa);
		return res;
	}

	res = i2c_transfer(client->adapter, rdwr_pa, rdwr_arg.nmsgs);
	while (i-- > 0) {
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user(data_ptrs[i], rdwr_pa[i].buf,
					 rdwr_pa[i].len))
				res = -EFAULT;
		}
		kfree(rdwr_pa[i].buf);
	}
	kfree(data_ptrs);
	kfree(rdwr_pa);
	return res;
}

static noinline int i2cdev_ioctl_smbus(struct i2c_client *client,
		unsigned long arg)
{
	// printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "in %s: PID:%d\n", __func__, (int)current->pid);
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp = {};
	int datasize, res;

	if (copy_from_user(&data_arg,
			   (struct i2c_smbus_ioctl_data __user *) arg,
			   sizeof(struct i2c_smbus_ioctl_data)))
		return -EFAULT;
	if ((data_arg.size != I2C_SMBUS_BYTE) &&
	    (data_arg.size != I2C_SMBUS_QUICK) &&
	    (data_arg.size != I2C_SMBUS_BYTE_DATA) &&
	    (data_arg.size != I2C_SMBUS_WORD_DATA) &&
	    (data_arg.size != I2C_SMBUS_PROC_CALL) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
	    (data_arg.size != I2C_SMBUS_I2C_BLOCK_DATA) &&
	    (data_arg.size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_dbg(&client->adapter->dev,
			"size out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.size);
		return -EINVAL;
	}
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((data_arg.read_write != I2C_SMBUS_READ) &&
	    (data_arg.read_write != I2C_SMBUS_WRITE)) {
		dev_dbg(&client->adapter->dev,
			"read_write out of range (%x) in ioctl I2C_SMBUS.\n",
			data_arg.read_write);
		return -EINVAL;
	}

	/* Note that command values are always valid! */

	if ((data_arg.size == I2C_SMBUS_QUICK) ||
	    ((data_arg.size == I2C_SMBUS_BYTE) &&
	    (data_arg.read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data */
		return i2c_smbus_xfer(client->adapter, client->addr,
				      client->flags, data_arg.read_write,
				      data_arg.command, data_arg.size, NULL);

	if (data_arg.data == NULL) {
		dev_dbg(&client->adapter->dev,
			"data is NULL pointer in ioctl I2C_SMBUS.\n");
		return -EINVAL;
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
	if (data_arg.size == I2C_SMBUS_I2C_BLOCK_BROKEN) {
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		data_arg.size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (data_arg.read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}
	res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
	      data_arg.read_write, data_arg.command, data_arg.size, &temp);
	if (!res && ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
		     (data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
		     (data_arg.read_write == I2C_SMBUS_READ))) {
		if (copy_to_user(data_arg.data, &temp, datasize))
			return -EFAULT;
	}
	return res;
}


static long vanilla_i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = file->private_data;
	unsigned long funcs;

	dev_dbg(&client->adapter->dev, "ioctl, cmd=0x%02x, arg=0x%02lx\n",
		cmd, arg);

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		if ((arg > 0x3ff) ||
		    (((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
			return -EINVAL;
		if (cmd == I2C_SLAVE && i2cdev_check_addr(client->adapter, arg))
			return -EBUSY;
		/* REVISIT: address could become busy later */
		client->addr = arg;
		return 0;
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_M_TEN;
		else
			client->flags &= ~I2C_M_TEN;
		return 0;
	case I2C_PEC:
		/*
		 * Setting the PEC flag here won't affect kernel drivers,
		 * which will be using the i2c_client node registered with
		 * the driver model core.  Likewise, when that client has
		 * the PEC flag already set, the i2c-dev driver won't see
		 * (or use) this setting.
		 */
		if (arg)
			client->flags |= I2C_CLIENT_PEC;
		else
			client->flags &= ~I2C_CLIENT_PEC;
		return 0;
	case I2C_FUNCS:
		funcs = i2c_get_functionality(client->adapter);
		return put_user(funcs, (unsigned long __user *)arg);

	case I2C_RDWR:
		return i2cdev_ioctl_rdwr(client, arg);

	case I2C_SMBUS:
		return i2cdev_ioctl_smbus(client, arg);

	case I2C_RETRIES:
		client->adapter->retries = arg;
		break;
	case I2C_TIMEOUT:
		/* For historical reasons, user-space sets the timeout
		 * value in units of 10 ms.
		 */
		client->adapter->timeout = msecs_to_jiffies(arg * 10);
		break;
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}

// /* redefined from drivers/tee/optee/device.c */
// static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
// {
// 	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
// 		return 1;
// 	else
// 		return 0;
// }

//
// static int invoke_command_rt_io_pta(struct tee_context *ctx, u32 session,
// 		       struct tee_shm *device_shm, u32 *shm_size)
// {
// 	int ret = 0;
// 	struct tee_ioctl_invoke_arg inv_arg;
// 	struct tee_param param[4];
//
// 	memset(&inv_arg, 0, sizeof(inv_arg));
// 	memset(&param, 0, sizeof(param));
//
// 	/* Invoke PTA_CMD_GET_DEVICES function */
// 	// inv_arg.func = PTA_CMD_GET_DEVICES;
// 	inv_arg.session = session;
// 	inv_arg.num_params = 4;
//
// 	/* Fill invoke cmd params */
// 	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
// 	param[0].u.memref.shm = device_shm;
// 	param[0].u.memref.size = *shm_size;
// 	param[0].u.memref.shm_offs = 0;
//
// 	ret = tee_client_invoke_func(ctx, &inv_arg, param);
// 	if (ret < 0) {
// 		pr_err("RT_IO_SEC_PTA invoke function err: %x\n",
// 		       inv_arg.ret);
// 		return -EINVAL;
// 	}
//
// 	*shm_size = param[0].u.memref.size;
//
// 	return 0;
//
// }

//
// int call_to_secure_world(void)
// {
// 	const uuid_t pta_uuid =
// 		UUID_INIT(0xd7cf3a38, 0x6626, 0x11e9,
// 			  0xa9, 0x23, 0x16, 0x81, 0xbe, 0x66, 0x3d, 0x3e);
// 	struct tee_ioctl_open_session_arg sess_arg;
// 	u32 shm_size = 0;
//
// 	struct tee_context *ctx = NULL;
// 	int rc;
//
// 	memset(&sess_arg, 0, sizeof(sess_arg));
//
// 	/* Open context with OP-TEE driver */
// 	ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
// 	if (IS_ERR(ctx))
// 		return -ENODEV;
//
// 	/* Open session with device enumeration pseudo TA */
// 	memcpy(sess_arg.uuid, pta_uuid.b, TEE_IOCTL_UUID_LEN);
// 	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
// 	sess_arg.num_params = 0;
//
// 	rc = tee_client_open_session(ctx, &sess_arg, NULL);
// 	if ((rc < 0) || (sess_arg.ret != TEEC_SUCCESS)) {
// 		/* RT-IO-SEC pseudo TA not found */
// 		rc = 0;
// 		printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "TA Not Found!\n");
// 		goto out_ctx;
// 	}
// 	printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "Open session successfully!\n");
//
// 	rc = invoke_command_rt_io_pta(ctx, sess_arg.session, NULL, &shm_size);
// 	if (rc < 0) {
// 		printk(KERN_INFO RT_IO_DRIVER_DEV_NAME "Invoke Command Failed!\n");
// 		goto out_sess;
// 	}
//
//
// 	// TODO Need to fix the logic
//
// 	goto out_sess;
//
// 	out_sess:
// 		tee_client_close_session(ctx, sess_arg.session);
//
// 	out_ctx:
// 		tee_client_close_context(ctx);
//
// 	return rc;
//
//
// }

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	// static int i=0;
	// if (i==0) {
	// 	printk(KERN_INFO RT_IO_DRIVER_DEV_NAME " %s: %d\n", __func__, (int)current->pid);
	// 	// i++;
	// 	call_to_secure_world();
	// }
	// if (i==5) {
	// 	i=-1;
	// }
	//
	// i++;

	// return the original call
	return vanilla_i2cdev_ioctl(file, cmd, arg);

}

static int i2cdev_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct i2c_client *client;
	struct i2c_adapter *adap;

	adap = i2c_get_adapter(minor);
	if (!adap)
		return -ENODEV;

	/* This creates an anonymous i2c_client, which may later be
	 * pointed to some address using I2C_SLAVE or I2C_SLAVE_FORCE.
	 *
	 * This client is ** NEVER REGISTERED ** with the driver model
	 * or I2C core code!!  It just holds private copies of addressing
	 * information and maybe a PEC flag.
	 */
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adap);
		return -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "i2c-dev %d", adap->nr);

	client->adapter = adap;
	file->private_data = client;

	return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations i2cdev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= i2cdev_read,
	.write		= i2cdev_write,
	.unlocked_ioctl	= i2cdev_ioctl,
	.open		= i2cdev_open,
	.release	= i2cdev_release,
};

/* ------------------------------------------------------------------------- */

static struct class *i2c_dev_class;

static int i2cdev_attach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;
	int res;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);

	i2c_dev = get_free_i2c_dev(adap);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	cdev_init(&i2c_dev->cdev, &i2cdev_fops);
	i2c_dev->cdev.owner = THIS_MODULE;
	res = cdev_add(&i2c_dev->cdev, MKDEV(I2C_MAJOR, adap->nr), 1);
	if (res)
		goto error_cdev;

	/* register this i2c device with the driver core */
	i2c_dev->dev = device_create(i2c_dev_class, &adap->dev,
				     MKDEV(I2C_MAJOR, adap->nr), NULL,
				     "i2c-%d", adap->nr);
	if (IS_ERR(i2c_dev->dev)) {
		res = PTR_ERR(i2c_dev->dev);
		goto error;
	}

	pr_debug("i2c-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->nr);
	return 0;
error:
	cdev_del(&i2c_dev->cdev);
error_cdev:
	put_i2c_dev(i2c_dev);
	return res;
}

static int i2cdev_detach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);

	i2c_dev = i2c_dev_get_by_minor(adap->nr);
	if (!i2c_dev) /* attach_adapter must have failed */
		return 0;

	cdev_del(&i2c_dev->cdev);
	put_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, adap->nr));

	pr_debug("i2c-dev: adapter [%s] unregistered\n", adap->name);
	return 0;
}

static int i2cdev_notifier_call(struct notifier_block *nb, unsigned long action,
			 void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return i2cdev_attach_adapter(dev, NULL);
	case BUS_NOTIFY_DEL_DEVICE:
		return i2cdev_detach_adapter(dev, NULL);
	}

	return 0;
}

static struct notifier_block i2cdev_notifier = {
	.notifier_call = i2cdev_notifier_call,
};

/* ------------------------------------------------------------------------- */

/*
 * module load/unload record keeping
 */

static int __init i2c_dev_init(void)
{
	int res;

	printk(KERN_INFO "i2c /dev entries driver\n");

	res = register_chrdev_region(MKDEV(I2C_MAJOR, 0), I2C_MINORS, "i2c");
	if (res)
		goto out;

	i2c_dev_class = class_create(THIS_MODULE, "i2c-dev");
	if (IS_ERR(i2c_dev_class)) {
		res = PTR_ERR(i2c_dev_class);
		goto out_unreg_chrdev;
	}
	i2c_dev_class->dev_groups = i2c_groups;

	/* Keep track of adapters which will be added or removed later */
	res = bus_register_notifier(&i2c_bus_type, &i2cdev_notifier);
	if (res)
		goto out_unreg_class;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(NULL, i2cdev_attach_adapter);

	return 0;

out_unreg_class:
	class_destroy(i2c_dev_class);
out_unreg_chrdev:
	unregister_chrdev_region(MKDEV(I2C_MAJOR, 0), I2C_MINORS);
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit i2c_dev_exit(void)
{
	bus_unregister_notifier(&i2c_bus_type, &i2cdev_notifier);
	i2c_for_each_dev(NULL, i2cdev_detach_adapter);
	class_destroy(i2c_dev_class);
	unregister_chrdev_region(MKDEV(I2C_MAJOR, 0), I2C_MINORS);
}

MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl> and "
		"Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C /dev entries driver");
MODULE_LICENSE("GPL");

module_init(i2c_dev_init);
module_exit(i2c_dev_exit);
