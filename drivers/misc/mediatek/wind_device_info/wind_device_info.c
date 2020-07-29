//ranyanhao@wind.com 20160928 begin 

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <wind_device_info.h>
#include <linux/fs.h>
#include <linux/device.h>
struct device_info_dev{
	struct cdev dev;
	struct semaphore sem;
};

#define DEVICE_INFO_TAG                  "[DEVICE/INFO] "
#define DEVICE_INFO_FUN(f)               printk(KERN_INFO DEVICE_INFO_TAG"%s\n", __FUNCTION__)
#define DEVICE_INFO_ERR(fmt, args...)    printk(KERN_ERR  DEVICE_INFO_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define DEVICE_INFO_LOG(fmt, args...)    printk(KERN_INFO DEVICE_INFO_TAG fmt, ##args)
#define DEVICE_INFO_DBG(fmt, args...)    printk(KERN_INFO DEVICE_INFO_TAG fmt, ##args) 

#define DEV_NAME "device_info"
#define CLASS_NAME "wind_device"

static dev_t device_info_devno = 0;
struct device_info_dev g_device_info_dev;
static struct class *g_device_info_classp =NULL; 
static int test_val = 0;


/*********************************************************PRODUCT_LINE_DEMAND_START****************************************************************/
//#include "../include/mt-plat/battery_common.h"
#define PRODUCT_LINE_PROC_FOLDER "product_line"
//#define PRODUCT_LINE_PROC_BATTERY_SOC "battery_soc"
#define PRODUCT_LINE_PROC_TRAY "tray"
static struct proc_dir_entry *product_line_proc_dir = NULL;
//static struct proc_dir_entry *proc_battery_soc_file = NULL;
static struct proc_dir_entry *proc_tray_file = NULL;

/*
static ssize_t battery_soc_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	char *ptr = buf;

	if (*pos) {
		return 0;
	}

	ptr += sprintf(ptr,"%d\n", BMT_status.UI_SOC2);

	*pos += ptr - buf;

	return (ptr -buf);

}
static struct file_operations proc_battery_soc_file_ops =
{
	.owner = THIS_MODULE,
	.read = battery_soc_read,
};
*/

extern int mt_get_gpio_in_base(unsigned long pin);
#define GPIO_3 3
static ssize_t tray_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	char *ptr = buf;
	size_t size = 0;
	
	if (*pos) {
		return 0;
	}

	size = mt_get_gpio_in_base(GPIO_3);
	if (size) { //out
		ptr += sprintf(ptr,"%d\n", 0);
	} else {    //in
		ptr += sprintf(ptr,"%d\n", 1);
	}

	*pos += ptr - buf;

	return (ptr -buf);
}
static struct file_operations proc_tray_file_ops =
{
	.owner = THIS_MODULE,
	.read = tray_read,
};


static int product_line_proc_init(void)
{
	product_line_proc_dir = proc_mkdir(PRODUCT_LINE_PROC_FOLDER, NULL);
	if (product_line_proc_dir == NULL)
	{
		printk(" %s: product_line_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}

/*
	proc_battery_soc_file = proc_create(PRODUCT_LINE_PROC_BATTERY_SOC, (S_IWUSR|S_IRUGO|S_IWUGO), 
		product_line_proc_dir, &proc_battery_soc_file_ops);
	if(proc_battery_soc_file == NULL)
	{
		printk(" %s: proc battery_soc file create failed!\n", __func__);
		remove_proc_entry( PRODUCT_LINE_PROC_BATTERY_SOC, product_line_proc_dir );
		return -ENOMEM;
	}
*/	

	proc_tray_file = proc_create(PRODUCT_LINE_PROC_TRAY, (S_IWUSR|S_IRUGO|S_IWUGO), 
		product_line_proc_dir, &proc_tray_file_ops);
	if(proc_tray_file == NULL)
	{
		printk(" %s: proc calibration file create failed!\n", __func__);
		remove_proc_entry( PRODUCT_LINE_PROC_TRAY, product_line_proc_dir );
		return -ENOMEM;
	}

	return 0 ;
}

static void product_line_proc_deinit(void)
{
//	remove_proc_entry( PRODUCT_LINE_PROC_BATTERY_SOC, product_line_proc_dir );
	remove_proc_entry( PRODUCT_LINE_PROC_TRAY, product_line_proc_dir );
}
/*********************************************************PRODUCT_LINE_DEMAND_END****************************************************************/



/*********************************************************MODULE INFO DEFINE START****************************************************************/

/*
 *  wind_device_info is global variable! when you want to display some hardware information,
 *	in your own driver add :
 *							#include <wind_device_info.h>
 *							extern wind_device_info_t wind_device_info;
 *	and fill hardware information to this variable .
 */
wind_device_info_t wind_device_info = {{"Unknown_Name","Unknown_Name","Unknown_Name","Unknown_Name"},
									   {0,"Unknown_Name"},
									   {0,0,"Unknown_Name"},
									   {0,0,"Unknown_Name"},
									   {"Unknow","O",0,0,"Unknow_Image_Version",{0,0,0,0,0,0,0,0,0,0}}};


/*
 *   gsensor_info
 */
static ssize_t show_gsensor_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	DEVICE_INFO_FUN();
	if(NULL != wind_device_info.sensor_info_name.g_gsensor_name)
		size = sprintf(buf, "%s\n", wind_device_info.sensor_info_name.g_gsensor_name);
    return size;
}

static DEVICE_ATTR(gsensor_info, 0664, show_gsensor_info, NULL);



/*
 *   alsensor_info
 */
static ssize_t show_alsensor_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	DEVICE_INFO_FUN();
	if(NULL != wind_device_info.sensor_info_name.g_alsensor_name)
		size = sprintf(buf, "%s\n", wind_device_info.sensor_info_name.g_alsensor_name);
    return size;
}

static DEVICE_ATTR(alsensor_info, 0664, show_alsensor_info, NULL);



/*
 *   msensor_info
 */
static ssize_t show_msensor_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	DEVICE_INFO_FUN();
	if(NULL != wind_device_info.sensor_info_name.g_msensor_name)
		size = sprintf(buf, "%s\n", wind_device_info.sensor_info_name.g_msensor_name);
    return size;
}

static DEVICE_ATTR(msensor_info, 0664, show_msensor_info, NULL);



/*
 *   hallsensor_info
 */
static ssize_t show_hallsensor_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	DEVICE_INFO_FUN();
	if(NULL != wind_device_info.sensor_info_name.g_hallsensor_name)
		size = sprintf(buf, "%s\n", wind_device_info.sensor_info_name.g_hallsensor_name);
    return size;
}

static DEVICE_ATTR(hallsensor_info, 0664, show_hallsensor_info, NULL);



/*
 *   lcm_info
 */
static ssize_t show_lcm_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;
	DEVICE_INFO_FUN();
	//modify by qiangang@wind-mobi.com 20170728 begin
	buf_temp += sprintf(buf_temp, "IC:%s", wind_device_info.lcm_module_info.ic_name);
	//buf_temp += sprintf(buf_temp, "vendor:0x%x-\n",wind_device_info.lcm_module_info.vendor);
	//modify by qiangang@wind-mobi.com 20170728 end
    return (buf_temp - buf);
}

static DEVICE_ATTR(lcm_info, 0664, show_lcm_info, NULL);



/*
 *   ctp_info
 */
static ssize_t show_ctp_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;
	DEVICE_INFO_FUN();
	buf_temp += sprintf(buf_temp, "IC:%s-", wind_device_info.ctp_module_info.ic_name);
	buf_temp += sprintf(buf_temp, "vendor:0x%x-",wind_device_info.ctp_module_info.vendor);
	buf_temp += sprintf(buf_temp, "fwvr:0x%x\n", wind_device_info.ctp_module_info.fwvr);
    return (buf_temp - buf);
}

static DEVICE_ATTR(ctp_info, 0664, show_ctp_info, NULL);



/*
 *   fp_info
 */
static ssize_t show_FP_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;

	DEVICE_INFO_FUN();
	buf_temp += sprintf(buf_temp, "IC:%s-", wind_device_info.fp_module_info.ic_name);
	buf_temp += sprintf(buf_temp, "vendor:0x%x-",wind_device_info.fp_module_info.vendor);
	buf_temp += sprintf(buf_temp, "fwvr:0x%x\n", wind_device_info.fp_module_info.fwvr);

    return (buf_temp - buf);
}

static DEVICE_ATTR(FP_info, 0664, show_FP_info, NULL);



/*
 *   battery_info
 */
static ssize_t show_battery_id_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	DEVICE_INFO_FUN();
	size= sprintf(buf, "Battery_ID = %d\n", wind_device_info.battery_data.g_bat_id);
    return size;
}

static ssize_t show_battery_boot_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;
	DEVICE_INFO_FUN();
	
	buf_temp += sprintf(buf_temp, "get_hw_ocv = %d\n", wind_device_info.battery_data.battery_boot_data[0]);
	buf_temp += sprintf(buf_temp, "HW_SOC = %d\n", wind_device_info.battery_data.battery_boot_data[1]);
	buf_temp += sprintf(buf_temp, "SW_SOC = %d\n", wind_device_info.battery_data.battery_boot_data[2]);
	buf_temp += sprintf(buf_temp, "rtc_fg_soc = %d\n", wind_device_info.battery_data.battery_boot_data[3]);
	buf_temp += sprintf(buf_temp, "gFG_capacity_by_c = %d\n", wind_device_info.battery_data.battery_boot_data[4]);
	buf_temp += sprintf(buf_temp, "gFG_DOD0 = %d\n", wind_device_info.battery_data.battery_boot_data[5]);
	buf_temp += sprintf(buf_temp, "gFG_15_vlot = %d\n", wind_device_info.battery_data.battery_boot_data[6]);
	
     return (buf_temp - buf);
}

static ssize_t show_battery_version_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;

	DEVICE_INFO_FUN();
	buf_temp += sprintf(buf_temp, "%s-", wind_device_info.battery_data.BAT_Model_Name);
	buf_temp += sprintf(buf_temp, "%s-", wind_device_info.battery_data.Battery_type);
	buf_temp += sprintf(buf_temp, "0%x-", wind_device_info.battery_data.g_bat_id);
	buf_temp += sprintf(buf_temp, "000%d-", wind_device_info.battery_data.Driver_and_Data_Flash_Version);
	buf_temp += sprintf(buf_temp, "%s\n", wind_device_info.battery_data.Image_Version);
	/*After Driver_and_Data_Flash_Version need to insert image version*/
	
	return (buf_temp - buf);
}

static DEVICE_ATTR(battery_id_info, 0664, show_battery_id_info, NULL);
static DEVICE_ATTR(battery_boot_info, 0664, show_battery_boot_info, NULL);
static DEVICE_ATTR(battery_version_info, 0664, show_battery_version_info, NULL);


/*
 *   camera_info
 */
 //lihaiyan@wind-mobi.com 20170427 begin
#if (defined CONFIG_WIND_DEF_PRO_E266L)||(defined CONFIG_WIND_DEF_PRO_E267L)||(defined CONFIG_WIND_DEF_PRO_E260L)||(defined CONFIG_WIND_DEF_PRO_E262L)
char g_imgsensor_name[3][32];
static ssize_t show_camera_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;
	
	DEVICE_INFO_FUN();
	buf_temp += sprintf(buf_temp, "(%s)", g_imgsensor_name[0]);
	buf_temp += sprintf(buf_temp, "(%s)", g_imgsensor_name[1]);
	buf_temp += sprintf(buf_temp, "(%s)\n", g_imgsensor_name[2]);
	return (buf_temp - buf);
}
#else
 
 
extern char g_invokeSensorNameStr[2][32];
static ssize_t show_camera_info(struct device *dev,struct device_attribute *attr, char *buf)
{
	char *buf_temp = buf;
	DEVICE_INFO_FUN();
	buf_temp += sprintf(buf_temp, "(%s)", g_invokeSensorNameStr[0]);
	buf_temp += sprintf(buf_temp, "(%s)\n",g_invokeSensorNameStr[1]);
    return (buf_temp - buf);
}
#endif
 //lihaiyan@wind-mobi.com 20170427 end
static DEVICE_ATTR(camera_info, 0664, show_camera_info, NULL);

/*********************************************************MODULE INFO DEFINE END****************************************************************/


static void attr_files_create(struct device *device)
{	
	device_create_file(device, &dev_attr_lcm_info);	
	device_create_file(device, &dev_attr_gsensor_info);	
	device_create_file(device, &dev_attr_msensor_info);	
	device_create_file(device, &dev_attr_ctp_info);	
	device_create_file(device, &dev_attr_FP_info); 
	device_create_file(device, &dev_attr_camera_info);	
	device_create_file(device, &dev_attr_battery_boot_info);	
	device_create_file(device, &dev_attr_battery_id_info);	
	device_create_file(device, &dev_attr_alsensor_info);
	device_create_file(device, &dev_attr_hallsensor_info);
	device_create_file(device, &dev_attr_battery_version_info);
}


static int device_info_open(struct inode *inode, struct file *filp)
{
	struct device_info_dev *device_info_dev = NULL;
	DEVICE_INFO_FUN();
	device_info_dev = container_of(inode->i_cdev, struct device_info_dev, dev);
	filp->private_data = device_info_dev;
	return 0;

}


static int device_info_release(struct inode *inode, struct file *filp)
{
	DEVICE_INFO_FUN();
	return 0;
}


static ssize_t device_info_read(struct file *filp, char __user * buf, size_t count, loff_t *offp)
{
	DEVICE_INFO_FUN();
	if(count > sizeof(int))
		return 0;
	if(copy_to_user(buf, &test_val, sizeof(int)))
	{
		return -EFAULT;
	}
	return sizeof(int);

	
}


static ssize_t device_info_write(struct file *filp, const char __user *buf, size_t count, loff_t *offp)
{
	DEVICE_INFO_FUN();
	if(count > sizeof(int))
		return 0;
	if(copy_from_user(&test_val, buf, sizeof(int)))
	{
		return -EFAULT;
	}
	return sizeof(int);	
}


static long  device_info_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{    
	int err = 0;
	DEVICE_INFO_FUN();
	switch(cmd)
	{
	}
	return err;
}


static struct file_operations device_info_fops =
{
    .owner = THIS_MODULE,
    .open = &device_info_open,
    .release = &device_info_release,
    .write = &device_info_write,
    .read = &device_info_read,
    .unlocked_ioctl = &device_info_unlocked_ioctl,
};


static int  device_info_init(void)
{
	int err;
    struct device *class_dev = NULL;
	struct device_info_dev *device_info_devp;
	DEVICE_INFO_FUN();
	device_info_devp = &g_device_info_dev;

	err = alloc_chrdev_region(&device_info_devno, 0, 1, DEV_NAME);
	if(err){
		DEVICE_INFO_ERR("register device number error!!!! \n");
		goto fail;
	}

	cdev_init(&device_info_devp->dev, &device_info_fops);
	device_info_devp->dev.owner = THIS_MODULE;

   	err = cdev_add(&device_info_devp->dev, device_info_devno, 1);
	if(err){
		DEVICE_INFO_ERR("cdev_add error!!!!! \n");
		goto err0;
	}

    g_device_info_classp = class_create(THIS_MODULE, CLASS_NAME);
    class_dev = (struct device *)device_create(g_device_info_classp, 
                                                   NULL, 
                                                   device_info_devno, 
                                                   NULL, 
                                                   DEV_NAME);
	attr_files_create(class_dev);
	product_line_proc_init();

	return 0;
	
err0:
	unregister_chrdev_region(device_info_devno, 1);
fail:	
	return err;

}


static void  device_info_exit(void)
{
	DEVICE_INFO_FUN();
	product_line_proc_deinit();
}

module_init(device_info_init);
module_exit(device_info_exit);
MODULE_DESCRIPTION("Wind Device Info");
MODULE_AUTHOR("liqiang<liqiang@wind-mobi.com>");
MODULE_LICENSE("GPL");
//ranyanhao@wind.com 20160928 end
