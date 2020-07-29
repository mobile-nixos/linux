
/** lihaiyan@wind-mobi.com 20170327 begin **/

/************************************************************************
** NOTICE ONE: What have you to do ?
**	1. add  psensor_proc_init() in xxx_prob() to init proc node
**	2. add  psensor_proc_remove() in xxx_remove() to remove proc node
**
** NOTICE TWO: How to use it ?
**	1. /proc/lightsensor/calibration 
** 
**   sys:  /sys/bus/platform/driver/als_ps/alsps_fac_cal
************************************************************************/

#include "alsps_calibrator.h"


#define LIGHTSENSOR_PROC_FOLDER 							"lightsensor"
#define LIGHTSENSOR_PROC_CALIBRATION_FILE 					"calibration"
#define LIGHTSENSOR_PROC_READBACK_FILE 						"readback"
#define LIGHTSENSOR_PROC_CALIBRATION_FACTOR_FILE 			"lux_per_count"

static struct proc_dir_entry *lightsensor_proc_dir 			= NULL;
static struct proc_dir_entry *proc_calibration_file 		= NULL;
static struct proc_dir_entry *proc_readback_file 			= NULL;
static struct proc_dir_entry *proc_calibration_factor_file 	= NULL;

#define LIGHT_BAR_400LUX    400
//set a default factor (factor = (400 * 1000)/ current_als )
int als_calibration_factor = 1000;  //default 100   
u16 cali_factor_400lux = 0;   //save the calibration factor after *983*01# calibration 


#define FILTER_ALS_VALUE_ENABLE    	1
/* als_buf[ 0 ] - als_buf[ FILTER_NUM ]: save als value of the last %FILTER_NUM% times ,to get the average in als_buf[ FILTER_NUM ]	*/
#define FILTER_NUM					3
unsigned short als_buf[FILTER_NUM + 1] = {0};  

//extern void start_als_value(void);
alsps_start_als_value_func   start_als_value_func = NULL;

int get_als_calibration_factor(void)
{	
	return als_calibration_factor;
} 

u16 filter_als_value(u16 als)
{
	//static unsigned int cnt = 0;
	//unsigned char i = 0;
	//unsigned long sum_value = 0;
	
	// 1. save the ALS value present in the array[cnt]
	//als_buf[cnt] = als;    

	// 2. plus all the member of arry and calculte the average
	//for(i = 0; i < FILTER_NUM; i++){
	//	sum_value += als_buf[cnt];
	//	printk("[TL5601C][%s]ALS value: als_buf[ %d ] = %d \n", __func__, cnt , als_buf[cnt]);
	//}
	//als_buf[FILTER_NUM] = sum_value / FILTER_NUM;
	als_buf[FILTER_NUM] = als;
	als = ( als_calibration_factor * als_buf[FILTER_NUM] ) / 1000;  //this value just for report ,not for calibration
	
	printk("[TL5601C][%s] als_buf[%d] = %d,  als = %d \n",__func__,FILTER_NUM, als_buf[FILTER_NUM], als);
	// 3.reset the count number
	//cnt++;
	//cnt %= FILTER_NUM; 
	// 4.return als_buf average value
	return als;
}



/******  als_factory_calibration readback  *******/
static ssize_t readback_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	u8 i = 0;
	u32 report_lux = 0;
	char *ptr = buf;
	
	// 1. collect the als average value for 30 times
	for (i = 0; i < 10; i++) {
		start_als_value_func();
    	report_lux += als_buf[ FILTER_NUM ];
		mdelay(100);
	}
	// 2. get the average
	report_lux =  report_lux / 10;
	printk("[TL5601C][%s] report_lux = %d \n",__func__,report_lux);
	// 3. use the calibration factor to recalculate the als value, and uploade it
	report_lux = ((report_lux) * cali_factor_400lux ) / 1000;
	als_calibration_factor = cali_factor_400lux;   
	
	if (*pos)
		return 0;
	
	ptr += sprintf(ptr, "%d+%d\n", report_lux, cali_factor_400lux);
	printk("[TL5601C][%s] report_lux = %d + cali_factor_400lux = %d\n ",__func__,report_lux, cali_factor_400lux);
	
    *pos += ptr - buf;

	return (ptr -buf);		
}

/******  als_factory_calibration readback  *******/
static struct file_operations proc_readback_file_ops =
{
	.owner = THIS_MODULE,
	.read = readback_read,
};


/******  als_factory_calibration function *******/
static ssize_t calibration_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	u8 i = 0;
	u32 report_lux = 0;
	uint16_t calibration_enable = 0;

    sscanf(buff, "%hu", &calibration_enable);

	if (calibration_enable == LIGHT_BAR_400LUX) {
		// 1. collect the als average value for 30 times
		for (i = 0; i < 10; i++) {
			start_als_value_func();
			report_lux += als_buf[ FILTER_NUM ];
			mdelay(100);
		}
		// 2. get the average
		report_lux /= 10;
		// 3. caculate the calibration factor 
		cali_factor_400lux = (LIGHT_BAR_400LUX * 1000) / report_lux;
		printk("[TL5601C][%s]ALS calibration factor: report_lux = %d, cali_factor_400lux = %d \n",__func__, report_lux, cali_factor_400lux);
		return len;
	} else {
		return -1;
	}
	return len;
}


/******  als_factory_calibration function *******/
static struct file_operations proc_calibration_file_ops =
{
	.owner = THIS_MODULE,
	.write = calibration_write,
};

static ssize_t calibration_factor_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	int als_calibration_factor_tmp = 0;
    sscanf(buff, "%d",&als_calibration_factor_tmp);
	als_calibration_factor = als_calibration_factor_tmp;
	printk("[TL5601C][%s]ALS als_calibration_factor = %d \n",__func__,als_calibration_factor);
    return len; 
}
static struct file_operations proc_calibration_factor_file_ops =
{
	.owner = THIS_MODULE,
	.write = calibration_factor_write,
};

int lightsensor_proc_init(void)
{
	printk("[TL5601C][%s]ALS\n",__func__);
	// 1. create proc directory
	lightsensor_proc_dir = proc_mkdir(LIGHTSENSOR_PROC_FOLDER, NULL);
	if (lightsensor_proc_dir == NULL)
	{
		printk(" %s: lightsensor_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
	// 2. create readback node file
	proc_readback_file = proc_create(LIGHTSENSOR_PROC_READBACK_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
				lightsensor_proc_dir, &proc_readback_file_ops);
	if(proc_readback_file == NULL)
	{
		printk(" %s: proc readback file create failed!\n", __func__);
		remove_proc_entry( LIGHTSENSOR_PROC_READBACK_FILE, lightsensor_proc_dir );
		return -ENOMEM;
	}
	// 3. create calibration node file
	proc_calibration_file = proc_create(LIGHTSENSOR_PROC_CALIBRATION_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
				lightsensor_proc_dir, &proc_calibration_file_ops);
	if(proc_calibration_file == NULL)
	{
		printk(" %s: proc calibration file create failed!\n", __func__);
		remove_proc_entry( LIGHTSENSOR_PROC_CALIBRATION_FILE, lightsensor_proc_dir );
		return -ENOMEM;
	}

	proc_calibration_factor_file = proc_create(LIGHTSENSOR_PROC_CALIBRATION_FACTOR_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
		lightsensor_proc_dir, &proc_calibration_factor_file_ops);
	if(proc_calibration_factor_file == NULL)
	{
		printk(" %s: proc_calibration_factor_file create failed!\n", __func__);
		remove_proc_entry( LIGHTSENSOR_PROC_CALIBRATION_FACTOR_FILE, lightsensor_proc_dir );
		return -ENOMEM;
	}
	return 0;
}

void lightsensor_proc_deinit(void)
{
	printk("[TL5601C][%s]ALS\n",__func__);
	remove_proc_entry( LIGHTSENSOR_PROC_READBACK_FILE, lightsensor_proc_dir );
	remove_proc_entry( LIGHTSENSOR_PROC_CALIBRATION_FILE, lightsensor_proc_dir );
	remove_proc_entry( LIGHTSENSOR_PROC_CALIBRATION_FACTOR_FILE, lightsensor_proc_dir );
}

/* lihaiyan@wind-mobi.com 20170331 end */


