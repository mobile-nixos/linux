
/** lihaiyan@wind-mobi.com 20170327 begin **/

/************************************************************************
** NOTICE ONE: What have you to do ?
**	1. add  psensor_proc_init() in xxx_prob() to init proc node
**	2. add  psensor_proc_remove() in xxx_remove() to remove proc node
**
** NOTICE TWO: How to use it ?
**	1. /proc/psensor/Data1 
************************************************************************/

#include "alsps_calibrator.h"
#include "tl5601c.h"


// the proc folder's name
#define PSENSOR_PROC_FOLDER "psensor"
// the node file name below proc folder
#define PSENSOR_PROC_CROSSTALK 								"Data1"
#define PSENSOR_PROC_THRESHOLD_HIGH 						"Data2"
#define PSENSOR_PROC_THRESHOLD_LOW 							"Data3"
#define PSENSOR_PROC_THRESHOLD_READ 						"data_read_5cm"

static struct proc_dir_entry *psensor_proc_dir 				= NULL;
static struct proc_dir_entry *proc_ps_crosstalk 			= NULL;
static struct proc_dir_entry *proc_ps_threshold_high 		= NULL;
static struct proc_dir_entry *proc_ps_threshold_low 		= NULL;
static struct proc_dir_entry *proc_ps_threshold_read 		= NULL;

int ps_crosstalk=0;
int ps_threshold_high=0;
int ps_threshold_low=0;
int ps_threshold_5cm = 0; 

//extern void write_ps_value(int ps_low_threshold, int ps_high_threshold);

alsps_write_ps_value_func  write_ps_value_func = NULL;


ssize_t write_ps_threshold_low(struct file *file, const char *buf, size_t len, loff_t *pos)
{
	int ps_threshold_low_tmp = 0;
	int ps_threshold_high_tmp = 0;
    sscanf(buf, "%d", &ps_threshold_low_tmp);
	ps_threshold_low = ps_threshold_low_tmp;
	
    if( (ps_crosstalk < ps_threshold_low) && (ps_threshold_low < ps_threshold_high))
    {
	    ps_threshold_low_tmp = ps_threshold_low - ps_crosstalk; 
        ps_threshold_high_tmp = ps_threshold_high - ps_crosstalk;
		write_ps_value_func(ps_threshold_low_tmp, ps_threshold_high_tmp);
	}
	return len;
}

ssize_t read_ps_threshold_low(struct file *file, char *buf, size_t len, loff_t *pos)
{
	char *ptr = buf;
	int ps_threshold_low_tmp = 0;
	ps_threshold_low_tmp  = ps_threshold_low;

	if (*pos)
		return 0;
	
    ptr += sprintf(ptr, "%d\n", ps_threshold_low_tmp);
    *pos += ptr - buf;

	return (ptr -buf);	
}

struct file_operations proc_ps_threshold_low_ops =
{
	.owner = THIS_MODULE,
	.write = write_ps_threshold_low,
	.read = read_ps_threshold_low,
};

/** ps_threshold_high node **/

ssize_t write_ps_threshold_high(struct file *file, const char *buf, size_t len, loff_t *pos)
{	 
	int ps_threshold_high_tmp = 0;
    sscanf(buf, "%d", &ps_threshold_high_tmp);
	ps_threshold_high = ps_threshold_high_tmp;
	
	return len;		
}

ssize_t read_ps_threshold_high(struct file *file, char *buf, size_t len, loff_t *pos)
{
	int ps_threshold_high_tmp = 0;
	char *ptr = buf;
	
	ps_threshold_high_tmp = ps_threshold_high;
	
	if (*pos)
		return 0;
	
    ptr += sprintf(ptr, "%d\n", ps_threshold_high_tmp);
    *pos += ptr - buf;

	return (ptr -buf);
}
		
struct file_operations proc_ps_threshold_high_ops =
{
	.owner = THIS_MODULE,
	.write = write_ps_threshold_high,
	.read = read_ps_threshold_high,
};


/** ps_crosstalk node **/

ssize_t write_ps_crosstalk(struct file *file, const char *buf, size_t len, loff_t *pos)
{
	int ps_crosstalk_tmp = 0;
    sscanf(buf, "%d", &ps_crosstalk_tmp);
	ps_crosstalk = ps_crosstalk_tmp;
	return len;
}

ssize_t read_ps_crosstalk(struct file *file,  char *buf, size_t len, loff_t *pos)
{
	char *ptr = buf;
	int ps_crosstalk_tmp = 0;
	ps_crosstalk_tmp = ps_crosstalk;

	if (*pos)
		return 0;
	
    ptr += sprintf(ptr, "%d\n", ps_crosstalk_tmp);
    *pos += ptr - buf;

	return (ptr -buf);		
}

struct file_operations proc_ps_crosstalk_ops =
{
	.owner = THIS_MODULE,
	.write = write_ps_crosstalk,
	.read = read_ps_crosstalk,
};


ssize_t read_ps_threshold_5cm(struct file *file,  char *buf, size_t len, loff_t *pos)
{
	char *ptr = buf;
	int ps_threshold_5cm_tmp = 0;
	ps_threshold_5cm_tmp = ps_threshold_5cm;

	if (*pos)
		return 0;
	
    ptr += sprintf(ptr, "%d\n", ps_threshold_5cm_tmp);
    *pos += ptr - buf;

	return (ptr -buf);		
}

struct file_operations proc_ps_threshold_read_ops =
{
	.owner = THIS_MODULE,
	.read = read_ps_threshold_5cm,
};



int psensor_proc_init(void)
{
	psensor_proc_dir = proc_mkdir(PSENSOR_PROC_FOLDER, NULL);
	if (psensor_proc_dir == NULL)
	{
		printk(" %s: psensor_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
	
	proc_ps_crosstalk = proc_create(PSENSOR_PROC_CROSSTALK, (S_IWUSR|S_IRUGO|S_IWUGO), 
				psensor_proc_dir, &proc_ps_crosstalk_ops);
	if(proc_ps_crosstalk == NULL)
	{
		printk(" %s: proc_ps_crosstalk create failed!\n", __func__);
		remove_proc_entry( PSENSOR_PROC_CROSSTALK, psensor_proc_dir );
		return -ENOMEM;
	}

	proc_ps_threshold_high = proc_create(PSENSOR_PROC_THRESHOLD_HIGH, (S_IWUSR|S_IRUGO|S_IWUGO), 
				psensor_proc_dir, &proc_ps_threshold_high_ops);
	if(proc_ps_threshold_high == NULL)
	{
		printk(" %s: proc_ps_threshold_high create failed!\n", __func__);
		remove_proc_entry( PSENSOR_PROC_THRESHOLD_HIGH, psensor_proc_dir );
		return -ENOMEM;
	}
	
	proc_ps_threshold_low = proc_create(PSENSOR_PROC_THRESHOLD_LOW, (S_IWUSR|S_IRUGO|S_IWUGO), 
				psensor_proc_dir, &proc_ps_threshold_low_ops);
	if(proc_ps_threshold_low == NULL)
	{
		printk(" %s: proc_ps_threshold_low create failed!\n", __func__);
		remove_proc_entry( PSENSOR_PROC_THRESHOLD_LOW, psensor_proc_dir );
		return -ENOMEM;
	}
	
	proc_ps_threshold_read = proc_create(PSENSOR_PROC_THRESHOLD_READ, (S_IWUSR|S_IRUGO|S_IWUGO), 
				psensor_proc_dir, &proc_ps_threshold_read_ops);
	if(proc_ps_threshold_low == NULL)
	{
		printk(" %s: proc_ps_threshold_read create failed!\n", __func__);
		remove_proc_entry( PSENSOR_PROC_THRESHOLD_READ, psensor_proc_dir );
		return -ENOMEM;
	}
	return 0 ;
}
void psensor_proc_remove(void)
{
    //remove_proc_entry( PSENSOR_PROC_FOLDER, psensor_proc_dir );
	remove_proc_entry( PSENSOR_PROC_CROSSTALK, psensor_proc_dir );
	remove_proc_entry( PSENSOR_PROC_THRESHOLD_HIGH, psensor_proc_dir );
	remove_proc_entry( PSENSOR_PROC_THRESHOLD_LOW, psensor_proc_dir );
	remove_proc_entry( PSENSOR_PROC_THRESHOLD_READ, psensor_proc_dir );
}


/** lihaiyan@wind-mobi.com 20170327 end **/
