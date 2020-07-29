
/** qiangang@wind-mobi.com 20171129 begin **/

/************************************************************************
** NOTICE ONE: What have you to do ?
**	1. add  sim_proc_init() in xxx_prob() to init proc node
**	2. add  sim_proc_remove() in xxx_remove() to remove proc node
**
** NOTICE TWO: How to use it ?
**	1. /proc/sim/sim1 
**  2. /proc/sim/sim2 
************************************************************************/

#include "mt-plat/mtgpio.h"
#include <linux/types.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
#include <mach/gpio_const.h>
#include <linux/proc_fs.h>
//#include "cust_gpio_usage.h"

#define SIM_PROC_FOLDER 							"sim"
#define SIM1_PROC_DETECT_PIN_FILE 					"sim1"
#define SIM2_PROC_DETECT_PIN_FILE					"sim2"


static struct proc_dir_entry *sim_proc_dir 			= NULL;
static struct proc_dir_entry *sim1_proc_dir_file 	= NULL;
static struct proc_dir_entry *sim2_proc_dir_file  	= NULL;



#define GPIO_SIM1_HOT_PLUG         (GPIO46 | 0x80000000)
#define GPIO_SIM2_HOT_PLUG         (GPIO45 | 0x80000000)

ssize_t read_sim1_pinstate(struct file *file, char *buf, size_t len, loff_t *pos)
{
    char *ptr = buf;
	int sim1 = 0;

    if (*pos)
    {    
        return 0;
    }

    sim1 = mt_get_gpio_in(GPIO_SIM1_HOT_PLUG);

    printk("qiangang read sim1=%d\n",sim1);
    
    ptr += sprintf(ptr, "%d\n", sim1);
    *pos += ptr - buf;

   return (ptr - buf);
}


/******  sim1 pinstate readback  *******/
static struct file_operations sim1_proc_file_ops =
{
	.owner = THIS_MODULE,
	.read = read_sim1_pinstate,
};


ssize_t read_sim2_pinstate(struct file *file, char *buf, size_t len, loff_t *pos)
{
    char *ptr = buf;
	int sim2 = 0;

   if (*pos)
       return 0;
	sim2 = mt_get_gpio_in(GPIO_SIM2_HOT_PLUG);
	printk("qiangang read sim2=%d\n",sim2);
    ptr += sprintf(ptr, "%d\n", sim2);
    *pos += ptr - buf;

   return (ptr - buf);
}


/******  sim2 pinstate readback  *******/
static struct file_operations sim2_proc_file_ops =
{
	.owner = THIS_MODULE,
	.read = read_sim2_pinstate,
};


int sim_proc_init(void)
{
	//printk("[SIM][%s]qiangang\n",__func__);
	// 1. create proc directory
	sim_proc_dir = proc_mkdir(SIM_PROC_FOLDER, NULL);
	if (sim_proc_dir == NULL)
	{
		printk(" %s: sim_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
	// 2. create SIM1 readback node file
	sim1_proc_dir_file = proc_create(SIM1_PROC_DETECT_PIN_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
				sim_proc_dir, &sim1_proc_file_ops);
	if(sim1_proc_dir_file == NULL)
	{
		printk(" %s: qq proc readback file create failed!\n", __func__);
		remove_proc_entry( SIM1_PROC_DETECT_PIN_FILE, sim_proc_dir );
		return -ENOMEM;
	}
	// 3. create SIM2 calibration node file
	sim2_proc_dir_file = proc_create(SIM2_PROC_DETECT_PIN_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
				sim_proc_dir, &sim2_proc_file_ops);
	if(sim1_proc_dir_file == NULL)
	{
		printk(" %s: qq proc readback file create failed!\n", __func__);
		remove_proc_entry( SIM2_PROC_DETECT_PIN_FILE, sim_proc_dir);
		return -ENOMEM;
	}
	return 0;
}

void sim_proc_deinit(void)
{
	printk("[SIM[%s]ALS\n",__func__);
	remove_proc_entry( SIM1_PROC_DETECT_PIN_FILE, sim_proc_dir );
	remove_proc_entry( SIM2_PROC_DETECT_PIN_FILE, sim_proc_dir );
}

/* qiangang@wind-mobi.com 20171129 end */


