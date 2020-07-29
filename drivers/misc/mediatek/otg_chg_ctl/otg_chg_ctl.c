//yutao@wind-mobi.com add usb swtich file point begin 20160625
//#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/wakelock.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
//#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "../power/mt6755/bq25890.h"
//#include <linux/irq.h>
#include <linux/kthread.h>



int           otg_chg_flag=0;
#define   OTG_CHG_SWITCH_CLASS_NAME   "otg_chg"
#define   OTG_CHG_DEVICE_NODE_NAME   "otg_chg"
#define   OTG_CHG_DEVICE_CLASS_NAME  "otg_chg" 
#define   OTG_CHG_DEVICE_FILE_NAME   "otg_chg"
static struct class *otg_chg_class = NULL;
static struct device* temp = NULL;  
//static void otg_chg_handle(void);
//static DECLARE_TASKLET(otg_chg_tasklet,otg_chg_handle,0);
static int otg_chg_major=0;
static int otg_chg_minor=0;
static DEFINE_MUTEX(otg_chg_mutex);
extern void msleep(unsigned int msecs);
struct task_struct *otg_chg_thread=NULL;
struct otg_chg_device
{
   
   int otg_chg_val;
   struct semaphore sem;
   struct cdev dev;

};

struct otg_chg_device* otg_chg_dev =NULL;


static struct file_operations otg_chg_fops={
	.owner=THIS_MODULE,
	
};


static int otg_chg_thread_func(void *data)
{
	if(otg_chg_flag==1)
	{
        mutex_lock(&otg_chg_mutex);
		//printk("otg_chg_thread_func otg_chg_flag = 1!!\n");
		bq25890_otg_en(1);
		msleep(50); 
		bq25890_chg_en(1);
		mutex_unlock(&otg_chg_mutex);
		kthread_stop(otg_chg_thread);
	}
	else 
	{
		mutex_lock(&otg_chg_mutex);
		//printk("otg_chg_thread_func otg_chg_flag = 0!!\n");
        bq25890_otg_en(0);                
		msleep(50);               
		bq25890_chg_en(0);
		mutex_unlock(&otg_chg_mutex);
		kthread_stop(otg_chg_thread);
	
	}

     return 0;
}
static ssize_t _otg_chg_show(struct otg_chg_device*dev,char*buf)
{
    int val=0;
	if(down_interruptible(&(dev->sem)))
		{
			return -ERESTARTSYS;
		}
	val=dev->otg_chg_val;


	up(&(dev->sem));

	return snprintf(buf,PAGE_SIZE,"%d\n",val);  //from kernel space to user space

}
#if 0
void otg_chg_handle(void)
{
    
	if(otg_chg_flag==1)
	{
        mutex_lock(&otg_chg_mutex);
		bq25890_otg_en(1);
		//msleep(50); 
		//bq25890_chg_en(1);
		mutex_unlock(&otg_chg_mutex);
	}
	else 
	{
		mutex_lock(&otg_chg_mutex);
        bq25890_otg_en(0);                
		// msleep(50);               
		// bq25890_chg_en(0);
		mutex_unlock(&otg_chg_mutex);
	
	}

}
#endif
static ssize_t _otg_chg_store(struct otg_chg_device *dev,const char * buf,size_t count)
{
	      int val=0;
	      val=simple_strtoul(buf, NULL, 10);
		if(down_interruptible(&(dev->sem)))
		 {
			return -ERESTARTSYS;
		 }
		
		   
	      dev->otg_chg_val=val;
		  if(val==1)
		  {
		     otg_chg_flag=1;
		     //tasklet_schedule(&otg_chg_tasklet);
		     	otg_chg_thread = kthread_create(otg_chg_thread_func, NULL, "otg_chg_thread_func");
			 if(!IS_ERR(otg_chg_thread))
			 	{	//printk("kthread start success!!\n");	
			 wake_up_process(otg_chg_thread);	
			      }
		     
			 }
		  else 
		  {
			 otg_chg_flag=0;
			otg_chg_thread = kthread_create(otg_chg_thread_func, NULL, "otg_chg_thread_func");
			 if(!IS_ERR(otg_chg_thread))
			 	{	//printk("kthread start success!!\n");	
			 wake_up_process(otg_chg_thread);	
			      }
		
		  }
		 
		
	      up(&(dev->sem));
		
              return count;

}

static ssize_t otg_chg_show(struct device *dev, struct device_attribute *attr,char *buf)
{


	struct otg_chg_device *gdev=(struct otg_chg_device*)dev_get_drvdata(dev);
	return _otg_chg_show(gdev,buf);
}

static ssize_t otg_chg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	struct otg_chg_device *gdev=(struct otg_chg_device*)dev_get_drvdata(dev);
	
	 return _otg_chg_store(gdev,buf,size);
	
}

 static int _otg_chg_setup_dev(struct otg_chg_device* dev)
 	{
 	
 	
 		int err;
		dev_t devno=MKDEV(otg_chg_major,otg_chg_minor);
		memset(dev,0,sizeof(struct otg_chg_device));
		cdev_init(&(dev->dev),&otg_chg_fops);
		dev->dev.owner=THIS_MODULE;
		dev->dev.ops=&otg_chg_fops;
		err=cdev_add(&(dev->dev),devno,1);
		if(err)
			{
				return err;

		       }
		dev->otg_chg_val=0;
		sema_init(&(dev->sem),1);
		return 0;
 	 }
 static DEVICE_ATTR(otg_chg_val, 0664 , otg_chg_show, otg_chg_store);
static int __init otg_chg_driver_init(void)
{
    int err = -1;  
    dev_t dev = 0;  
   

   //printk(KERN_ALERT"Initializing OTG_CHG device.\n");
   
   err = alloc_chrdev_region(&dev, 0, 1, OTG_CHG_DEVICE_NODE_NAME); 
   
       if(err < 0) {  
       printk(KERN_ALERT"Failed to alloc char dev region.\n");  
        goto fail;  
		}
		
	otg_chg_major=MAJOR(dev);
	otg_chg_minor=MINOR(dev);
	
	otg_chg_dev=kmalloc(sizeof(struct otg_chg_device),GFP_KERNEL);
	    if(!otg_chg_dev) {  
       err = -ENOMEM;  
      // printk(KERN_ALERT"Failed to alloc usb_switch_dev.\n");  
        goto unregister;  
    }

	err=_otg_chg_setup_dev(otg_chg_dev);
	   if(err) {  
      // printk(KERN_ALERT"Failed to setup dev: %d.\n", err);  
        goto cleanup;  
		} 
		
	otg_chg_class = class_create(THIS_MODULE, OTG_CHG_DEVICE_CLASS_NAME);  
       if(IS_ERR(otg_chg_class)) {  
        err = PTR_ERR(otg_chg_class);  
      // printk(KERN_ALERT"Failed to create OTG_CHG class.\n");  
       goto destroy_cdev;  
    } 

	   
	    temp = device_create(otg_chg_class, NULL, dev, "%s", OTG_CHG_DEVICE_FILE_NAME);  
		
  if(IS_ERR(temp)) {  
       err = PTR_ERR(temp);  
       //printk(KERN_ALERT"Failed to create OTG_CHG device.");  
        goto destroy_class;  
   }

	    err = device_create_file(temp, &dev_attr_otg_chg_val);  
    if(err < 0) {  
      // printk(KERN_ALERT"Failed to create attribute val.");                  
       goto destroy_device;  
    }  
  
    dev_set_drvdata(temp, otg_chg_dev);  
	
  //  printk(KERN_ALERT"Initializing OTG_CHG device  successfully.\n");
    return 0;
	
destroy_device:
     device_destroy(otg_chg_class,dev);	
destroy_class:
	class_destroy(otg_chg_class);
	

destroy_cdev:
	cdev_del(&(otg_chg_dev->dev));

cleanup:
	
	kfree(otg_chg_dev);
unregister:
     unregister_chrdev_region(MKDEV(otg_chg_major, otg_chg_minor), 1);    
		
fail:

     return err;


   
   
   

}

static void __exit otg_chg_driver_exit(void)
{
	dev_t devno = MKDEV(otg_chg_major, otg_chg_minor);  
	//printk(KERN_ALERT"Destroy OTG_CHG device.\n");
	
	  if(otg_chg_class) {  
        device_destroy(otg_chg_class, MKDEV(otg_chg_major, otg_chg_minor));  
        class_destroy(otg_chg_class);  
    } 

    if(otg_chg_dev) {  
        cdev_del(&(otg_chg_dev->dev));  
        kfree(otg_chg_dev);  
   }
unregister_chrdev_region(devno, 1); 
}




module_init(otg_chg_driver_init);
module_exit(otg_chg_driver_exit);
//yutao@wind-mobi.com add usb swtich file point end 20160625
