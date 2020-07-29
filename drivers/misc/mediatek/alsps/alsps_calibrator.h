/* lihaiyan@wind-mobi.com 20170331 begin */

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include "../../../../arch/powerpc/boot/stdlib.h"



#define ALSPS_DEBUG_ENABLE 		1

#if (ALSPS_DEBUG_ENABLE)
#define ALSPS_DEBUG(fmt, args...) printk(KERN_ERR "[TL5601C][%s][%d]"fmt"\n", __func__, __LINE__, ##args)
#else
#define ALSPS_DEBUG(fmt, args...) no_printk(KERN_ERR "[TL5601C][%s][%d]"fmt"\n", __func__, __LINE__, ##args)
#endif


typedef void (*alsps_start_als_value_func)(void);
typedef void (*alsps_write_ps_value_func)(int ps_low_threshold, int ps_high_threshold);

extern  alsps_start_als_value_func   start_als_value_func;
extern  alsps_write_ps_value_func    write_ps_value_func;

extern int ps_threshold_5cm; 
//wangjun@wind-mobi.com 20170728 begin
extern int ps_crosstalk;
extern int ps_threshold_high;
extern int ps_threshold_low;
//wangjun@wind-mobi.com 20170728 end
extern int als_calibration_factor; //wangjun@wind-mobi.com 20170601
extern int get_als_calibration_factor(void);
extern u16 filter_als_value(u16 als);
extern int lightsensor_proc_init(void);
extern void lightsensor_proc_deinit(void);


extern int psensor_proc_init(void);
extern void psensor_proc_remove(void);

/* lihaiyan@wind-mobi.com 20170331 end */



