/************************************************************************
** OWER: lihaiyan@wind-mobi.com
** DATE: 20170412
**
** NOTICE ONE: What have you to do ?
**  1. add  tp_gesture_proc_init() in xxx_probe() to init proc node
**  2. add  tp_gesture_proc_remove() in xxx_remove() to remove proc node
**
** NOTICE TWO: How to use it ?
**  1. /proc/wind_tp/wind_gesture 
************************************************************************/

#include "tp_gesture.h"

// the proc folder's name
#define TPD_PROC_FOLDER                                 "wind_tp"

// the node file name below proc folder
#define TPD_PROC_GESTURE                                "wind_gesture"

static struct proc_dir_entry *tpd_proc_dir              = NULL;
static struct proc_dir_entry *tpd_gesture_mode          = NULL;

unsigned int g_GestureWakeupMode_wind                   = 0x00000000;


void calculate_gesture(unsigned int GestureWakeModetmp, unsigned int gesture_bit_mask)
{
    if ((GestureWakeModetmp & gesture_bit_mask) == gesture_bit_mask)
    {
        g_GestureWakeupMode_wind = g_GestureWakeupMode_wind | gesture_bit_mask;
    }
    else
    {
        g_GestureWakeupMode_wind = g_GestureWakeupMode_wind & (~gesture_bit_mask);
    }
}


ssize_t write_gesture(struct file *file, const char *buf, size_t len, loff_t *pos)
{
    unsigned int WakeupModeTmp = 0;
    
    if( NULL != buf )
    {
        sscanf(buf, "%x", &WakeupModeTmp);
        
        printk("[TPD][GESTURE] WakeupModeTmp = 0x%x \n",WakeupModeTmp);
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_DOUBLE_CLICK_FLAG   );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_UP_DIRECT_FLAG      );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_DOWN_DIRECT_FLAG    );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_LEFT_DIRECT_FLAG    );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_RIGHT_DIRECT_FLAG   );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_M_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_W_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_C_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_E_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_V_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_O_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_S_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_Z_FLAG              );
        calculate_gesture( WakeupModeTmp, GESTURE_MODE_L_FLAG              );
        printk("[TPD][GESTURE] g_GestureWakeupMode_wind = 0x%x  , %d \n", g_GestureWakeupMode_wind, g_GestureWakeupMode_wind);
    }
    return len;
}

ssize_t read_gesture(struct file *file, char *buf, size_t len, loff_t *pos)
{
    char *ptr = buf;

    if (*pos)
        return 0;
    printk("[TPD][GESTURE] g_GestureWakeupMode_wind = 0x%x, %d \n", g_GestureWakeupMode_wind, g_GestureWakeupMode_wind);
    ptr += sprintf(ptr, "%x\n", g_GestureWakeupMode_wind);
    *pos += ptr - buf;

    return (ptr -buf);
}

struct file_operations proc_gesture_ops =
{
    .owner = THIS_MODULE,
    .write = write_gesture,
    .read  = read_gesture,
};

int tp_gesture_proc_init(void)
{
    tpd_proc_dir = proc_mkdir(TPD_PROC_FOLDER, NULL);
    if (tpd_proc_dir == NULL)
    {
        printk(" %s: tpd_proc_dir file create failed!\n", __func__);
        return -ENOMEM;
    }

    tpd_gesture_mode = proc_create(TPD_PROC_GESTURE, (S_IWUSR|S_IRUGO|S_IWUGO), 
            tpd_proc_dir, &proc_gesture_ops);
    if(tpd_gesture_mode == NULL)
    {
        printk(" %s: tpd_gesture_mode create failed!\n", __func__);
        remove_proc_entry( TPD_PROC_GESTURE, tpd_gesture_mode );
        return -ENOMEM;
    }

    return 0 ;
}

void tp_gesture_proc_remove(void)
{
    remove_proc_entry( TPD_PROC_GESTURE, tpd_proc_dir );
}

/************************************************************************
** OWER: lihaiyan@wind-mobi.com
** DATE: 20170412
************************************************************************/
