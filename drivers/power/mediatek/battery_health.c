#include <linux/mm.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/printk.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <linux/wakelock.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <mt-plat/mtk_rtc.h>
#include <linux/jiffies.h>
#include	<linux/slab.h>

#define FILE_OP_READ   0
#define FILE_OP_WRITE	1

//ASUS_BS battery health upgrade +++
#define BAT_HEALTH_NUMBER_MAX 21
struct BAT_HEALTH_DATA{
	int magic;
	int bat_current;
	unsigned long long bat_current_avg;
	unsigned long long accumulate_time; //second
	unsigned long long accumulate_current; //uA
	int bat_health;
	unsigned long start_time;
	unsigned long end_time;
	
};
struct BAT_HEALTH_DATA_BACKUP{
    char date[20];
    int health;
};
 
//ASUS_BS battery health upgrade +++
#define	BATTERY_HEALTH_UPGRADE_TIME 1 //ASUS_BS battery health upgrade
#define	BATTERY_METADATA_UPGRADE_TIME 60 //ASUS_BS battery health upgrade
#define BAT_HEALTH_DATA_OFFSET  0x0
#define BAT_HEALTH_DATA_MAGIC  0x86
#define BAT_HEALTH_DATA_BACKUP_MAGIC 0x87
#define ZE620KL_DESIGNED_CAPACITY 3924 //mAh //Design Capcaity *0.95 = 3924
#define BAT_HEALTH_DATA_FILE_NAME   "/batinfo/bat_health"
#define BAT_HEALTH_DATA_SD_FILE_NAME   "/batinfo/.bh"
#define BAT_HEALTH_DATA_SDCARD_FILE_NAME   "/sdcard/.bh"
#define BAT_HEALTH_START_LEVEL 70
#define BAT_HEALTH_END_LEVEL 100
static bool g_bathealth_initialized = false;
static bool g_bathealth_trigger = false;
static bool g_last_bathealth_trigger = false;
static bool g_health_debug_enable = true;
static bool g_health_upgrade_enable = true;
static int g_health_upgrade_index = 0;
static int g_health_upgrade_start_level = BAT_HEALTH_START_LEVEL;
static int g_health_upgrade_end_level = BAT_HEALTH_END_LEVEL;
static int g_health_upgrade_upgrade_time = BATTERY_HEALTH_UPGRADE_TIME;
static int g_bat_health_avg;

extern ssize_t battery_GetBatteryPercent(void);
extern ssize_t battery_GetBatteryChargecurrent(void);
extern int get_bat_charging_current_level(void);
extern kal_bool bat_is_charging(void);
extern int read_tbat_value(void);

static struct BAT_HEALTH_DATA g_bat_health_data = {
    .magic = BAT_HEALTH_DATA_MAGIC,
    .bat_current = 0,
    .bat_current_avg = 0,
    .accumulate_time = 0,
    .accumulate_current = 0,
    .bat_health = 0
};
static struct BAT_HEALTH_DATA_BACKUP g_bat_health_data_backup[BAT_HEALTH_NUMBER_MAX] = {
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0},
	{"", 0}
};
struct delayed_work battery_health_work;
 
//ASUS_BS battery health upgrade +++
static void battery_health_data_reset(void)
{
	g_bat_health_data.bat_current = 0;
	g_bat_health_data.bat_current_avg = 0;
	g_bat_health_data.accumulate_time = 0;
	g_bat_health_data.accumulate_current = 0;
	g_bat_health_data.start_time = 0;
	g_bat_health_data.end_time = 0;
	g_bathealth_trigger = false;
	g_last_bathealth_trigger = false;
}

static unsigned long jiffies_to_sec(const unsigned long j)
{
	return jiffies_to_msecs(j) / 1000;
}

static int file_op(const char *filename, loff_t offset, char *buf, int length, int operation)
{
	int filep;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(FILE_OP_READ == operation)
		filep= sys_open(filename, O_RDONLY | O_CREAT | O_SYNC, 0666);
	else if(FILE_OP_WRITE == operation)
		filep= sys_open(filename, O_RDWR | O_CREAT | O_SYNC, 0666);
	else {
		pr_err("Unknown partition op err!\n");
		return -1;
	}
	if(filep < 0) {
		pr_err("open %s err! error code:%d\n", filename, filep);
		return -1;
	}
	else
		pr_info("open %s success!\n", filename);

	sys_lseek(filep, offset, SEEK_SET);
	if(FILE_OP_READ == operation)
		sys_read(filep, buf, length);
	else if(FILE_OP_WRITE == operation) {
		sys_write(filep, buf, length);
		sys_fsync(filep);
	}
	set_fs(old_fs);
	sys_close(filep);
	return length;
}

static int batt_health_csc_backup(void){
	int rc=0, i=0;
	struct BAT_HEALTH_DATA_BACKUP buf[BAT_HEALTH_NUMBER_MAX];
	char buf2[BAT_HEALTH_NUMBER_MAX][30];

	memset(&buf,0,sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX);
	memset(&buf2,0,sizeof(char)*BAT_HEALTH_NUMBER_MAX*30);
	
	rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
		(char*)&buf, sizeof(struct BAT_HEALTH_DATA)*BAT_HEALTH_NUMBER_MAX, FILE_OP_READ);
	if(rc < 0) {
		pr_err("Read bat health file failed!\n");
		return rc;
	}

	for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
		if(buf[i].health!=0){
			sprintf(&buf2[i-1][0], "%s [%d]\n", buf[i].date, buf[i].health);
		}
	}

	rc = file_op(BAT_HEALTH_DATA_SD_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
	(char *)&buf2, sizeof(char)*BAT_HEALTH_NUMBER_MAX*30, FILE_OP_WRITE);
	if(rc < 0 ) 
		pr_err("Write bat health file failed!\n");

	rc = file_op(BAT_HEALTH_DATA_SDCARD_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
	(char *)&buf2, sizeof(char)*BAT_HEALTH_NUMBER_MAX*30, FILE_OP_WRITE);
	if(rc < 0 ) 
		pr_err("Write bat health to /sdcard/.bh file failed!\n");


	battery_log(BAT_LOG_CRTI, "%s Done! \n",__FUNCTION__);
	return rc;
}

static int resotre_bat_health(void)
{
	int i=0, rc = 0;

	memset(&g_bat_health_data_backup,0,sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX);
	
	/* Read cycle count data from emmc */
	rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
		(char*)&g_bat_health_data_backup, sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX, FILE_OP_READ);
	if(rc < 0) {
		pr_err("Read bat health file failed!\n");
		return -1;
	}

	battery_log(BAT_LOG_CRTI, "%s: index(%d)\n",__FUNCTION__, g_bat_health_data_backup[0].health);
	for(i=1; i<BAT_HEALTH_NUMBER_MAX;i++){
		battery_log(BAT_LOG_CRTI, "%s %d\n",g_bat_health_data_backup[i].date, g_bat_health_data_backup[i].health);
	}

	g_health_upgrade_index = g_bat_health_data_backup[0].health;
	g_bathealth_initialized = true;

	batt_health_csc_backup();
	return 0;
}

static int backup_bat_health(void)
{
	int bat_health, rc;
	struct timespec ts;
	struct rtc_time tm; 
	int health_t;
	int count=0, i=0;
	unsigned long long bat_health_accumulate=0;

	getnstimeofday(&ts); 
	rtc_time_to_tm(ts.tv_sec,&tm); 

	bat_health = g_bat_health_data.bat_health;

	if(g_health_upgrade_index == BAT_HEALTH_NUMBER_MAX-1){
		for (i=1;i<BAT_HEALTH_NUMBER_MAX-1;i++) {
			g_bat_health_data_backup[i] = g_bat_health_data_backup[i+1];
		}
	}else{
		g_health_upgrade_index++;
	}

	battery_log(BAT_LOG_CRTI, "%s: g_health_upgrade_index(%d)\n",__FUNCTION__, g_health_upgrade_index);
	sprintf(g_bat_health_data_backup[g_health_upgrade_index].date, "%d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900,tm.tm_mon+1, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
	g_bat_health_data_backup[g_health_upgrade_index].health = bat_health;
	g_bat_health_data_backup[0].health = g_health_upgrade_index;

	battery_log(BAT_LOG_CRTI, "%s ===== Health history ====\n",__FUNCTION__);
	for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
		if(g_bat_health_data_backup[i].health!=0){
			count++;
			bat_health_accumulate += g_bat_health_data_backup[i].health;
			battery_log(BAT_LOG_CRTI, "%s %02d:%d\n",__FUNCTION__,i,g_bat_health_data_backup[i].health);
		}
	}
	battery_log(BAT_LOG_CRTI, "%s ========================\n",__FUNCTION__);

	if(count==0){
		pr_err("%s battery health value is empty\n",__FUNCTION__);
		return -1;
	}
	health_t = bat_health_accumulate*10/count;
	g_bat_health_avg = (int)(health_t + 5)/10;
	g_bat_health_data_backup[g_health_upgrade_index].health = g_bat_health_avg;

	rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
		(char *)&g_bat_health_data_backup, sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX, FILE_OP_WRITE);
	if(rc<0){
		pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_HEALTH_DATA_FILE_NAME);
	}

	return rc;
}

static void update_battery_health(void)
{
	int bat_capacity, bat_current, delta_p;
	unsigned long T;
	int health_t;

	battery_log(BAT_LOG_CRTI, "======update_battery_health\n");

	if(g_health_upgrade_enable != true){
		return;
	}
	
	if(g_bathealth_initialized != true){
		resotre_bat_health();
		return;
	}
/*
	if(!chip->online_status){
		if(g_last_bathealth_trigger == true){
			battery_health_data_reset();
		}
		return;
	}
*/	
	if (bat_is_charging() == KAL_FALSE) {
		if(g_last_bathealth_trigger == true){
			battery_health_data_reset();
		}
		battery_log(BAT_LOG_CRTI, "======bat no charging\n");
		return;
	}
	else
		battery_log(BAT_LOG_CRTI, "======bat is charging\n");

	//fg_get_prop_capacity(chip, &bat_capacity);
	bat_capacity = battery_GetBatteryPercent();
	battery_log(BAT_LOG_CRTI, "======bat_capacity=%d\n",bat_capacity);
	battery_log(BAT_LOG_CRTI, "======bat_current=%ld\n",battery_GetBatteryChargecurrent());
	battery_log(BAT_LOG_CRTI, "======battery_meter_get_battery_percentage=%d\n",battery_meter_get_battery_percentage());
	battery_log(BAT_LOG_CRTI, "======get_bat_charging_current_level=%d\n",get_bat_charging_current_level());

	if(bat_capacity == g_health_upgrade_start_level && g_bat_health_data.start_time == 0){
		g_bathealth_trigger = true;
		//g_bat_health_data.start_time = asus_qpnp_rtc_read_time();
		g_bat_health_data.start_time = get_jiffies_64();
		battery_log(BAT_LOG_CRTI, "======start_time=%lu\n",g_bat_health_data.start_time);
	}
	if(bat_capacity > g_health_upgrade_end_level){
		g_bathealth_trigger = false;
	}
	if(g_last_bathealth_trigger == false && g_bathealth_trigger == false){
		return;
	}
	
	if( g_bathealth_trigger ){		
		//fg_get_battery_current(chip, &bat_current);
		bat_current = battery_GetBatteryChargecurrent() * 1000;
		battery_log(BAT_LOG_CRTI, "======bat_current=%d\n",bat_current);

		g_bat_health_data.accumulate_time += g_health_upgrade_upgrade_time;
		g_bat_health_data.bat_current = bat_current;
		g_bat_health_data.accumulate_current += g_bat_health_data.bat_current;
		g_bat_health_data.bat_current_avg = g_bat_health_data.accumulate_current/g_bat_health_data.accumulate_time;
			
		if(g_health_debug_enable)
			pr_info("%s accumulate_time(%llu), accumulate_current(%llu)mA, bat_current(%d)mA, bat_current_avg(%llu)mA, bat_capacity(%d)\n",__FUNCTION__, g_bat_health_data.accumulate_time, g_bat_health_data.accumulate_current/1000, g_bat_health_data.bat_current/1000, g_bat_health_data.bat_current_avg/1000, bat_capacity);

		if(bat_capacity >= g_health_upgrade_end_level){			
			//g_bat_health_data.end_time = asus_qpnp_rtc_read_time();
			g_bat_health_data.end_time = get_jiffies_64();
			battery_log(BAT_LOG_CRTI, "======start_time=%lu, end_time=%lu\n",g_bat_health_data.start_time, g_bat_health_data.end_time);
			
			delta_p = g_health_upgrade_end_level - g_health_upgrade_start_level;
			T = jiffies_to_sec(g_bat_health_data.end_time - g_bat_health_data.start_time);
			health_t = (g_bat_health_data.bat_current_avg*T)*10/(unsigned long long)(ZE620KL_DESIGNED_CAPACITY*delta_p)/(unsigned long long)360;
			g_bat_health_data.bat_health = (int)((health_t + 5)/10);
	
			backup_bat_health();
			batt_health_csc_backup();
			battery_log(BAT_LOG_CRTI, "%s battery health = (%d,%d), T(%lu), bat_current_avg(%llu)mA\n",__FUNCTION__, g_bat_health_data.bat_health, g_bat_health_avg, T, g_bat_health_data.bat_current_avg/1000);
			
			battery_health_data_reset();
		}else{
				//do nothing
		}
	}else{
		battery_health_data_reset();
	}
	g_last_bathealth_trigger = g_bathealth_trigger;
}

/* 
* test_battery_health - Create battery health value for test
* @count: the number of create battery health value
*
* note: after testing, to delete /batinfo/bat_health, to delete /batinfo/.bh, to delete /sdcard/.bh, then reboot
*/
static int test_battery_health(int count)
{
	int rc;
	struct timespec ts;
	struct rtc_time tm; 
	int i=0;

	battery_log(BAT_LOG_CRTI, "======test_battery_health\n");

	getnstimeofday(&ts); 
	rtc_time_to_tm(ts.tv_sec,&tm); 

	memset(&g_bat_health_data_backup,0,sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX);

	if (count < 1 || count >= BAT_HEALTH_NUMBER_MAX) {
		count = BAT_HEALTH_NUMBER_MAX - 1;
	}
	g_health_upgrade_index = count;
	g_bat_health_data_backup[0].health = count;

	battery_log(BAT_LOG_CRTI, "%s: ======Create count(%d)\n",__FUNCTION__, g_bat_health_data_backup[0].health);
	for(i=1; i<=count;i++){
		sprintf(g_bat_health_data_backup[i].date, "%d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900,tm.tm_mon+1, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		g_bat_health_data_backup[i].health = i;
		battery_log(BAT_LOG_CRTI, "%s %d\n",g_bat_health_data_backup[i].date, g_bat_health_data_backup[i].health);
	}
	
	battery_log(BAT_LOG_CRTI, "%s ========================\n",__FUNCTION__);

	rc = file_op(BAT_HEALTH_DATA_FILE_NAME, BAT_HEALTH_DATA_OFFSET,
		(char *)&g_bat_health_data_backup, sizeof(struct BAT_HEALTH_DATA_BACKUP)*BAT_HEALTH_NUMBER_MAX, FILE_OP_WRITE);
	if(rc<0){
		pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_HEALTH_DATA_FILE_NAME);
	}

	rc = batt_health_csc_backup();
	return rc;
}


static int batt_health_config_proc_show(struct seq_file *buf, void *data)
{
	int count=0, i=0;
	unsigned long long bat_health_accumulate=0;
	
	seq_printf(buf, "start level:%d\n", g_health_upgrade_start_level);
	seq_printf(buf, "end level:%d\n", g_health_upgrade_end_level);
	seq_printf(buf, "upgrade time:%d\n", g_health_upgrade_upgrade_time);
	seq_printf(buf, "upgrade index:%d\n", g_health_upgrade_index);

	for(i=1;i<BAT_HEALTH_NUMBER_MAX;i++){
		if(g_bat_health_data_backup[i].health!=0){
			count++;
			bat_health_accumulate += g_bat_health_data_backup[i].health;
		}
	}
	g_bat_health_avg = bat_health_accumulate/count;	
	seq_printf(buf, "health_avg: %d\n", g_bat_health_avg);

	return 0;
}

static int batt_health_config_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, batt_health_config_proc_show, NULL);
}

static ssize_t batt_health_config_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
{
	int command=0;
	int value = 0;
	char buf[30] = {0};
	size_t buf_size;
	char *start = buf;

	buf_size = min(count, (size_t)(sizeof(buf)-1));
	if (copy_from_user(buf, buffer, buf_size)) {
		pr_err("Failed to copy from user\n");
		return -EFAULT;
	}
	buf[buf_size] = 0;

	sscanf(start, "%d", &command);
	while (*start++ != ' ');
	sscanf(start, "%d", &value);

	switch(command){
		case 1:
			g_health_upgrade_start_level = value;
			battery_log(BAT_LOG_CRTI, "health upgrade start_level = %d;\n", value);
		break;
		case 2:
			g_health_upgrade_end_level = value;
			battery_log(BAT_LOG_CRTI, "health upgrade end_level = %d;\n", value);
		break;
		case 3:
			g_health_upgrade_upgrade_time = value;
			battery_log(BAT_LOG_CRTI, "health upgrade time = %d;\n", value);
		break;
		case 4:
			test_battery_health(value);
		break;
		default:
			battery_log(BAT_LOG_CRTI, "input error!Now return\n");
			return count;
	}

	return count;
}

static const struct file_operations batt_health_config_fops = {
	.owner = THIS_MODULE,
	.open = batt_health_config_proc_open,
	.read = seq_read,
	.write = batt_health_config_write,
	.release = single_release,
};
 
static void battery_health_upgrade_data_polling(int time) {
	cancel_delayed_work(&battery_health_work);
	schedule_delayed_work(&battery_health_work, time * HZ);
}

static void battery_health_worker(struct work_struct *work)
{
	update_battery_health();
	battery_health_upgrade_data_polling(g_health_upgrade_upgrade_time);
}

//======================== ASUS_BSP battery safety upgrade ========================

 struct fg_dev {
 //ASUS_BSP battery safety upgrade +++
	 unsigned long condition1_battery_time;
	 unsigned long condition2_battery_time;
	 int condition1_cycle_count;
	 int condition2_cycle_count;
	 unsigned long condition1_temp_vol_time;
	 unsigned long condition2_temp_vol_time;
	 unsigned long condition1_temp_time;
	 unsigned long condition2_temp_time;
	 unsigned long condition1_vol_time;
	 unsigned long condition2_vol_time;
 //ASUS_BSP battery safety upgrade ---
 };
  
 //ASUS_BSP battery safety upgrade +++
 /* Cycle Count Date Structure saved in emmc
 + * magic - magic number for data verification
 + * charge_cap_accum - Accumulated charging capacity
 + * charge_last_soc - last saved soc before reset/shutdown
 + * [0]:battery_soc [1]:system_soc [2]:monotonic_soc
 + */
 struct CYCLE_COUNT_DATA{
	 int magic;
	 int cycle_count;
	 unsigned long battery_total_time;
	 unsigned long high_vol_total_time;
	 unsigned long high_temp_total_time;
	 unsigned long high_temp_vol_time;
	 u32 reload_condition;
 };
 
#define HIGH_TEMP   350
#define HIGHER_TEMP 450
#define FULL_CAPACITY_VALUE 100
#define BATTERY_USE_TIME_CONDITION1  (12*30*24*60*60) //12Months
#define BATTERY_USE_TIME_CONDITION2  (18*30*24*60*60) //18Months
#define CYCLE_COUNT_CONDITION1  100
#define CYCLE_COUNT_CONDITION2  400
#define HIGH_TEMP_VOL_TIME_CONDITION1 (15*24*60*60)  //15Days
#define HIGH_TEMP_VOL_TIME_CONDITION2 (30*24*60*60)  //30Days
#define HIGH_TEMP_TIME_CONDITION1     (6*30*24*60*60) //6Months
#define HIGH_TEMP_TIME_CONDITION2     (12*30*24*60*60) //12Months
#define HIGH_VOL_TIME_CONDITION1     (6*30*24*60*60) //6Months
#define HIGH_VOL_TIME_CONDITION2     (12*30*24*60*60) //12Months
 
 enum calculation_time_type {
	 TOTOL_TIME_CAL_TYPE,
	 HIGH_VOL_CAL_TYPE,
	 HIGH_TEMP_CAL_TYPE,
	 HIGH_TEMP_VOL_CAL_TYPE,
 };
 //ASUS_BSP battery safety upgrade ---
   
  struct fg_dev *g_fg = NULL;
 #define REPORT_CAPACITY_POLLING_TIME 180
  char battery_name[64] = "";
 //ASUS_BSP battery safety upgrade +++
#define CYCLE_COUNT_DATA_MAGIC  0x85
#define CYCLE_COUNT_FILE_NAME   "/batinfo/.bs"
#define BAT_PERCENT_FILE_NAME   "/batinfo/Batpercentage"
#define BAT_SAFETY_FILE_NAME   "/batinfo/bat_safety"
#define CYCLE_COUNT_SD_FILE_NAME   "/sdcard/.bs"
#define BAT_PERCENT_SD_FILE_NAME   "/sdcard/Batpercentage"
#define BAT_CYCLE_SD_FILE_NAME   "/sdcard/Batcyclecount"
#define CYCLE_COUNT_DATA_OFFSET  0x0
#define	BATTERY_SAFETY_UPGRADE_TIME 1*60 // one hour
 
 static bool g_cyclecount_initialized = false;
 //extern bool rtc_probe_done;
 static struct CYCLE_COUNT_DATA g_cycle_count_data = {
	 .magic = CYCLE_COUNT_DATA_MAGIC,
	 .cycle_count=0,
	 .battery_total_time = 0,
	 .high_vol_total_time = 0,
	 .high_temp_total_time = 0,
	 .high_temp_vol_time = 0,
	 .reload_condition = 0
 };
 struct delayed_work battery_safety_work;
 //ASUS_BSP battery safety upgrade ---
  //[---]ASUS : Add variables
   
 static int g_fv_setting; //ASUS_BSP battery safety upgrade
 static int FV_JEITA_uV; //ASUS_BSP battery safety upgrade
 //ASUS_BSP battery safety upgrade +++
 static void set_full_charging_voltage(void)
 {
	 if(0 == g_cycle_count_data.reload_condition){
		 g_fv_setting = 0x4C; //4.36V
		 //fg_set_constant_chg_voltage(g_fg, 4350 * 1000);
		 FV_JEITA_uV = 4360000;
	 }else if(1 == g_cycle_count_data.reload_condition){
		 g_fv_setting = 0x47; //4.30V
		 //fg_set_constant_chg_voltage(g_fg, 4300 * 1000);
		 FV_JEITA_uV = 4310000;
	 }else if(2 == g_cycle_count_data.reload_condition){
		 g_fv_setting = 0x42; //4.25V
		 //fg_set_constant_chg_voltage(g_fg, 4250 * 1000);
		 FV_JEITA_uV = 4260000;
	 }
	 battery_log(BAT_LOG_CRTI, "g_fv_setting=%x \n",g_fv_setting);
 }
 //ASUS_BSP battery safety upgrade ---
  
 //ASUS_BSP battery safety upgrade +++
 static void init_battery_safety(struct fg_dev *fg)
 {
	 fg->condition1_battery_time = BATTERY_USE_TIME_CONDITION1;
	 fg->condition2_battery_time = BATTERY_USE_TIME_CONDITION2;
	 fg->condition1_cycle_count = CYCLE_COUNT_CONDITION1;
	 fg->condition2_cycle_count = CYCLE_COUNT_CONDITION2;
	 fg->condition1_temp_vol_time = HIGH_TEMP_VOL_TIME_CONDITION1;
	 fg->condition2_temp_vol_time = HIGH_TEMP_VOL_TIME_CONDITION2;
	 fg->condition1_temp_time = HIGH_TEMP_TIME_CONDITION1;
	 fg->condition2_temp_time = HIGH_TEMP_TIME_CONDITION2;
	 fg->condition1_vol_time = HIGH_VOL_TIME_CONDITION1;
	 fg->condition2_vol_time = HIGH_VOL_TIME_CONDITION2;
 }
 
 static int backup_bat_percentage(void)
 {
	 char buf[1]={0};
	 int bat_percent = 0, rc;
 
	 if(0 == g_cycle_count_data.reload_condition){
		 bat_percent = 0;
	 }else if(1 == g_cycle_count_data.reload_condition){
		 bat_percent = 95;
	 }else if(2 == g_cycle_count_data.reload_condition){
		 bat_percent = 90;
	 }
	 sprintf(buf, "%d\n", bat_percent);
	 battery_log(BAT_LOG_CRTI, "bat_percent=%d;reload_condition=%d\n", bat_percent, g_cycle_count_data.reload_condition);  
 
	 rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&buf, sizeof(char), FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_PERCENT_FILE_NAME);
 
	 return rc;
 }

#if 0
 static int backup_bat_cyclecount(void)
 {
	 char buf[30]={0};
	 int rc;
 
	 sprintf(buf, "%d\n", g_cycle_count_data.cycle_count);
	 battery_log(BAT_LOG_CRTI, "cycle_count=%d\n", g_cycle_count_data.cycle_count);	 
 
	 rc = file_op(BAT_CYCLE_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&buf, sizeof(char)*30, FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_CYCLE_SD_FILE_NAME);
 
 
	 battery_log(BAT_LOG_CRTI, "Done! rc(%d)\n",rc);
	 return rc;
 }
#endif

 static int backup_bat_safety(void)
 {
	 char buf[70]={0};
	 int rc;
 
	 sprintf(buf, "%lu,%d,%lu,%lu,%lu\n", 
		 g_cycle_count_data.battery_total_time, 
		 g_cycle_count_data.cycle_count, 
		 g_cycle_count_data.high_temp_total_time, 
		 g_cycle_count_data.high_vol_total_time,
		 g_cycle_count_data.high_temp_vol_time);	 
 
	 rc = file_op(BAT_SAFETY_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&buf, sizeof(char)*70, FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_SAFETY_FILE_NAME);
 
	 return rc;
 }
 
 static int init_batt_cycle_count_data(void)
 {
	 int rc = 0;
	 struct CYCLE_COUNT_DATA buf;
 
	 /* Read cycle count data from emmc */
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
	 if(rc < 0) {
		 pr_err("Read cycle count file failed!\n");
		 return rc;
	 }
 
	 /* Check data validation */
	 if(buf.magic != CYCLE_COUNT_DATA_MAGIC) {
		 pr_err("data validation!\n");
		 file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
		 return -1;
	 }else {
		 /* Update current value */
		 battery_log(BAT_LOG_CRTI, "Update current value!\n");
		 g_cycle_count_data.cycle_count = buf.cycle_count;
		 g_cycle_count_data.high_temp_total_time = buf.high_temp_total_time;
		 g_cycle_count_data.high_temp_vol_time = buf.high_temp_vol_time;
		 g_cycle_count_data.high_vol_total_time = buf.high_vol_total_time;
		 g_cycle_count_data.reload_condition = buf.reload_condition;
		 g_cycle_count_data.battery_total_time = buf.battery_total_time;
 
		 rc = backup_bat_percentage();
		 if(rc < 0){
			 pr_err("backup_bat_percentage failed!\n");
			 return -1;
		 }

#if 0 
		 rc = backup_bat_cyclecount();
		 if(rc < 0){
			 pr_err("backup_bat_cyclecount failed!\n");
			 return -1;
		 }
#endif
		 
		 rc = backup_bat_safety();
		 if(rc < 0){
			 pr_err("backup_bat_cyclecount failed!\n");
			 return -1;
		 }
 
		 battery_log(BAT_LOG_CRTI, "reload_condition=%d;high_temp_total_time=%lu;high_temp_vol_time=%lu;high_vol_total_time=%lu;battery_total_time=%lu\n",
			 buf.reload_condition, buf.high_temp_total_time,buf.high_temp_vol_time,buf.high_vol_total_time,buf.battery_total_time);
	 }
	 battery_log(BAT_LOG_CRTI, "Cycle count data initialize success!\n");
	 g_cyclecount_initialized = true;
	 set_full_charging_voltage();
	 return 0;
 }
 
 static void write_back_cycle_count_data(void)
 {
	 int rc;
 
	 backup_bat_percentage();
	 //backup_bat_cyclecount();
	 backup_bat_safety();
	 //batt_safety_csc_backup();
	 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("%s:Write file:%s err!\n", __FUNCTION__, CYCLE_COUNT_FILE_NAME);
 }
 
 static void asus_reload_battery_profile(struct fg_dev *fg, int value){
 
	 //save current status
	 write_back_cycle_count_data();
 
	 //reloade battery
	 //reload_battery_profile(chip);
	 set_full_charging_voltage();
 
	 battery_log(BAT_LOG_CRTI, "!!new profile is value=%d\n",value);
 }
 
 static void asus_judge_reload_condition(struct fg_dev *fg)
 {
	 int temp_condition = 0;
	 int cycle_count = 0;
//	 bool full_charge;
	 unsigned long local_high_vol_time = g_cycle_count_data.high_vol_total_time;
	 unsigned long local_high_temp_time = g_cycle_count_data.high_temp_total_time;
	 //unsigned long local_high_temp_vol_time = g_cycle_count_data.high_temp_vol_time;
	 unsigned long local_battery_total_time = g_cycle_count_data.battery_total_time;
 
	 temp_condition = g_cycle_count_data.reload_condition;
	 if(temp_condition >= 2){ //if condition=2 will return
		 return ;
	 }
/* 
	 //only full charger can load new profile
	 full_charge = fg->charge_done;
	 if(!full_charge)
		 return ;
*/ 
	 //1.judge battery using total time
	 if(local_battery_total_time >= fg->condition2_battery_time){
		 g_cycle_count_data.reload_condition = 2;
		 goto DONE;
	 }else if(local_battery_total_time >= fg->condition1_battery_time &&
		 local_battery_total_time < fg->condition2_battery_time){
		 g_cycle_count_data.reload_condition = 1;
	 }
 
	 //2. judge battery cycle count
	 cycle_count = g_cycle_count_data.cycle_count;
#if 0 //disable reloade condition with cycle_count
	 if(cycle_count >= chip->condition2_cycle_count){
		 g_cycle_count_data.reload_condition = 2;
		 goto DONE;
	 }else if(cycle_count >= chip->condition1_cycle_count &&
		 cycle_count < chip->condition2_cycle_count){
		 g_cycle_count_data.reload_condition = 1;
	 }
#endif
#if 0 //disable reloade condition with high_temp_vol
	 //3. judge high temp and voltage condition
	 if(local_high_temp_vol_time >= fg->condition2_temp_vol_time){
		 g_cycle_count_data.reload_condition = 2;
		 goto DONE;
	 }else if(local_high_temp_vol_time >= fg->condition1_temp_vol_time &&
		 local_high_temp_vol_time < fg->condition2_temp_vol_time){
		 g_cycle_count_data.reload_condition = 1;
	 }
#endif
 
	 //4. judge high temp condition
	 if(local_high_temp_time >= fg->condition2_temp_time){
		 g_cycle_count_data.reload_condition = 2;
		 goto DONE;
	 }else if(local_high_temp_time >= fg->condition1_temp_time &&
		 local_high_temp_time < fg->condition2_temp_time){
		 g_cycle_count_data.reload_condition = 1;
	 }
 
	 //5. judge high voltage condition
	 if(local_high_vol_time >= fg->condition2_vol_time){
		 g_cycle_count_data.reload_condition = 2;
		 goto DONE;
	 }else if(local_high_vol_time >= fg->condition1_vol_time &&
		 local_high_vol_time < fg->condition2_vol_time){
		 g_cycle_count_data.reload_condition = 1;
	 }
 
 DONE:
	 if(temp_condition != g_cycle_count_data.reload_condition)
		 asus_reload_battery_profile(fg, g_cycle_count_data.reload_condition);
 
 }
 
 unsigned long last_battery_total_time = 0;
 unsigned long last_high_temp_time = 0;
 unsigned long last_high_vol_time = 0;
 unsigned long last_high_temp_vol_time = 0;
 
 static void calculation_time_fun(int type)
 {
	 unsigned long now_time;
	 unsigned long temp_time = 0;
 
	 now_time = rtc_read_hw_time();
	 if(now_time < 0){
		 pr_err("asus read rtc time failed!\n");
		 return ;
	 }
 
	 switch(type){
		 case TOTOL_TIME_CAL_TYPE:
			 if(0 == last_battery_total_time){
				 last_battery_total_time = now_time;
				 battery_log(BAT_LOG_CRTI, "now_time=%lu;last_battery_total_time=%lu\n", now_time, g_cycle_count_data.battery_total_time);
			 }else{
				 temp_time = now_time - last_battery_total_time;
				 if(temp_time > 0)
					 g_cycle_count_data.battery_total_time += temp_time;
				 last_battery_total_time = now_time;
			 }
		 break;
 
		 case HIGH_VOL_CAL_TYPE:
			 if(0 == last_high_vol_time){
				 last_high_vol_time = now_time;
				 battery_log(BAT_LOG_CRTI, "now_time=%lu;high_vol_total_time=%lu\n", now_time, g_cycle_count_data.high_vol_total_time);
			 }else{
				 temp_time = now_time - last_high_vol_time;
				 if(temp_time > 0)
					 g_cycle_count_data.high_vol_total_time += temp_time;
				 last_high_vol_time = now_time;
			 }
		 break;
 
		 case HIGH_TEMP_CAL_TYPE:
			 if(0 == last_high_temp_time){
				 last_high_temp_time = now_time;
				 battery_log(BAT_LOG_CRTI, "now_time=%lu;high_temp_total_time=%lu\n", now_time, g_cycle_count_data.high_temp_total_time);
			 }else{
				 temp_time = now_time - last_high_temp_time;
				 if(temp_time > 0)
					 g_cycle_count_data.high_temp_total_time += temp_time;
				 last_high_temp_time = now_time;
			 }
		 break;
 
		 case HIGH_TEMP_VOL_CAL_TYPE:
			 if(0 == last_high_temp_vol_time){
				 last_high_temp_vol_time = now_time;
				 battery_log(BAT_LOG_CRTI, "now_time=%lu;high_temp_vol_time=%lu\n", now_time, g_cycle_count_data.high_temp_vol_time);
			 }else{
				 temp_time = now_time - last_high_temp_vol_time;
				 if(temp_time > 0)
					 g_cycle_count_data.high_temp_vol_time += temp_time;
				 last_high_temp_vol_time = now_time;
			 }
		 break;
	 }
 }
 
#if 0
 static void calculation_battery_time_fun(unsigned long time)
 {
	 static unsigned long last_time = 0;
	 unsigned long count_time = 0;
 
	 if(last_time > 0){
		 count_time = ((time-last_time)>0) ? (time-last_time):0;
		 g_cycle_count_data.battery_total_time += count_time;
		 last_time = time;
		 return ;
	 }
 
	 if(g_cycle_count_data.battery_total_time <= time){
		  g_cycle_count_data.battery_total_time = time;
	 }else{//add up time when pull out the battery
		 g_cycle_count_data.battery_total_time += time;
	 }
	 battery_log(BAT_LOG_CRTI, "time=%lu;battery_total_time=%lu\n", time,g_cycle_count_data.battery_total_time);
	 last_time = time;
 }
#endif 
 
 static int write_test_value = 0;
 static void update_battery_safe(struct fg_dev *fg)
 {
	 int rc;
	 int temp;
	 int capacity;
	 unsigned long now_time;
 
	 battery_log(BAT_LOG_CRTI, "+++");

	 if(g_cyclecount_initialized != true){
		 rc = init_batt_cycle_count_data();
		 if(rc < 0){
			 pr_err("cyclecount is not initialized");
			 return;
		 }
	 }
	 
	 temp = read_tbat_value() * 10;
	 battery_log(BAT_LOG_CRTI, "temp=%d\n", temp);
	 if (temp < 0) {
		 pr_err("Error in getting battery temp, temp=%d\n", temp);
		 return;
	 }
 
	 capacity = battery_GetBatteryPercent();
	 battery_log(BAT_LOG_CRTI, "capacity=%d\n", capacity);
	 if (capacity < 0) {
		 pr_err("Error in getting capacity, capacity=%d\n", capacity);
		 return;
	 }
 
	 now_time = rtc_read_hw_time();
 
	 if(now_time < 0){
		 pr_err("asus read rtc time failed!\n");
		 return ;
	 }
 
#if 0
	 if(write_test_value != 1){ //skip battery time test
		 calculation_battery_time_fun(now_time);
	 }
#endif
	 calculation_time_fun(TOTOL_TIME_CAL_TYPE);
 
	 if(capacity == FULL_CAPACITY_VALUE){
		 calculation_time_fun(HIGH_VOL_CAL_TYPE);	 
	 }else{
		 last_high_vol_time = 0; //exit high vol
	 }
 
	 if(temp >= HIGHER_TEMP){
		 calculation_time_fun(HIGH_TEMP_CAL_TYPE);
	 }else{
		 last_high_temp_time = 0; //exit high temp
	 }
 
	 if(temp >= HIGH_TEMP && capacity == FULL_CAPACITY_VALUE){
		 calculation_time_fun(HIGH_TEMP_VOL_CAL_TYPE);
	 }else{
		 last_high_temp_vol_time = 0; //exit high temp and vol
	 }
 
	 asus_judge_reload_condition(fg);
	 write_back_cycle_count_data();
	 battery_log(BAT_LOG_CRTI, "---");
 }
 
 void battery_safety_upgrade_data_polling(int time) {
	 cancel_delayed_work(&battery_safety_work);
	 schedule_delayed_work(&battery_safety_work, time * HZ);
 }
 
 void battery_safety_worker(struct work_struct *work)
 {
	 update_battery_safe(g_fg);
	 battery_safety_upgrade_data_polling(BATTERY_SAFETY_UPGRADE_TIME); // update each hour
 }
  
 //ASUS_BSP battery safety upgrade +++
 static int batt_safety_proc_show(struct seq_file *buf, void *data)
 {
	 int rc =0;
 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc < 0 )
		 pr_err("write cycle count file error\n");
	 
	 seq_printf(buf, "---show battery safety value---\n");
	 seq_printf(buf, "cycle_count:%d\n", g_cycle_count_data.cycle_count);
	 seq_printf(buf, "battery_total_time:%lu\n", g_cycle_count_data.battery_total_time);
	 seq_printf(buf, "high_temp_total_time:%lu\n", g_cycle_count_data.high_temp_total_time);
	 seq_printf(buf, "high_vol_total_time:%lu\n", g_cycle_count_data.high_vol_total_time);
	 seq_printf(buf, "high_temp_vol_time:%lu\n", g_cycle_count_data.high_temp_vol_time);
	 seq_printf(buf, "reload_condition:%d\n", g_cycle_count_data.reload_condition);
 
	 return 0;
 }
 static int batt_safety_proc_open(struct inode *inode, struct file *file)
 {
	 return single_open(file, batt_safety_proc_show, NULL);
 }
 
 static ssize_t batt_safety_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
 {
	 int value=0;
	 unsigned long time = 0;
	 char buf[30] = {0};
	 size_t buf_size;
	 char *start = buf;
 
	 buf_size = min(count, (size_t)(sizeof(buf)-1));
	 if (copy_from_user(buf, buffer, buf_size)) {
		 pr_err("Failed to copy from user\n");
		 return -EFAULT;
	 }
	 buf[buf_size] = 0;
 
	 sscanf(start, "%d", &value);
	 while (*start++ != ' ');
	 sscanf(start, "%lu", &time);
 
	 write_test_value = value;
 
	 switch(value){
		 case 1:
			 g_cycle_count_data.battery_total_time = time;
		 break;
		 case 2:
			 g_cycle_count_data.cycle_count = (int)time;
		 break;
		 case 3:
			 g_cycle_count_data.high_temp_vol_time = time;
		 break;
		 case 4:
			 g_cycle_count_data.high_temp_total_time = time;
		 break;
		 case 5:
			 g_cycle_count_data.high_vol_total_time = time;
		 break;
		 default:
			 battery_log(BAT_LOG_CRTI, "input error!Now return\n");
			 return count;
	 }
	 asus_judge_reload_condition(g_fg);
	 battery_log(BAT_LOG_CRTI, "value=%d;time=%lu\n", value, time);
 
	 return count;
 }
 
 static const struct file_operations batt_safety_fops = {
	 .owner = THIS_MODULE,
	 .open = batt_safety_proc_open,
	 .read = seq_read,
	 .write = batt_safety_proc_write,
	 .release = single_release,
 };
 
 static int batt_safety_csc_proc_show(struct seq_file *buf, void *data)
 {
	 int rc =0;
 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc < 0 )
		 pr_err("write cycle count file error\n");
	 
	 seq_printf(buf, "---show battery safety value---\n");
	 seq_printf(buf, "cycle_count:%d\n", g_cycle_count_data.cycle_count);
	 seq_printf(buf, "battery_total_time:%lu\n", g_cycle_count_data.battery_total_time);
	 seq_printf(buf, "high_temp_total_time:%lu\n", g_cycle_count_data.high_temp_total_time);
	 seq_printf(buf, "high_vol_total_time:%lu\n", g_cycle_count_data.high_vol_total_time);
	 seq_printf(buf, "high_temp_vol_time:%lu\n", g_cycle_count_data.high_temp_vol_time);
	 seq_printf(buf, "reload_condition:%d\n", g_cycle_count_data.reload_condition);
 
	 return 0;
 }
 static int batt_safety_csc_proc_open(struct inode *inode, struct file *file)
 {
	 return single_open(file, batt_safety_csc_proc_show, NULL);
 }
 
 static int batt_safety_csc_erase(void){
	 int rc =0;
	 char buf[1]={0};
 
	 g_cycle_count_data.battery_total_time = 0;
	 g_cycle_count_data.cycle_count = 0;
	 g_cycle_count_data.high_temp_total_time = 0;
	 g_cycle_count_data.high_temp_vol_time = 0;
	 g_cycle_count_data.high_vol_total_time = 0;
	 g_cycle_count_data.reload_condition = 0;
 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&g_cycle_count_data, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc < 0 )
		 pr_err("Write file:%s err!\n", CYCLE_COUNT_FILE_NAME);
 
	 rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&buf, sizeof(char), FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("Write file:%s err!\n", BAT_PERCENT_FILE_NAME);
 
	 battery_log(BAT_LOG_CRTI, "Done! rc(%d)\n",rc);
	 return rc;
 }
 
 static int batt_safety_csc_backup(void){
	 int rc = 0;
	 struct CYCLE_COUNT_DATA buf;
 //  char buf2[1]={0};
 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
	 if(rc < 0) {
		 pr_err("Read cycle count file failed!\n");
		 return rc;
	 }
 
	 rc = file_op(CYCLE_COUNT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc < 0 ) 
		 pr_err("Write cycle count file failed!\n");
#if 0
	 rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&buf2, sizeof(char), FILE_OP_READ);
	 if(rc < 0) {
		 pr_err("Read cycle count percent file failed!\n");
		 return rc;
	 }
 
	 rc = file_op(BAT_PERCENT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&buf2, sizeof(char), FILE_OP_WRITE);
	 if(rc < 0 ) 
		 pr_err("Write cycle count percent file failed!\n");
 
	 battery_log(BAT_LOG_CRTI, "Done! rc(%d)\n",rc);
#endif
	 battery_log(BAT_LOG_CRTI, "Done!\n");
	 return rc;
 }
 
 static int batt_safety_csc_restore(void){
	 int rc = 0;
	 struct CYCLE_COUNT_DATA buf;
	 char buf2[1]={0};
 
	 rc = file_op(CYCLE_COUNT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_READ);
	 if(rc < 0) {
		 pr_err("Read cycle count file failed!\n");
		 return rc;
	 }
 
	 rc = file_op(CYCLE_COUNT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&buf, sizeof(struct CYCLE_COUNT_DATA), FILE_OP_WRITE);
	 if(rc < 0 ) 
		 pr_err("Write cycle count file failed!\n");
 
	 rc = file_op(BAT_PERCENT_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char*)&buf2, sizeof(char), FILE_OP_READ);
	 if(rc < 0) {
		 pr_err("Read cycle count percent file failed!\n");
		 return rc;
	 }
 
	 rc = file_op(BAT_PERCENT_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
	 (char *)&buf2, sizeof(char), FILE_OP_WRITE);
	 if(rc < 0 ) 
		 pr_err("Write cycle count percent file failed!\n");
 
	 init_batt_cycle_count_data();
	 battery_log(BAT_LOG_CRTI, "Done! rc(%d)\n",rc);
	 return rc;
 }
 
 //ASUS_BS battery health upgrade +++
 static void batt_health_upgrade_debug_enable(bool enable)
 {
	g_health_debug_enable = enable;
	battery_log(BAT_LOG_CRTI, "%d\n",g_health_debug_enable);
 }
 
 static void batt_health_upgrade_enable(bool enable)
 {
	g_health_upgrade_enable = enable;
	battery_log(BAT_LOG_CRTI, "%d\n",g_health_upgrade_enable);
 }
 
 //ASUS_BS battery health upgrade ---
 
 static int batt_safety_csc_getcyclecount(void){
	 char buf[30]={0};
	 int rc;
 
	 sprintf(buf, "%d\n", g_cycle_count_data.cycle_count);
	 battery_log(BAT_LOG_CRTI, "cycle_count=%d\n", g_cycle_count_data.cycle_count);
 
	 rc = file_op(BAT_CYCLE_SD_FILE_NAME, CYCLE_COUNT_DATA_OFFSET,
		 (char *)&buf, sizeof(char)*30, FILE_OP_WRITE);
	 if(rc<0)
		 pr_err("%s:Write file:%s err!\n", __FUNCTION__, BAT_CYCLE_SD_FILE_NAME);
 
 
	 battery_log(BAT_LOG_CRTI, "Done! rc(%d)\n",rc);
	 return rc;
 }
 
 static ssize_t batt_safety_csc_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
 {
	 int value=0;
	 char buf[2] = {0};
	 size_t buf_size;
	 char *start = buf;
 
	 buf_size = min(count, (size_t)(sizeof(buf)-1));
	 if (copy_from_user(buf, buffer, buf_size)) {
		 pr_err("Failed to copy from user\n");
		 return -EFAULT;
	 }
	 buf[buf_size] = 0;
 
	 sscanf(start, "%d", &value);
 
	 switch(value){
		 case 0: //erase
			 batt_safety_csc_erase();
		 break;
		 case 1: //backup /persist to /sdcard
			 batt_safety_csc_backup();
		 break;
		 case 2: //resotre /sdcard from /persist 
			 batt_safety_csc_restore();
		 break;
		 case 3: //write cycle count to /sdcard
			 batt_safety_csc_getcyclecount();
		 break;
		 case 4: //copy bat_health_data to /sdcard from /persist
			 batt_health_csc_backup();
			 break;
		 case 5: // disable battery health debug log
			 batt_health_upgrade_debug_enable(false);
			 break;
		 case 6: // enable battery health debug log
			 batt_health_upgrade_debug_enable(true);
			 break;
		 case 7: // disable battery health upgrade
			 batt_health_upgrade_enable(false);
			 break;
		 case 8: // enable battery health upgrade
			 batt_health_upgrade_enable(true);
			 break;
		 default:
			 pr_err("input error!Now return\n");
			 return count;
	 }
 
	 return count;
 }
 
 static const struct file_operations batt_safety_csc_fops = {
	 .owner = THIS_MODULE,
	 .open = batt_safety_csc_proc_open,
	 .read = seq_read,
	 .write = batt_safety_csc_proc_write,
	 .release = single_release,
 };
 
 static int cycle_count_proc_show(struct seq_file *buf, void *data)
 {
	 seq_printf(buf, "---show cycle count value---\n");
	 seq_printf(buf, "cycle count:%d\n", g_cycle_count_data.cycle_count);
 
	 return 0;
 }
 
 static int cycle_count_proc_open(struct inode *inode, struct file *file)
 {
	 return single_open(file, cycle_count_proc_show, NULL);
 }
 
 static const struct file_operations cycle_count_fops = {
	 .owner = THIS_MODULE,
	 .open = cycle_count_proc_open,
	 .read = seq_read,
	 .release = single_release,
 };
 
 static int condition_value_proc_show(struct seq_file *buf, void *data)
 {
	 if(!g_fg){
		 pr_err("chip oem is NULL!");
		 return -1;
	 }
 
	 seq_printf(buf, "---show condition value---\n");
	 seq_printf(buf, "condition1 battery time %lu\n", g_fg->condition1_battery_time);
	 seq_printf(buf, "condition2 battery time %lu\n", g_fg->condition2_battery_time);
	 seq_printf(buf, "condition1 cycle count %d\n", g_fg->condition1_cycle_count);
	 seq_printf(buf, "condition2 cycle count %d\n", g_fg->condition2_cycle_count);
	 seq_printf(buf, "condition1 temp time %lu\n", g_fg->condition1_temp_time);
	 seq_printf(buf, "condition2 temp time %lu\n", g_fg->condition2_temp_time);
	 seq_printf(buf, "condition1 temp&vol time %lu\n", g_fg->condition1_temp_vol_time);
	 seq_printf(buf, "condition2 temp&vol time %lu\n", g_fg->condition2_temp_vol_time);
	 seq_printf(buf, "condition1 vol time %lu\n", g_fg->condition1_vol_time);
	 seq_printf(buf, "condition2 vol time %lu\n", g_fg->condition2_vol_time);
 
	 return 0;
 }
 
 static int condition_value_proc_open(struct inode *inode, struct file *file)
 {
	 return single_open(file, condition_value_proc_show, NULL);
 }
 
 static ssize_t condition_value_proc_write(struct file *file,const char __user *buffer,size_t count,loff_t *pos)
 {
	 int value = 0;
	 unsigned long condition1_time = 0;
	 unsigned long condition2_time = 0;
	 char buf[count];
	 char *start = buf;

	 if (copy_from_user(buf, buffer, count-1)) {
		 pr_err("Failed to copy from user\n");
		 return -EFAULT;
	 }
	 buf[count] = 0;
 
	 sscanf(start, "%d", &value);
	 while (*start++ != ' ');
	 sscanf(start, "%lu", &condition1_time);
	 while (*start++ != ' ');
	 sscanf(start, "%lu", &condition2_time);
 
	 if(value && condition2_time <= condition1_time){
		 pr_err("input value error,please input correct value!\n");
		 return count;
	 }
 
	 switch(value){
		 case 0:
			 init_battery_safety(g_fg);
			 g_cycle_count_data.reload_condition = 0;
		 break;
		 case 1:
			 g_fg->condition1_battery_time = condition1_time;
			 g_fg->condition2_battery_time = condition2_time;
		 break;
		 case 2:
			 g_fg->condition1_cycle_count = (int)condition1_time;
			 g_fg->condition2_cycle_count = (int)condition2_time;
		 break;
		 case 3:
			 g_fg->condition1_temp_vol_time = condition1_time;
			 g_fg->condition2_temp_vol_time = condition2_time;
		 break;
		 case 4:
			 g_fg->condition1_temp_time = condition1_time;
			 g_fg->condition2_temp_time = condition2_time;
		 break;
		 case 5:
			 g_fg->condition1_vol_time = condition1_time;
			 g_fg->condition2_vol_time = condition2_time;
		 break;
	 }
 
	 battery_log(BAT_LOG_CRTI, "value=%d;condition1_time=%lu;condition2_time=%lu\n", value, condition1_time, condition2_time);
	 return count;
 }
 
 static const struct file_operations condition_value_fops = {
	 .owner = THIS_MODULE,
	 .open = condition_value_proc_open,
	 .read = seq_read,
	 .write = condition_value_proc_write,
	 .release = single_release,
 };
 
 static void create_batt_cycle_count_proc_file(void)
 {
	 struct proc_dir_entry *asus_batt_cycle_count_dir = proc_mkdir("Batt_Cycle_Count", NULL);
	 struct proc_dir_entry *asus_batt_cycle_count_proc_file = proc_create("cycle_count", 0666,
		 asus_batt_cycle_count_dir, &cycle_count_fops);
	 struct proc_dir_entry *asus_batt_batt_safety_proc_file = proc_create("batt_safety", 0666,
		 asus_batt_cycle_count_dir, &batt_safety_fops);
	 struct proc_dir_entry *asus_batt_batt_safety_csc_proc_file = proc_create("batt_safety_csc", 0666,
		 asus_batt_cycle_count_dir, &batt_safety_csc_fops);
	 struct proc_dir_entry *asus_batt_safety_condition_proc_file = proc_create("condition_value", 0666,
		 asus_batt_cycle_count_dir, &condition_value_fops);
	 struct proc_dir_entry *batt_health_config_proc_file = proc_create("batt_health_config", 0666,
		asus_batt_cycle_count_dir, &batt_health_config_fops);
	 
	 if (!asus_batt_cycle_count_dir)
		 printk("batt_cycle_count_dir create failed!\n");
	 if (!asus_batt_cycle_count_proc_file)
		 printk("batt_cycle_count_proc_file create failed!\n");
	 if(!asus_batt_batt_safety_proc_file)
		 printk("batt_safety_proc_file create failed!\n");
	 if(!asus_batt_batt_safety_csc_proc_file)
		 printk("batt_safety_csc_proc_file create failed!\n");
	 if (!asus_batt_safety_condition_proc_file)
		 printk(" create asus_batt_safety_condition_proc_file failed!\n");
	 if (!batt_health_config_proc_file)
		printk(" create batt_health_config_proc_file failed!\n");
 }
 
 //Write back batt_cyclecount data before restart/shutdown 
 static int reboot_shutdown_prep(struct notifier_block *this,
				   unsigned long event, void *ptr)
 {
	 switch(event) {
	 case SYS_RESTART:
	 case SYS_POWER_OFF:
		 /* Write data back to emmc */
		 write_back_cycle_count_data();
		 break;
	 default:
		 break;
	 }
	 return NOTIFY_DONE;
 }
 /*  Call back function for reboot notifier chain  */
 static struct notifier_block reboot_blk = {
	 .notifier_call  = reboot_shutdown_prep,
 };
 
 static int __init battery_health_safety_init(void)
{
	 g_fg = kmalloc(sizeof(struct fg_dev), GFP_KERNEL);
	 if (!g_fg) {
	 	pr_err("kmalloc(struct fg_dev) failed!\n");
		return -ENOMEM;
	 }
	 
	INIT_DELAYED_WORK(&battery_safety_work, battery_safety_worker); //ASUS_BSP battery safety upgrade
	INIT_DELAYED_WORK(&battery_health_work, battery_health_worker); //battery_health_work
	
	//ASUS_BSP battery safety upgrade +++
	init_battery_safety(g_fg);
	create_batt_cycle_count_proc_file();
	register_reboot_notifier(&reboot_blk);
	schedule_delayed_work(&battery_safety_work, 30 * HZ);
	//ASUS_BSP battery safety upgrade ---
	 
	battery_health_data_reset();
	
	schedule_delayed_work(&battery_health_work, 30 * HZ); //battery_health_work
	return 0;
}

module_init(battery_health_safety_init);

MODULE_DESCRIPTION("Battery Health Safety Driver");
MODULE_LICENSE("GPL");
