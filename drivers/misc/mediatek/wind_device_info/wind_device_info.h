#ifndef _WIND_DEVICE_INFO_H_
#define _WIND_DEVICE_INFO_H_

typedef struct sensor_info {
	char* g_gsensor_name;
	char* g_alsensor_name;
	char* g_msensor_name;
	char* g_hallsensor_name;
}sensor_info_t;

typedef struct module_info {
	unsigned int fwvr;
	unsigned int vendor;
	char ic_name[30];
}module_info_t;

typedef struct lcm_info {
	unsigned int vendor;
	char* ic_name;
}lcm_info_t;



/*
 * This battery_info struct due to different project fill different data.
 *
 * For ASUS project :
 * BAT_Model_Name  				 : 0~7
 * Battery_type					 : 8
 * g_bat_id						 : 9~10
 * Driver_and_Data_Flash_Version : 11~14
 * Image_Version				 : 15~xx
 *
 * For others project may be you just need fill g_bat_id.
 */

typedef struct battery_info {
	char BAT_Model_Name[9];
    char Battery_type[2] ;
	unsigned int g_bat_id;
	unsigned int Driver_and_Data_Flash_Version;
	char Image_Version[50];
	int battery_boot_data[10];
}battery_info_t;

typedef struct wind_device_info {
	sensor_info_t 	sensor_info_name;
	lcm_info_t 		lcm_module_info;
	module_info_t 	ctp_module_info;
	module_info_t 	fp_module_info;
	battery_info_t 	battery_data;
}wind_device_info_t;

#endif