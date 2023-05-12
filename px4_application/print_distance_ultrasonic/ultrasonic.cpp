//////////////////////////////////////////////////////////////////////////////////
// Company: McMaster University
// Engineer: Ali Abbasi
//
// Create Date: 04/29/2023 10:21:03 AM
// Design Name: fmuk66_ultrasonic_px4
// Target Devices: NXP RDDRONE-FMUK66
// Description: This px4 application prints the distance received from the ultrasonic
// module connected to the autopilot
//////////////////////////////////////////////////////////////////////////////////

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <uORB/topics/distance_sensor.h>

extern "C" __EXPORT int print_distance_ultrasonic_main(int argc, char *argv[]);

int print_distance_ultrasonic_main(int argc, char *argv[])
{
	int distance_sensor_handle;
	distance_sensor_s sensor_data;
	
	distance_sensor_handle = orb_subscribe(ORB_ID(distance_sensor));
	orb_set_interval(distance_sensor_handle, 200);
	
	while(1)
	{
		orb_copy(ORB_ID(distance_sensor), distance_sensor_handle, &sensor_data);
	
		PX4_INFO("Distance = %fm", (double)sensor_data.current_distance);
	
		px4_usleep(200000);
	}
	
	return 0;
}
