#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_value.h>

#include <uORB/topics/distance_sensor.h>

#include <uORB/topics/test_motor.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/sensor_combined.h>

#define DC_MOTOR	0
#define SERVO_MOTOR 1

#define AUTONOMOUS	0
#define RC			1

#define SLOW		0
#define MODERATE	1
#define FAST		2

#define DRIVE		0
#define REVERSE		1
#define AUTO		2

#define COLOR_RED	1
#define COLOR_YELLOW 4
#define COLOR_GREEN 2

int decode_drive_mode_switch(float swValue);
double map_stick_value_to_steering(float lrStickValue);
int decode_speed_mode_switch(float swValue, float slowKeyValue, float fastKeyValue);
int decode_gear_mode_switch(float swValue);
double map_stick_value_to_motor_speed(float fbStickValue, int direction, int speed_mode);
void drive_servo(double servo_steering);
void stop_servo();
void drive_dc(double motor_speed);
void stop_dc();
void turn_on_led(int color);

extern "C" __EXPORT int drive_motors_rc_main(int argc, char *argv[]);

int drive_motors_rc_main(int argc, char *argv[])
{
	px4_sleep(4);

	int rc_channels_handle;
	rc_channels_s rc_channels_data;
	rc_channels_handle = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(rc_channels_handle, 200);

	debug_value_s debug_data;
	int debug_handle = orb_subscribe(ORB_ID(debug_value));
	orb_set_interval(debug_handle, 500);

	int sensor_combined_handle;
	sensor_combined_s sensor_data;
	sensor_combined_handle = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_combined_handle, 200);

	int distance_sensor_handle;
	distance_sensor_s distance_sensor_data;
	distance_sensor_handle = orb_subscribe(ORB_ID(distance_sensor));
	orb_set_interval(distance_sensor_handle, 200);

	int drive_mode = RC;

	double servo_steering = 0; // a number between 0 to 1
	double motor_speed = 0; // a number between 0 to 1
	double error_value = 0.05;
	double speed_change = 0;

	int speed_mode = SLOW;
	int gear_mode = AUTO;
	int led_color = COLOR_GREEN;

	while(1)
	{
		// read from RC
		orb_copy(ORB_ID(rc_channels), rc_channels_handle, &rc_channels_data);

		/*Speed -> CH3
		Steering -> CH1
		Speed Mode -> CH5
		Immediate Fast Mode -> CH7
		Immediate Slow Mode -> CH8
		Gear Mode -> CH6
		Drive Mode -> CH9*/

		drive_mode = decode_drive_mode_switch(rc_channels_data.channels[8]);

		if (drive_mode == RC)
		{
			//--------------------Servo Motor--------------------

			servo_steering = map_stick_value_to_steering(rc_channels_data.channels[0]);
			//PX4_INFO("Channel Value: %f, Driver Value: %f", (double)rc_channels_data.channels[0], (double)servo_steering);
			drive_servo(servo_steering);
		
			//--------------------DC Motor--------------------

			speed_mode = decode_speed_mode_switch(rc_channels_data.channels[4], rc_channels_data.channels[7], rc_channels_data.channels[6]);

			// Assigning a color to the LED based on the speed mode
			switch (speed_mode)
			{
				case FAST:
					led_color = COLOR_RED;
					break;
				case MODERATE:
					led_color = COLOR_YELLOW;
					break;
				case SLOW:
					led_color = COLOR_GREEN;
					break;
			}

			turn_on_led(led_color);

			gear_mode = decode_gear_mode_switch(rc_channels_data.channels[5]);

			//PX4_INFO("gear_mode = %d", gear_mode);

			//motor_speed = ((double)rc_channels_data.channels[2]/2 + 0.5);
			motor_speed = map_stick_value_to_motor_speed(rc_channels_data.channels[2], gear_mode, speed_mode);
		
			//PX4_INFO("motorSpeed = %f", motor_speed + error);

			orb_copy(ORB_ID(sensor_combined), sensor_combined_handle, &sensor_data);
	
			speed_change = (double)sensor_data.accelerometer_m_s2[0] * -0.015;

			//PX4_INFO("speed_change = %f", speed_change);

			//PX4_INFO("Channel Value: %f, Driver Value: %f", (double)rc_channels_data.channels[2], (double)motor_speed);
			drive_dc(motor_speed + speed_change + error_value);
		}

		if (drive_mode == AUTONOMOUS)
		{
			//----------------Read from MAVLink------------------
			orb_copy(ORB_ID(debug_value), debug_handle, &debug_data);

			//--------------------Servo Motor--------------------

			// reading the preferred direction
			if(debug_data.ind == 0)
				servo_steering = 1;
			else if(debug_data.ind == 1)
				servo_steering = 0.5;
			else if(debug_data.ind == 2)
				servo_steering = 0;

			drive_servo(servo_steering);
		
			//--------------Read from ultrasonic--------------
			orb_copy(ORB_ID(distance_sensor), distance_sensor_handle, &distance_sensor_data);

			//--------------------DC Motor--------------------

			// reading the distance - received from MAVLink debug message
			// if(debug_data.value < 50 && debug_data.value > 15)					
			// 	motor_speed = 0.52 + error_value;
			// else if(debug_data.value <= 15)
			// 	motor_speed = 0.49 + error_value;
			// else
			// 	motor_speed = 0.54 + error_value;
 
			// reading the distance - received from distance sensor messaage
			if((double)distance_sensor_data.current_distance*100 < 50 && (double)distance_sensor_data.current_distance*100 > 15)					
				motor_speed = 0.52 + error_value;
			else if((double)distance_sensor_data.current_distance*100 <= 15)
				motor_speed = 0.49 + error_value;
			else
				motor_speed = 0.54 + error_value;

			drive_dc(motor_speed);
		}
	}

	// stop servo motor
	PX4_INFO("The steering will be in middle");
	stop_servo();

	// stop dc motor
	PX4_INFO("The motor will be stopped");
	stop_dc();

	return 0;
}

// Mapping from the value of a 2-toggle switch to the drive mode
int decode_drive_mode_switch(float swValue)
{
	if(swValue <= -0.951020f)
		return RC;
	else if(swValue >= 1)
		return AUTONOMOUS;
	return RC;
}

// generating the servo angle between 0 to 1
double map_stick_value_to_steering(float lrStickValue)
{
	return ((double)-lrStickValue/2 + 0.5);
}

// Mapping from the value of a 3-toggle switch and two keys to a speed mode
int decode_speed_mode_switch(float swValue, float slowKeyValue, float fastKeyValue)
{
	if (slowKeyValue >= 1.0f)
		return SLOW;
	else if (fastKeyValue >= 1.0f)
		return FAST;
	else
	{
		if(swValue <= -0.951020f)
			return SLOW;
		else if(swValue >= 0.008160f && swValue <= 0.008165f)
			return MODERATE;
		else if(swValue >= 1)
			return FAST;
	}
	return SLOW;
}

// Mapping from the value of a 3-toggle switch to the gear mode
int decode_gear_mode_switch(float swValue)
{
	if(swValue <= -0.951020f)
		return DRIVE;
	else if(swValue >= 0.008160f && swValue <= 0.008165f)
		return REVERSE;
	else if(swValue >= 1)
		return AUTO;
	return SLOW;
}

// Generating the motor speed between 0 to 1
// if in auto mode - mapping to [0.4:0.6]:
//		if in slow mode: linePoints {(-0.951020, 0.45), (1, 0.55)}
//		if in fast mode: : linePoints {(-0.951020, 0.35), (1, 0.65)}
// if in drive mode - mapping to [0.5:1]:
//		if in slow mode: linePoints {(-0.951020, 0.5), (1, 0.6)}
//		if in moderate mode: linePoints {(-0.951020, 0.5), (1, 0.75)}
//		if in fast mode: linePoints {(-0.951020, 0.5), (1, 1)}
// if in reverse mode - mapping to [0.5:0]:
//		if in slow mode: linePoints {(-0.951020, 0.5), (1, 0.4)}
//		if in moderate mode: linePoints {(-0.951020, 0.5), (1, 0.25)}
//		if in fast mode: linePoints {(-0.951020, 0.5), (1, 0)}
double map_stick_value_to_motor_speed(float fbStickValue, int gear_mode, int speed_mode)
{
	double motor_speed = 0.5;

	if (gear_mode == AUTO)
	{
		switch (speed_mode)
		{
			case SLOW:
				//motor_speed = fbStickValue * 0.051255f + 0.498744f;
				motor_speed = fbStickValue * 0.102510f + 0.497489f;
				break;
			case FAST:
				motor_speed = fbStickValue * 0.153765f + 0.496234f;
				break;
		}
		return motor_speed;
	}

	else
	{
		if (fbStickValue < 0.008163f)
		{
			return 0.5;
		}

		if (gear_mode == DRIVE)
		{
			switch (speed_mode)
			{
			case SLOW:
				motor_speed = fbStickValue * 0.051255f + 0.548744f;
				break;
			case MODERATE:
				motor_speed = fbStickValue * 0.128138f + 0.621861f;
				break;
			case FAST:
				motor_speed = fbStickValue * 0.256276f + 0.743723f;
				break;
			}
			return motor_speed;
		}

		else if (gear_mode == REVERSE)
		{
			switch (speed_mode)
			{
			case SLOW:
				motor_speed = fbStickValue * -0.051255f + 0.451255f;
				break;
			case MODERATE:
				motor_speed = fbStickValue * -0.128138f + 0.378138f;
				break;
			case FAST:
				motor_speed = fbStickValue * -0.256276f + 0.256276f;
				break;
			}
			return motor_speed;
		}
	}
	return 0.5;
}

void drive_servo(double servo_steering)
{
	test_motor_s test_motor_servo;
	uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

	test_motor_servo.timestamp = hrt_absolute_time();
	test_motor_servo.motor_number = SERVO_MOTOR;
	test_motor_servo.value = servo_steering;
	test_motor_servo.action = test_motor_s::ACTION_RUN;
	test_motor_servo.driver_instance = 0;
	test_motor_servo.timeout_ms = 0;
	
	test_motor_pub.publish(test_motor_servo);
}

void stop_servo()
{
	test_motor_s test_motor_servo;
	uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

	test_motor_servo.timestamp = hrt_absolute_time();
	test_motor_servo.motor_number = SERVO_MOTOR;
	test_motor_servo.value = 0.5;
	test_motor_servo.driver_instance = 0;
	test_motor_servo.timeout_ms = 0;

	test_motor_pub.publish(test_motor_servo);
}

void drive_dc(double motor_speed)
{
	test_motor_s test_motor_dc;
	uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

	test_motor_dc.timestamp = hrt_absolute_time();
	test_motor_dc.motor_number = DC_MOTOR;
	test_motor_dc.value = motor_speed;
	test_motor_dc.action = test_motor_s::ACTION_RUN;
	test_motor_dc.driver_instance = 0;
	test_motor_dc.timeout_ms = 0;

	test_motor_pub.publish(test_motor_dc);
}

void stop_dc()
{
	test_motor_s test_motor_dc;
	uORB::Publication<test_motor_s> test_motor_pub(ORB_ID(test_motor));

	test_motor_dc.timestamp = hrt_absolute_time();
	test_motor_dc.motor_number = DC_MOTOR;
	test_motor_dc.value = 0.5;
	test_motor_dc.driver_instance = 0;
	test_motor_dc.timeout_ms = 0;

	test_motor_pub.publish(test_motor_dc);
}

void turn_on_led(int color)
{
	led_control_s led_control;
	uORB::Publication<led_control_s> led_control_pub(ORB_ID(led_control));

	led_control.timestamp = hrt_absolute_time();
	led_control.led_mask = 0xFF;
	led_control.color = color;
	led_control.mode = led_control_s::MODE_ON;
	led_control.num_blinks = 0;
	led_control.priority = led_control_s::MAX_PRIORITY;
	led_control._padding0[0] = 0;
	led_control._padding0[1] = 0;
	led_control._padding0[2] = 0;

	led_control_pub.publish(led_control);
}