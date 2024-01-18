#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/quaternion.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "icm20948.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu imu_msg;
rcl_clock_t clock;
IMU_EN_SENSOR_TYPE enMotionSensorType;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    //get clock
    rcl_time_point_value_t now;
    rcl_clock_get_now(&clock, &now);

    imu_msg.header.stamp.sec = RCL_NS_TO_S(now);
    imu_msg.header.stamp.nanosec = now % 1000000000;
    
    // get imu data
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

    // convert stAngles to degree values because icm20948.c multiplies it with 57.3 we have to normalize it again 
    stAngles.fYaw = stAngles.fYaw/57.3;
    stAngles.fPitch = stAngles.fPitch/57.3;
    stAngles.fRoll = stAngles.fRoll/57.3;
    
    // convert imu data
    double cosYaw = cos(stAngles.fYaw * 0.5);
    double sinYaw = sin(stAngles.fYaw * 0.5);
    double cosPitch = cos(stAngles.fPitch * 0.5);
    double sinPitch = sin(stAngles.fPitch * 0.5);
    double cosRoll = cos(stAngles.fRoll * 0.5);
    double sinRoll = sin(stAngles.fRoll * 0.5);
    double qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    double qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    double qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    double qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

    //normalize quaternion to make sure it only represents an rotation
    double magnitude = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx = qx / magnitude;
    qy = qy / magnitude;
    qz = qz / magnitude;
    qw = qw / magnitude;

    //set imu data
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link");

    //Check: This seems ok. is not visualized in rviz
    imu_msg.angular_velocity.x = stGyroRawData.s16X ;
    imu_msg.angular_velocity.y = stGyroRawData.s16Y ;
    imu_msg.angular_velocity.z = stGyroRawData.s16Z ;
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;
    
    imu_msg.linear_acceleration.x =  stAccelRawData.s16X ;
    imu_msg.linear_acceleration.y = stAccelRawData.s16Y ;
    imu_msg.linear_acceleration.z = stAccelRawData.s16Z ;
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    // publish 
    rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();
    //init clock
    rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    //imu init
    imuInit(&enMotionSensorType);
    
    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

