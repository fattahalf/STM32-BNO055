/*
 * roshandler.cpp
 *
 *  Created on: Mar 25, 2021
 *      Author: Fattah .Alf
 */

#include <ros.h>
#include <roshandler.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;
sensor_msgs::Imu BNO;
ros::Publisher bno("BNO", &BNO);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void) {
	nh.initNode();
	nh.advertise(bno);
}

void loop(void) {
	BNO.orientation.x = IMU_Quarternion[0];
	BNO.orientation.y = IMU_Quarternion[1];
	BNO.orientation.z = IMU_Quarternion[2];
	BNO.orientation.w = IMU_Quarternion[3];

	bno.publish(&BNO);
	nh.spinOnce();
	HAL_Delay(200);
}
