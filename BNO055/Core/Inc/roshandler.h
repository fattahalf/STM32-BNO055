/*
 * roshandler.h
 *
 *  Created on: Mar 25, 2021
 *      Author: Fattah .Alf
 */

#ifndef INC_ROSHANDLER_H_
#define INC_ROSHANDLER_H_

float IMU_Euler[3], IMU_Quarternion[4];

#ifdef __cplusplus
 extern "C" {
#endif

void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ROSHANDLER_H_ */
