/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: */

#ifndef TURTLEBOT3_MECANUM_H_
#define TURTLEBOT3_MECANUM_H_

#define NAME                             "Mecanum"

#define WHEEL_RADIUS                     0.03           // meter
#define WHEEL_SEPARATION_X               0.1005           // meter 
#define WHEEL_SEPARATION_Y               0.085           // meter 
#define TURNING_RADIUS                   0.080           // meter 
#define ROBOT_RADIUS                     0.105           // meter 
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              0.22       // m/s  
#define MAX_ANGULAR_VELOCITY             2.84      // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //TURTLEBOT3_MECANUM_H_
