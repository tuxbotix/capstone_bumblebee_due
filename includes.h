/**
Copyright (c) 2015, A.A. Darshana Sanjeewan Adikari
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Additional notes:
01.06.2015 - Version 1.0 (stable)
**/



#define TRAJ_INDEX_MAX 5
#define LOOK_AHEAD 400
#define DT 20000
#define DEADBAND 60 //when to stop control algo , specified in mm
#define TRAJECTORY_LED 22
#define MOTION_LED 23
#define MOTION_LED 24
#define FREQ 1000000/DT

#define V_MAX 15 //10 mm per 10ms -> 1000ms per sec*
/**
32, 34, 36, 38
31,33,35,37, 39
*/
#define CALIB_ON_SW 37
#define CALIB_ACCL_COMPASS_SELECT 39
#define CALIB_ACCL_GYRO_LED 38
#define CALIB_COMPASS_LED 40
