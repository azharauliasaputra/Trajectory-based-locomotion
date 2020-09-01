//
//  SSGA.h
//  Neural Oscillator
//
//  Created by Azhar Aulia Saputra on 1/12/15.
//  Copyright (c) 2015 Azhar Aulia Saputra. All rights reserved.
//

#ifndef __trajectory__
#define __trajectory__


#define PHI (float)57.295779513082320876798154814105
#define phi (float)3.1428571428571428571428571428571

void MotionPatternGeneration(int fMth, int tMax, int timing);
extern float AngleJoint[2][6], AngleRobot, Speed;
extern float Sf;
extern double touch[2];
extern int reset,t;
extern float A, ratio, f;


extern double Dt, valTime;

#endif /* defined(__Neural_Oscillator__SSGA__) */