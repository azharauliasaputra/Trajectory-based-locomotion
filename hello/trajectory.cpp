
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "trajectory.h"
#include "main.h"

#define Frame1 76
#define Frame2 76
#define Frame3 152

float Resultan0, Resultan1, Resultan2, sC, aX, aY, aZ, Alfa;
unsigned char Injak = 1,Geser;
float Angle[6];
float UpFoot, theta=0, LengthFoot, BesideFoot;

float LengthX1,LengthY1,LengthZ1,Heading1,LengthX2,LengthY2,LengthZ2,Heading2, LengthTangan, BuffSudut[3],BuffSudut1[3];
float xFoot1, yFoot1, zFoot1, xFoot2, yFoot2, zFoot2, hFoot1, hFoot2,
LxFoot1, LyFoot1, LzFoot1, LxFoot2, LyFoot2, LzFoot2, LhFoot1, LhFoot2,
mTangan;
/**************************************************/
// b = jarak langkah awal (engkel kanan dan engkel kiri)
// j = jarak langkah akhir (engkel kanan dan engkel kiri)
// h = tinggi;
// Xa = kordinat engkel dalam sumbu-x
// t = waktu saat ini
// t0 = waktu setengah melangkah
// t1 = waktu awal melangkah
// t2 = waktu akhir melangkah
float AngleJoint[2][6];
float b,B, f, h0R=0, h1R=0, h0L=0, h1L=0, XaL, XaR, YaL, YaR,Xp,ZaL,ZaR,Zp,YB=0, awalKn, awalKr, akhirKn, akhirKr;
int t0,t1,t2,step;
float a0,a1,a2,a3,ap0,ap1,ap2,ap3,alf_x, alf_z, alf_zData, SyData,tInc,tInc2,A1=0,A2=100, Hdef = 140, lastA = 0, AA=0, LastL, LastR;
int VektorTrajektori1, A_Disturb, A_Motion, Count_A;
float dSf;

/**********************************************************************************************/
int dMtn = 0, handdataX1[20], handdataX2[20], handdataY1[20], handdataY2[20], handdataZ1[20], handdataZ2[20]
, legdataX1[20], legdataX2[20], legdataY1[20], legdataY2[20], legdataZ1[20], legdataZ2[20]
, legdataH1[20], legdataH2[20], dataTime[20], alBody[20];
float sum_Ta=0, sum_Tb=0;
int dTime, maxStep = 0, tstep;
float xInp1,xInp2,yInp1,yInp2,zInp1,zInp2,hInp1,hInp2;
float xHand1,xHand2,yHand1,yHand2,zHand1,zHand2;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
float sA = 0, sB = 0,tT,tT1,outBezier, Tbez;
float arx,ary;
float p0x,p1x,p2x,p3x,p0y,p1y,p2y,p3y,p0z,p1z,p2z,p3z, SuA[3]={0,0,0}, SuB[3]={0,0,0};
float p0x2,p1x2,p2x2,p3x2,p0y2,p1y2,p2y2,p3y2,p0z2,p1z2,p2z2,p3z2, SuA2[3]={0,0,0}, SuB2[3]={0,0,0};
float Lp0x,Lp1x,Lp2x,Lp3x,Lp0y,Lp1y,Lp2y,Lp3y,Lp0z,Lp1z,Lp2z,Lp3z, LSuA[3]={0,0,0}, LSuB[3]={0,0,0};
float Lp0x2,Lp1x2,Lp2x2,Lp3x2,Lp0y2,Lp1y2,Lp2y2,Lp3y2,Lp0z2,Lp1z2,Lp2z2,Lp3z2, LSuA2[3]={0,0,0}, LSuB2[3]={0,0,0};
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
int MassHead, EngkelX_1, ReadyChange;
float Yg, dataYg, YaRt, YaRx, YaRz, YaLt, YaLx, YaLz;
int fMth, lastMotion = -1,LastDataAction = 0;
float LHeading0, LHeading1,RHeading0, RHeading1, dHeading,miringGakBiasa1,miringGakBiasa2;
int SdMotion, lastSdMotion = -1, dfMth, Rstep, nStep, kStep, aShoot, lastErr_Y;
int outCTPy,outCTPx,GakBiasa = 0;
float AngleBody = 0, aBody = 0, aRobot = 0, lastAngleBody = 0,ratio,A,h,Sf = -5,Sy,Speed;
float AngleJointServo[12] = {
	0,0,0,0,0,0,0,0,0,0,0,0
};
float XmoveL=0, YmoveL=0, ZmoveL=0, lastXLmove=0,lastXL,lastXR,lastXp, XmoveR=0, YmoveR=0, ZmoveR=0, lastXRmove=0, Xpelvis =0, lastXpelvis=0;
char buffer[100],buffer1[100];
/**********************************************************************************************/

float *dat;
float orL2[3], orL3[3];
float OrLink1[3]={0,56,0},cLink1[3],OrLink2[3]={0,56,0},cLink2[3];
float Origin1[3]={0,112,0},Origin2[3]={0,112,0};
/**********************************************************************************************/
short int GoalPosition[20];
short int GoalSpeed[12]={10,10,10,10,10,10,10,10,10,10,10};
/**/
short int PosDefault[18]={
	664, //id 1
	356,//id 2
	514, //id 3
	502, //id 4
	509-40, //id 5
	511+40, //id 6
	525-90, //id 7
	500+90, //id 8
	507+50, //id 9
	512-50, //id 10
	514, //id 11
	504, //id 12
};
short int DefaultServo[18]={
	664, //id 1
	356, //id 2
	514, //id 3
	502, //id 4
	509-40, //id 5
	511+40, //id 6
	525-90, //id 7
	500+90, //id 8
	507+50, //id 9
	512-50, //id 10
	514, //id 11
	504, //id 12
	450, //id 13 
	573, //id 14
	556, //id 15
	469, //id 16
	458, //id 17
	579, //id 18
}; 
int aaa;
float AngleRobot= 100,AngleSamping= 0;

void InversKinematic(float x, float y, float z, int Heading){
		
	Angle[0] = (float)Heading;
	Resultan0 = sqrt(z*z + x*x);
	
	Alfa = 	(atan2(z,x) * PHI) - Heading;
	aZ = (sin(Alfa / PHI) * Resultan0);
	aX = (cos(Alfa / PHI) * Resultan0);
	
	Angle[1] = atan2(aZ,y) * PHI;
	Resultan1 = sqrt(aZ*aZ + y*y);
	Resultan2 = sqrt(Resultan1*Resultan1 + aX*aX);
	if(Resultan2 >= (Frame1+Frame2))Resultan2 = Frame1 + Frame2;
	sC = acos((float)((float)(Frame1*Frame1)+(float)(Frame1*Frame1)-(float)(Resultan2*Resultan2))/(float)(2 * Frame1 * Frame2)) * PHI;
	sA = asin((float)aX/Resultan2) * PHI;
	sB = asin((float)((float)Frame2 * sin(sC / PHI))/Resultan2) * PHI;
	Angle[2] = 0 - ( sA + sB );
	Angle[3] = 180 - sC;
	Angle[4] = (Angle[2] + Angle[3]);
	Angle[5] = -Angle[1];
}
void InversKinematicLeg(float x, float y, float z, int Heading){
		float Resultan0, Resultan1, Resultan2, sA, sB, sC, aX, aY, aZ, Alfa;
		
		Angle[0] = Heading;
        Resultan0 = sqrt(z*z + x*x);
        if(Resultan0 > (Frame1+Frame2))Resultan0 = Frame1+Frame2;
	
		Alfa = 	(atan2(z,x) * PHI) - Heading;
		aZ = z + (sin(Alfa / PHI) * Resultan0);
		aX = x - (cos(Alfa / PHI) * Resultan0);
	
		Angle[1] = atan2(aZ,y) * PHI;
		
        Resultan1 = sqrt(aZ*aZ + y*y);
        if(Resultan1 > (Frame1+Frame2))Resultan1 = Frame1+Frame2;
        Resultan2 = sqrt(Resultan1*Resultan1 + aX*aX);
        if(Resultan2 > (Frame1+Frame2))Resultan2 = Frame1+Frame2;
	
		sC = acos(((Frame1*Frame1)+(Frame1*Frame1)-(Resultan2*Resultan2))/(2 * Frame1 * Frame2)) * PHI;
		sA = asin(x/Resultan2) * PHI;
		sB = asin((Frame2 * sin(sC / PHI))/Resultan2) * PHI;
	
		Angle[2] = 0 - ( sA + sB );
		Angle[3] = 180 - sC;
		Angle[4] = (Angle[2] + Angle[3]);
		Angle[5] = -Angle[1];
}
void CoordinateServo(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2){
    AngleBody = 1.2;
    
    InversKinematicLeg(x1, y1, z1, Heading1);
				AngleJointServo[0] = Angle[0];
				if(y1 > y2 && Angle[1] >= 0){
						AngleJointServo[2] = (Angle[1]  - AngleBody * sin(Angle[0] / PHI)) ;
				}else if(y1 < y2 && Angle[1] >= 0){
						AngleJointServo[2] = (Angle[1]  - AngleBody * sin(Angle[0] / PHI)) ;
				}else{
						AngleJointServo[2] = (Angle[1]  - AngleBody * sin(Angle[0] / PHI));
				}
				
				if(y1>y2){
						AngleJointServo[8] = (Angle[4] + AngleRobot);//(y1 - y2)/3);
				}else{
						AngleJointServo[8] = Angle[4] + AngleRobot;
				}
				AngleJointServo[4] = Angle[2] - AngleBody * cos(Angle[0] / PHI);
				AngleJointServo[6] = Angle[3];
				AngleJointServo[10] = Angle[5];
				
    AngleJoint[0][0] = AngleJointServo[0]/PHI;
    AngleJoint[0][1] = -AngleJointServo[2]/PHI;
    AngleJoint[0][2] = AngleJointServo[4]/PHI;
    AngleJoint[0][3] = AngleJointServo[6]/PHI;
    AngleJoint[0][4] = -AngleJointServo[8]/PHI;
    AngleJoint[0][5] = -AngleJointServo[10]/PHI;
    
	InversKinematicLeg(x2, y2, z2, Heading2);
				AngleJointServo[1] = Angle[0];
				if(y1 < y2 && Angle[1] <= 0){ 
						AngleJointServo[3] = (Angle[1] - AngleBody * sin(Angle[0] / PHI)) ;
				}else if(y1 > y2 && Angle[1] <= 0){ 
						AngleJointServo[3] = (Angle[1] - AngleBody * sin(Angle[0] / PHI)) ;
				}else{ 
						AngleJointServo[3] = Angle[1]  - AngleBody * sin(Angle[0] / PHI);
				}
				if(y2>y1){
						AngleJointServo[9] = (Angle[4] + AngleRobot);//(y2 - y1)/3);
				}else{
						AngleJointServo[9] = Angle[4] + AngleRobot;
				}
				AngleJointServo[5] = Angle[2]  - AngleBody * cos(Angle[0] / PHI);
				AngleJointServo[7] = Angle[3];
				AngleJointServo[11] = (Angle[5]);
    
    AngleJoint[1][0] = AngleJointServo[1]/PHI;
    AngleJoint[1][1] = -AngleJointServo[3]/PHI;
    AngleJoint[1][2] = AngleJointServo[5]/PHI;
    AngleJoint[1][3] = AngleJointServo[7]/PHI;
    AngleJoint[1][4] = -AngleJointServo[9]/PHI;
    AngleJoint[1][5] = -AngleJointServo[11]/PHI;
    
}
void CalculateServoValue(){
	int i;
	for(i=0;i<12;i++){
			if( i == 2 || i == 2 || i == 3 || i == 5  || i == 6 || i == 9){
					GoalPosition[i] = (int)(DefaultServo[i] - (int)((AngleJointServo[i]*1024)/300));
			}else{
					GoalPosition[i] = (int)(DefaultServo[i] + (int)((AngleJointServo[i]*1024)/300));
			}
	}
}
void ChangeDataMotion(int fMth){

	if(fMth == 0){
        Sf = 0; A = 0; SyData = 0; h = 0;alf_x = 0;alf_zData = 0;YB = 0;dHeading = 0; aBody = 0; aRobot = 0;
	}else if(fMth == 100){
		Sf = 0; A = 0; SyData = 6; h = 18;alf_x = 1;alf_zData = 10;YB = 0;dHeading = 0; aBody = 0; aRobot = 0; Speed = 0;
	}else if(fMth == 101){
		dHeading = 7; aBody = 0;
	}else if(fMth == 102){
		dHeading = -7; aBody = 0;
	}else if(fMth == 120){
		Sf = 0; A = 30; ratio = 0.6; SyData = 5; h = 24;alf_x = 4;alf_zData = 1;YB = 0; aBody = 0;
	}else if(fMth == 200){
        Sf = 20; A = 0; SyData = 5; h = 15;alf_x = 1;alf_zData = 10;YB = 0;dHeading = 0; aBody = 2; aRobot = 0;
	}else if(fMth == 201){
		Sf = 20; A = 0; SyData = 4; h = 15;alf_x = 1;alf_zData = 10;YB = 0;dHeading = 6; aBody = 7;
	}else if(fMth == 202){
		Sf = 20; A = 0; SyData = 4; h = 15;alf_x = 1;alf_zData = 10;YB = 0;dHeading = -6; aBody = 7;
	}else if(fMth == 300){
		Sf = 45; A = 0; SyData = 4; h = 15;alf_x = 1;alf_zData = 10;YB = 0;dHeading = 0; aBody = 7;
	}
//    aBody = 0.1;
    
}
void MotionControl(int fMth){
		SdMotion = fMth%100;
		lastSdMotion = lastMotion%100;
		if(fMth != 0){
						if(lastMotion < fMth){
								dfMth = lastMotion + (SdMotion - lastSdMotion);
								if(lastMotion <= fMth - 100) fMth = dfMth + 100;
								else fMth = dfMth;
						}else if(lastMotion > fMth){
								dfMth = lastMotion + (SdMotion - lastSdMotion);
								if(lastMotion >= fMth + 100) fMth = dfMth - 100;
								else fMth = dfMth;
						}
				if(fMth == 0){
						if(lastMotion < fMth){
								if(lastMotion <= fMth - 100) fMth = fMth + 100;
								else fMth = fMth;
						}else if(lastMotion > fMth){
								if(lastMotion >= fMth + 100) fMth = fMth - 100;
								else fMth = fMth;
						}
				}
		}else{
						if(lastMotion < fMth){
								dfMth = lastMotion + (SdMotion - lastSdMotion);
								if(lastMotion <= fMth - 100) fMth = dfMth + 100;
								else fMth = dfMth;
						}else if(lastMotion > fMth){
								dfMth = lastMotion + (SdMotion - lastSdMotion);
								if(lastMotion >= fMth + 100) fMth = dfMth - 100;
								else fMth = dfMth;
						}
		}
		if(lastMotion != fMth){
				ChangeDataMotion(fMth);
		}
		lastMotion = fMth;
		//sprintf(cccc,"%d\t%d\t\n",lastfMth,fMth);
		//USART_puts(USART6,cccc);
}
int dataDSP,FSR1_z,FSR2_z;

int  con = 0;
float LastHand1_1=0, LastHand2_1=0, LastHand1_2=0, LastHand2_2=0;
void TrajectoryHand(int t2, int t1, int t, char step){
	float tIn,tIn2, HandZ1, HandZ2, HandY;
	tIn = ((float)t - (float)t1)/((float)t2 - (float)t1);
	if(step == 2){
		if(t<=t0){
			tIn2 = ((float)t - (float)t1)/((float)t0 - (float)t1);
			HandZ2 = LastHand2_2 - (LastHand2_2-Sy)*sin((phi/2)*tIn2);
			LastHand2_1 = HandZ2;
			HandZ1 = LastHand1_2;
			LastHand1_1 = HandZ1;
		}else{
			tIn2 = ((float)t - (float)t0)/((float)t2 - (float)t0);
			HandZ2 = LastHand2_1 - (LastHand2_1+Sy)*sin((phi/2)*tIn2);
			LastHand2_2 = HandZ2;
			HandZ1 = LastHand1_1;
			LastHand1_2 = HandZ1;
		}			
		HandY = sin(phi*tIn)*Sf;

	}else if(step == 1){
		
		if(t<=t0){	
			tIn2 = ((float)t - (float)t1)/((float)t0 - (float)t1);
			HandZ1 = LastHand1_2 - (LastHand1_2-Sy)*sin((phi/2)*tIn2)*sin((phi/2)*tIn2);
			LastHand1_1 = HandZ1;
			HandZ2 = LastHand2_2;
			LastHand2_1 = HandZ2;
		}else{
			tIn2 = ((float)t - (float)t0)/((float)t2 - (float)t0);
			HandZ1 = LastHand1_1 - (LastHand1_1+Sy)*sin((phi/2)*tIn2)*sin((phi/2)*tIn2);
			LastHand1_2 = HandZ1;
			HandZ2 = LastHand2_1;
			LastHand2_2 = HandZ2;
		}
		HandY = -sin(phi*tIn)*Sf;
	}
	
	//printf("%d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",-40+step*10,XmoveR,xFoot2,yFoot2,LastHand1_2,LastHand1_1,HandZ1);
	GoalPosition[12] = DefaultServo[12] + 1*HandY;
	GoalPosition[13] = DefaultServo[13] + 1*HandY;
	GoalPosition[14] = DefaultServo[14] - 1*HandZ2;
	GoalPosition[15] = DefaultServo[15] - 1*HandZ1;
	GoalPosition[16] = DefaultServo[16] - 1*HandZ2;
	GoalPosition[17] = DefaultServo[17] - 1*HandZ1;
}
void TrajectoryWalking(int fMth, int t2, int t1, char step){
	static float dataMiring=0;
	static int timing = t2-t1;
	int DSP = 3;
    
    
    dataDSP = 0;
	DSP = dataDSP;
	float tInc3;
	//dataMiring = 15 * sin((float)(err_y*2)/PHI);
	//alf_x = 1;
	a0 = -B/2;
    a1 = alf_x*B;
    a2 = f*3/2 + B - 2*alf_x*B - f*alf_x;
    a3 = alf_x*B + alf_x*f - f - (B/2);
	tInc = (float)(t - (t1+DSP))/(float)((t2-DSP) - (t1+DSP));
	tInc3 = (float)(t - (t1))/(float)((t2) - (t1));
	
	t0 = (t2 + t1)/2;
	//Trajectory Engkel;
	if(step == 2){
		// Kaki knn melangkah/mengayun;
		
		/* Coronal Position (Tampak Depan - Sumbu Z) */
		
		ZaL = (((akhirKr/2)*(1 - ratio) - awalKr)/2)*(1 + cos(phi*((float)(t + (t2 - t1) - t1)/(float)(t2 - t1))))+ awalKr;
		ZaR = (((akhirKn/2)*(1 + ratio) - awalKn)/2)*(1 + cos(phi*((float)(t + (t2 - t1)- t1)/(float)(t2 - t1))))+ awalKn ;
		
		alf_z = alf_zData;
		Sy = SyData;
		/* Sagital Position (Tampak samping - Sumbu X) */
		
		Xp = a3*pow(tInc3,3)+ a2*pow(tInc3,2)+ a1*tInc3+ a0;
		if(Injak == 0){
			if(t > (t1+DSP) && t <= (t2-DSP)){
				if(t>t0)XaR = (b + f)*((tInc) - (1 / 2*phi)*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
				else XaR = (b + f)*((tInc) - (1 / 2*phi)*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
				YaRt = ((2 - (cos(2 * phi * tInc)+1))* h/2) + (h0R + ((h1R - h0R)*tInc));
				YaRx = YaRt*cos(AngleRobot/ PHI) + XaR *sin(AngleRobot/ PHI) - (lastXR-lastXL)*sin(AngleRobot/ PHI);
				YaRz = YaRt*cos(AngleSamping/ PHI) + (ZaR + awalKr + awalKn)*sin(AngleSamping/ PHI);
				YaR = YaRx;

			}else if (t <= (t1+DSP)){
				tInc = 0;
				XaR = (b + f)*((tInc) - (1 / (2*phi))*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
			}else if ( t > (t2-DSP)){
				tInc = 1;
				XaR = (b + f)*((tInc) - (1 / (2*phi))*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
			}
		}
        if(GakBiasa == 1){
            YaR -= 1;
            YaL += 0.4;
        }else{
            YaL = h0L + ((h1L - h0L)*tInc);
            if(h0L > 0){
                if(YaL < 0)YaL = 0;
            }else{
                if(YaL > 0)YaL = 0;
            }
        }
        if(YaL > 10)YaL = 10;
        else if(YaL < -20)YaL = -20;
        if(YaR > 30)YaR = 30;
        else if(YaR < -20)YaR = -20;
        
        XaL = 0;
				
		hFoot1 = LHeading0 + (LHeading1 - LHeading0)*tInc;
		hFoot2 = RHeading0 + (RHeading1 - RHeading0)*tInc;
		//YB = 5;
		if(t<=t0){
			//YB = YB;
			con = 1;
			ap0 = YB;
			ap1 = -alf_z;
			ap2 = 2*alf_z - 3*Sy - 3*YB;
			ap3 = 2*Sy - alf_z + 2*YB;
			tInc2 = (float)(t - t1)/(float)(t0 - t1);
			Zp = ap3*pow(tInc2,3)+ ap2*pow(tInc2,2)+ ap1*tInc2+ ap0;
		}else{
			con = 2;
			YB = 0;
			ap0 = -Sy;
			ap1 = 0;
			ap2 = 3*Sy -alf_z + 3*(-YB);
			ap3 = alf_z - 2*Sy - 2*(-YB);
			tInc2 = (float)(t - t0)/(float)(t2 - t0);
			Zp = ap3*pow(tInc2,3)+ ap2*pow(tInc2,2)+ ap1*tInc2+ ap0;
		}
	}else{
		// Kaki kr melangkah/mengayun;
		/* Sagital Position (Tampak samping - Sumbu X) */
		Xp = a3*pow(tInc3,3)+ a2*pow(tInc3,2)+ a1*tInc3+ a0;
		if(Injak == 0){
			if(t > (t1+DSP) && t <= (t2-DSP)){
				if(t>t0)XaL = (b + f)*((tInc) - (1 / 2*phi)*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
				else XaL = (b + f)*((tInc) - (1 /2*phi)*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
				YaLt = ((2 - (cos(2 * phi * tInc)+1))* h/2) + (h0L + ((h1L - h0L)*tInc));
				YaLx = YaLt*cos(AngleRobot/ PHI) + XaL *sin(AngleRobot/ PHI) - (lastXL-lastXR)*sin(AngleRobot/ PHI);
				YaL = YaLx;

			}else if (t <= (t1+DSP)){
				tInc = 0;
				XaL = (b + f)*((tInc) - (1 / (2*phi))*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
			}else if ( t > (t2-DSP)){
				tInc = 1;
				XaL = (b + f)*((tInc) - (1 / (2*phi))*sin((2*phi)*(tInc)))*cos(AngleRobot/ PHI) - b;
			}	
		}
        if(GakBiasa == 1){
            YaL -= 1;
            YaR += 0.4;
        }
        else{
            YaR = (h0R + ((h1R - h0R)*tInc));
            if(h0R > 0){
                if(YaR < 0)YaR = 0;
            }else{
                if(YaR > 0)YaR = 0;
            }
        }
        if(YaL > 30)YaL = 30;
        else if(YaL < -20)YaL = -20;
        if(YaR > 10)YaR = 10;
        else if(YaR < -20)YaR = -20;
        
		XaR = 0;
			
		hFoot1 = LHeading0 + (LHeading1 - LHeading0)*tInc;
		hFoot2 = RHeading0 + (RHeading1 - RHeading0)*tInc;
			
		/* Coronal Position (Tampak Depan - Sumbu Z) */
		ZaL = (((akhirKr/2)*(1 - ratio) - awalKr)/2)*(1 - cos(phi*((float)(t - t1)/(float)(t2 - t1))))+ awalKr ;
		ZaR = (((akhirKn/2)*(1 + ratio) - awalKn)/2)*(1 - cos(phi*((float)(t - t1)/(float)(t2 - t1))))+ awalKn ;
				
		alf_z = -alf_zData;
		Sy = -SyData;
		//YB = -10;
		if(t<=t0){
			//YB = -YB;
			con = 3;
			ap0 = YB;
			ap1 = -alf_z;
			ap2 = 2*alf_z - 3*Sy - 3*(YB);
			ap3 = 2*Sy - alf_z + 2*(YB);
			tInc2 = (float)(t - t1)/(float)(t0 - t1);
			Zp = ap3*pow(tInc2,3)+ ap2*pow(tInc2,2)+ ap1*tInc2+ ap0;
		}else{
			con = 4;
			YB = 0;
			ap0 = -Sy;
			ap1 = 0;
			ap2 = 3*Sy -alf_z + 3*YB;
			ap3 = alf_z - 2*Sy - 2*YB;
			tInc2 = (float)(t - t0)/(float)(t2 - t0);
			Zp = ap3*pow(tInc2,3)+ ap2*pow(tInc2,2)+ ap1*tInc2+ ap0;
		}
				
	}
		
	xFoot1 = XaL - Xp ;
	xFoot2 = XaR - Xp ;
	zFoot1 = ZaL - Zp ;
	zFoot2 = ZaR - Zp ;
	yFoot1 = YaL;
	yFoot2 = YaR;
    
    //sprintf(buffer,"%d\t%d\t%.2f\n",t,step,zFoot1);
    printf("%d\t%d\t%.2f\n",t,step,yFoot2);
}

double Dt=0, valTime=0;

void MotionPatternGeneration(int fMth, int tMax, int timing){
    
    t = (int)Dt;
    
    if(reset == 1)lastMotion = 0;
	if(t == 0)t2 = timing;
	if(t > -1){
		ReadyChange = 0;
		LastDataAction = 0;
		int a = rand()%13;
		a = 15;
        
        if(AngleBody < aBody)    AngleBody += 0.05;
        else AngleBody -= 0.05;
        
        if(AngleRobot < aRobot)    AngleRobot += 0.05;
        else AngleRobot -= 0.05;
                
        AngleRobot = -roll*20;
        if(Sf < 0) AngleRobot = 0;
        
        AngleSamping = pitch*20;
        
        if(t > (t2-a) && t <= (t2+1) && Injak == 0){
			if(step == 1){
				if(touch[0] == 1 ){// Kaki kiri melangkah/mengayun;
					Injak = 1; Dt = t2+1;
				}else{
					Injak = 0;
				}
			}else if(step == 2){// Kaki kanan melangkah/mengayun;
				if(touch[1] == 1 ){
					Injak = 1; Dt = t2+1;
				}else{
					Injak = 0;
				}
			}
			//Injak = 1;t=t2+1;
		}
        t = (int)Dt;
//        f = Sf;
		if(t > t2 && Injak == 1){
			MotionControl(fMth);
			LHeading0 = LHeading1;
			RHeading0 = RHeading1;
			
			if(Injak == 1){
				if(step == 1){
					ReadyChange = 1;
					b = XaL;
					B = xFoot1 + xFoot1;
					h0L = YaL; h1L = 0;
					h0R = YaR; h1R = 0;
					LHeading1 = -dHeading;
					RHeading1 = dHeading;	
					YB = Zp;
				}
				else if(step == 2){
					b = XaR;
					B = xFoot2 + xFoot2;
					h0R = YaR; h1R = 0;
					h0L = YaL; h1L = 0;
					LHeading1 = dHeading;
					RHeading1 = -dHeading;
					YB = Zp;
				}
			}				
            f = Sf;
//            if(Sf > 0 || Sf < 0){
//                SyData = 3; h = 18+abs(f)/5 ;alf_x = 1;alf_zData = 10;YB = 0;
//            }else{
//                SyData = 0; h = 0;alf_x = 0;alf_zData = 0;YB = 0;dHeading = 0;
//            }
			t1 = t2;
			t2 += timing;
			step ++;
			if(step > 2)step = 1;
			Injak  =0;
							
			awalKn = ZaR;
			awalKr = ZaL;
								
			if(step == 1){
				if(A == 0){
					akhirKn = 0;										akhirKr = 0;
				}else if(A < 0){
					akhirKn = 0;										akhirKr = 0;
				}else{
					akhirKn = A*2;										akhirKr = -A*2;
				}
			}else{
				if(A == 0){
					akhirKn = 0;        								akhirKr = 0;
				}else if(A > 0){
					akhirKn = 0;        								akhirKr = 0;
				}else{
					akhirKn = -A*2; 									akhirKr = A*2;
				}
			}
			Count_A --;
			GakBiasa = 0;
			lastXLmove = XmoveL;
			lastXRmove = XmoveR;
			lastXpelvis = Xpelvis;
			lastXL = xFoot1;
			lastXR = xFoot2;
			lastXp = Xp;
			
		}
		else if(t > t2 && Injak == 0){
			Dt = t2;
            t = (int)Dt;
			GakBiasa = 1;
		}
			
		TrajectoryWalking(0,t2,t1,step);
		TrajectoryHand(t2,t1,t,step);
		// convert to the lenght of movement.
		if(step == 1){
			XmoveL = lastXLmove + (xFoot1 - lastXL) - (xFoot2 - lastXR);
			//XmoveL = 0;
		}else{
			XmoveR = lastXRmove + (xFoot2 - lastXR) - (xFoot1 - lastXL);
		}
		Xpelvis = XmoveL - xFoot1;
		 //+ Sf/27;
		//AngleBody = 0;
		CoordinateServo(xFoot1,Hdef-yFoot1, zFoot1, hFoot1, xFoot2, Hdef-yFoot2, zFoot2, hFoot2);	
		//CalculateServoValue();
		//forwardKinematic();
			
		
		if(ReadyChange == 0){		
/****************/
			legdataX1[0] = (int)(xFoot1+15 - (Sf/8));
			legdataY1[0] = (int)(yFoot1);
			legdataZ1[0] = (int)(zFoot1);
			legdataH1[0] = (int)(hFoot1);
			legdataX2[0] = (int)(xFoot2+15- (Sf/8));
			legdataY2[0] = (int)(yFoot2);
			legdataZ2[0] = (int)(zFoot2);
			legdataH2[0] = (int)(hFoot2);
					
			alBody[0] = (int)AngleBody;
			dataTime[0] = 1;
				
			LxFoot1 = xFoot1;
			LyFoot1 = yFoot1;
			LzFoot1 = zFoot1;
			LxFoot2 = xFoot2;
			LyFoot2 = yFoot2;
			LzFoot2 = zFoot2;
/**************/
		}
        Dt ++;
        Dt += valTime;
        
        if(Dt > (t2+1)) Dt = (t2+1);
		t = (int)(Dt);
	}
}
int itServo;
int Torque[12];
float output_Y = 0;
float TrajectoryPendulum(int tMax);

void runTrajectory(void){	//mainä÷êî
  
   /* Initialize */
   
   ///////////////////////////////////////////////////////
   int mth = 0;
	//dataDSP = 8;
   ///////////////////////////////////////////////////////
    
  while(1){
        MotionPatternGeneration(mth, 10000, 44);

  }
}
