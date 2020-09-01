#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "texturepath.h"
#include "main.h"
#include "vector.h"
#include "trajectory.h"

#ifdef dDOUBLE                      // 単精度と倍精度の両方に対応する
#define dsDrawSphere dsDrawSphereD  // ためのおまじない
#endif

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#endif

dWorldID      world;        // ìÆóÕäwåvéZópÇÃÉèÅ[ÉãÉh (for dynamics)
dSpaceID      space;        // è’ìÀåüèoópÇÃÉXÉyÅ[ÉX (for collision)
dGeomID       ground, foot[8], tstick;       // ínñ  (ground)
dJointGroupID contactgroup; // ê⁄êGì_ÉOÉãÅ[Év (contact group for collision)
dsFunctions   fn;           // ÉhÉçÅ[ÉXÉ^ÉbÉtÇÃï`âÊä÷êî (function of drawstuff)

#define LINK_NUM 6  // ëSÉäÉìÉNêî (total number of links)
#define JT_NUM   6  // ëSÉWÉáÉCÉìÉgêî (total number of joints)
#define LEG_NUM  2  // ëSãrêî (total number of legs)

#define PI  3.141592653589793238462643383279
#define RAD  (180/PI)

double stabilityCal = 0;
double stabilityAccCal = 0;
double velocityCal = 0;
int t=0;
int ballON = 0;

void simLoop(int pause);
// start simulation - set viewpoint

float xyzz[3] = {  0.0f,   0.0f, 0.5f};
float hprz[3] = { 0.0f, -90.0f, 0.0f};
static  float   xyz[3] = {0.2317f,-0.8817f,0.6500f};
//static  float   xyz[3] = {0.0317f,-0.7817f,0.6500f};
static  float   hpr[3] = {101.0000f,-25.5000f,0.0000f};
float xyzg[3] = {    0.6f, 0.4f, 0.5f};          // éãì_[m]
float hprg[3] = { -145.50f, -18.5f, 0.0f};          // éãê¸[Åã]
//static  float   hpr[3] = {101.0000f,-25.5000f,0.0000f};
float xyzx[3] = {  0.5f,   0.0f, 0.15f};
float hprx[3] = { -180.0f, -2.0f, 0.0f};

float xyzy[3] = {  0.0f,   -0.5f, 0.15f};
float hpry[3] = { 88.0f, -6.00f, 0.0f};
//float   xyz2[3] = {0.8317,-2.9817,2.0};
float xyz2[3] = {0.2317f,-0.8817f,0.6500f};

static float xyzn[3],hprn[3];

dGeomID  ballground; // 地面
//dJointGroupID contactgroup; // コンタクトグループ
dReal ballr = 0.1, ballm  = 5.0;
//dsFunctions fn;



typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  dReal    m,r,x,y,z; // éøó (weight)ÅCîºåa(radius)ÅCà íu(positin:x,y,z)
} MyLink;
typedef struct {       // MyObject構造体
    dBodyID ballbody;        // ボディ(剛体)のID番号（動力学計算用）
    dGeomID ballgeom;        // ジオメトリのID番号(衝突検出計算用）
    double  balll,ballr,ballm;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;
MyObject ball;

MyLink leg[LEG_NUM][LINK_NUM],torso, feet[2], tfeet[2][4], hand[2][4], head, stick, base; // ãr(leg)ÅCì∑ëÃ(torso)

dReal  THETA[LEG_NUM][LINK_NUM] = {{0,0,0,0},{0.2,1,0,0}}; // ñ⁄ïWäpìx(target angle)
dReal  THETA_HAND[2][4] = {{-0.2,0.2,-0.2,-0.5},{-0.2,-0.2,0.2,-0.5}}; // ñ⁄ïWäpìx(target angle)
dReal  SX = 0, SY = 0, SZ = 1.35;           // ì∑ëÃèdêSÇÃèâä˙à íu(initial positon of COG)

dReal  l1 = 0.10, l2 = 0.02, l3 = 0.4, l4  = 0.4, l5 = 0.06, l6 = 0.02;// ÉäÉìÉNí∑ 0.25 (lenth of links)
dReal  r1 = 0.02, r2 = 0.05, r3 = 0.05, r4 = 0.04, r5 = 0.02, r6 = 0.03;// leg radius
dReal  hl1 = 0.12, hl2 = 0.02, hl3 = 0.35, hl4  = 0.37;
dReal  hr1 = 0.02, hr2 = 0.03, hr3 = 0.03, hr4 = 0.02;// hand radius

dReal  lx = 0.30, ly= 0.09, lz = 0.6;         // body sides
//dReal  lx2 = 0.10, ly2= 0.3, lz2 = 0.05;         // body sides
dReal  lx2 = 0.115, ly2= 0.220, lz2 = 0.03;         // body sides
dReal  lx3 = 0.115, ly3= 0.220, lz3 = 0.03;         // body sides
dReal  rb1 = 0.01, rl1 = 0.008;
dReal  br = 0.075, bl = 0.075;

// for leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dReal  cx1 = (lx)/2, cy1 = (0);     // (temporal variable)

dReal  c_x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1 ,cx1 ,cx1 ,cx1}, // (center of joints)
                                 {-cx1,-cx1,-cx1 ,-cx1,-cx1,-cx1}};
dReal  c_y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1 , cy1, cy1, cy1}, // (center of joints)
                                 { cy1, cy1, cy1 , cy1, cy1, cy1}};
dReal  c_z[LEG_NUM][LINK_NUM] = {// (center of joints)
            {0-lz/2, -l1-lz/2, -l1-(r2+lz)/2 ,-(l1+l3)-lz/2, -(l1+l3+l4)-lz/2, -(l1+l3+l4+l5)-(lz)/2},
            {0-lz/2, -l1-lz/2, -l1-(r2+lz)/2 ,-(l1+l3)-lz/2, -(l1+l3+l4)-lz/2, -(l1+l3+l4+l5)-(lz)/2}};
// for hand %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//dReal  cx1 = (lx)/2, cy1 = (0);     // (temporal variable)

dReal  hc_x[2][4] = {{ (cx1+hl1/4), (cx1+hl1/2), (cx1+hl1/2) ,(cx1+hl1/2)}, // (center of joints)
                    {-(cx1+hl1/4),-(cx1+hl1/2),-(cx1+hl1/2) ,-(cx1+hl1/2)}};
dReal  hc_y[2][4] = {{ cy1, cy1, cy1 , cy1}, // (center of joints)
                    { cy1, cy1, cy1 , cy1}};
dReal  hc_z[2][4] = {// (center of joints)
                    {0+lz/2, lz/2, (-hr2+lz)/2 ,-(hl3)+lz/2},
                    {0+lz/2, lz/2, (-hr2+lz)/2 ,-(hl3)+lz/2}};

static void makeBall()
{
    dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
    dMass ballmass;
    
    ball.ballbody = dBodyCreate(world);
    dMassSetZero(&ballmass);
    dMassSetSphereTotal(&ballmass,ballm,ballr);
    dBodySetMass(ball.ballbody,&ballmass);
    dBodySetPosition(ball.ballbody, x0, y0, z0);
    ball.ballr      = ballr;
    ball.ballgeom   = dCreateSphere(space,ball.ballr); // 球ジオメトリの生成
    dGeomSetBody(ball.ballgeom,ball.ballbody);         // ボディとジオメトリの関連付け
}
void drawball(){
    
    dsSetColor(1.0,0.0,0.0);
    dsDrawSphere(dBodyGetPosition(ball.ballbody),dBodyGetRotation(ball.ballbody),ballr);
}
float PosStick[3] = {0,-1,1.3};
float slad = 0.8, rlad=0.015;
void makeStick(){
    
    dMatrix3 Rb;
    dMass mass;
    
    base.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,100, 0.3, 0.3, PosStick[2]+0.2);
    dBodySetMass(base.body,&mass);
    base.geom = dCreateBox(space, 0.3, 0.3, PosStick[2]+0.2);
    dGeomSetBody(base.geom, base.body);
    dBodySetPosition(base.body, PosStick[0], PosStick[1], (PosStick[2]+0.2)/2);
    
    stick.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,0.1, 1,rlad*3,slad);
    dMassAdjust(&mass,1);
    dBodySetMass(stick.body,&mass);
    stick.geom = dCreateCapsule(space,rlad*3,slad);
    dBodySetPosition(stick.body, PosStick[0], PosStick[1], PosStick[2]);
    dGeomSetBody(stick.geom, stick.body);
    dRFromZAxis(Rb, 0, 0, 1);
    dBodySetRotation(stick.body, Rb);
    
    
    dRFromZAxis(Rb, 0, 1, 0);
    dGeomSetRotation(stick.geom, Rb);
    
    tstick = stick.geom;
    
    stick.joint = dJointCreateSlider(world,0);
    dJointAttach(stick.joint, base.body, stick.body);
    dJointSetSliderAxis(stick.joint, 0, 1, 0);
    
}
void drawStick(){
    
    dVector3 sides;
    
    dsSetColor(1.3,2.3,1.3);
    dsDrawCapsule(dBodyGetPosition(stick.body),
                  dBodyGetRotation(stick.body),slad,rlad*3);
    dsSetColor(1.3,0,0);
    dGeomBoxGetLengths(base.geom,sides);
    dsDrawBox(dBodyGetPosition(base.body),
              dBodyGetRotation(base.body),sides);
}
void  makeRobot()
{
    dMatrix3 Rb;
    
    dReal torso_m = 1.6;                    // (weight of torso)
    dReal feet_m1 = 0.05;                    // (weight of feet)
    dReal feet_m2 = 0.05;                    // (weight of feet)
    dReal tfeet_m = 0.002;
    dReal mhead = 0.3;
    // for leg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dReal  l1m = 0.05,l2m = 0.05, l3m = 0.4, l4m = 0.3,l5m = 0.05, l6m = 0.05; // (weight of links)

    dReal x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1 ,cx1 ,cx1 ,cx1},// (link position)
                                  {-cx1,-cx1,-cx1 ,-cx1,-cx1,-cx1}};
    dReal y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1 , cy1, cy1, cy1},// (link position)
                                  { cy1, cy1, cy1 , cy1, cy1, cy1}};
    dReal z[LEG_NUM][LINK_NUM] = {                                  // (link position)
            {0-lz/2, -l1/2-lz/2, c_z[0][2]-l3/2, c_z[0][3]-l4/2, c_z[0][4]-l5/2, c_z[0][5]-l5/2},
            {0-lz/2, -l1/2-lz/2, c_z[1][2]-l3/2, c_z[1][3]-l4/2, c_z[1][4]-l5/2, c_z[1][5]-l5/2}};
    dReal r[LINK_NUM]          =  { r1, r2, r3, r4, r5, r6}; // (radius of links)
    dReal length[LINK_NUM]     =  { l1, l2, l3, l4, l5, l6}; // (length of links)
    dReal weight[LINK_NUM]     =  {l1m,l2m,l3m,l4m,l5m,l6m}; // (weight of links)
    dReal axis_x[LEG_NUM][LINK_NUM] = {{ 0, 0, 1, 1, 1, 0},{ 0, 0, 1, 1, 1, 0}};
    dReal axis_y[LEG_NUM][LINK_NUM] = {{ 0, 1, 0, 0, 0, 1},{ 0, 1, 0, 0, 0, 1}};
    dReal axis_z[LEG_NUM][LINK_NUM] = {{ 1, 0, 0, 0, 0, 0},{ 1, 0, 0, 0, 0, 0}};
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    // for hand %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dReal hx[2][4] = {{ (cx1+hl1/4), (cx1+hl1/2), (cx1+hl1/2) ,(cx1+hl1/2)}, // (center of joints)
                        {-(cx1+hl1/4),-(cx1+hl1/2),-(cx1+hl1/2) ,-(cx1+hl1/2)}};
    dReal hy[2][4] = {{ cy1, cy1, cy1 , cy1},// (link position)
                    { cy1, cy1, cy1 , cy1}};
    dReal hz[2][4] = {                                  // (link position)
        {0+lz/2, +lz/2, hc_z[0][2]-hl3/2, hc_z[0][3]-hl4/2},
        {0+lz/2, lz/2, hc_z[1][2]-hl3/2, hc_z[1][3]-hl4/2}};
    dReal hr[4]          =  { hr1, hr2, hr3, hr4}; // (radius of links)
    dReal hlength[4]     =  { hl1, hl2, hl3, hl4}; // (length of links)
    dReal hweight[4]     =  {l1m/3,l2m/3,l3m/3,l4m/3}; // (weight of links)
    dReal haxis_x[2][4] = {{ 1, 0, 0, 1,},{ 1, 0, 0, 1}};
    dReal haxis_y[2][4] = {{ 0, 1, 0, 0,},{ 0, 1, 0, 0}};
    dReal haxis_z[2][4] = {{ 0, 0, 1, 0,},{ 0, 0, 1, 0}};
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    dReal x_f[2][4] = {{x[0][5]+lx2/2, x[0][5]-lx2/2, x[0][5]+lx2/2, x[0][5]-lx2/2},
        {x[1][5]+lx2/2, x[1][5]-lx2/2, x[1][5]+lx2/2, x[1][5]-lx2/2}};
    dReal y_f[2][4] = {{y[0][5]+ly2/2, y[0][5]+ly2/2, y[0][5]-ly2/2, y[0][5]-ly2/2},
        {y[1][5]+ly2/2, y[1][5]+ly2/2, y[1][5]-ly2/2, y[1][5]-ly2/2}};
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // (crate a torso)
	dMass mass;
	torso.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,torso_m, lx, ly, lz);
    dBodySetMass(torso.body,&mass);
    torso.geom = dCreateBox(space,lx, ly, lz);
    dGeomSetBody(torso.geom, torso.body);
    dBodySetPosition(torso.body, SX, SY, SZ);
    //head
	head.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,mhead, 1, br, bl);
    dMassAdjust(&mass,mhead);
    dBodySetMass(head.body,&mass);
    head.geom = dCreateCapsule(space,br,bl);
    dBodySetPosition(head.body, SX, SY, SZ+lz/2+br/2+bl/2);
    dGeomSetBody(head.geom, head.body);
    dRFromZAxis(Rb, 0, 0, 1);
    dBodySetRotation(head.body, Rb);
    
    
    // (crate a feet)
	feet[0].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,feet_m1, lx2, ly2, lz2);
    dBodySetMass(feet[0].body,&mass);
    feet[0].geom = dCreateBox(space,lx2, ly2, lz2);
    dGeomSetBody(feet[0].geom, feet[0].body);
    dBodySetPosition(feet[0].body, SX+x[0][5], SY+y[0][5] + ly2/3, SZ+z[0][5]-l6);
    
    feet[1].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,feet_m2, lx3, ly3, lz3);
    dBodySetMass(feet[1].body,&mass);
    feet[1].geom = dCreateBox(space,lx3, ly3, lz3);
    dGeomSetBody(feet[1].geom, feet[1].body);
    dBodySetPosition(feet[1].body, SX+x[1][5], SY+y[1][5] + ly3/3, SZ+z[1][5]-l6);
    //dBodySetPosition(feet1.body, 0, 0, -1.5);
    int m = 0;
    for(int k = 0; k < 2; k++){
        for(int l = 0; l < 4; l++){
            
            tfeet[k][l].body  = dBodyCreate(world);
            dMassSetZero(&mass);
            dMassSetCapsule(&mass,tfeet_m, 1, rb1, rl1);
            dMassAdjust(&mass,tfeet_m);
            dBodySetMass(tfeet[k][l].body,&mass);
            tfeet[k][l].geom = dCreateCapsule(space,rb1,rl1);
            dBodySetPosition(tfeet[k][l].body, SX+x_f[k][l], SY+y_f[k][l]  + ly2/3, SZ+z[k][5]-l6-0.01);
            //dBodySetPosition(tfeet[k][l].body, SX+x[1][5], SY+y[1][5], SZ+z[1][5]-l6);
            dGeomSetBody(tfeet[k][l].geom, tfeet[k][l].body);
            dRFromZAxis(Rb, 0, 0, 1);
            dBodySetRotation(tfeet[k][l].body, Rb);
            foot[m] =tfeet[k][l].geom;
            m++;
        }
    }
    
    ///// for leg ////////////////////////////////////////////////////////////////////////////////////
    
    dMatrix3 R;
    dRFromAxisAndAngle(R,1,0,0,M_PI/2);
    for (int i = 0; i < LEG_NUM; i++) {
        for (int j = 0; j < LINK_NUM; j++) {
            leg[i][j].body = dBodyCreate(world);
            if (j == 1) dBodySetRotation(leg[i][j].body,R);
            dBodySetPosition(leg[i][j].body, SX+x[i][j], SY+y[i][j], SZ+z[i][j]);
            dMassSetZero(&mass);
            dMassSetCapsuleTotal(&mass,weight[j],3,r[j],length[j]);
            dBodySetMass(leg[i][j].body, &mass);
            leg[i][j].geom = dCreateCapsule(space,r[j],length[j]);
            dGeomSetBody(leg[i][j].geom,leg[i][j].body);
        }
    }
    //(create links and attach legs to the torso)
    for (int i = 0; i < LEG_NUM; i++) {
        for (int j = 0; j < LINK_NUM; j++) {
            leg[i][j].joint = dJointCreateHinge(world, 0);
            if (j == 0)
                dJointAttach(leg[i][j].joint, torso.body, leg[i][j].body);
            else
                dJointAttach(leg[i][j].joint, leg[i][j-1].body, leg[i][j].body);
            dJointSetHingeAnchor(leg[i][j].joint, SX+c_x[i][j], SY+c_y[i][j],SZ+c_z[i][j]);
            dJointSetHingeAxis(leg[i][j].joint, axis_x[i][j], axis_y[i][j],axis_z[i][j]);
        }
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////
    
    ///// for HAND ////////////////////////////////////////////////////////////////////////////////////
    
    dMatrix3 R1;
    dRFromAxisAndAngle(R1,0,1,0,M_PI/2);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            hand[i][j].body = dBodyCreate(world);
            if (j == 1) dBodySetRotation(hand[i][j].body,R);
            if (j == 0) dBodySetRotation(hand[i][j].body,R1);
            dBodySetPosition(hand[i][j].body, SX+hx[i][j], SY+hy[i][j], -0.03 + SZ+hz[i][j]);
            dMassSetZero(&mass);
            dMassSetCapsuleTotal(&mass,hweight[j],3,hr[j],hlength[j]);
            dBodySetMass(hand[i][j].body, &mass);
            hand[i][j].geom = dCreateCapsule(space,hr[j],hlength[j]);
            dGeomSetBody(hand[i][j].geom,hand[i][j].body);
        }
    }
    //(create links and attach legs to the torso)
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            hand[i][j].joint = dJointCreateHinge(world, 0);
            if (j == 0)
                dJointAttach(hand[i][j].joint, torso.body, hand[i][j].body);
            else
                dJointAttach(hand[i][j].joint, hand[i][j-1].body, hand[i][j].body);
            dJointSetHingeAnchor(hand[i][j].joint, SX+hc_x[i][j], SY+hc_y[i][j],-0.03 + SZ+hc_z[i][j]);
            dJointSetHingeAxis(hand[i][j].joint, haxis_x[i][j], haxis_y[i][j],haxis_z[i][j]);
        }
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////
    
//    
    feet[0].joint=dJointCreateFixed(world,0);
	dJointAttach(feet[0].joint,leg[0][5].body,feet[0].body);
	dJointSetFixed(feet[0].joint);
    feet[1].joint=dJointCreateFixed(world,0);
	dJointAttach(feet[1].joint,leg[1][5].body,feet[1].body);
	dJointSetFixed(feet[1].joint);
    //head
    head.joint=dJointCreateFixed(world,0);
	dJointAttach(head.joint,torso.body,head.body);
	dJointSetFixed(head.joint);
    
    for(int k = 0; k < 2; k++){
        for(int l = 0; l < 4; l++){
            
            tfeet[k][l].joint=dJointCreateFixed(world,0);
            dJointAttach(tfeet[k][l].joint,feet[k].body,tfeet[k][l].body);
            dJointSetFixed(tfeet[k][l].joint);
        }
    }
}

/*** ÉçÉ{ÉbÉgÇÃï`âÊ (draw a robot) ***/
void drawRobot()
{
   dReal r,length;
   dVector3 sides;

    // BODY
    dsSetColor(1.3,2.3,1.3);
    dGeomBoxGetLengths(torso.geom,sides);
    dsDrawBox(dBodyGetPosition(torso.body),
              dBodyGetRotation(torso.body),sides);
    dsSetColor(1,0,0);
    dGeomBoxGetLengths(feet[0].geom,sides);
    dsDrawBox(dBodyGetPosition(feet[0].body),
              dBodyGetRotation(feet[0].body),sides);
    dGeomBoxGetLengths(feet[1].geom,sides);
    dsDrawBox(dBodyGetPosition(feet[1].body),
              dBodyGetRotation(feet[1].body),sides);
    //head
    dGeomCapsuleGetParams(head.geom, &r,&length);
    dsDrawCapsule(dBodyGetPosition(head.body),
                  dBodyGetRotation(head.body),br,bl);
    
    for(int k = 0; k < 2; k++){
        for(int l = 0; l < 4; l++){
            dGeomCapsuleGetParams(tfeet[k][l].geom, &r,&length);
            dsDrawCapsule(dBodyGetPosition(tfeet[k][l].body),
                          dBodyGetRotation(tfeet[k][l].body),rl1,rb1);
        }
    }
    
    
    // LEG
    for (int i = 0; i < LEG_NUM; i++) {
        for (int j = 0; j < LINK_NUM; j++ ) {
            dGeomCapsuleGetParams(leg[i][j].geom, &r,&length);
            if (j== 0) dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                                     dBodyGetRotation(leg[i][j].body),0.5*length,1.2*r);
            else       dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                                     dBodyGetRotation(leg[i][j].body),length,r);
        }
    }
    // HAND
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++ ) {
            dGeomCapsuleGetParams(hand[i][j].geom, &r,&length);
            if (j== 0) dsDrawCapsule(dBodyGetPosition(hand[i][j].body),
                                     dBodyGetRotation(hand[i][j].body),0.5*length,1.2*r);
            else       dsDrawCapsule(dBodyGetPosition(hand[i][j].body),
                                     dBodyGetRotation(hand[i][j].body),length,r);
        }
    }
    
}
double touch[2]={0,0};
double touchF[8]={0,0,0,0,0,0,0,0};
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  // if ((o1 != ground) && (o2 != ground)) return;

  static const int N = 20;
  dContact contact[N];
  int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (int i=0; i<n; i++) {
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
        contact[i].surface.mu   = dInfinity; //2.0;
      contact[i].surface.soft_erp = 0.9;
      contact[i].surface.soft_cfm = 1e-5;
	    dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach(c,b1,b2);
    }
  }
    
    
    if((o1==foot[0] && o2==ground)||(o2==foot[0] && o1==ground))touchF[0]=1;
    if((o1==foot[1] && o2==ground)||(o2==foot[1] && o1==ground))touchF[1]=1;
    if((o1==foot[2] && o2==ground)||(o2==foot[2] && o1==ground))touchF[2]=1;
    if((o1==foot[3] && o2==ground)||(o2==foot[3] && o1==ground))touchF[3]=1;
    if((o1==foot[4] && o2==ground)||(o2==foot[4] && o1==ground))touchF[4]=1;
    if((o1==foot[5] && o2==ground)||(o2==foot[5] && o1==ground))touchF[5]=1;
    if((o1==foot[6] && o2==ground)||(o2==foot[6] && o1==ground))touchF[6]=1;
    if((o1==foot[7] && o2==ground)||(o2==foot[7] && o1==ground))touchF[7]=1;
    
    static const int ballN = 10; // 接触点数の最大値
    dContact ballcontact[ballN];     // 接触点
    if(ballON == 1){
    // 接触している物体のどちらかが地面ならisGroundに非0をセット
    int isGround = ((ballground == o1) || (ballground == o2));
    
    // 衝突情報の生成 nは衝突点数
    int nb =  dCollide(o1,o2,ballN,&ballcontact[0].geom,sizeof(dContact));
    if (isGround)  {
        for (int i = 0; i < nb; i++) {
            ballcontact[i].surface.mode = dContactBounce; // 接触面の反発性を設定
            ballcontact[i].surface.bounce = 0.7;          // 反発係数(0.0から1.0)
            ballcontact[i].surface.bounce_vel = 0.0;      // 反発に必要な最低速度
            
            // 接触ジョイントの生成
            dJointID cb = dJointCreateContact(world,contactgroup,
                                             &ballcontact[i]);
            // 接触している２つの剛体を接触ジョイントにより拘束
            dJointAttach(cb,dGeomGetBody(ballcontact[i].geom.g1),
                         dGeomGetBody(ballcontact[i].geom.g2));
        }
    }
    }
    
    
}

/*** ãtâ^ìÆäwÇÃåvéZ (calculate inverse kinematics ***/
void  inverseKinematics(dReal x, dReal y, dReal z,
                        dReal *ang1, dReal *ang2, dReal *ang3,int posture)
{
    dReal l1a = 0, l3a = l3 + r3/2;
    
    double c3 = (x*x + z*z + (y-l1a)*(y-l1a) - (l2*l2+l3a*l3a))/(2*l2*l3a);
    double s2 = (y-l1a) / (l2 + l3a*c3);
    double c2 = sqrt(1 - s2 * s2);
    double c1 = (l2 + l3a*c3)*c2/sqrt(x*x+z*z);
    // printf("c3=%f s2=%f c2=%f c1=%f \n", c3,s2,c2,c1);
    if (sqrt(x*x+y*y+z*z) > l2 + l3) {
        printf(" Target point is out of range \n");
    }
    
    switch (posture) {
        case 1: // épê®ÇP (posture 1)
            *ang1 =   atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1);
            *ang2 = - atan2(s2,c2);
            *ang3 =   atan2(sqrt(1-c3*c3),c3); break;
        case 2: // épê®ÇQ (posture 2)
            *ang1=   atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
    *ang2= - atan2(s2,c2);
    *ang3= - atan2(sqrt(1-c3*c3),c3); break;
  case 3:  // épê®ÇR (posture 3)
    *ang1 =   M_PI + (atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1));
    *ang2 = - M_PI +  atan2(s2,c2);
    *ang3 = - atan2(sqrt(1-c3*c3),c3); break;
  case 4:  // épê®ÇS (posture 4)
    *ang1 =  M_PI + atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
    *ang2 = -M_PI + atan2(s2,c2);
    *ang3 =  atan2(sqrt(1-c3*c3),c3); break;
  }
}

// get present position robot in simulation
float xyz0[3] = {  100.0f,  -100.2f, 0.2f};  // éãì_[m] (View point)
float hpr0[3] = {191.0f, -50.0f, 0.0f};  // éãê¸[Åã] (View direction)

float xyz1[3] = {  3.0f,  0.0f, 0.5f};  // éãì_[m] (View point)
float hpr1[3] = {-180.0f, -10.0f,0.0f};  // éãê¸[Åã] (View direction)
float data_pos[3];



/*** éãì_Ç∆éãê¸ÇÃê›íË (Set view point and direction) ***/
void start()
{
    int i;
    float xyz[3] = {  2.0f,  -2.2f, 1.5f};  // éãì_[m] (View point)
    float hpr[3] = {121.0f, -10.0f, 0.0f};  // éãê¸[Åã] (View direction)
    
    // éãì_Ç∆éãê¸ÇÃê›íË (Set View point and direction)
    
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
    dsSetCapsuleQuality(6);
}
int mtn = 100;
double data_kp = 20;
float speed = 0.0;
double OFFSETX = 0, OFFSETY = 0;
bool PushControl = 0;
int viewPoint = 0;
static void command (int cmd)
{
    int i;
    switch (cmd) {
        case 'a': case 'A':
            printf("test\n");
            for(i = 0; i<3; i++){
                xyz0[i] = data_pos[i] + xyz1[i];
            }
            dsSetViewpoint(xyz0,hpr0);
            break;
        case 'x':
            mtn = 300;
            break;
        case 'y':
            break;
        case 'z':
            mtn = 202;
            break;
        case 'c':
            viewPoint = 1;
            break;
        case 'v':
            viewPoint = 0;
            break;
        case '1':
            break;
        case '2':
            break;
        case '3':
            break;
        case 'q':
            break;
    }
}
/*** ÉhÉçÅ[ÉXÉ^ÉbÉtÇÃê›íË (Set drawstuff) ***/
void setDrawStuff() {
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command  = &command;;
    fn.path_to_textures = "/Users/Azhar/ode-0.13/drawstuff/textures";
}
/*** Pêßå‰ (P control) ***/
/*** ï‡çsêßå‰ (gait control) ***/
int tt = 0;
float moveStick = 0;
void Pcontrol()
{
  dReal kp = data_kp, fMax = 100.0;
    //if(tt >= 700)kp =8;
  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
      dReal tmp = dJointGetHingeAngle(leg[i][j].joint);
      dReal diff = THETA[i][j] - tmp;
      dReal u = kp * diff;
        dJointSetHingeParam(leg[i][j].joint,  dParamVel, u);
        if(j<4)      dJointSetHingeParam(leg[i][j].joint, dParamFMax, fMax);
        else         dJointSetHingeParam(leg[i][j].joint, dParamFMax, 50);
    }
      
  }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            dReal tmp = dJointGetHingeAngle(hand[i][j].joint);
            dReal diff = THETA_HAND[i][j] - tmp;
            dReal u = kp * diff;
            dJointSetHingeParam(hand[i][j].joint,  dParamVel, u);
            dJointSetHingeParam(hand[i][j].joint, dParamFMax, fMax);
        }
    }
    static dReal kps = 2.0;
    static dReal fmaxs = 3.95;
    dReal tmp = dJointGetSliderPosition(stick.joint);
    dReal u = kp * (moveStick-tmp);
//    if(u > 2) u = 2;
//    else if(u < -2) u = -2;
    dJointSetSliderParam(stick.joint, dParamVel, u);
    dJointSetSliderParam(stick.joint, dParamFMax, fmaxs);
    
    if(tmp <= moveStick) moveStick = 0;
}
double min(double a, double b){
    double x;
    if(a > b) x = b;
    else x = a;
    return x;
}
double max(double a, double b){
    double x;
    if(a > b) x = a;
    else x = b;
    return x;
}
// get condition of robot
double vel1 = 0, vel0 = 0, d_y = 0, d_y0 = 0;
double len_x[3] ={0,0,0} ,len_y[3]={0,0,0}, len[2], alfa=0;
double Acc = 0, dataVel[2], kec[50], ang[4], velx[2], Accl[2], pos[2];
double kecepatan, Smax, Smin;
double roll = 0, pitch = 0, angvelPitch = 0, angvelRoll;
void GetPositionObject(){
    int i;
    const dReal *pos0= dBodyGetPosition(torso.body);     // current position
    const dReal *rot0= dBodyGetRotation(torso.body);     // current posture
    const dReal *vel = dBodyGetLinearVel(torso.body);
    
    angvelRoll = (rot0[9] - roll)/0.005;
    angvelRoll = (rot0[8] - pitch)/0.005;
    roll = rot0[9]+0.00;
    pitch = rot0[8];
    
    for(i = 0; i<3; i++){
        data_pos[i] = pos0[i];
    }
    
    len_x[2] = len_x[1]; len_y[2] = len_y[1];
    len_x[1] = len_x[0]; len_y[1] = len_y[0];
    len_x[0] = pos0[0]; len_y[0] = pos0[1];
    
    velx[1] = velx[0];
    velx[0] = vel[1] * cos(rot0[1] * PI/2) + vel[1] * sin(rot0[1] * PI/2);
    
    Accl[0] = (velx[0] - velx[1])/0.02;
    
    
//    printf("||||| %f |||||| %f ||||||\n",touch[0], touch[1]);
    
    if(roll >= -0.4 && roll <= 0.4 && pitch >= -0.4 && pitch <= 0.4) reset = 0;
    else reset = 1;
    
    if(touchF[0] == 1 || touchF[1] == 1 ||touchF[2] == 1 ||touchF[3] == 1)touch[0] = 1;else touch[0] = 0;
    if(touchF[4] == 1 || touchF[5] == 1 ||touchF[6] == 1 ||touchF[7] == 1)touch[1] = 1;else touch[1] = 0;
    
    //if(pos[1] > Smax || pos[1] < Smin){
    //    Sf = 50 * (pos[1] - (Smin + (Smax - Smin)/2));
        //AngleRobot = -rot0[9] * 90;
    //}else{
    //    Sf = 0;
       // AngleRobot = 0;
   // }
    //
    /******************************************************************/
    
    //printf("3 = %.3f\t4 : %.3f\t0 : %.3f\t1 : %.3f\t2 : %.3f\n\r",rot0[2],rot0[7],rot0[8],rot0[9],rot0[10]);
    
}

double P[10][3];
double Pfeet[8][3];
double Vel[10][3];
double Vel0[10][3];
double Ac[10][3];
double Pw[3];
double G = 9.8;
void ZMPPointData(){
    const dReal *p1 = dBodyGetPosition(tfeet[0][0].body);     // current position
    const dReal *p2 = dBodyGetPosition(tfeet[0][1].body);     // current position
    const dReal *p3 = dBodyGetPosition(tfeet[0][2].body);     // current position
    const dReal *p4 = dBodyGetPosition(tfeet[0][3].body);     // current position
    const dReal *p5 = dBodyGetPosition(tfeet[1][0].body);     // current position
    const dReal *p6 = dBodyGetPosition(tfeet[1][1].body);     // current position
    const dReal *p7 = dBodyGetPosition(tfeet[1][2].body);     // current position
    const dReal *p8 = dBodyGetPosition(tfeet[1][3].body);     // current position
    for(int i=0; i<3; i++){
        Pfeet[0][i] = p1[i] - Pw[i];
        Pfeet[1][i] = p2[i] - Pw[i];
        Pfeet[2][i] = p3[i] - Pw[i];
        Pfeet[3][i] = p4[i] - Pw[i];
        Pfeet[4][i] = p5[i] - Pw[i];
        Pfeet[5][i] = p6[i] - Pw[i];
        Pfeet[6][i] = p7[i] - Pw[i];
        Pfeet[7][i] = p8[i] - Pw[i];
    }
    for(int i=0; i<8; i++){
//        printf("%.3f\t%.3f\n",Pfeet[i][0] ,Pfeet[i][1]);
    }
    
    
}
int SupportingArea(double px, double py){
    int DSP = 0;
    double a[12][2];
    double a2[12][2];
    double x[12];
    double n[12][2];
    double p[2];
    
    double baa[2];
    double Theta[12];
    
    for(int i=0; i<2; i++){
        a[0][i] = Pfeet[0][i];
        a2[0][i] = Pfeet[2][i];
        a[1][i] = Pfeet[5][i];
        a2[1][i] = Pfeet[7][i];
        a[2][i] = Pfeet[0][i];
        a2[2][i] = Pfeet[1][i];
        a[3][i] = Pfeet[2][i];
        a2[3][i] = Pfeet[3][i];
        a[4][i] = Pfeet[4][i];
        a2[4][i] = Pfeet[5][i];
        a[5][i] = Pfeet[6][i];
        a2[5][i] = Pfeet[7][i];
        a[6][i] = Pfeet[0][i];
        a2[6][i] = Pfeet[4][i];
        a[7][i] = Pfeet[1][i];
        a2[7][i] = Pfeet[5][i];
        a[8][i] = Pfeet[2][i];
        a2[8][i] = Pfeet[6][i];
        a[9][i] = Pfeet[3][i];
        a2[9][i] = Pfeet[7][i];
    }
    
    p[0] = px;
    p[1] = py;

    for(int i=0; i<10; i++){
        n[i][0] = a2[i][0] - a[i][0];
        n[i][1] = a2[i][1] - a[i][1];
        
        double *ba = vectorSubtraction(a[i], p ,2);
        double *bb = vectorScale(1.0/(vectorValue(n[i], 2)),n[i],2);
        double *bc = vectorScale(dotProduct(ba, bb, 2),bb, 2);
        
        baa[0] = 1;
        baa[1] = 0;
        Theta[i] = acos(dotProduct(vectorSubtraction(ba, bc, 2), baa, 2)/(vectorValue(vectorSubtraction(ba, bc, 2), 2) * vectorValue(baa, 2)));
        x[i] = vectorValue(vectorSubtraction(ba, bc, 2) , 2);
//        
//        if(i == 0){
//            printf("ba = %f\t%f\n", ba[0], ba[1]);
//            printf("bc = %f\t%f\n", bc[0], bc[1]);
//        }
//        printf("d = %f\n", x[i]);
    }
    
    if((x[0] >= 0 && x[1] <= 0) || (x[0] <= 0 && x[1] >= 0)){
        
        DSP = 1;
        
        if(((x[4] <= 0 || x[2] <= 0) && (x[5] >= 0 || x[3] >= 0))  || ((x[4] >= 0 || x[2] >= 0) && (x[5] <= 0 || x[3] <= 0))){
            if(((x[6] <= 0 || x[7] <= 0) && (x[8] >= 0 || x[9] >= 0))  || ((x[6] >= 0 || x[7] >= 0) && (x[8] <= 0 || x[9] <= 0))){
                DSP = 1;
            }
        }
    }
//    printf("%f\t%f\n", p[0], p[1]);
//    printf("%f\t%f\t%f\t%f\n", a[0][0], a[0][1], a2[0][0],  a2[0][1]);
//    printf("%f\t%f\t%f\t%f\n", a[1][0], a[1][1], a2[1][0],  a2[1][1]);
//    printf("Theta = %f\tTheta = %f\t%f\t%f\t", Theta[0]*RAD, Theta[1]*RAD, x[0], x[1]);
    
    return DSP;
}

double xZMP, yZMP;
void ZMPcalculation(){
    
    
    dMass mass[10];
    const dReal **p;
    
    const dReal *p1 = dBodyGetPosition(torso.body);     // current position
    const dReal *v1 = dBodyGetLinearVel(torso.body);     // current posture
    dBodyGetMass(torso.body, &mass[0]);
    const dReal *p2 = dBodyGetPosition(hand[0][2].body);     // current position
    const dReal *v2 = dBodyGetLinearVel(hand[0][2].body);     // current posture
    dBodyGetMass(torso.body, &mass[1]);
    const dReal *p3 = dBodyGetPosition(hand[0][3].body);     // current position
    const dReal *v3 = dBodyGetLinearVel(hand[0][3].body);     // current posture
    dBodyGetMass(torso.body, &mass[2]);
    const dReal *p4 = dBodyGetPosition(hand[1][2].body);     // current position
    const dReal *v4 = dBodyGetLinearVel(hand[1][2].body);     // current posture
    dBodyGetMass(torso.body, &mass[3]);
    const dReal *p5 = dBodyGetPosition(hand[1][3].body);     // current position
    const dReal *v5 = dBodyGetLinearVel(hand[1][3].body);     // current posture
    dBodyGetMass(torso.body, &mass[4]);
    const dReal *p6 = dBodyGetPosition(leg[0][2].body);     // current position
    const dReal *v6 = dBodyGetLinearVel(leg[0][2].body);     // current posture
    dBodyGetMass(torso.body, &mass[5]);
    const dReal *p7 = dBodyGetPosition(leg[0][3].body);     // current position
    const dReal *v7 = dBodyGetLinearVel(leg[0][3].body);     // current posture
    dBodyGetMass(torso.body, &mass[6]);
    const dReal *p8 = dBodyGetPosition(leg[1][2].body);     // current position
    const dReal *v8 = dBodyGetLinearVel(leg[1][2].body);     // current posture
    dBodyGetMass(torso.body, &mass[7]);
    const dReal *p9 = dBodyGetPosition(leg[1][3].body);     // current position
    const dReal *v9 = dBodyGetLinearVel(leg[1][3].body);     // current posture
    dBodyGetMass(torso.body, &mass[8]);
    const dReal *p10 = dBodyGetPosition(head.body);     // current position
    const dReal *v10 = dBodyGetLinearVel(head.body);     // current posture
    dBodyGetMass(torso.body, &mass[9]);
    
    for(int i=0; i<3; i++){
        P[0][i] = p1[i];
        Vel[0][i] = v1[i];
        P[1][i] = p2[i];
        Vel[1][i] = v2[i];
        P[2][i] = p3[i];
        Vel[2][i] = v3[i];
        P[3][i] = p4[i];
        Vel[3][i] = v4[i];
        P[4][i] = p5[i];
        Vel[4][i] = v5[i];
        P[5][i] = p6[i];
        Vel[5][i] = v6[i];
        P[6][i] = p7[i];
        Vel[6][i] = v7[i];
        P[7][i] = p8[i];
        Vel[7][i] = v8[i];
        P[8][i] = p9[i];
        Vel[8][i] = v9[i];
        P[9][i] = p10[i];
        Vel[9][i] = v10[i];
    }
    for(int i=0; i<10; i++){
        for(int j=0; i<3; i++){
            Ac[i][j] = (Vel[i][j] - Vel0[i][j])/0.02;
            Vel0[i][j] = Vel[i][j];
        }
    }
    Pw[0] = P[0][0];
    Pw[1] = P[0][1];
    Pw[2] = P[0][2];
    
    double A1[2] = {0,0};
    double A2[2] = {0,0};
    double B1[2] = {0,0};
    double B2[2] = {0,0};
    for(int j=0; j<2; j++){
        for(int i=0; i<10; i++){
            A1[j] += (double)(mass[i].mass) * (Ac[i][2] + Ac[0][2] + G) * P[i][j];
            A2[j] += (double)(mass[i].mass) * (Ac[i][2] + Ac[0][2] + G);
            B1[j] += (double)(mass[i].mass) * (Ac[i][j] + Ac[0][j]) * (P[i][2] + P[0][2]);
            B2[j] += (double)(mass[i].mass) * (Ac[i][2] + Ac[0][2] + G);
    
        }
    }
    xZMP = (A1[0]/A2[0] - B1[0]/B2[0]) - Pw[0];
    yZMP = (A1[1]/A2[1] - B1[1]/B2[1]) - Pw[1];
    
    ZMPPointData();
    int DSP = SupportingArea(xZMP, yZMP);
//    printf("\nx = %f\ty = %f\tDSP = %d\n", xZMP, yZMP, DSP);
}
int reset = 0, lastreset = 0;
void initialStart(){
    
    
    touchF[0] = 0; touchF[1] = 0;
    touchF[2] = 0; touchF[3] = 0;
    touchF[4] = 0; touchF[5] = 0;
    touchF[6] = 0; touchF[7] = 0;
    
    // ï‡çsêßå‰ (gait control)
    dSpaceCollide(space,0,&nearCallback); // è’ìÀåüèo (collision detection)
    dWorldStep(world, 0.02);              // ÉXÉeÉbÉvçXêV (step a simulation)
    dJointGroupEmpty(contactgroup);       // ê⁄êGì_ÉOÉãÅ[ÉvÇãÛ (empty jointgroup)
    
    drawRobot();
    for(int i = 0; i<3; i++){
        xyz0[i] = data_pos[i] + xyz1[i];
    }
    
    reset = 0;
    
    
    if(reset == 1){
        
        dInitODE();
        
        setDrawStuff();
        world        = dWorldCreate();
        space        = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground       = dCreatePlane(space,0,0,1,0);
        dWorldSetGravity(world, 0, 0, -9.81);
        dWorldSetCFM(world, 1e-3); // CFMÇÃê›íË (global CFM)
        dWorldSetERP(world, 0.9);  // ERPÇÃê›íË (global ERP)
        makeRobot();
    }
    
}

// make trajectory walking
#define rad 57.295779513082320876798154814105
#define sp2 1.4142135623730950488016887242097

dReal angle1, angle2, angle3;
int Impulse = 0;
int posture = 2;
int ModeRunning = 0, timeToLearn=0;
void walk()
{
    double theta0[JT_NUM] = {0,-0.02,-PI/1};
    double LEG0_0,LEG0_1,LEG0_2,LEG0_3;
    static int steps = 0;
    
    
    GetPositionObject();
    ZMPcalculation();
    
        MotionPatternGeneration(mtn, 10000, 30);
    
        THETA[0][0] = AngleJoint[0][0];
        THETA[0][1] = AngleJoint[0][1];
        THETA[0][2] = AngleJoint[0][2];
        THETA[0][3] = AngleJoint[0][3];
        THETA[0][4] = AngleJoint[0][4];
        THETA[0][5] = AngleJoint[0][5];
        
        THETA[1][0] = AngleJoint[1][0];
        THETA[1][1] = AngleJoint[1][1];
        THETA[1][2] = AngleJoint[1][2];
        THETA[1][3] = AngleJoint[1][3];
        THETA[1][4] = AngleJoint[1][4];
        THETA[1][5] = AngleJoint[1][5];
        
        //THETA_HAND[0][0] = -OutputData[0] * 100 + DataTheta[3]/1.3 + 0.23;
        //THETA_HAND[1][0] = -OutputData[1] * 100 + DataTheta[1]/1.3 + 0.23;
        //THETA_HAND[0][0] = data_out[0];
        //THETA_HAND[1][0] = data_out[0];

    
//        fprintf(OutputFile2,"%d\t",tt);
//        for (int leg_no = 0; leg_no < LEG_NUM; leg_no++) {
//            for(int joint_no = 0; joint_no < JT_NUM; joint_no++){
//                fprintf(OutputFile2,"%.3f\t",THETA[leg_no][joint_no]);
//            }
//        }
//        fprintf(OutputFile2,"\n");
    
        initialStart();
    
        Pcontrol(); // (P control)
}

void simLoop(int pause)
{
    if (!pause) {
        walk();
    }
    if(viewPoint == 1){
        for(int i = 0; i<2; i++){
            xyz0[i] = data_pos[i] + xyz1[i];
        }
        dsSetViewpoint(xyz0,hpr1);
    }
}
int main(int argc, char *argv[])
{
    int tt=0;
    
    dInitODE();
    
    setDrawStuff();
    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    ground       = dCreatePlane(space,0,0,1,0);
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetCFM(world, 1e-3); // CFMÇÃê›íË (global CFM)
    dWorldSetERP(world, 0.9);  // ERPÇÃê›íË (global ERP)
    
    makeRobot();
    makeStick();
    //for(int l = 0; l < 3; l++){
    dsSimulationLoop(argc,argv,800,480,&fn);
        //exit(1);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
