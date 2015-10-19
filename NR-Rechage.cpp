// LRF2D_thetaDock_MultiSegFitCOG_35cm.cpp' d=y15m9d29. released y15m10d09.
// was'MAIN_chargeNav.cpp using Hough-Transform'a= Liu,JinYong. 2015-08-20.Fri. //changed names of some variables.
// Use git test
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include "common/utils.h"
#include "messages/MessageInteraction.h"
#include "messages/AGVMessage.h"
#include "map/map_convert.h"
#include "map/traval_map.h"


#include <highgui.h>
#include <time.h>
#include "math.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//
//#include "NRF_Messages/MessageInteraction.h"
//#include "NRF_Common/NRF_Utils.h"
//#include "NRF_Common/NRF_ParamReader.h"
//#include "NRF_Common/NRF_Geometry.h"

#include"PointStruct.h"
// #include <opencv/cv.h>
// #include <opencv/highgui.h>

/// SURO 头文件

//USING_NAMESPACE_NR_FC_COMMON();
using namespace NJRobot;


#define constVel 50
int zoneDeg = 45;
int zoneDegHalf =50;
#define zoneDegWide 88
// bool enter_flag = 0
double prevOKmidY =0; //keep prevY'to avoid sudden false detection wn 60cm
bool isDocked =false;
bool isNearDock = false;
// bool isVaryMidPnt = true; // only not-varying midPnt can be chosen.
bool isFirstScan = true;
int seqCurScan =0;
Point_t preMidPnt;
//


//
bool Recharge_enable = false;
bool isNewDeviateAtDocking = false;
double preDistYmid=0;
double preDistXmid=0;
double preDistMidTheta=0;
// double line1k,line1b,line2k,line2b;
int scanIter = 1;
int cur_segC_indexMatching =0;
int cur_segC_indexFound =0;
double YawRateScale = 0.01;
double YawRateScaleF = 0.001; //  = 0.001;
// double HX_theta, theta; // HXtheta2min' HX_theta' AGV'theta_yaw2pntE
double pre_theta; 
// double mid_HX_theta;
// int i,j;
double midXfoundA, midYfoundA, midDistfoundA, midAnglefoundA; // re'excl'
double angle150found, LengthBfound, LengthCfound; // Global var;

#define Center2COG 340					// Laser scanner'激光扫描器中心点'距离小车中心距离为 340mm' 
#define Center_Sick 340					// SICK Laser scanner'激光扫描器中心点'距离小车中心距离为 340mm' 
#define Safe_dist 600					//小车中心距离充电桩距离 600mm
#define DistYtoDockStation 650 // 60cm'

// #define Ashort 386.37
#define Ashort 482.963
#define ConstSegShort 250 //25cm
// #define ConstSegShort 200
#define segBthres 80


// bool FirstScanTrue = 1;
double CVDrawScale = 0.1; //0.01
// cv::Mat cvRenderMatrix2D = cv::Mat::zeros(720,640,CV_8UC3);
// FILE *fp;
#define num 361					//共取361个激光数据 '在541个数据时'取361个中间位置的数据点.
#define angular_resolution_DEG 0.5
#define MIN_DEGREE -45
const int LMS270deg_DATA_length = 541; 

Point_t *LRF2DdataFrame = new Point_t[LMS270deg_DATA_length];
std::vector<Lmap_segment_t> segsStdVecGlobal;
//! The number of lines we have in the lineList (Double this number to get point count)
int lineCount =0;

//! Define the size of the list
// #define SEG_LIST_SIZE 109
//! A custom list of all line points

#define LINE_LIST_SIZE 256
Point_t lineList[LINE_LIST_SIZE][2];
Cluster_PointIndex_t Cluster_PointIndex[LINE_LIST_SIZE];

// #define pi 3.1415926535897932384626433832795
const double pi = 3.14159265358979;

#ifndef DEG2RAD
#define DEG2RAD pi/180.0
#endif
// #define RAD2DEG 180.0/pi

Mesg_RobotSpeed cur_vel;  //x前进方向，y右方向，y方向的速度一般不控制，直接给0， 只要给出Vx和w即可
Mesg_ControlTask feedbackInfo;
Mesg_NavigationTask recharge_motion;

boost::mutex g_mutex_laser;

#define RECHARGE_SHAPE_SUBSIZE 23
#define Conf_distmax 930					//小车调整确认距离
#define Conf_distmin 900

// ' SICK LMS-1xx'270deg' n=541;
//2e'n=682'for' Hokuyo_URG-04LX-UG01'240deg'682points. 
double rangeData[LMS270deg_DATA_length];
double angleRAD_Data[LMS270deg_DATA_length];
double angleDEG_Data[LMS270deg_DATA_length];

int segFindIndex =-1;
int clusterFindIndex =-1;
double segLengthB=0;
double segLengthC=0;

bool enter_flag = 0, leave_flag = 0;
bool leave_flagTo = 0;
int weight;
int no_find_times;

void updateLaser(Mesg_Laser LRFdata2D);
void updateMotionFlags(Mesg_NavigationTask recharge_motion);

void readScanPntsCOG(int DataSize, Mesg_Laser LRFdata2D);
//
bool VarDeviationOK(double Length, double refLength, double thresVar)
{
  if ( fabs(Length - refLength) < fabs(thresVar))
	return true;
  else 
	return false;
}
// :: 5b. 'show angles-between-consecutive-LineSegments.
bool segAB_angles_find(Point_t* midPnt_p, segBCA_t* segBCA_p, int seg_start_i)
{
	// cur_segC_indexFound = seg_start_i+1;
	bool tagFoundSegBC = 0;
	int foundBCindex = -1;
  // int segFitIndex = -1; // not use globalVar. //index of found {segA}and'its'joining-segB
  const double ConstObtuseADeg = 150.0;
  // const double ConstLengthA = 48.3; // '+ LengthShort
  // const double ConstSegLength = 25.0;

  unsigned int segs_number = numSegs_curScan; //was' segs_Index
  int seg_point_numB, seg_point_numC;
  double xB1,yB1, xB2,yB2; // LineSegmentB: B1,B2
  double xC1,yC1;
  double xC2,yC2; // LineSegmentC: C1,C2;
  
  int tmp_seg_point_num = 0;	// The given point distance

  double cosObtuse_A;  // globalVar
  double Obtuse_A_deg;
  double LengthB, LengthC,  LengthA,  LengthSqrDiff;
  LengthB = LengthC = LengthA = LengthSqrDiff = -1;
  // seg_point_num wn a segment 'depends on the distance between LRF2D and ChargingStation. // show it as an indicator.

  // if (segs_number==1 || segs_number-seg_start_i<1)
  if (segs_number==1 || segs_number-seg_start_i<1)
  {
      cur_segC_indexMatching = segs_number;
	printf("seg==1 remaining!!! \n");
	return false;
  }
  int zoneDegSeat = zoneDeg;
  // isNearDock = false;
  for(unsigned int j=seg_start_i; j< (segs_number-1) ; j++) //vs'ros:'sensor_msgs::LaserScan scan2D.ranges.size(); 
  {	

	if (isNearDock==false) zoneDegSeat = zoneDeg; // 60deg
	  else zoneDegSeat = zoneDegWide; // 70deg
	  
	  if  (LineSegments[j].start_point_index <= (270 - zoneDegSeat *2) || LineSegments[j].end_point_index >= (270 + zoneDegSeat *2) )
		  ; // non-joining LineSeg'

	  else if  (LineSegments[j+1].start_point_index - LineSegments[j].end_point_index >5 || distP2Pval(LineSegments[j+1].p1, LineSegments[j].p2)>=75 ) //8cm'
		  ; // non-joining LineSeg'
	  else {
	//// was' Length'B'C'A	
	// seg_point_numB = LineSegments[j].num_seg_points;
	xB1 = LineSegments[j].p1.x; yB1= LineSegments[j].p1.y;
	xB2 = LineSegments[j].p2.x; yB2= LineSegments[j].p2.y;	
	LengthB = distP2P( &LineSegments[j].p1, &LineSegments[j].p2);
	
	LengthC = distP2P( &LineSegments[j+1].p1, &LineSegments[j+1].p2);
	// seg_point_numC = LineSegments[j].num_seg_points;
	xC1= LineSegments[j+1].p1.x; yC1= LineSegments[j+1].p1.y;	
	xC2= LineSegments[j+1].p2.x; yC2= LineSegments[j+1].p2.y;
	
	// now for two neighour segments (eg. not joining)' {x1b,y1b}-{x2a,y2a}
	// tmp'double newC2atB_x, newC2atB_y; // double offset_CtoBx = xC1 - xB2; // double offset_CtoBy = yC1 - yB2;
	Point_t newC2toB;
	newC2toB.x = xC2 - (xC1 - xB2);
	newC2toB.y = yC2 - (yC1 - yB2);

	// consecutive-joining-segA'segB: Length of '(segA.point1, to segB.point2).
	LengthA = distP2P( &LineSegments[j].p1, &newC2toB);
	// was'err: LengthA = distP2P( &LineSegments[j].p1, &LineSegments[j+1].p2); 
	
	// unknown'a^2 == b^2+c^2-2bccos(A) //re'角A是钝角. 钝角三角形的长边. '公式中: a^2 == b^2 + c^2 + 2bc*cos(pi-A) //其中'锐角(pi-A)
	// ie. sqrt(a^2) = 48.2962913145; where a^2 == 2,332.53175473 = b^2 + c^2 + 1,082.53175473; (where b==c==25)
	LengthSqrDiff = (LengthB*LengthB) + (LengthC*LengthC) -(LengthA*LengthA) ;
	// 90deg. 
	cosObtuse_A = LengthSqrDiff / (2*LengthB*LengthC);
	Obtuse_A_deg = acos(cosObtuse_A) * 180/pi; // rad -> degree;
	// printf(" 2LineAngle_deg %lf\n", Obtuse_A_deg);
	if ( fabs(Obtuse_A_deg - 150) >= 15.0)	{
		//printf("\n Obtuse_A varTooLarge: %lf", Obtuse_A_deg);
	continue;
	}

	// if (distP2Pval(LineSegments[j].p1, LineSegments[j].p2) == 0) //ok
	// if (distP2Pval(LineSegments[j].p1, LineSegments[j].p2) <= 70) //70mm //

	double vertAtanRad_B = LineSegments[j].slopeVert;
	// LineSegments[j].slopeVert
	double vertSize = sqrt ( LineSegments[j].xVert * LineSegments[j].xVert + LineSegments[j].yVert * LineSegments[j].yVert );
	double vertAcosRadB = acos (LineSegments[j].xVert / vertSize);
	double vertAcosDeg_B = vertAcosRadB * RAD2DEG;
	//:: printf("\n (seg_%d'th; slopeV_aTanRad %lf, slopeV_acos_Deg %lf, slopeV_aTanDeg %lf", j, LineSegments[j].slopeVert, vertAcosDeg_B, RAD2DEG* LineSegments[j].slopeVert );

	double vertSizeC = sqrt ( LineSegments[j+1].xVert * LineSegments[j+1].xVert + LineSegments[j+1].yVert * LineSegments[j+1].yVert );
	double vertAcosRadC = acos (LineSegments[j+1].xVert / vertSizeC);
	double vertAtanRad_C = LineSegments[j+1].slopeVert;
	// printf("\n~ (seg:=%d'th; slopeV_Rad %lf, slopeV_acos_Rad %lf, slopeV_acos_Rad %lf, slopeV_deg %lf", j+1, LineSegments[j+1].slopeVert, vertAcosRadC, vertAcosRadC*RAD2DEG, RAD2DEG* vertAtanRadC );

	double degBC = 180 - abs( vertAcosRadC* RAD2DEG - vertAcosDeg_B);
	if ( (degBC - 150) >= 15.0)   {
		//:: printf("\n _ excl' degBC: %lf wuErr >= 8deg", degBC);
		continue; // 700mm 
	  }
	//
	// eg'wn'4nd quad'XiangXian: atanRad < 0 valid && invalid: 0< acosRad < pi/2;
	if (LineSegments[j+1].xVert >0 && LineSegments[j+1].yVert <0)
		vertAcosRadC = vertAtanRad_C;
	if (LineSegments[j].xVert >0 && LineSegments[j].yVert <0)
		vertAcosRadB = vertAtanRad_B;

	// eg'wn'2nd quad'XiangXian: atanRad < 0 invalid && valid_ acosRad >pi/2;
	// tmp_var' atanRad_B'_C' eg. already considered vertSlope'wn' 1st-quad'Xiangxian'
	// if (LineSegments[j+1].xVert <0 && LineSegments[j+1].yVert >0)
	if (vertAcosRadC >0 && vertAtanRad_C <0)
		vertAtanRad_C = vertAcosRadC;
	if (vertAcosRadB >0 && vertAtanRad_B <0)
		vertAtanRad_B = vertAcosRadB;

	// double vertRadC, vertRadB;
	Point_t pnt_mid;

	if(LineSegments[j].slopeK!=LineSegments[j+1].slopeK)
	{
		pnt_mid.x = (LineSegments[j+1].slopeB-LineSegments[j].slopeB)/(LineSegments[j].slopeK-LineSegments[j+1].slopeK);
		// TODO'
		pnt_mid.y = LineSegments[j].slopeK*pnt_mid.x + LineSegments[j].slopeB;
	}
	else
		printf("NO JD_POINT!\n"); // never happens.

	//pnt_mid.y = 0.5 * ( LineSegments[j].p2.y + LineSegments[j+1].p1.y);
	pnt_mid.dist = sqrt( pnt_mid.x*pnt_mid.x + pnt_mid.y*pnt_mid.y);

	// (segC's vertSlope + vertSlope-to-segB')/2 : parallel to Jiao'PingFeng'xian.
	double vertRad_mid = (vertAtanRad_C + vertAtanRad_B ) * 0.5; 
	// double vertRad_mid = (vertRadC + vertRadB ) * 0.5; 

	// eg' 2th'xiangxian: tan_'<0;
	// HX_theta2min = fabs(vertRad_mid*RAD2DEG - 90); // degree'
	double HX_theta = vertRad_mid *RAD2DEG;
	// if( HX_theta2min<0 ) HX_theta2min += 180; // never happens due-to fabs();
	if( HX_theta <0 ) HX_theta += 180; // never happens due-to fabs();

	//was'double slopeK_biSector = tan(vertRad_mid);
	double vertRad_mid_k = tan(vertRad_mid);
	// printf(", AGVvertMid %lf,", vertRad_mid*RAD2DEG);

	double JDmid_K;
	// double theta2pntE;

	if (pnt_mid.x != 0){
		JDmid_K = pnt_mid.x / pnt_mid.y ; // vs'JDx'JDy(a=LiuJY)' ==midPnt.x,midPnt.y;
		// theta2pntE = fabs (atan(JD_Ktan) )* RAD2DEG; // use fabs on''atan() coulde be negative value'<0;
	}
	else JDmid_K = 57295779.513082;
	double theta_JDK = atan(JDmid_K) * RAD2DEG;

	// double theta_midPntToMidLine  = atan( fabs( JDmid_K -vertRad_mid_k) / fabs( 1+ JDmid_K* vertRad_mid_k) ) * RAD2DEG;
	double theta_midPntToMidLine = (HX_theta -90) + theta_JDK;
	// eg. see Fig.
	 //:: printf(" LineB'C %lf, %lf", LengthB, LengthC);

		midPnt_p->y = pnt_mid.y;
		midPnt_p->x = pnt_mid.x;
		midPnt_p->dist = pnt_mid.dist;
		midPnt_p->thetaRAD = acos(pnt_mid.x / pnt_mid.dist);
		midPnt_p->thetaDEG = midPnt_p->thetaRAD * RAD2DEG;
		midPnt_p->scan_point_index = (int)(LineSegments[j+1].start_point_index + LineSegments[j].end_point_index)/2;

	cur_segC_indexMatching = j+1;
	//:: printf("\n cur'SegBC'{%lf %lf} 'deg'%f/seg_%d~%d", LengthB, LengthC, Obtuse_A_deg, j, j+1);

	if ( (fabs(LengthC-ConstSegShort)<=segBthres ) && ( fabs(LengthB - ConstSegShort)<=segBthres) && ( fabs(LengthA-Ashort)<=segBthres*1.5) && ( fabs(Obtuse_A_deg - 150)<=8) )
	{
			segBCA_p->HXthetaDock = HX_theta; 
			segBCA_p->theta2vertDock = theta_midPntToMidLine; // VelDock_p

			segBCA_p->lengthBfound = LengthB;
			segBCA_p->lengthCfound = LengthC;
			segBCA_p->lengthAfound = LengthA;
			segBCA_p->Angle150found = Obtuse_A_deg;
			segBCA_p->SegBCindex = j+1;

			// printf("\n F0'SegBC'{%lf %lf} 'deg'%f/seg_%d~%d", LengthB, LengthC, Obtuse_A_deg, j, j+1);
			cur_segC_indexFound = j+1;
			// LengthAfound  = LengthA;
		return true;
	}
 // 2e'TODO'' multiple matching segBC's. +e'wn'Callback'

  } //  else-- joining LineSegs'

  }//end'for'segments.
  
  double Obtuse_ACluster_deg = -1;
  // LengthB = LengthC = LengthA = LengthSqrDiff = -1;

  return false;
}
////


// Vector2<double> pointA denotes 'the point P3 (x3,y3) 'and 'The perpendicular point P0 is the closest point to P3(x3,y3) on the Line<Pt1,Pt2>.
// was':: double DistanceToLine(Vector2<double> linePt1, Vector2<double> linePt2, Vector2<double> point)
// :: double DistanceToLine(Point_t linePt1, Point_t linePt2, Point_t point)

double getLengthP2P(Point_t pt1, Point_t pt2)
{
  double sqrSegmentLength = (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y);
  return sqrt( sqrSegmentLength );
}

double getSegSlope(Point_t pt1, Point_t pt2)
{
  return ( (pt2.y - pt1.y) / (pt2.x - pt1.x) );
}

// fits the line segment 's' to the point set 'pnts[i_low] ... pnts[i_high]'
void Lmap_line_fitting(Lmap_segment_t *segA_p, Point_t *LRF2DPoints, int i_low, int i_high)
{
  int num_of_points = i_high - i_low + 1;
  
  double x_sum = 0.0, y_sum = 0.0;
  for(int i=i_low; i<=i_high; i++)
  {
	x_sum += LRF2DPoints[i].x;
	y_sum += LRF2DPoints[i].y;
  }
  x_sum = x_sum/(double)num_of_points;
  y_sum = y_sum/(double)num_of_points;
  
  double numerator = 0.0, denominator_Length = 0.0;
  double min_x = MAXDOUBLE;
  double min_y = MAXDOUBLE; 
  
  double max_x = -MAXDOUBLE; 
  double max_y = -MAXDOUBLE;
  
  for(int i=i_low; i<=i_high; i++)
  {
	numerator   += (y_sum - LRF2DPoints[i].y) * (x_sum - LRF2DPoints[i].x);
	denominator_Length +=  ( (y_sum - LRF2DPoints[i].y) * (y_sum - LRF2DPoints[i].y) ) - ( (x_sum - LRF2DPoints[i].x) * (x_sum - LRF2DPoints[i].x)  );
	
	if( min_x > LRF2DPoints[i].x )
	{ min_x = LRF2DPoints[i].x; }
	
	if( max_x < LRF2DPoints[i].x )
	{ max_x = LRF2DPoints[i].x; }
	
	if( min_y > LRF2DPoints[i].y )
	{ min_y = LRF2DPoints[i].y; }
	
	if( max_y < LRF2DPoints[i].y )
	{ max_y = LRF2DPoints[i].y; }
  }
  numerator *= -2.0;
  double angle    = atan2( numerator, denominator_Length ) / 2.0;
  double distance = ( x_sum*cos(angle) ) + ( y_sum*sin(angle) );
  
  // segA_p->segLength = distance;
  // segA_p->slope = angle;
  segA_p->num_seg_points = i_high - i_low +1;
  segA_p->start_point_index = i_low;
  segA_p->end_point_index = i_high;
  
  double pntA_x, pntA_y;
  pntA_x = LRF2DPoints[i_low].x;
  pntA_y = LRF2DPoints[i_low].y;

  double pntB_x, pntB_y;
  pntB_x = LRF2DPoints[i_high].x;
  pntB_y = LRF2DPoints[i_high].y;

  Point_t pnt1, pnt2;
  if(angle< pi /4.0 && angle> - pi/4.0)
  {
	pnt1.x = ( distance - (min_y)*sin(angle) ) / cos(angle);
	pnt1.y = min_y;
	pnt2.x = ( distance - (max_y)*sin(angle) ) / cos(angle);
	pnt2.y = max_y;
	//pnt2.x 'smaller'
	// re' switched p1'p2 which were [p2,p1]'ie. reverted wrt cluster_ith[p1,p2] // but still could be reverted(p2,p1)
	if (pntA_y < pntB_y)
	{ segA_p->p1 = pnt1; segA_p->p2 = pnt2;}
	else if (pntA_y > pntB_y)
	{ segA_p->p1 = pnt2; segA_p->p2 = pnt1;}
	// else if (pntA_y == pntB_y) // '+

  }
  else{
 // angle>= M_PI/4.0 || angle<= -M_PI/4.0;
	// should note: seg->(p1,p2 ) could be reverted wrt cluster_ith(p1,p2)
	pnt1.x = min_x;
	pnt1.y = ( distance - (min_x)*cos(angle) ) / sin(angle);
	//pnt2.y 'smaller'
	pnt2.x = max_x;
	pnt2.y = ( distance - (max_x)*cos(angle) ) / sin(angle);
	// re' switched p1'p2 which were [p2,p1]'ie. reverted wrt cluster_ith[p1,p2]// but still could be reverted(p2,p1)

	if (pntA_x < pntB_x)
	{ segA_p->p1 = pnt1; segA_p->p2 = pnt2;}
	else if (pntA_x > pntB_x)
	{ segA_p->p1 = pnt2; segA_p->p2 = pnt1;}
	// else if (pntA_x == pntB_x) '+
  }

  if ( (pntA_y == pntB_y) || (pntA_x == pntB_x) )
  { segA_p->p1 = LRF2DPoints[i_low]; 
    segA_p->p2 = LRF2DPoints[i_high]; 
  }
  segA_p->segLength = distP2Pval(segA_p->p2, segA_p->p1);
  // segA_p->slopeRAD = atan ( (segA_p->p2.y - segA_p->p1.y) / (segA_p->p2.x - segA_p->p1.x) );
  if (segA_p->p2.x != segA_p->p1.x)
	segA_p->slopeK = (segA_p->p2.y - segA_p->p1.y) / (segA_p->p2.x - segA_p->p1.x);
  else
	segA_p->slopeK =  57295779.513082;	 //做下边界处理 tan(89.999999)=57295779.513082;
  segA_p->slopeB = segA_p->p2.y - segA_p->slopeK * segA_p->p2.x;
  // slope_RAD = atan();
  
  // see'This' ::DistanceToLine( linePt1, linePt2, Point_t point); // use (0,0) to find the Perpendicular intersect point onto Seg_B'seg_C;
  Point_t linePt1 = segA_p->p1;
  Point_t linePt2 = segA_p->p2;
  double distSqrSeg = (linePt1.x - linePt2.x) * (linePt1.x - linePt2.x) + (linePt1.y - linePt2.y) * (linePt1.y - linePt2.y);
  // we get u=
  double u = ( (0.0 - linePt1.x) * (linePt2.x - linePt1.x) + (0.0 - linePt1.y) * (linePt2.y - linePt1.y) ) / distSqrSeg;
  
  // Get the point of intersection (x',y') the tangent Line 'from (0,0):
  // Point_t newPoint; {newPoint.x, newPoint.y)
  double xVert = linePt1.x + u * (linePt2.x - linePt1.x);
  double yVert = linePt1.y + u * (linePt2.y - linePt1.y);
  segA_p->xVert= xVert;
  segA_p->yVert= yVert;
  
	double vertSize = sqrt ( xVert * xVert + yVert * yVert );
	double vertAcosRad_i = acos( xVert / vertSize);
  segA_p->slopeVertAcos = vertAcosRad_i;
  segA_p->slopeVert = atan ( yVert /xVert);
	  //was' atan ( yVert /xVert); // == RAD; but 2nd'xiangxian: must add-with pi'

  //:: printf("%d-The line is: X = %lfY + %lf segLength:%lf\n", segA_p->num_seg_points, segA_p->slopeK, segA_p->slopeB, segA_p->segLength);

}

// Vector2<double> pointA denotes 'the point P3 (x3,y3) 'and 'The perpendicular point P0 is the closest point to P3(x3,y3) on the Line<Pt1,Pt2>.
double DistanceToLineJD( Point_t linePt1, Point_t linePt2, Point_t point, Point_t* pntJD)
{
	double segmentSlope = getSegSlope(linePt1, linePt2);
	// This equation '' by Paul Bourke 'October 1988 'see: http://paulbourke.net/geometry/pointlineplane/
	// was'  http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/
	// where 'on this Line', We have point P'  = P1 + u (P2 - P1) // is a point on the line' defined through two points P1 (x1,y1) and P2 (x2,y2)'
	// wrt point P3 (x3,y3) , The perpendicular point P0 is the closest point to P3(x3,y3) on the line satisfies' (P3 - P0) dot (P2 - P1) = 0 
	// ie. [P3 - P1 - u(P2 - P1)] dot (P2 - P1) = 0 
	double distSqr_p1_p2 = (linePt1.x - linePt2.x) * (linePt1.x - linePt2.x) + (linePt1.y - linePt2.y) * (linePt1.y - linePt2.y);
	// we get u=
	double u = ( (point.x - linePt1.x) * (linePt2.x - linePt1.x) + (point.y - linePt1.y) * (linePt2.y - linePt1.y) ) / distSqr_p1_p2;

	// now Get the perpendicular point'
	// ie. the point of intersection (x',y') of the tangent Line:
	// x = x1 + u (x2 - x1)
	// y = y1 + u (y2 - y1) 
	//see: http://paulbourke.net/geometry/pointlineplane/
	// Point_t newPoint;
	pntJD->x = linePt1.x + u * (linePt2.x - linePt1.x);
	pntJD->y = linePt1.y + u * (linePt2.y - linePt1.y);

	// re' This newPoint 'is useful when the (P1,P2) wrt (P1,P3) is obtuse angle >= 90degree'
	//!! We can use u= () and u'=(_)

	// Return the distance from the point to the line
	double distSqr_newP0_p = (point.x - pntJD->x) * (point.x - pntJD->x) + (point.y - pntJD->y) * (point.y - pntJD->y);
	double dist_newP0_p = sqrt( distSqr_newP0_p );
	// printf("\n dist_P3_seg=newP0_ %lf, segmentDeg(DEG)= %lf ", dist_newP0_p, atan(segmentSlope) *180/pi );
	return dist_newP0_p;
}

// fits the line segment 's' to the point set 'pnts[i_low] ... pnts[i_high]'
void Lmap_line_fittingLS(Lmap_segment_t *segA_p, Point_t *LRF2DPoints, int i_low, int i_high)
{
	double k1, b1, xhe = 0, yhe = 0, xyhe = 0, x2he = 0, y2he = 0;
	// +ex' :: Lmap_line_fitting()'from'Carmen-0.6.5-beta' 'better than' 最小二乘法'

	for( int i=i_low; i<i_high; i++ )
	{
		xhe += LRF2DPoints[i].x;
		yhe += LRF2DPoints[i].y;
		xyhe += LRF2DPoints[i].x*LRF2DPoints[i].y;
		x2he += LRF2DPoints[i].x*LRF2DPoints[i].x;
		y2he += LRF2DPoints[i].y*LRF2DPoints[i].y;
	}
	int n1 = i_high-i_low+1;
	if(y2he-yhe*yhe/n1  !=0)
	{
		k1 = ( xyhe-(xhe*yhe)/n1 ) / ( y2he - yhe*yhe/n1 );
		b1 = xhe/n1 - k1*yhe/n1;
		// k1 = -1.0 * k1;
		//printf("%d\nThe line is: X = %lfY + %lf ", n1, k1, b1);
	}

	Point_t pnt1, pnt2; // JD_1, JD2
	pnt1.x = LRF2DPoints[i_low].x; 
	pnt1.y = k1 * pnt1.x + b1;

	pnt2.x = LRF2DPoints[i_high].x; 
	pnt2.y = k1 * pnt2.x + b1;

	segA_p->p2 = pnt1;
	segA_p->p1 = pnt2; 
	// segA_p->segLength = distance;
	// segA_p->slope = angle;
	segA_p->num_seg_points = i_high - i_low +1;
	segA_p->start_point_index = i_low;
	segA_p->end_point_index = i_high;

	// else if (pntA_x == pntB_x) '+

	segA_p->segLength = distP2Pval(segA_p->p2, segA_p->p1);
	// segA_p->slopeRAD = atan ( (segA_p->p2.y - segA_p->p1.y) / (segA_p->p2.x - segA_p->p1.x) );
	segA_p->slopeK = k1;
	segA_p->slopeB = b1;
	// was' segA_p->slopeK = (segA_p->p2.y - segA_p->p1.y) / (segA_p->p2.x - segA_p->p1.x);
	// slope_RAD = atan();
	// .slope

	// 2ex'2follow'This'see' ::DistanceToLine( linePt1, linePt2, Point_t point, Point_t* pnt_intersect_p); // use (0,0) to find the Perpendicular intersect point onto Seg_B'seg_C;
	// Point_t pntZero;
	// pntZero.x=0; 	pntZero.y=0;
	Point_t linePt1 = segA_p->p1;
	Point_t linePt2 = segA_p->p2;
	// DistToLineJD(pnt1, pnt2，pntZero, &pnt2);
	double distSqrSeg = (linePt1.x - linePt2.x) * (linePt1.x - linePt2.x) + (linePt1.y - linePt2.y) * (linePt1.y - linePt2.y);
	// we get u=
	double u = ( (0.0 - linePt1.x) * (linePt2.x - linePt1.x) + (0.0 - linePt1.y) * (linePt2.y - linePt1.y) ) / distSqrSeg;

	// Point_t newPoint; {newPoint.x, newPoint.y)
	double xVert = linePt1.x + u * (linePt2.x - linePt1.x);
	double yVert = linePt1.y + u * (linePt2.y - linePt1.y);
	segA_p->xVert= xVert;
	segA_p->yVert= yVert;

	double vertSize = sqrt ( xVert * xVert + yVert * yVert );
	double vertAcosRad_i = acos( xVert / vertSize);
	segA_p->slopeVertAcos = vertAcosRad_i;
	segA_p->slopeVert = atan ( yVert /xVert);
	//was' atan ( yVert /xVert); // == RAD; but 2nd'xiangxian: must add-with pi'
	printf("%d-The line is: X = %lfY + %lf segLength:%lf\n", n1, k1, b1, segA_p->segLength);

}
// Vector2<double> pointA denotes 'the point P3 (x3,y3) 'and 'The perpendicular point P0 is the closest point to P3(x3,y3) on the Line<Pt1,Pt2>.
double DistanceToLine(Point_t linePt1, Point_t linePt2, Point_t point)
{
  double segmentSlope = getSegSlope(linePt1, linePt2);
  // This equation '' by Paul Bourke 'October 1988 'see: http://paulbourke.net/geometry/pointlineplane/
  // was'  http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/
  // where 'on this Line', We have point P'  = P1 + u (P2 - P1) // is a point on the line' defined through two points P1 (x1,y1) and P2 (x2,y2)'
  // wrt point P3 (x3,y3) , The perpendicular point P0 is the closest point to P3(x3,y3) on the line satisfies' (P3 - P0) dot (P2 - P1) = 0 
  // ie. [P3 - P1 - u(P2 - P1)] dot (P2 - P1) = 0 
  double distSqr_p1_p2 = (linePt1.x - linePt2.x) * (linePt1.x - linePt2.x) + (linePt1.y - linePt2.y) * (linePt1.y - linePt2.y);
  // we get u=
  double u = ( (point.x - linePt1.x) * (linePt2.x - linePt1.x) + (point.y - linePt1.y) * (linePt2.y - linePt1.y) ) / distSqr_p1_p2;
  
  // now Get the perpendicular point'
  // ie. the point of intersection (x',y') of the tangent Line:
  // x = x1 + u (x2 - x1)
  // y = y1 + u (y2 - y1) 
  //see: http://paulbourke.net/geometry/pointlineplane/
  Point_t newPoint;
  newPoint.x = linePt1.x + u * (linePt2.x - linePt1.x);
  newPoint.y = linePt1.y + u * (linePt2.y - linePt1.y);
  
  // re' This newPoint 'is useful when the (P1,P2) wrt (P1,P3) is obtuse angle >= 90degree'
  //!! We can use u= () and u'=(_)
  
  // Return the distance from the point to the line
  double distSqr_newP0_p = (point.x - newPoint.x) * (point.x - newPoint.x) + (point.y - newPoint.y) * (point.y - newPoint.y);
  double dist_newP0_p = sqrt( distSqr_newP0_p );
  // printf("\n dist_P3_seg=newP0_ %lf, segmentDeg(DEG)= %lf ", dist_newP0_p, atan(segmentSlope) *180/pi );
  return dist_newP0_p;
}

int FitLineScanXY(int pt1, int pt2, Mesg_Laser LRFdata)
{
	return 0;
}

// int FitLine(int pt1, int pt2, Point_t *ScanPoints)
int FitLineZone(int pt1, int pt2, Point_t *ScanPoints, int tinyDeg)
{
  //printf(" \ curSeg_= %d ", segs_Index);
  // printf("\n _[0~540].range: %lf, %lf, %lf, %lf, %lf", ScanPoints[0].dist, ScanPoints[135].dist, ScanPoints[270].dist, ScanPoints[405].dist, ScanPoints[540].dist);
   if (pt1==0 && pt2==(LMS270deg_DATA_length-1) ) {
	  segs_Index =0;
	  lineCount =0;
  }

  int num_of_points = pt2 - pt1 + 1;  
  // check number of points
  if( num_of_points < samg_min_num ) {
	  return 0; 
  }
  
  // 2 - Find the gratest distance on the given line
  double dist_tmp = 0.0;	// The given point distance
  int pointIndex = 0;		// The point index
  
  if (pt1 >= pt2) return 0;
  int i=pt1;
  int j = pt2;

//TODO
  // Zone: ie.'excl'two-side pnts'with range_i>=4m'
   int j_tmp = pt2; 
  for (int i_tmp = pt1; (i_tmp <= pt2) &&  (j_tmp >= pt1); i_tmp++){
    if ( (ScanPoints[i].dist >= 4000 ) || i< (270 - tinyDeg*2) ) // 20deg' 
	{ pt1 = ++i; } 
    if ( (ScanPoints[j].dist >= 4000 ) || j> (270 + tinyDeg*2) ) 
	{ pt2 = --j;  }

	j_tmp++;
	// if ( (ScanPoints[i].dist !=0 ) && (ScanPoints[j].dist !=0 ) ) break;

  }
  //:: printf("\n ptA %d, ptB %d; i_range_%dth=%lf; j_range_%dth=%lf", pt1, pt2, pt1-1, ScanPoints[i].dist, pt2+1, ScanPoints[j+1].dist); // re'2e'offset=1'
	//TODO' sparse scan-data'eg' both end-points being {0.00,0.00}

  if (pt1 >= pt2) 
		return 0;

  for( i = pt1; i <= pt2; i++) {
	double dist = DistanceToLine(ScanPoints[pt1], ScanPoints[pt2], ScanPoints[i]);
	if(dist > dist_tmp) {
	  pointIndex = i;
	  dist_tmp = dist;
	}
  }
  // printf("\n %d'th point for splitting; dist2curLine %lf; dynThres %lf", pointIndex, dist_tmp, SEG_DLineDistThreshold );
  // 'Split-and-Merge 方法的主要思想是，
  // above. (1).先用区域的两个端点拟合一条直线，'计算区域内各个数据点'到该直线的距离，'选取距离最大的数据点，
  // 2. '以距离(此两端点所成直线)最大的该数据点为边界，将区域划分为'两个，分别用同样的方法对两个区域再次进行划分。
  // 以此类推，直到区域的分割点'距离直线(区域端点所成直线)'的距离值'小于一定阈值为止。
  double distAB = distP2Pval(ScanPoints[pt1], ScanPoints[pt2]);
  // when distAB <=3.0m use 动态阈值 D_thr = L * \rho. // 取 \rho = 0.05. // ref' Msc'09'nudt,基于激光测距仪的移动机器人二维地图创建问题研究  渠瀛  国防科学技术大学  2009-11-01  硕士. 导师】 张辉.nudt
  double SEG_DLineDistThreshold = distAB * 0.05;
  // double SEG_SLineDistThreshold = 1.0; // eg. SEG_LineDistBounds; // eg. 0.3'const // static threshold.

  if (distAB < 500) //50cm
	  SEG_DLineDistThreshold = 32; // mm'2e'eg.30mm' 33mm

  // // when distAB >=400mm use static threshold-val.
  if (distAB > 500) 
	  SEG_DLineDistThreshold = 33; // mm
  
  // 3 - Is it out of our bounds? (Beta in the algorithm paper)
  //vs' const' ==0.3; const' samg_max_gap' was' const SEG_LineDistBounds; 're' fragments' 
  if(dist_tmp > SEG_DLineDistThreshold) 
  //wrt' var' SEG_DLineDistThreshold
  {
	if (pt1 >= pointIndex) return 0;
	// Apply the new function to the two new line segments
	int ret = FitLineZone(pt1, pointIndex, ScanPoints, tinyDeg); // left segment
	  //:: printf(" pt:%d, curSegsNum= %d)", pt1, segs_Index);
    if (pointIndex >= pt2) return 0;
	ret = FitLineZone(pointIndex, pt2, ScanPoints, tinyDeg); // Right segment
	  //:: printf(" (pt:%d curSegsNum= %d", pointIndex, segs_Index);
  }  
  // 4 - All points fit to a line, create a line and pop-back from this function call
  // je 'add'. LineFitting
  else if(dist_tmp <= SEG_DLineDistThreshold) // was' const SEG_LineDistBounds;
  {
	  num_of_points = pt2 - pt1 + 1;
	  // was' >=50 'thus excluded'valid lineSeg'
      if ( (num_of_points >=60 && distAB >= 300) ) {
	  	  //printf("\n &gt MaxNum'ie. %d-pnts 'from'(%d~ %d'th)", samg_min_num, pt1, pt2);
		  return 0; // err!! no chance to work on Big-Clusters to be split-into-twoClusters.
	  }
	  else if( num_of_points < samg_min_num ) {
		  //printf("\n &lt num=%d-pnts. excl'(%d~ %d'th)", samg_min_num, pt1, pt2 );
		  return 0; 
	  }
  
	double distAB = distP2P( &ScanPoints[pt1], &ScanPoints[pt2]);

	Lmap_segment_t segTmp; // Local.
	
	// line-fitting after 'split'-check
	Lmap_line_fitting(&segTmp, ScanPoints, pt1, pt2); 
	
	//segSize must wn'+/-10cm of ConstSegShort':20cm'
	if ( (segTmp.segLength -80 >= ConstSegShort) || (segTmp.segLength +100 <= ConstSegShort) ) //5cm'
	{
		// TODO'		
		return 0;
	}
	// Register the new line points
	LineSegments[segs_Index] = segTmp;
	// Register the new line points
	LineSegments[segs_Index] = segTmp;

	// double slope_RAD = LineSegments[segs_Index].slopeRAD; 
	// double slope_DEG = RAD2DEG* LineSegments[segs_Index].slopeRAD;

	//segFit: could use LineCount
	// Cluster_PointIndex[lineCount].start_point_i = pt1;
	// Cluster_PointIndex[lineCount].end_point_i = pt2;

	++lineCount;
	++segs_Index;
	// segs_Index : numSegs_curScan
  }
  // printf(" \ curSegsNum= %d ", segs_Index);
  return segs_Index;
}
////
void readScanPntsCOG(int DataSize, Mesg_Laser LRFdata2D)
{
  double angle_deg_ith = MIN_DEGREE; 
  double range, dist2COG = -1;
   // read LRF2Dscan'
	
	for(int i=0;i< DataSize;i++) // 270-
	{
		// double angleRad2LRF = LRFdata2D.laser_data(i).angle() *DEG2RAD;	//deg->rad //  + 90
		double angleDeg = LRFdata2D.laser_data(i).angle();
		double angleRad2LRF = ( angleDeg+90)*DEG2RAD;	//deg->rad //  + 90
		//double dist =  LRFdata2D.laser_data(i).dist()/1000;           //mm -> m
		range =  LRFdata2D.laser_data(i).dist();		//mm 所有dist不大于8000
		dist2COG = sqrt(range*range + Center_Sick*Center_Sick - 2*range*Center_Sick*cos(pi+ angleDeg*DEG2RAD ));		// a^2 = b^2+c^2-2bccos(A)

		//fprintf(fp,"%d %lf %lf\n", i, angle, dist1);

		double cosangle = (dist2COG*dist2COG +Center_Sick*Center_Sick-range*range)/(2*dist2COG*Center_Sick);				// cos(b) = (a^2+c^2-b^2)/(2ac)
		double angleRad2COG = acos(cosangle);

		if (i<=270){
			angle_deg_ith = 90 - angleRad2COG*RAD2DEG; // LRFdata2D.laser_data(i).angle(); // SICK-LRF2D'msg:90deg
			angleRAD_Data[i] = pi/2 - angleRad2COG; // angle_deg_ith * DEG2RAD;
			LRF2DdataFrame[i].x = dist2COG * sin( angleRad2COG );
		}
		else if (i>=271){
			LRF2DdataFrame[i].x = (-1) * dist2COG * sin( angleRad2COG); 
			angle_deg_ith = 90 + angleRad2COG*RAD2DEG; // LRFdata2D.laser_data(i).angle(); // SICK-LRF2D'msg:90deg
			angleRAD_Data[i] = pi/2 + angleRad2COG; // angle_deg_ith * DEG2RAD;
		}
		// double angle_rad_ith = angle_deg_ith *pi/180;	//deg->rad
		// printf(" %d", (int)dist2COG);
		
		//只检测2.0m范围内的点
		// if( dist2COG> 3000 ) dist2COG =0;

		rangeData[i]= dist2COG;
		angleDEG_Data[i] = angle_deg_ith;
		// printf("\n range_index %d, angleDeg= %lf, range_ %d =%lf'cm", i, angle_deg_ith, i, rangeData[i] /10 );

		LRF2DdataFrame[i].scan_point_index = i;
		LRF2DdataFrame[i].y = dist2COG * cos( angleRad2COG); 

		// LRF2DdataFrame[index].theta = arcsin(y_ith/x_ith); // ?
		LRF2DdataFrame[i].thetaDEG = angle_deg_ith;
		LRF2DdataFrame[i].thetaRAD = angle_deg_ith*DEG2RAD;
		// LRF2DdataFrame[i].dist = sqrt(x_ith * x_ith + y_ith *y_ith);
		LRF2DdataFrame[i].dist = dist2COG;

	} //end'for-Loop
	 // fclose(fp);
}

////



// +'qv'回调函数updateLaser()，在这里面进行处理并将机器人控制速度发布出去
void updateLaser(Mesg_Laser LRFdata2D)
{ 
	// bool FirstScanTrue =1; // global'var.
	// int segs_num =0; // not used.

	if(Recharge_enable)
	{
 bool foundDocking = 1;
 bool foundDockingSeat = 1;

 if(g_mutex_laser.try_lock())
 {
	int scanDataSize = 0;
	scanDataSize = LRFdata2D.laser_data_size();
	//printf("\n %d'th scan ; size: %d", scanIter, scanDataSize);

	if (scanDataSize ==0) {
		//printf("\n !! %d'th scan-size == %d (0)", ++scanIter, scanDataSize);
		return;
	}

   readScanPntsCOG(scanDataSize, LRFdata2D);
   // readScanPntsLRF(scanDataSize, LRFdata2D);

	  // part-02a: split==dist2LineCluster 'excl' Line-Fitting 'excl'segsMerge'
	  // part-02: LineFitting + split + merge	  

	//   int zoneDeg = 30;//   int zoneDeg = 30;
   int segs_num =0;
   zoneDeg = zoneDegWide; // 2e
   if (preDistYmid >940 ) // wn'93cm'64cm
	   zoneDeg = zoneDeg;
   if (preDistYmid <=940 && preDistYmid >=640) // wn'93cm'64cm
	   zoneDeg = zoneDegHalf;
   if (preDistYmid !=0 && preDistYmid <=640) // wn'93cm'64cm
	   zoneDeg = zoneDegWide;

   if (isFirstScan==true){
	    segs_num = FitLineZone(0, scanDataSize - 1, LRF2DdataFrame, 45);
		isFirstScan=false;
		++seqCurScan;
   }
   if (isFirstScan ==false){
		// segs_num = FitLineScanXY(0, scanDataSize - 1, LRFdata2D);
		segs_num = FitLineZone(0, scanDataSize - 1, LRF2DdataFrame, zoneDeg);
   }
	numSegs_curScan = segs_num;
	segs_Index =0;

	if (numSegs_curScan==0)  {
		printf("\n numSegs==0, ");
		// segs_Index =0;
		// FirstScanTrue =1;
		return;
	}
	Point_t PntLeft, PntRH;
	Point_t midPntA, midPnt2;
	segBCA_t segABC1, segABC2;

	VelDocking_t velDockA, velDock2;
	int ret = -1;
	// ret = segAB_angles_find(&midPntA, &velDockA, &segABC1, 0); 
	ret = segAB_angles_find(&midPntA, &segABC1, 0); 
	// ret = segAB_angles_find(&midPnt, &PntLeft, &PntRH); 
	//:: showLRF2DSegmentsCVmatrix();//openCV
	if (ret ==0){
		//:: printf(" nr_%d not found DockingSeat, midXa %lf, midYa %lf\n", no_find_times, midPntA.x, midPntA.y);
		// printf("%d  x=%d,y=%d\n", weight,matchVal_x*2,matchVal_y*2-100);
		no_find_times++;
		//cur_vel.set_vx(0);  
		//cur_vel.set_vy(0); 
		//cur_vel.set_w(0);
		if( no_find_times>500 )
		{  // 超过500次没有发现充电桩
			//feedbackInfo.set_feedback_type(feedbackInfo.No_Find_Station);
			//SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo);
			enter_flag = false;
		}
       cur_segC_indexMatching =0;
		cur_segC_indexFound=0;
		return;
		//SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.No_Find_Station);
	}
	double prevBtoThres, prevCtoThres, prevAtoThres, prevDegtoThres;
	double curBtoThres, curCtoThres, curAtoThres, curDegtoThres;
		prevBtoThres = fabs(segABC1.lengthBfound -ConstSegShort);
		prevCtoThres = fabs(segABC1.lengthCfound -ConstSegShort);
		prevAtoThres = fabs(segABC1.lengthAfound -ConstSegShort*1.932); // cos15deg*2
		prevDegtoThres = fabs(segABC1.Angle150found -150);

	bool isVaryMidPntA =false;
	if (seqCurScan !=1) {
		isVaryMidPntA = ( fabs(midPntA.thetaDEG - preMidPnt.thetaDEG) >= 50 || fabs(midPntA.x - preMidPnt.x )>=500  || fabs(midPntA.dist - preMidPnt.dist )>=500 ); // 200mm
		// ie. MidPntAA ==false'
	}

	bool isWorsePrevSegBC = false;
	if (isVaryMidPntA == true)
		isWorsePrevSegBC = true; // ie. MidPntAA ==false'
	else if (isVaryMidPntA == false)
		isWorsePrevSegBC = ( prevBtoThres + prevCtoThres + prevAtoThres + prevDegtoThres > curBtoThres + curCtoThres + curAtoThres + curDegtoThres) ||  ( prevBtoThres > curBtoThres && prevCtoThres > curCtoThres && prevAtoThres > curAtoThres );

	if (ret==1 && enter_flag && fabs(midPntA.x) <= 999) 
		printf("\n F0'SegBC'{%lf %lf} 'deg'%f/seg_%d", segABC1.lengthBfound, segABC1.lengthCfound, segABC1.Angle150found, cur_segC_indexMatching);

	bool ret2=false;
	if (ret==1 && fabs(midPntA.x) >= 999 ) 
	    ret2 = segAB_angles_find(&midPnt2, &segABC2, cur_segC_indexMatching);  // cur_segC_indexFound seems bad'
	
	bool SegBC2ndOK = false;
	for (; ret2 == true && cur_segC_indexMatching < numSegs_curScan && midPnt2.x <= 590 && midPnt2.x > -590; ) // && cur_segC_indexFound!=0 // -99cm
	{
		if (enter_flag) printf("\n midX,midY {%lf,%lf}, distMid2 %lf", midPnt2.x, midPnt2.y, midPnt2.dist);
		if (enter_flag) printf("\n F2'SegBC'{%lf %lf} 'deg'%f/seg_%d", segABC2.lengthBfound, segABC2.lengthCfound, segABC2.Angle150found, cur_segC_indexMatching);
		// ret2 = segAB_angles_find(&midPnt2, &velDock2, &segABC1, cur_segC_indexFound); 
		curBtoThres = fabs(segABC2.lengthBfound -ConstSegShort);
		curCtoThres = fabs(segABC2.lengthCfound  -ConstSegShort);
		curAtoThres = fabs(segABC2.lengthAfound -ConstSegShort*1.932); // cos15deg*2
		curDegtoThres = fabs(segABC2.Angle150found -150);
		// segABC1.SegBCindex;

		bool isVaryMidPnt2 =true;
		isWorsePrevSegBC = false;
		//0e'y15m10d09+1' midPnt2.dist <= midPntA.dist'
		isWorsePrevSegBC = ( midPnt2.dist <= midPntA.dist ) || (prevBtoThres + prevCtoThres + prevAtoThres + prevDegtoThres > curBtoThres + curCtoThres + curAtoThres + curDegtoThres) ||  ( prevBtoThres > curBtoThres && prevCtoThres > curCtoThres && prevAtoThres > curAtoThres );

		// bool isVaryMidPnt = false;
		if (seqCurScan !=1) {
			isVaryMidPnt2 = ( fabs(midPnt2.thetaDEG - preMidPnt.thetaDEG) >= 10 || fabs(midPnt2.x - preMidPnt.x )>=500  || fabs(midPnt2.dist - preMidPnt.dist )>=500 ); // 200mm'50cm
		}
		// if (isVaryMidPnt2==true) printf("\n midPnt2'vary {%lf %lf}, do-nothing ", midPnt2.x, midPnt2.y);

		// SegBC2ndOK = false;
		if ( isWorsePrevSegBC && isVaryMidPnt2==false && fabs(midPnt2.x) <= 999) //wn'99cm
		{
			// // only not-varying midPnt can be chosen.
			if (enter_flag) printf("\n curSegBC'{%lf %lf} deg'%lf'vs' Worse'BC'%lf %lf. andle %lf'deg", segABC2.lengthBfound, segABC2.lengthCfound, segABC2.Angle150found, segABC1.lengthBfound, segABC1.lengthCfound, segABC1.Angle150found);
			midPntA.y = midPnt2.y;
			midPntA.x = midPnt2.x;
			midPntA.dist = midPnt2.dist;
			midPntA.thetaRAD = midPnt2.thetaRAD;
			midPntA.thetaDEG = midPnt2.thetaDEG;
			midPntA.scan_point_index = midPnt2.scan_point_index;
			SegBC2ndOK = true;

			// segABC1.HXthetaDock = velDock2.HXthetaDock;
			// segABC1.theta2vertDock = velDock2.theta2vertDock;
			segABC1.theta2vertDock = segABC2.theta2vertDock;
			segABC1.HXthetaDock = segABC2.HXthetaDock;
			
			prevBtoThres = curBtoThres;
			prevCtoThres = curCtoThres;
			prevAtoThres = curAtoThres;
			prevDegtoThres = curDegtoThres;
		}
		ret2 = segAB_angles_find(&midPnt2, &segABC2, cur_segC_indexMatching);  // cur_segC_indexFound seems bad'
		// else 'prevB'C'better.
	} // end-for-Loop.
	if (seqCurScan ==1)	{
		++seqCurScan;
	}
	if (isVaryMidPntA == true && SegBC2ndOK == false){
		printf("\n _ ret_ isVaryMidPntA && SegBC2ndOK false, discard cur'%d'th'scan ", scanIter);
		isVaryMidPntA =false;
		SegBC2ndOK = true;
		// printf("\n _ ret _ err: excl'midPnts'curScan");
		//return;
	}
	if (isVaryMidPntA ==false){
		preMidPnt.y = midPntA.y;
		preMidPnt.x = midPntA.x;
		preMidPnt.dist = midPntA.dist;
		preMidPnt.thetaRAD = midPntA.thetaRAD ;
		preMidPnt.thetaDEG = midPntA.thetaDEG ;
		preMidPnt.scan_point_index = midPntA.scan_point_index ;
	}
	if (isVaryMidPntA == true && SegBC2ndOK == false){
		printf("\n _ ret _ err: excl'midPnts'curScan");
		zoneDegHalf -=5;

		return;
	}

	// if (ret2 ==0) //OK.
		cur_segC_indexFound=0;
       cur_segC_indexMatching =0;

	   //TODO'
	if (preDistYmid <= DistYtoDockStation){
		bool isYdocked =true;
	 }
	preDistYmid= midPntA.y;

	// HX_theta = segABC1.HX_theta;
	// if (ret==1) printf("\n midX,midY2COG {%lf,%lf}, dist2mid %lf", midPntA.x, midPntA.y, midPntA.dist);
	// printf(" dist2mid %lf ",  midPntA.dist);

	double midXfound, midYfound, midDistfound, midAnglefound;
	double midYtoCOG =  midPntA.y;
	 // re' segAB_angles_find() assigned midPnt_p->y' midPnt_p->x  'midPnt_p->scan_point_index '
	midXfound = midPntA.x; // midPnt.x
	midDistfound= midPntA.dist;
	midYfound = midPntA.y; // re'COG' // already offset by Center2COG; neednot add Center2COG here.
  // midYfound = midPntA.y + Center2COG; // re'COG''wrt'LRF'

	double JDmidA_K;
	if (midPntA.x != 0){
		JDmidA_K = midXfound / midYfound ; // vs'JDx'JDy(a=LiuJY)' ==midPnt.x,midPnt.y;
		// theta2pntE = fabs (atan(JD_Ktan) )* RAD2DEG; // use fabs on''atan() coulde be negative value'<0;
	}
	else JDmidA_K = 57295779.513082;
	double thetaA_JDK = atan(JDmidA_K) * RAD2DEG;
    
	double HX_theta = segABC1.HXthetaDock; 
	double HXdelta = HX_theta - 90;

	double theta2 = -1.0 * ( (HX_theta -90) + thetaA_JDK );

	double theta = -1.0 * segABC1.theta2vertDock;
	// double theta = segABC1.theta2vertDock;

	if(pre_theta == 0) {
		pre_theta = theta;
	}
	
	 double rotateXYrad = (HX_theta -90)*DEG2RAD;

	 double miX_nH = midXfound * cos(rotateXYrad) + midYtoCOG * sin(rotateXYrad);
	 double miY_nH = -midXfound * sin(rotateXYrad) + midYtoCOG * cos(rotateXYrad);

	 if (ret==1 && enter_flag){
		 // printf("\n\n theta_m %lf, theta %lf", theta2, theta);
		 printf("\n docking: midXnH,YnH {%lf,%lf}, theta %lf, HX %lf", miX_nH, miY_nH, theta, HX_theta);
		 // printf(" midXm, midYm {%lf,%lf}, HXdelta %lf", midXfound, midYtoCOG, HX_theta -90);
		 // printf(", HXdelta %lf", HX_theta -90);
		 // printf("\n midXnF,midXnF {%lf %lf\n", miX_nF, miY_nF);
	 }
	 if (ret==1 && leave_flag){
		 printf("\n Leaving: HX %lf, theta %lf, midXnH,YnH {%lf,%lf} ", HX_theta, theta, miX_nH, miY_nH);
	 }
	double JiaoDianJDx = fabs(midYfound); //ref'a=LiuJY': JDx 'at y-axis
	// int LengthGn2Mid = 1100;
	double yG_n = miY_nH - 900; // LengthToMid// 90cm'
	double xG_n = miX_nH;
	double dist2xyG_n = sqrt (xG_n* xG_n + yG_n*yG_n);

	double pntGdeg =90;
	if (xG_n !=0)
		pntGdeg = atan(yG_n / xG_n) * RAD2DEG;
	else pntGdeg =90;

	double yawRateK =0;
	double speedK =0;
	
	
 // if(enter_flag)
//  if(isDocked!=true)
//if(1)
if(enter_flag)
  {
    int safeRangeYtoLRF = Safe_dist - 110; //50cm'49cm
	if( miY_nH <= DistYtoDockStation &&  miY_nH < (DistYtoDockStation -52) && fabs(miY_nH - prevOKmidY) >77 ) 
	{
			cur_vel.set_vx(constVel *0.2/1000.0);  
			cur_vel.set_vy(0); 
			cur_vel.set_w( yawRateK );
	}
    if( miY_nH <= DistYtoDockStation && miY_nH >= (DistYtoDockStation -52) && fabs(miX_nH) <=30 && fabs(miY_nH - prevOKmidY) <=77 ) //0.6m // x:wn'2cm
    { 
	    isDocked=true;
		// double scanOKnth = scanIter;
	    cur_vel.set_vx(0);  
	    cur_vel.set_vy(0); 
	    cur_vel.set_w(0);
		yawRateK =0; speedK = 0;
	    //feedbackInfo.set_flag(feedbackInfo.ECTF_RECHARGE_ENTER_STATION);//set_feedback_type(feedbackInfo.Enter_Station_Succ);	//进站成功
		feedbackInfo.set_flag(feedbackInfo.ECTF_RECHARGE_ENTER_STATION);
		feedbackInfo.set_result(0); // successfully
	    SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
	    printf("\n wn 49cm'OK! midX,midY2COG {%lf,%lf} Enter Station Successfully!\n", midXfound, midYfound);
		//printf(" feedbackInfo %d, Enter Station Successfully!\n", feedbackInfo.feedback_type() );
	    enter_flag = false;
	    //SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.Enter_Station_Succ);
    }
    else if( miY_nH <= DistYtoDockStation &&  miY_nH >= (DistYtoDockStation -52) && fabs(miX_nH) >30 && fabs(miY_nH - prevOKmidY) <=77 ) //0.6m // x:wn'2cm
    { 
	    isDocked=true;
	    cur_vel.set_vx(0);  
	    cur_vel.set_vy(0); 
	    cur_vel.set_w(0);
		yawRateK =0; speedK = 0;
	    feedbackInfo.set_flag(feedbackInfo.ECTF_RECHARGE_ENTER_STATION);//feedbackInfo.set_feedback_type(feedbackInfo.Enter_Station_Succ);	//进站成功
		feedbackInfo.set_result(0); // successfully
	    SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
		printf("\n X_nH >2cm' wn 49cm'! midX,midY2COG {%lf,%lf} HX'deg: %lf !\n", midXfound, midYfound, HX_theta);
		//printf(" feedbackInfo %d, Enter Station Successfully!\n", feedbackInfo.feedback_type() );
	    enter_flag = false;
	    //SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.Enter_Station_Succ);
    }
  else  // >=60cm' re' 45deg'at'58cm'ie.34cm'COG+'24cm(19.3+5cm)'
  { 
	if( miY_nH < Conf_distmin-100) // wn'80cm'
	{
		isNearDock = false;
		//if(  fabs(HXdelta) > 1.0 ) 
		{
			// printf("\n  wn'90cm: vx =35; yawRate %lf _eg. |HXdelta|>1.0", HX_theta-90);
			yawRateK = (HX_theta-90)*YawRateScale;
			speedK = 0;
			cur_vel.set_vx(constVel*0.9/1000.0);  
			cur_vel.set_vy(0); 
			cur_vel.set_w( yawRateK );
		}
		//else { // | HXdelta| <= 1.0
		//	printf("\n  wn'90cm: vx =35.  yawRate Zero.");
		//	cur_vel.set_vx(constVel);  
		//	cur_vel.set_vy(0); 
		//	cur_vel.set_w(0);
		//}
	} // &lt 90cm
	else if( miY_nH < Conf_distmin && miY_nH >= (Conf_distmin -100) ) // wn'90cm'
	{
		isNearDock = false;
		//if(  fabs(HXdelta) > 1.0 ) 
		{
			// printf("\n  wn'90cm: vx =35; yawRate %lf _eg. |HXdelta|>1.0", HX_theta-90);
			yawRateK = (HX_theta-90)*YawRateScale;
			speedK = 0;
			cur_vel.set_vx(constVel*0.9/1000.0);  
			cur_vel.set_vy(0); 
			cur_vel.set_w( yawRateK );
		}
		//else { // | HXdelta| <= 1.0
		//	printf("\n  wn'90cm: vx =35.  yawRate Zero.");
		//	cur_vel.set_vx(constVel);  
		//	cur_vel.set_vy(0); 
		//	cur_vel.set_w(0);
		//}
	} // &lt 90cm

	// wn'90cm'93cm'
	// if( midYtoCOG< (Conf_distmax- Center2COG) && midYtoCOG>(Conf_distmin -Center2COG))
	else if( miY_nH<Conf_distmax && miY_nH>Conf_distmin)
	{
		isNearDock = false;
		//偏差过大，直接认为进站失败
		//if( fabs(theta)>10 ) // eg. 20deg'
		if(fabs(miX_nH)>30) // 3cm
		{
			yawRateK =0;
			speedK = 0;
			cur_vel.set_vx(0);  
			cur_vel.set_vy(0); 
			cur_vel.set_w(0);
			feedbackInfo.set_flag(Mesg_ControlTask::ECTF_RECHARGE_ENTER_STATION);
			feedbackInfo.set_result(1);	//failed
			SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
			printf(" wn'(90cm,93cm) theta Deviation %lf deg 'too LARGE! Enter Station Failed! HX: %lf \n", theta, HX_theta);
			enter_flag = false;
			//SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.Enter_Station_Fail);
		}
		else if( fabs(HXdelta) > 1.0 ) 
		{
			isNearDock = false;
			// printf("\n  wn'(90cm,93cm): vx =35; yawRate %lf _eg. |HXdelta| >1.0", HX_theta-90);
			yawRateK =(HX_theta-90)*YawRateScale ;
			speedK = 35;
			cur_vel.set_vx(constVel*0.2/1000.0);  //==5
			cur_vel.set_vy(0); 	
			cur_vel.set_w( yawRateK );
		}
		else {		
			//eg' ( |HXdelta| <=1.0)
			// printf("\n wn'(90cm,93cm): vx =35; yawRate Zero");
			yawRateK =0; speedK = 35;
			cur_vel.set_vx(constVel*0.2/1000.0);  
			cur_vel.set_vy(0); 
			cur_vel.set_w( (HX_theta-90)*YawRateScale );
		}
	} // wn'93cm'90cm'

	else   // beyond'(90cm',93cm)
	{
		isNearDock = false;
		if( fabs(miX_nH)<30 ) //3cm
		{
			if( HXdelta > 2.0 || HXdelta <- 2.0 ) { // ie. |HX_theta -90| >1.0
				// printf("\n 93cm+' vx =35; yawRate %lf _eg. |HXdelta|>1.0", HX_theta-90);
				yawRateK = theta*YawRateScaleF;
				speedK = 35;
				cur_vel.set_vx(constVel/1000.0);  
				cur_vel.set_vy(0); 	
				cur_vel.set_w( theta*YawRateScaleF); // *0.001
			}
			else // | HXdelta| &lt 1.0
			{		
				// printf("\n 93cm+' vx =35; yawRate Zero");
				yawRateK= (HX_theta-90)*YawRateScaleF + theta*YawRateScaleF;
				speedK = 35;
				cur_vel.set_vx(constVel/1000.0);  
				cur_vel.set_vy(0); 
				cur_vel.set_w( yawRateK ); // *0.001
			}
		} // |theta| &lt 1.5

		// |theta| &gt 1.5
		else 
		{
			if(fabs(miX_nH)>300)//see LiuJY'commented' 30cm
			{
				isNearDock = false;
					// must be run'at first scan.  /// |theta - preTheta | &gte 1.5
					if(fabs(theta-pre_theta)>5.0)
					{
						yawRateK =0; 
						speedK = 35;
						cur_vel.set_vx(constVel/1000.0);  
						cur_vel.set_vy(0);
						cur_vel.set_w(0);
						// printf("\n vx =35; yawRate Zero, theta %lf  ~ pre_theta %lf >>5deg ", theta, pre_theta );
					}
					else
					{
						// if(miX_nH>0) printf("\n miXnH %lf >0, vx =35; yaw(HX-120) %lf , theta %lf ~ pre_theta %lf", miX_nH*0.1, (HX_theta-120+theta)*YawRateScale, theta, pre_theta );
						// else if(miX_nH<=0) printf("\n miXnH %lf <0, vx =35; yaw(HX-60) %lf , theta %lf  ~ pre_theta %lf", miX_nH*0.1, (HX_theta-60+theta)*YawRateScale, theta, pre_theta );
						speedK = 35;
						if(miX_nH>0) yawRateK = (HX_theta-120)*YawRateScale + theta*YawRateScale;
						else if(miX_nH>0) yawRateK = (HX_theta-60)*YawRateScale + theta*YawRateScale;

						cur_vel.set_vx(constVel/1000.0);  
						cur_vel.set_vy(0); 

						if(miX_nH>0)
							cur_vel.set_w( yawRateK);
							// cur_vel.set_w( (HX_theta-120)*YawRateScale + theta*YawRateScale);
						else
							cur_vel.set_w( yawRateK);
							// cur_vel.set_w( (HX_theta-60)*YawRateScale + theta*YawRateScale);
						// see LiuJY 'commented'//see LiuJY
					}
			}
			else // (fabs(miX_nH)<=300)
			{
				isNearDock = false;
					// must be run'at first scan.  /// |theta - preTheta | &gte 1.5
					yawRateK = (HX_theta-(90+miX_nH*0.1) + theta)*YawRateScale;
					speedK = 35;
					// printf("\n vx =35; yawRate %lf, miXnH %lf'cm, theta %lf, (theta -pre_theta) %lf",  (HX_theta-(90+miX_nH*0.1) + theta)*YawRateScale, miX_nH*0.1, theta, theta-pre_theta );
					cur_vel.set_vx(constVel/1000.0);  
					cur_vel.set_vy(0);
						cur_vel.set_w( yawRateK ); // (HX_theta-(90+miX_nH*0.1))*YawRateScale + theta*YawRateScale

					// see LiuJY 'commented'
				//see LiuJY
			}
		} /// |theta| &gte 1.5
	}  // beyond'(90cm',93cm)

	} // beyond''60cm'60cm'

	if (foundDocking == false){
				printf("进站第%d次没有发现充电桩！\n", no_find_times);
				// printf("%d  x=%d,y=%d\n", weight,matchVal_x*2,matchVal_y*2-100);

				no_find_times++;
				speedK=0; yawRateK =0;
				cur_vel.set_vx(0);  
				cur_vel.set_vy(0); 
				cur_vel.set_w(0);
				if( no_find_times>500 )
				{									// 超过500次没有发现充电桩
					feedbackInfo.set_flag(Mesg_ControlTask::ECTF_RECHARGE_ENTER_STATION);
					feedbackInfo.set_result(2);	// no find station
					SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
					enter_flag = false;
				}
				//SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.No_Find_Station);

	} // was' not found Matching Template [23]

		SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_vel);
		printf("\n vx =%lf; yawRate %lf, miXnH %lf'cm, HXdelta %lf, theta %lf, (theta-prev) %lf", speedK, yawRateK, miX_nH*0.1, HXdelta, theta, theta-pre_theta );
		// printf("当前Vx=%lf，当前角速度=%lf\n",cur_vel.vx(), cur_vel.w());
		// printf("Vx=%lf, yawRate=%lf\n",cur_vel.vx(), cur_vel.w());
		pre_theta = theta;
		prevOKmidY = miY_nH;
 
	// double midXfound, midYfound;
	double angle150found, LengthBfound, LengthCfound; // Global var;


  ++scanIter;
 } // (enter_flag)

 if(leave_flag || leave_flagTo == true)
 {
	// Point_t midPnt3;
	// segBCA_t segABC3;
	bool retChuZhan = true; // already ensured' already Found midPnt.
		// bool retChuZhan = segAB_angles_find(&midPnt3, &segABC3, 0); 
	    // bool retChuZhan = true;

		if (retChuZhan == true)
		{
			// midPntA
				// printf("发现充电桩!\n");
				double HX_theta3 = HX_theta; // segABC3.HXthetaDock;
				double mid_HX_theta3 = fabs(90-HX_theta3);
				printf("\n Leaving: midXc, midYc {%lf,%lf}, HXdelta3 %lf", miX_nH, miY_nH, HX_theta3 -90); // midPntA.x

					// double theta3 = -1.0 * segABC3.theta2vertDock;
						// 导航过程 已知夹角theta与航向角HX_theta，给出最优的Vx与w

						if( miY_nH >Conf_distmax+400 )					// 出站成功 midPntA.y // >=1m
						{
							cur_vel.set_vx(0);  
							cur_vel.set_vy(0); 
							cur_vel.set_w(0);
							printf(" midY {%lf} >=93cm Leave Station Successfully!\n", midPntA.y);
							feedbackInfo.set_flag(Mesg_ControlTask::ECTF_RECHARGE_LEAVE_STATION);
							feedbackInfo.set_result(0);	// leave successfully
							SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
							leave_flag = false;
							enter_flag = false;
						}
						else
						{
							if( mid_HX_theta3 > 1.0 )
							{
								cur_vel.set_vx(-1.2*constVel/1000.0);  
								cur_vel.set_vy(0); 						
								cur_vel.set_w(-1.0*(HX_theta3-90)*0.001);
							}
							else // HXdelta <=1.0 deg;
							{										
								cur_vel.set_vx(-1.4*constVel/1000.0);  
								cur_vel.set_vy(0); 
								cur_vel.set_w(0);
							}
						}
			} 
			else if (retChuZhan == false)
			{
				printf("出站第%d次没有发现充电桩！\n", no_find_times);
				// printf("%d   %d,%d\n", weight,matchVal_x*2,matchVal_y*2-100);
				no_find_times++;
				cur_vel.set_vx(0);  
				cur_vel.set_vy(0); 
				cur_vel.set_w(0);

				if( no_find_times>30 )										// 超过300次没有发现充电桩
				{
					feedbackInfo.set_flag(Mesg_ControlTask::ECTF_RECHARGE_LEAVE_STATION);
					feedbackInfo.set_result(1);	// Leave Station failed
					printf("Leave Station failed!\n");
					SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
					leave_flag = false;
					enter_flag = false;
				}
			}
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_vel);
			// printf("Leaving: 当前Vx=%lf，当前角速度=%lf\n",cur_vel.vx(), cur_vel.w());
			printf("Leaving: Vx=%lf, yawRate=%lf\n",cur_vel.vx(), cur_vel.w());

   } // leave-flag

    if( enter_flag == false && leave_flag==false && leave_flagTo ==true && no_find_times<=30 && miY_nH <= DistYtoDockStation+20 && fabs(miX_nH) <=100 ) //0.6m // x:wn'2cm
    { 
	    isDocked=true;
	    cur_vel.set_vx(0);  
	    cur_vel.set_vy(0); 
	    cur_vel.set_w(0);
		// yawRateK =0; speedK = 0;
	    feedbackInfo.set_flag(Mesg_ControlTask::ECTF_RECHARGE_ENTER_STATION);	//进站成功
		feedbackInfo.set_result(0);
	    SubPubManager::Instance()->m_controltask.GetPublisher()->publish(feedbackInfo);
		printf("\n docked'OK!  HX %lf, theta %lf, midX,midYd {%lf,%lf} Entered Station!", HX_theta, theta, miX_nH*0.1, miY_nH*0.1 ); // midYfound
		printf(" Yh270: %lf, XYh-1:{%lf, %lf}, XYh+1:{%lf, %lf},\n", LRF2DdataFrame[270].y*0.1, LRF2DdataFrame[269].x*0.1, LRF2DdataFrame[269].y*0.1, LRF2DdataFrame[271].x*0.1, LRF2DdataFrame[271].y*0.1); 
		printf(" \n");
		++scanIter;
	    enter_flag = false;
		leave_flagTo = false;
	    //SubPubManager::Instance()->m_rechargefeedback.GetPublisher()->publish(feedbackInfo.Enter_Station_Succ);
    }

 }
 g_mutex_laser.unlock();
 }
}

void updateMotionFlags(Mesg_NavigationTask recharge_motion)
{
	printf("get Navi_task it's %d\n",recharge_motion.flag());
	switch(recharge_motion.flag())
	{
		case recharge_motion.CHARGE_GO_IN:	enter_flag = true;	Recharge_enable = true; break;
		case recharge_motion.CHARGE_GO_OUT:	leave_flag = true;	Recharge_enable = true; break;
		default: enter_flag = leave_flag = Recharge_enable = false;	break;
	}
	// for( i=0; i<23; i++ ) subTemplate_y[i] = i*1.0-11.0;
	if(enter_flag)
	{
		printf("get charge_in task\n");
		pre_theta = 0;
		no_find_times = 0;
	//	enter_flag = true;
		leave_flag = false;
		leave_flagTo = false;
	}
	if(leave_flag)
	{
		printf("get charge_out task\n");
		no_find_times = 0;
	//	leave_flag = true;
		enter_flag = false;
		leave_flagTo = false;		
	}
}

int main(int argc,char**argv)
{
	using namespace std;

	//============ROS/SURO publisher=======================//
	std::cout<<"done\nInitializing SURO...";
	//	STD_OUT_DEBUG(Module::Module_Navigation,"is setting up ...");
	//	NODE.init(Module::Module_Navigation);
	//  STD_OUT_DEBUG(Module::Module_RechargeNav,"is setting up ...");
	NODE.init(Module::Module_RechargeNav);

	//	fp = fopen("data1.txt","w");//打开文档，写入

	SubPubManager::Instance()->m_navtask.Initialize(Topic::Topic_NavTask,updateMotionFlags);

	SubPubManager::Instance()->m_laser.Initialize(Topic::Topic_Laser, updateLaser);						//接收激光数据
	SubPubManager::Instance()->m_robotspeed.Initialize(Topic::Topic_Speed, NULL);						//发布机器人速度
	SubPubManager::Instance()->m_controltask.Initialize(Topic::Topic_CtrlTask, NULL);					//发布充电反馈信息

	Sleep(500);

	//进入spin函数，程序循环等待，当激光数据到来后，调用'回调函数updateLaser()，在这里面进行处理并将机器人控制速度发布出去
	NODE.spin();
}
