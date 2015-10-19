/**************************************************************\
File: global.h 
Desc: CARMEN.sf.net

\**************************************************************/

// Inclusion guard
#ifndef POINT_STRUCT_H
#define POINT_STRUCT_H

// #ifdef __cplusplus
// extern "C" { 
// #endif

#include <math.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX

// const double MAXDOUBLE = DBL_MAX;
unsigned int segs_Index = 0;
unsigned int numSegs_curScan = 0;

	//! The split-and-merge algorithm divergent disance from a line to be a new line
	//!  (known as beta in the algorithm paper)
	double SEG_LineDistBounds = 0.3; // mm / ?cm? m ?

//  use the fitting algorithm when merging lines
bool sam_use_fitting_split = false;

const double Lmap_epsilon = 0.001;

	// Split'n'Merge:  max. distance of neighbouring points, thus a segment will be created. 
	 // (E.g. if 'distance>sam_max_gap' then the point set will be splited)
const double samg_max_gap  = 0.3;
	
	// 'defn'= 'minimum number of points on a line segment.
const int samg_min_num = 5; // == 5
  // check number of points '+

	// Split'n'Merge: minimum length of a line segment
const double samg_min_segment_length = 0.4; // mm'

	// Split'n'Merge: max. distance of a point to a segment in the "split"-step
	 // (e.g.: if This distance &lt > sam_tolerance, then the point set will be splited)
	  // re' With smaler values you get less line segments, but more accurate Line-Segments.
const double samg_tolerance  = 0.1;


// Includes
#include "math.h"
// #define SEG_LIST_SIZE 109
#define SEG_LIST_SIZE 340

typedef struct {
  int scan_point_index; // for selected Point_t[] 
  double x;
  double y;
  double thetaRAD;
  double thetaDEG;
  double dist; // use it to exclude invalid out-of-range points' // equals to ROS:msg & suroROS:msg' 'Home_Laser LRFdata2D.laser_data(i).dist()
} Point_t, *Point_p; // Point_t Point wrt local LRF-coordinates' // with LRF-center as origin--(0,0)-of-Local-coordinate.

typedef struct {
 double HXthetaDock; 
 double theta2vertDock; // pnt2COG_to_vertLine;
 // double pre_theta;
} VelDocking_t;

typedef struct {
 double lengthL;
 double asinL; 
 double dist2L;
 Point_t pntNW;
 Point_t pntSW;
 double theta2dock; // pnt2COG_to_vertLine;
 double HXthetaDock; 
} segRoller_t;


typedef struct {
 double lengthBfound;
 double lengthCfound;
 double lengthAfound; 
 double Angle150found;
 double SegBCindex;
 double theta2vertDock; // pnt2COG_to_vertLine;
 double HXthetaDock; 
} segBCA_t;

typedef struct {
 int start_point_i; 
 int end_point_i;
} Cluster_PointIndex_t;

typedef struct{
  // int seg_index;
  Point_t p1, p2;
  double slopeK; // relative slope' wrt'LRF. // slopeRAD use atan or acos?
  double slopeB;
  // see'This' ::DistanceToLine( linePt1, linePt2, Point_t point); // use (0,0) to find the Perpendicular intersect point onto Seg_B'seg_C;
  double slopeVert; // atan: 2nd'xiangxian: must add-with pi'
  double slopeVertAcos; // but 4th'xiangxian: TODO.
  double xVert, yVert;
  double segLength;
  int start_point_index; 
  int end_point_index; 
  int num_seg_points; // called weight;
} Lmap_segment_t;

Lmap_segment_t LineSegments[SEG_LIST_SIZE];
// Point_t lineList[LINE_LIST_SIZE][2];
// std:vector<Lmap_segment_set_t> segVec;
//vs' Lmap_segment_set_t *segSet;
// Lmap_segment_t segTmp;

typedef struct{
  int num_segs;
  Lmap_segment_t *segs_p;
} Lmap_segment_set_t;


typedef struct {
  double x;
  double y;
  double theta;
  double t_vel;
  double r_vel;
} traj_Point_t, *traj_Point_p;

double Lmap_angle_between(Point_p p1, Point_p p2)
{
  return atan2(p2->y - p1->y, p2->x - p1->x);
}

//! Returns the length of this point // eg'as vector
double getLengthVec(Point_t p)
{
	return sqrt(p.x*p.x + p.y*p.y);
}

double distP2Pval(Point_t linePt1, Point_t linePt2) 
{
  return sqrt( (linePt1.x - linePt2.x) * (linePt1.x - linePt2.x) + (linePt1.y - linePt2.y) * (linePt1.y - linePt2.y) );
}

double distP2P(Point_p p1, Point_p p2) 
{
  return sqrt((p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

double square2(double val)
{
  return (val*val);
}

typedef struct{
  double laser_max_length;
  double laser_opening_angle;
  double laser_start_angle;
  double sam_tolerance;
  double sam_max_gap;
  double sam_min_length;
  int    sam_min_num;
  int    sam_use_fit_split; 
  double merge_max_dist;
  double merge_min_relative_overlap;
  double merge_overlap_min_length;
  double merge_uniformly_distribute_dist;
} Lmap_params_t;

#endif // Inclusion guard
