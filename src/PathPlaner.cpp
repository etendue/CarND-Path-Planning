/*
 * PathPlaner.cpp
 *
 *  Created on: Jul 23, 2017
 *      Author: etendue
 */

#include "PathPlaner.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>
#include "spline.h"


using namespace std;

//define constants
const double MAX_VEL = 20;//
const double MPH_50 = 22.35; // 50mph
const double MAX_ACCEL = 10; //10m/s²
const double MAX_JERK = 10; //10m/s3
const double SAFT_DISTANCE = 30; // the distance between front car or back car.
const double CRASH_DISTANCE = 10; //if distance is small than this, change lane may crash
const double CAR_LENGTH = 4;
const double MAX_DISTANCE = 1.0e20; // maximum distance, used when car is not detected.
const double dT= 0.02;//20ms time step


PathPlaner::PathPlaner() {
  ego.sd={0,0};
  ego.v = 0;
  ego.exist = true;
  ego.lane_id =0;
  last_predict_v = 0;
  last_predict_sd.s = numeric_limits<double>::lowest();
  last_predict_sd.d = numeric_limits<double>::lowest();
  max_s = 6945.554;
  last_x = 0;
  last_y = 0;
}

PathPlaner::~PathPlaner() {
  // TODO Auto-generated destructor stub
}
void PathPlaner::processInputData(const InputData& data)
{

	//update the states of self
	ego = data.ego_update;

	if(last_predict_sd.s == numeric_limits<double>::lowest() ){
		last_predict_sd = ego.sd;
		auto xy = getXYFromSD(ego.sd.s, ego.sd.d);
		last_x = xy[0];
		last_y = xy[1];
	}


	//check other vehicles

	env.front.exist = false;
	env.front.sd = {MAX_DISTANCE,MAX_DISTANCE};
	env.front_left.exist = false;
	env.front_left.sd = {MAX_DISTANCE,MAX_DISTANCE};
	env.front_right.exist = false;
	env.front_right.sd = {MAX_DISTANCE,MAX_DISTANCE};
	env.back_left.exist = false;
	env.back_left.sd = {-MAX_DISTANCE,-MAX_DISTANCE};
	env.back_right.exist = false;
	env.back_right.sd = {-MAX_DISTANCE,-MAX_DISTANCE};

	for(int i=0; i<data.sensor_data.size();i++){
		vector<double> other = data.sensor_data[i];
		Car car;
		car.exist = true;
		car.v = sqrt(pow(other[3],2)+pow(other[4],2));
		car.sd.s = other[5];
		car.sd.d = other[6];

		car.lane_id = ceil(other[6] / 4);

		//left side
		if(car.lane_id == (ego.lane_id - 1)){
			//back
			if(car.sd.s < ego.sd.s){
				//first detected or a more near vehicle is detected
				if(!env.back_left.exist || car.sd.s > env.back_left.sd.s){
					env.back_left = car;
				}

			}
			//front
			else{
				if(!env.front_left.exist ||car.sd.s < env.front_left.sd.s){
					env.front_left = car;
				}
			}
		}
		//right side
		else if(car.lane_id== (ego.lane_id +1)){
			//back
			if (car.sd.s < ego.sd.s) {
				//first detected or a more near vehicle is detected
				if (!env.back_right.exist || car.sd.s > env.back_right.sd.s) {
					env.back_right = car;
				}

			}
			//front
			else {
				if (!env.front_right.exist || car.sd.s < env.front_right.sd.s) {
					env.front_right = car;
				}
			}
		}
		// same lane
		else if(car.lane_id == ego.lane_id){
			//check only vehicle in the front
			if (car.sd.s > ego.sd.s) {
				//first detected
				if (!env.front.exist || car.sd.s < env.front.sd.s) {
					env.front = car;
				}
			}
		}
	}
}
bool PathPlaner::generatePrediction()
{
	double score_keep = -99;
	double score_left = -99;
	double score_right= -99;

	int lane_id = ceil(last_predict_sd.d /4);

	score_keep = getKeepLaneScore();
	if (lane_id > 1) {
		score_left = getChangeLeftScore();
	}
	if (lane_id < 3) {
		score_right = getChangeRightScore();
	}

	string movement;
	if(score_keep >= score_left && score_keep >= score_right)
	{
		//choose keep lane
		getKeepLaneTrajectory();
		movement = "↑";
	}else if(score_left > score_right){
		getChangeToLeftLaneTrajectory();
		movement = "←";
	}else{
		getChangeToRightLaneTrajectory();
		movement = "→";
	}

	string car_pos ="|     | ego |     |";
	if(ego.lane_id <2){
		   car_pos ="| ego |     |     |";
	}else if(ego.lane_id>2){
		   car_pos ="|     |     | ego |";
	}

	cout <<" "<<car_pos;

    cout<<"    SPEED:"<<setw(5)<<setprecision(3)<<left<<last_predict_v;
    cout<<"    Next Move:"<<movement;
	cout<<"    SCORES:(LEFT:"<<setw(5)<<setprecision(3)<<left<<score_left;
	cout<<" KEEP:"<<setw(5)<<setprecision(3)<<left<<score_keep;
	cout<<" RIGHT:"<<setw(5)<<setprecision(3)<<left<<score_right;
	cout<<") \r"<<flush;


	return true;

}
void PathPlaner::getChangeToRightLaneTrajectory()
{
	return getChangeToLeftLaneTrajectory(false);
}
void PathPlaner::getChangeToLeftLaneTrajectory(bool left){

	double d;
	bool slowDown = false;
	if(left){
		d = (ego.lane_id - 2) * 4 + 2;
		d = d<2? 2:d;
		if (env.front_left.exist && env.front_left.sd.s - ego.sd.s < SAFT_DISTANCE && env.front_left.v < last_predict_v) {
			slowDown = true;
		}
	}else
	{
		d = (ego.lane_id) * 4 + 2;
		d = d>10? 10:d;
		if (env.front_right.exist && env.front_right.sd.s - ego.sd.s < SAFT_DISTANCE && env.front_right.v < last_predict_v) {
			slowDown = true;
		}
	}

	if (env.front.exist && env.front.sd.s - ego.sd.s < SAFT_DISTANCE
			&& env.front.v < last_predict_v) {
		slowDown = true;
	}

	vector<double> x_path;
	vector<double> y_path;
	double delta_v;

	if (slowDown) {
		//DO slow down
		delta_v =  -2;
		if(last_predict_v > env.front.v)
		{
			delta_v -= last_predict_v - env.front.v;
		}

		delta_v = (last_predict_v + delta_v)< 0 ? -fabs(last_predict_v):delta_v;

	} else {
		double delta_v = (MAX_VEL - last_predict_v) > 2.5 ? 2.5 : (MAX_VEL - last_predict_v);
	}


	generateChangeLanePath(delta_v, d, x_path, y_path);

	for(int i= 0; i<x_path.size();i++)
	{
		last_prediction.x.push_back(x_path[i]);
		last_prediction.y.push_back(y_path[i]);
	}

}
void PathPlaner::getKeepLaneTrajectory(){


  bool slowDown = false;
  if (env.front.exist && env.front.sd.s - ego.sd.s < SAFT_DISTANCE &&env.front.v <last_predict_v) {
    slowDown = true;
  }

  double d = (ego.lane_id-1) * 4 + 2;
  vector<double> x_path;
  vector<double> y_path;

  double delta_v;

  if (slowDown) {
	  //DO slow down
	  delta_v =  -2;
	  if(last_predict_v > env.front.v)
	  {
		  delta_v -= last_predict_v - env.front.v;
	  }

	  delta_v = (last_predict_v + delta_v)< 0 ? -fabs(last_predict_v):delta_v;

  } else{

      delta_v = (MAX_VEL - last_predict_v)>2.5?2.5 : (MAX_VEL - last_predict_v);
  }


  generateKeepLanePath(delta_v, d, x_path, y_path);

  for (int i = 0; i < x_path.size(); i++) {
	  last_prediction.x.push_back(x_path[i]);
	  last_prediction.y.push_back(y_path[i]);
  }

}

void PathPlaner::generateKeepLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path) {
	/***
	 * generate point along d direction.
	 **/

	double t = 1.0;
	double v0 = last_predict_v;
	double s0 = last_predict_sd.s;
	double d0 = last_predict_sd.d;
	double delta_d = d_target - last_predict_sd.d;
	double jerk_s = 0;
	double jerk_d = 0;

	vector<double> waypoints_t(5);
	vector<double> waypoints_x(5);
	vector<double> waypoints_y(5);
	vector<double> waypoints_s(5);
	vector<double> waypoints_d(5,d0);

	bool overSpeed = false;
	do{
		jerk_s = MAX_JERK *copysign(1,dv);
		//check if it hits the front car in case slow down
		if(dv <0)
		{
		     double safe_distance = env.front.sd.s - s0 + env.front.v * t + CAR_LENGTH ;
		     double s_travel = v0*t + jerk_s*pow(t/2,3);
		     while(s_travel > safe_distance)
		     {
		    	 t +=dT;
		    	 safe_distance = env.front.sd.s - s0 + env.front.v * t + CAR_LENGTH ;
		    	 s_travel = v0*t + jerk_s*pow(t/2,3);
		     }
		}

		double jerk_s_needed = dv / pow(t / 2, 2);
		if (fabs(jerk_s_needed) < fabs(jerk_s)) {
			jerk_s = jerk_s_needed;
		}

		double delta_s1 = v0 * (t / 4) + jerk_s * pow(t / 4, 3) / 6;	//distance is third polynomial curve
		double delta_s2 = v0 * (t / 2) + jerk_s * pow(t / 2, 3) / 6;	//distance is third polynomial curve
		double delta_s3 = v0 * (t * 3 / 4)+ jerk_s * pow(t, 3) * 25 / 384;
		double delta_s4 = v0 * (t) + jerk_s * pow(t / 2, 3);

	    jerk_d = sqrt(MAX_JERK*MAX_JERK - jerk_s*jerk_s)*copysign(1,delta_d);
	     // the maximum distance the car can travel with given jerk  -- j*(t/4)³*2 = d

	    if (fabs(jerk_d) > 0.001) {
			double d_max = jerk_d * pow(t / 4, 3) * 2;
			if (fabs(delta_d) < fabs(d_max)) {
				jerk_d = delta_d/(pow(t / 4, 3) *2);
			}
			double delta_d1 = jerk_d * pow(t / 4, 3)/ 6 ;//distance is third polynomial curve
			double delta_d2 = jerk_d * pow(t / 4, 3);
			waypoints_d = {d0, d0 + delta_d1, d0 + delta_d2, d0 + 2 * delta_d2 - delta_d1, d0 + 2 * delta_d2};
		}

	    waypoints_t = { 0, t / 4, t / 2, 3 * t / 4, t };
	    waypoints_s = { s0, s0 + delta_s1, s0 + delta_s2,s0+delta_s3, s0 + delta_s4};
	    waypoints_s[0] = fmod(waypoints_s[0],max_s);
	    waypoints_s[1] = fmod(waypoints_s[1],max_s);
	    waypoints_s[2] = fmod(waypoints_s[2],max_s);
	    waypoints_s[3] = fmod(waypoints_s[3],max_s);
	    waypoints_s[4] = fmod(waypoints_s[4],max_s);

	    for (size_t i = 0; i < 5; i++) {
	    	vector<double> xy = getXYFromSD(waypoints_s[i], waypoints_d[i]);
	    	waypoints_x[i]=  xy[0];
	    	waypoints_y[i] = xy[1];
	    }

	    tk::spline spline_x;
	    tk::spline spline_y;
	    spline_x.set_points(waypoints_t, waypoints_x);
	    spline_y.set_points(waypoints_t, waypoints_y);

	    x_path.clear();
	    y_path.clear();
	    double x_prev = last_x;
	    double y_prev = last_y;

	    overSpeed = false;
	    for (size_t i = 1; i <= int(t / dT); i++) {
	    	x_path.push_back(spline_x(i * dT));
	    	y_path.push_back(spline_y(i * dT));
	    	double dx = x_path.back() - x_prev;
	    	double dy = y_path.back() - y_prev;

	    	if (sqrt(dx * dx + dy * dy) > MPH_50  * dT) {
	    		dv -=1;
	    		cout << "!!!Over speed!!! " <<t<<" dv:"<<dv<<  endl;
	    		overSpeed = true;
	    		break;
	    	}
	    	x_prev = x_path.back();
	    	y_prev = y_path.back();
	    }
	} while (overSpeed && t > dT);

	last_predict_v = v0 + jerk_s * pow(t / 2, 2);
	last_predict_sd.s = waypoints_s.back();
	last_predict_sd.d = waypoints_d.back();
	last_x = x_path.back();
	last_y = y_path.back();
}
void PathPlaner::generateChangeLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path) {
	/***
	 * generate point along d direction.
	 **/
	double t = 2.32;
	double v0 = last_predict_v;
	double s0 = last_predict_sd.s;
	double d0 = last_predict_sd.d;
	//here we consider the cross direction first
	double delta_d = d_target - d0;

	double jerk_d = MAX_JERK*0.8 *copysign(1,delta_d);
	double jerk_s = sqrt(MAX_JERK*MAX_JERK - jerk_d*jerk_d) *copysign(1,dv);

	vector<double> waypoints_t(5);
	vector<double> waypoints_x(5);
	vector<double> waypoints_y(5);
	vector<double> waypoints_s(5);
	vector<double> waypoints_d(5);


	// the maximum distance the car can travel with given jerk  -- j*(t/4)³*2 = d
	double d_max = fabs(jerk_d * pow(t / 4, 3) * 2);
	while (d_max < fabs(delta_d)) {
		t += dT;
		d_max = fabs(jerk_d * pow(t / 4, 3) * 2);
	}

	bool overSpeed = false;

	do {

		double delta_d1 = jerk_d * pow(t / 4, 3) / 6;//distance is third polynomial curve
		double delta_d2 = jerk_d * pow(t / 4, 3);

		double jerk_s_needed = dv / pow(t / 2, 2);
		if (fabs(jerk_s_needed) < fabs(jerk_s)) {
			jerk_s = jerk_s_needed;
		}

		double delta_s1 = v0 * (t / 4) + jerk_s * pow(t / 4, 3) / 6;//distance is third polynomial curve
		double delta_s2 = v0 * (t / 2) + jerk_s * pow(t / 2, 3) / 6;//distance is third polynomial curve
		double delta_s3 = v0 * (t * 3 / 4) + jerk_s * pow(t, 3) * 25 / 384;
		double delta_s4 = v0 * (t) + jerk_s * pow(t / 2, 3);

		waypoints_t = { 0, t / 4, t / 2, 3 * t / 4, t };
		waypoints_d = { d0, d0 + delta_d1, d0 + delta_d2, d0 + 2 * delta_d2 - delta_d1, d0 + 2 * delta_d2 };
		waypoints_s = { s0, s0 + delta_s1, s0 + delta_s2,s0+delta_s3, s0 + delta_s4};
		waypoints_s[0] = fmod(waypoints_s[0],max_s);
		waypoints_s[1] = fmod(waypoints_s[1],max_s);
		waypoints_s[2] = fmod(waypoints_s[2],max_s);
		waypoints_s[3] = fmod(waypoints_s[3],max_s);
		waypoints_s[4] = fmod(waypoints_s[4],max_s);

		for (size_t i = 0; i < 5; i++) {
			vector<double> xy = getXYFromSD(waypoints_s[i], waypoints_d[i]);
			waypoints_x[i]=  xy[0];
			waypoints_y[i] = xy[1];
		}

		tk::spline spline_x;
		tk::spline spline_y;
		spline_x.set_points(waypoints_t, waypoints_x);
		spline_y.set_points(waypoints_t, waypoints_y);

		x_path.clear();
		y_path.clear();
		double x_prev = last_x;
		double y_prev = last_y;

		overSpeed = false;
		for (size_t i = 1; i <= int(t / dT); i++) {
			x_path.push_back(spline_x(i * dT));
			y_path.push_back(spline_y(i * dT));
			double dx = x_path.back() - x_prev;
			double dy = y_path.back() - y_prev;

			if (sqrt(dx * dx + dy * dy) > MPH_50 * dT) {
				cout << "!!!Over speed!!! " <<t<<" dv:"<<dv<<  endl;
                 if(dv>0){
                	 dv=0;
                 }else
                 {
                	 t -=dT;
                 }

				 overSpeed = true;
				 break;
			}
			x_prev = x_path.back();
			y_prev = y_path.back();
		}
	} while (overSpeed && t > dT);


	last_predict_v = v0 + jerk_s * pow(t / 2, 2);
	last_predict_sd.s = waypoints_s.back();
	last_predict_sd.d = waypoints_d.back();
	last_x = x_path.back();
	last_y = y_path.back();
}

double PathPlaner::getKeepLaneScore() {
	double score = 0;
	//keep lane has per default 1 point more score.
	score +=2;
	if(!env.front.exist){
		score +=1;
		env.front.v = MAX_VEL;
		env.front.sd.s = MAX_DISTANCE;
	}

	if ((env.front.sd.s - ego.sd.s) / SAFT_DISTANCE > 10) {
		score += 10;
	} else {
		score += (env.front.sd.s - ego.sd.s) / SAFT_DISTANCE;
	}


	if((env.front.sd.s - ego.sd.s) > SAFT_DISTANCE)
	{
		score +=MAX_VEL;
	}else{
		score += env.front.v - ego.v;
	}


	return score;
}


double PathPlaner::getChangeLeftScore() {
	double score = 0;

	if(!env.front_left.exist)
	{
		score += 1;
		env.front_left.v = MAX_VEL;
		env.front_left.sd.s = MAX_DISTANCE;
	}

	if ((env.front_left.sd.s - ego.sd.s) / SAFT_DISTANCE > 10) {
		score += 10;
	} else {
		score += (env.front_left.sd.s - ego.sd.s) / SAFT_DISTANCE;
	}

	if ((env.front_left.sd.s - ego.sd.s) < CRASH_DISTANCE) {
		score -= 99;
	}


	if ((env.front_left.sd.s - ego.sd.s) > SAFT_DISTANCE) {
		score += MAX_VEL;
	}else
	{
		score += env.front_left.v - ego.v;
	}

	if(env.back_left.exist)
	{
		score -=1;
		if((ego.sd.s - env.back_left.sd.s )< SAFT_DISTANCE)
		{
			score -=1;
			//check if it hits by back car
			if( (ego.sd.s - env.back_left.sd.s )< CRASH_DISTANCE) {
				score -=99;
			}
		}
	}
	return score;
}

double PathPlaner::getChangeRightScore() {
	double score = 0;

	if (!env.front_right.exist) {
		score += 1;
		env.front_right.v = MAX_VEL;
		env.front_right.sd.s = MAX_DISTANCE;
	}

	if ((env.front_right.sd.s - ego.sd.s) / SAFT_DISTANCE > 10) {
		score += 10;
	} else {
		score += (env.front_right.sd.s - ego.sd.s) / SAFT_DISTANCE;
	}

	if ((env.front_right.sd.s - ego.sd.s) < CRASH_DISTANCE) {
		score -= 99;
	}


	if ((env.front_right.sd.s - ego.sd.s) > SAFT_DISTANCE) {
		score += MAX_VEL;
	}else
	{
		score += env.front_right.v - ego.v;
	}

	if (env.back_right.exist) {
		score -= 1;
		if ((ego.sd.s - env.back_right.sd.s) < SAFT_DISTANCE) {
			score -= 1;
			//check if it hits by back car
			if ((ego.sd.s - env.back_right.sd.s) < CRASH_DISTANCE) {
				score -= 99;
			}
		}
	}
	return score;
}
