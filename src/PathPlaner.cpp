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
const double MAX_VEL = 21;//m/s  50 mph
const double MAX_ACCEL = 10; //10m/s²
const double MAX_JERK = 10; //10m/s3
const double CAR_LENGTH = 4;
const double dT= 0.02;//20ms time step


PathPlaner::PathPlaner() {
  ego.a =0;
  ego.sd={0,0};
  ego.v = 0;
  ego.lane_id =0;
  last_predict_v = 0;
  last_predict_sd.s = numeric_limits<double>::lowest();
  last_predict_sd.d = numeric_limits<double>::lowest();
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
	}


	//check other vehicles

	env.front.exist = false;
	env.front.sd = {1e10,1e10};
	env.front_left.exist = false;
	env.front_left.sd = {1e10,1e10};
	env.front_right.exist = false;
	env.front_right.sd = {1e10,1e10};
	env.back_left.exist = false;
	env.back_left.sd = {-1e10,-1e10};
	env.back_right.exist = false;
	env.back_right.sd = {-1e10,-1e10};

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

    cout<<" V:"<<setw(8)<<setprecision(6)<<last_predict_v;
	cout<<" LEFT :"<<setw(8)<<setprecision(6)<<score_left;
	cout<<" KEEP :"<<setw(8)<<setprecision(6)<<score_keep;
	cout<<" RIGHT:"<<setw(8)<<setprecision(6)<<score_right;
	cout<<" CAR in LANE:" <<ego.lane_id<<" Movement:"<<movement;
    //cout<<"\r"<<flush;
    cout<<endl;

    if(last_predict_v >MAX_VEL)
    {
    	cout<<" Over Speed:!!!"<<endl;
    }
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
		if (env.front_left.exist && env.front_left.sd.s - ego.sd.s < 40 && env.front_left.v < last_predict_v) {
			slowDown = true;
		}
	}else
	{
		d = (ego.lane_id) * 4 + 2;
		if (env.front_right.exist && env.front_right.sd.s - ego.sd.s < 40 && env.front_right.v < last_predict_v) {
			slowDown = true;
		}
	}

	if (env.front.exist && env.front.sd.s - ego.sd.s < 40
			&& env.front.v < last_predict_v) {
		slowDown = true;
	}

	vector<double> s_path;
	vector<double> d_path;

	int N = 116;

	/***
	 * get the path of d first
	 */
	double jerk_d = MAX_JERK;
	jerk_d = get_d_path(jerk_d, d, N, d_path);
	double jerk_s = sqrt(MAX_JERK * MAX_JERK - jerk_d * jerk_d);
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

	/***
	 * get path of s
	 */
	get_s_path(jerk_s,delta_v, N, s_path);

	for(int i= 0; i<N;i++)
	{
		last_prediction.d.push_back(d_path[i]);
		last_prediction.s.push_back(s_path[i]);
	}

}
void PathPlaner::getKeepLaneTrajectory(){

  double d = (ego.lane_id-1) * 4 + 2;
  double v;
  bool slowDown = false;
  if (env.front.exist && env.front.sd.s - ego.sd.s < 40 &&env.front.v <last_predict_v) {
    slowDown = true;
  }

  vector<double> s_path;
  vector<double> d_path;

  int N = 50;

  /***
   * get the path of d first
   */

  double jerk_s = MAX_JERK;
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

  /***
   * get path of s
   */
  jerk_s = get_s_path(jerk_s,delta_v,N,s_path);


  double jerk_d = sqrt(MAX_JERK * MAX_JERK - jerk_s * jerk_s);
  get_d_path(jerk_d,d,N,d_path);

	for (int i = 0; i < N; i++) {
		last_prediction.d.push_back(d_path[i]);
		last_prediction.s.push_back(s_path[i]);
	}
}


double PathPlaner::get_d_path(double jerk, double target_d, int N,vector<double>& d_path) {
	/***
	 * generate point along d direction.
	 **/

	double t = N*dT;
	// up down down up;
	// the distance between now and target
	double delta_d = target_d - last_predict_sd.d;
	// the maximum distance the car can travel with given jerk  -- j*(t/4)³*2 = d
	double d_max = jerk * pow(t / 4, 3) * 2;
	double d_travel;

	if(fabs(delta_d)<0.1){
		d_path.resize(N,last_predict_sd.d);
	}else{

		if (fabs(delta_d) > d_max) {
			d_travel = d_max;
			jerk = jerk*copysign(1, delta_d);
		} else {
			d_travel = fabs(delta_d);
			jerk = d_travel / (2 * pow(t / 4, 3))*copysign(1, delta_d);
		}

		double d0 = last_predict_sd.d;
		double delta_d1 = jerk * pow(t / 4, 3) / 6 ;//distance is third polynomial curve
		double delta_d2 = jerk * pow(t / 4, 3);
		vector<double> waypoints_t = { 0, t / 4, t / 2, 3 * t / 4, t };
		vector<double> waypoints_d = { d0, d0 + delta_d1, d0 + delta_d2, d0
				+ 2 * delta_d2 - delta_d1, d0 + 2 * delta_d2 };
		tk::spline sp;
		sp.set_points(waypoints_t, waypoints_d);

		for (size_t i = 1; i <= N; i++) {
			d_path.push_back(sp(dT * i));
		}
		last_predict_sd.d = d_path.back();
	}

	return jerk;
}
double PathPlaner::get_s_path(double jerk, double dv,int N,vector<double>& s_path) {
	/***
	 * generate point along d direction.
	 **/

	double t = N * dT;
	double v0 = last_predict_v;
	double s0 = last_predict_sd.s;

	if (dv < 0.01 && dv >0) {
		//spare the manouver, keep the current speed;
		for (size_t i = 1; i <= N; i++) {
			double s = s0 + last_predict_v * i * dT;
			s_path.push_back(fmod(s,max_s));
		}
		last_predict_sd.s = s_path.back();
		//last_predict_v = last_predict_v;
	}else
	{
		double jerk_needed =fabs(dv/pow(t/2,2));
		if(jerk_needed< fabs(jerk)){
			jerk = jerk_needed *copysign(1,dv);
		}else{
			jerk = jerk *copysign(1,dv);
		}

		double delta_s1 = v0*(t/2) + jerk*pow(t/2,3)/6 ;//distance is third polynomial curve
		double delta_s2 = v0*t + jerk*pow(t/2,3);
		vector<double> waypoints_t = {0,t/2,t};
		vector<double> waypoints_s = { s0, s0 + delta_s1, s0 + delta_s2};
		tk::spline sp;
		sp.set_points(waypoints_t,waypoints_s);

		for (size_t i = 1; i <= N; i++) {
			s_path.push_back(fmod(sp(dT*i),max_s));
		}
		last_predict_sd.s = s_path.back();
		last_predict_v = v0 + jerk*pow(t/2,2);
	}

	return jerk;

}

double PathPlaner::getKeepLaneScore() {
	double score = 0;

	//keep lane has per default 1 point more score.
	score +=2;
	if(!env.front.exist){
		score +=1;
		env.front.v = MAX_VEL;
		env.front.sd.s = 1e10;
	}

	if((env.front.sd.s - ego.sd.s) > 40)
	{
		score +=MAX_VEL;
	}else{
		score += env.front.v - ego.v;
	}


	return score;
}


double PathPlaner::getChangeLeftScore() {
	double score = 0;
	if (!env.front.exist) {
		score += 1;
		env.front.v = MAX_VEL;
		env.front.sd.s = 1e10;
	}

	if(!env.front_left.exist)
	{
		score += 1;
		env.front_left.v = MAX_VEL;
		env.front_left.sd.s = 1e10;
	}


	if ((env.front_left.sd.s - ego.sd.s) > 40) {
		score += MAX_VEL;
	}else
	{
		score += env.front.v - ego.v;
	}

	if(env.back_left.exist)
	{
		score -=1;
		if((ego.sd.s - env.back_left.sd.s )< 40)
		{
			score -=1;
			//check if it hits by back car
			if( (ego.sd.s - env.back_left.sd.s )< 10) {
				score -=99;
			}
		}
	}
	return score;
}

double PathPlaner::getChangeRightScore() {
	double score = 0;
	if (!env.front.exist) {
		score += 1;
		env.front.v = MAX_VEL;
		env.front.sd.s = 1e10;
	}

	if (!env.front_right.exist) {
		score += 1;
		env.front_right.v = MAX_VEL;
		env.front_right.sd.s = 1e10;
	}

	if ((env.front_right.sd.s - ego.sd.s) > 40) {
		score += MAX_VEL;
	}else
	{
		score += env.front.v - ego.v;
	}

	if (env.back_right.exist) {
		score -= 1;
		if ((ego.sd.s - env.back_right.sd.s) < 40) {
			score -= 1;
			//check if it hits by back car
			if ((ego.sd.s - env.back_right.sd.s) < 10) {
				score -= 99;
			}
		}
	}
	return score;
}
