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


using namespace std;

//define constants
const double maximum_s = 1.0e20;
const double maximum_speed = 22.352 ;//m/s  50 mph
const double maximum_speed_d = 9.752892864; //m/s
const int maximum_lane = 3;
const double maximum_steer = 0.436332;//25degree
const double max_change_lane_duration = 3;//the change lane duration shall within 3 seconds
const double max_accel = 10; //10m/s²
const double max_accel_d = 4.22618261741;//25degree
const double max_jerk = 10; //10m/s3
const double car_width = 2;
const double maximum_cost = 100000;
const int D_SAMPLE_SIZE = 10;
const int S_SAMPLE_SIZE = 10;
const double CAR_LENGTH = 4;
const double dt= 0.02;//20ms time step

class Cost;

PathPlaner::PathPlaner() {
  self.exist = true;
  self.state.s = VectorXd(3);
  self.state.d = VectorXd(3);

  //my speed:7.79597 accel:-3.20324 target speed:17.4176
  self.state.s<<0,18.2644,-0.398246;
  self.state.d<<1,0,0;
  PointState2D target;
  target.s = VectorXd(3);
  target.d = VectorXd(3);
  target.s << 13.2366,18.5042,0;
  target.d << 0,0,0;
  vector<PointState2D> path(N);
  for(int i=0;i<N;i++)
  {
    path[i].s = VectorXd(3);
    path[i].d = VectorXd(3);
  }

//  generateValidPath(target,path);
//  generatePathWithLimits(target,path,true,true);
//  for(int i=0;i<N;i++)
//  {
//     cout << path[i].s[0]<<" "<<path[i].s[1]<<" "<<path[i].s[2]<<" ";
//     cout << path[i].d[0]<<" "<<path[i].d[1]<<" "<<path[i].d[2]<<endl;
//  }

 // generatePathWithLimits(target,path,false,true);

}

PathPlaner::~PathPlaner() {
  // TODO Auto-generated destructor stub
}
void PathPlaner::processingInputData(const InputData& data)
{

	//update the states of self
	self.state = data.self_state;
	self.lane_id = ceil(self.state.d[0]/4.0);

	//check other vehicles

	env.front.exist = false;
	env.front_left.exist = false;
	env.front_right.exist = false;
	env.back_left.exist = false;
	env.back_right.exist = false;

	for(int i=0; i<data.sensor_data.size();i++){
		vector<double> other = data.sensor_data[i];
		Vehicle v;
		v.state.s = VectorXd(3);
		v.state.d = VectorXd(3);
		v.exist = true;
		v.state.s[1] = sqrt(pow(other[3],2)+pow(other[4],2));
		v.state.s[0] = other[5];
		v.state.s[2] = 0;
		v.lane_id = ceil(other[6] / 4);

		//left side
		if(v.lane_id == (self.lane_id - 1)){
			//back
			if(v.state.s[0] < self.state.s[0]){
				//first detected or a more near vehicle is detected
				if(!env.back_left.exist || v.state.s[0] > env.back_left.state.s[0]){
					env.back_left = v;
				}

			}
			//front
			else{
				if(!env.front_left.exist ||v.state.s[0] < env.front_left.state.s[0]){
					env.front_left = v;
				}
			}
		}
		//right side
		else if(v.lane_id== (self.lane_id +1)){
			//back
			if (v.state.s[0] < self.state.s[0]) {
				//first detected or a more near vehicle is detected
				if (!env.back_right.exist || v.state.s[0] > env.back_right.state.s[0]) {
					env.back_right = v;
				}

			}
			//front
			else {
				if (!env.front_right.exist || v.state.s[0] < env.front_right.state.s[0]) {
					env.front_right = v;
				}
			}
		}
		// same lane
		else if(v.lane_id == self.lane_id){
			//check only vehicle in the front
			if (v.state.s[0] > self.state.s[0]) {
				//first detected
				if (!env.front.exist || v.state.s[0] < env.front.state.s[0]) {
					env.front = v;
				}
			}
		}
	}
}
void PathPlaner::getBestTrajectory(vector<PointState2D>& trj)
{
	//vector<PointDynamicsSD> keep_lane;
	getBestJMP_KeepLane(trj);
	//getFollow_KeepLane(trj);
	return;

}
double PathPlaner::getFollow_KeepLane(vector<PointState2D>& path){

	double target_speed = maximum_speed;


	if (env.front.exist) {
		target_speed = env.front.state.s[1];
	}
	double middle_speed = (target_speed + self.state.s[0])/2;
	double speed = self.state.s[1];
	double accel = 0;
	double s = self.state.s[0];

	for (int i = 0; i < N; i++) {

		if (speed < middle_speed) {
			accel += max_jerk*0.9 * dt;
		} else{
			accel -= max_jerk*0.9 * dt;
			if(accel<0)
			  accel = 0;
		}
		s += speed * dt + 0.5 * accel * dt * dt;
		speed += accel *dt;
		if(speed >target_speed)
		{
		  speed = target_speed;
		}

		path[i].s<< s, speed, accel;
		path[i].d <<  10, 0, 0;
	}

	return 0.0;

}
double PathPlaner::getBestJMP_KeepLane(vector<PointState2D>& path){


	PointState2D target;
	target.s = VectorXd(3);
	target.d = VectorXd(3);
	target.d << self.lane_id * 4.0 - 2, 0, 0;
	target.s << self.state.s[0] + 30, maximum_speed, 0;
	double cost;

	if (env.front.exist) {
		//set target is front car with safe distance CAR_LENGTH
		target.s << env.front.state.s[0] - CAR_LENGTH, env.front.state.s[1], 0;

		generateValidPath(target, path);
	} else {
		// cost = generatePathWithLimits(target,path,false,true);
		generateValidPath(target, path);
	}

	return cost;
}

void PathPlaner::getCurve(const vector<VectorXd>& best_coeff,
		vector<PointState2D>& path) {
	VectorXd tv(6);
	for (int i = 0; i < N; i++) {
		double t = dt * i;
		tv << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;
		double s, d;
		double speed_s, speed_d;
		double accel_s, accel_d;
		double jerk_s, jerk_d;
		//calculate longitude trajectory
		s = best_coeff[0].dot(tv);
		speed_s = best_coeff[1].dot(tv);
		accel_s = best_coeff[2].dot(tv);
		//calculate lateral trajectory
		d = best_coeff[3].dot(tv);
		speed_d = best_coeff[4].dot(tv);
		accel_d = best_coeff[5].dot(tv);
		path[i].s << s, speed_s, accel_s;
		path[i].d << d, speed_d, accel_d;
	}
}

double PathPlaner::generatePathWithLimits(const PointState2D &target_state,vector<PointState2D>& path, bool distance_limit, bool jerk_limit)
{
  double T;
  double best_T = 1000; // the shorter T the better of
  double best_S = 0; //range for 1 second to go
  vector<VectorXd>  best_coeff;

  bool found_Upbound = false;
  double range = 1;  //start range of guess to reach target speed
  double max_range = 256;  // within this range the car can speed from 0 to maximum
  double min_range = 0.1;  //search stop if change of range is less then this value
  double delta_r = 1;  //delta range for searching a valid range


  PointState2D candidate_state;
  candidate_state = target_state;
  //in caase the distance is confined, stop the first loop
  if (distance_limit == true) {
    range = candidate_state.s[0] - self.state.s[0];
    delta_r = 0;
  }

  double range_low = range;
  ////////////////////////////////////////
  /////////////1. loop search a range which fit the JMT
  int count=0;

  do{

    T = range/maximum_speed;
    candidate_state.s[0] = self.state.s[0] + range;
    //cout<<"S is:"<<range<<" dS:"<<delta_r<<endl;
    ///////////////////////////////////
    /////////2. loop search for time duration fit for JMT
    bool found_valid = false;
    double max_T = T * 32;
    double min_T = T/256;
    double delta_T = T;
    double T_low = T;

    while(T < max_T && delta_T > min_T){
      //cout<<"  T is now:"<<T<<" dT:"<<delta_T<<endl;
      CurveFlag flags;
      vector<VectorXd> coefficients = generateJMTPath(self.state,candidate_state,T,flags);

      if(!flags.overjerk && !flags.overaccel && !flags.overspeed){
        if(!flags.underspeed){// a valid path is found
//          double s_in_one_second = -self.state.s[0];
//          for (int i=0; i<coefficients[0].size();i++) {
//            s_in_one_second += coefficients[0][i];
//          }

          if (T < best_T) {
            best_T = T;
            best_S = range;
            best_coeff = coefficients;
          }
          found_valid = true;
        }
        T = (T_low + T)/2;
        delta_T = (T - T_low)/2; //half the search step
        found_Upbound = true;
        //cout<<"   found Upbound  with T:"<<T <<" S:"<<range<<endl;
      }else
      {
        if(found_Upbound){
          T_low = T;
          T += delta_T;
          delta_T = (T - T_low)/2; //half the search step
        }else
        {
          T_low = T;
          T= T + delta_T;
          delta_T *=2;
        }
      }

      count ++;
    }
    ///////////////////////////////////////////
    ////////loop for adjust range
    if(found_valid){
      range = (range_low + range)/2;
      delta_r = (range - range_low)/2;
    }else
    {
      if (found_Upbound){
        range_low = range;
        range += delta_r;
        delta_r = (range - range_low)/2;
      }else{
        range_low = range;
        range += delta_r;
        delta_r *=2;
      }
    }
  } while (range < max_range && delta_r > min_range && !distance_limit);

	if (best_T < 1000) {
		getCurve(best_coeff, path);
		cout << "Search: " << count << " best S:" << best_S << " velocity:"
				<< self.state.s[1] << "Target Speed:" << target_state.s[1]
				<< " accel:" << self.state.s[2] << endl;
	} else {
		cout << "!!!!!!!!No valid curve found for distance_limit:"
				<< distance_limit << " range:" << range << " foundUpbound:"
				<< found_Upbound << " my speed:" << self.state.s[1] << " target speed:" << candidate_state.s[1]
				<< " accel:"<< self.state.s[2] << endl;
		generateSafePath(target_state, path);
	}

  return best_T;

}
double PathPlaner::generateSafePath(const PointState2D &target_state,vector<PointState2D>& path){

	double delta_v = target_state.s[1] - self.state.s[1];
	double s;
	double speed_s;
	double accel_s;
	s = self.state.s[0];
	speed_s =  self.state.s[1];
	accel_s =  self.state.s[2];

	for (int i = 0; i < N; i++) {
		if(speed_s >target_state.s[1])
		{
			accel_s -=max_jerk *dt;
			accel_s = accel_s<-max_accel? -max_accel:accel_s;
		}else
		{
			if(accel_s>0){
				accel_s -= max_jerk *dt;
				accel_s = accel_s<0? 0:accel_s;
			}else{
				accel_s +=max_jerk*dt;
				accel_s = accel_s>=0? 0:accel_s;
			}
		}
		speed_s += accel_s * dt;

		s += speed_s * dt;

		path[i].s << s, speed_s, accel_s;
		path[i].d << self.state.d;
	}
	cout<<"Safe trajectory: speed increase to :"<< path.back().s[1]<<" S:"<<path.back().s[0]<<" Accel:"<<path.back().s[2]<<endl;
}

bool PathPlaner::generateValidPath(const PointState2D& target,vector<PointState2D>& path)
{

	//init and final states
	PointState2D target_copy;
	target_copy = target;

	double distance_s = target.s[0] - self.state.s[0];
	double distance_d = target.d[0] - self.state.d[0];

	double T_guess =  distance_s/((target.s[1] + self.state.s[1])/2);
	double T_lowbound, T_highbound;

	T_lowbound = T_guess/2;
	T_highbound = T_guess*2;




	random_device rd;
	default_random_engine gen(rd());

	normal_distribution<double> s_pertub(target.s[0], fabs(distance_s / 10.0));
	normal_distribution<double> d_pertub(target.d[0], fabs(distance_d / 10.0));
	uniform_real_distribution<double> T_pertub(T_lowbound,T_highbound);


	double best_T = 1000;
	CurveFlag flags;

	int MAX_TRY = 500;
	int counter = 0;
	vector<VectorXd> best_curve;
	while (counter < MAX_TRY) {
		double T_sample = T_pertub(gen);
		double s_sample = s_pertub(gen);
		double d_sample = d_pertub(gen);

		target_copy.s[0] = s_sample;
		target_copy.d[0] = d_sample;

		//curve coefficients for longitude and lateral
		vector<VectorXd> coeff=generateJMTPath(self.state, target_copy, T_sample, flags);
		if(!flags.overaccel && !flags.overjerk&&!flags.overspeed&&!flags.underspeed){
			if(best_T > T_sample){
				best_T = T_sample;
				best_curve = coeff;
			}
		}

		counter++;
	}

	if(best_T <1000){
		getCurve(best_curve, path);
		cout<<"Found Curve:  best T:"<<best_T<<" velocity:"<<self.state.s[1]<<
				"Target Speed:"<<target.s[1]<<" accel:"<<self.state.s[2] <<" distance:"<<distance_s<<endl;
		return true;
	}else{
		cout<<"!!!!!!!!No valid curve found for speed:"<<self.state.s[1]<<
				"Target Speed:"<<target.s[1]<<" accel:"<<self.state.s[2]<<" distance:"<<distance_s<<endl;
		generateSafePath(target,path);
		return false;
	}
}
vector<VectorXd> PathPlaner::generateJMTPath(const PointState2D& start,const PointState2D& target, double T, CurveFlag& flags)
{

  VectorXd s_start = self.state.s;
  VectorXd s_target = target.s;
  VectorXd d_start = self.state.d;
  VectorXd d_target = target.d;
  flags = {false,false,false,false};

  //curve coefficients for longitude and lateral
  VectorXd s_co, d_co;
  s_co = JMT(s_start, s_target, T);
  d_co = JMT(d_start, d_target, T);

  //validate the curve
  //coefficients for velocity, acceleration and jerk,
  VectorXd s_veloc(6), s_accl(6), s_jerk(6);
  s_veloc << s_co[1], 2 * s_co[2], 3 * s_co[3], 4 * s_co[4], 5 * s_co[5], 0;
  s_accl << 2 * s_co[2], 6 * s_co[3], 12 * s_co[4], 20 * s_co[5], 0, 0;
  s_jerk << 6 * s_co[3], 24 * s_co[4], 60 * s_co[5], 0, 0, 0;

  VectorXd d_veloc(6), d_accl(6), d_jerk(6);
  d_veloc << d_co[1], 2 * d_co[2], 3 * d_co[3], 4 * d_co[4], 5 * d_co[5], 0;
  d_accl << 2 * d_co[2], 6 * d_co[3], 12 * d_co[4], 20 * d_co[5], 0, 0;
  d_jerk << 6 * d_co[3], 24 * d_co[4], 60 * d_co[5], 0, 0, 0;

  VectorXd tv(6);

  int sample_number = 51;
  double delta_t = T / sample_number;

  for (int i = 0; i < sample_number; i++) {

    double t = delta_t * i;
    tv << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;

    double s, d;
    double speed_s, speed_d;
    double accel_s, accel_d;
    double jerk_s, jerk_d;

    //calculate longitude trajectory
    s = s_co.dot(tv);
    speed_s = s_veloc.dot(tv);
    accel_s = s_accl.dot(tv);
    jerk_s = s_jerk.dot(tv);

    //calculate lateral trajectory
    d = d_co.dot(tv);
    speed_d = d_veloc.dot(tv);
    accel_d = d_accl.dot(tv);
    jerk_d = d_jerk.dot(tv);

    if (speed_s < 0) {
      flags.underspeed = true;
      break;
    }
    if ((speed_s * speed_s + speed_d * speed_d) > maximum_speed * maximum_speed) {
      flags.overspeed = true;
      break;
    }
    if ((accel_s * accel_s + accel_d * accel_d) > max_accel * max_accel) {
      flags.overaccel = true;
      break;
    }
    if ((jerk_s * jerk_s + jerk_d * jerk_d) > max_jerk * max_jerk) {
      flags.overjerk = true;
      break;
    }
  }
  //if(!flags.overaccel && !flags.overjerk && !flags.overspeed && !flags.underspeed)
    //cout<<"found a valid curve with S:"<<s_target[0]<<" T:"<<T<<endl;
  return {s_co,s_veloc,s_accl,d_co,d_veloc,d_accl};
}
VectorXd PathPlaner::JMT(const VectorXd& start,const VectorXd& target, double T){

	double si = start[0];
	double sdi =start[1];
	double sddi = start[2];
	double sf=target[0];
	double sdf=target[1];
	double sddf=target[2];

	MatrixXd mat(3,3);
	VectorXd v(3);
	double T2 = T*T;
	double T3 = T2*T;
	double T4 = T3*T;
	double T5 = T4*T;
	mat << T3,T4,T5,
		  3*T2,4*T3,5*T4,
		  6*T,12*T2,20*T3;
	v << sf-(si+sdi*T+0.5*sddi*T2), sdf-(sdi + sddi*T), sddf-sddi;

	Eigen::VectorXd va = mat.inverse() *v;

	VectorXd coeff(6);
	coeff<<si,sdi,0.5*sddi,va[0],va[1],va[2];
	return coeff;
 }
