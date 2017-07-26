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
#include "spline.h"
using namespace std;

//define constants
const double maximum_s = 1.0e20;
const double maximum_speed = 22.352 ;//m/s  50 mph
const int maximum_lane = 3;
const double maximum_steer = 0.436332;//25degree
const double max_change_lane_duration = 3;//the change lane duration shall within 3 seconds
const double max_accel = 10; //10m/s²
const double max_accel_d = 4.22618261741;//25degree
const double max_jerk = 10; //10m/s3
const double car_width = 2;
const double maximum_cost = 100000;
const int N =50;
const int D_SAMPLE_SIZE = 10;
const int S_SAMPLE_SIZE = 10;
const double CAR_LENGTH = 4;
const double dt= 0.02;//20ms time step

class Cost;

PathPlaner::PathPlaner() {
}

PathPlaner::~PathPlaner() {
  // TODO Auto-generated destructor stub
}
void PathPlaner::processingInputData(const InputData& data)
{

	//update the states of self
	self.exist = true;
	self.s = data.self_s;
	self.d = data.self_d;
	self.vx = -1;
	self.vy = -1;
	self.v = data.self_speed;
	self.yaw = data.self_yaw;
	self.lane_id = ceil(data.self_d/4);

	next_waypoint_s = data.waypoint_s;
	next_waypoint_d = data.waypoint_d;


	//waypoints for trajectory to
  double distance_s = next_waypoint_s - self.s;
  double distance_d = self.lane_id*4 -2 - self.d;
  //quadratic function root=(-b+sqrt(b^2 -4ac))/2a
  ref_time_s = (-self.v + sqrt(self.v*self.v + 2*max_accel*distance_s))/max_accel;
  ref_time_d = sqrt(fabs(2*distance_d/max_accel_d));
  cout<<ref_time_d<<" "<<ref_time_s<<endl;

  if(data.previous_s.size()>0){
    help_s = data.previous_s[0];
    help_d = data.previous_d[0];
  }else{

    if (self.v != 0) {
      help_s = self.s + self.v * cos(self.yaw) * dt;
      help_d = self.d + self.v * sin(self.yaw) * dt;
    }else{
      help_s = self.s + copysign(0.01,distance_s); //requirement of spline
      help_d = self.d + copysign(0.01,distance_d);
    }
  }

	//check other vehicles

	env.front.exist = false;
	env.front_left.exist = false;
	env.front_right.exist = false;
	env.back_left.exist = false;
	env.back_right.exist = false;

	//update states of other vehicles detected in environment
	auto updateVechile = [](Vehicle& v, const vector<double>& data){
		v.s = data[5];
		v.d = data[6];
		v.vx = data[3];
		v.vy = data[4];
		v.v = sqrt(pow(v.vx,2) + pow(v.vy,2));
		v.lane_id =  ceil(data[6] / 4);
	};
	for(auto other : data.sensor_data){
		//vehicle ={id,x,y,vx,vy,s,d}
		int lane_id = ceil(other[6] / 4);

		//left side
		if(lane_id == (self.lane_id + 1)){
			//back
			if(other[5] < self.s){
				//first detected
				if(!env.back_left.exist){
					env.back_left.exist = true;
					updateVechile(env.back_left,other);
				}
				// a more near vehicle is detected
				else if(other[5] > env.back_left.s)
				{
					updateVechile(env.back_left,other);
				}
			}
			//front
			else{
				if(!env.front_left.exist){
					env.front_left.exist = true;
					updateVechile(env.front_left,other);
				}
				// a more near vehicle is detected
				else if(other[5] <env.front_left.s){
					updateVechile(env.front_left,other);
				}
			}
		}
		//right side
		else if(lane_id == (self.lane_id -1)){
			//back
			if (other[5] < self.s) {
				//first detected
				if (!env.back_right.exist) {
					env.back_right.exist = true;
					updateVechile(env.back_right, other);
				}
				// a more near vehicle is detected
				else if (other[5] > env.back_right.s) {
					updateVechile(env.back_right, other);
				}
			}
			//front
			else {
				if (!env.front_right.exist) {
					env.front_right.exist = true;
					updateVechile(env.front_right, other);
				}
				// a more near vehicle is detected
				else if (other[5] < env.front_right.s) {
					updateVechile(env.front_right, other);
				}
			}
		}
		// same lane
		else if(lane_id == self.lane_id){
			//check only vehicle in the front
			if (other[5] > self.s) {
				//first detected
				if (!env.front.exist) {
					env.front.exist = true;
					updateVechile(env.front, other);
				}
				// a more near vehicle is detected
				else if (other[5] > env.front.s) {
					updateVechile(env.front, other);
				}
			}
		}
	}

}

void PathPlaner::getBestTrajectory(vector<double>&ss,vector<double> &dd)
{


  Trajectory opt_trj;
  double td_min = ref_time_d;
  double ts_min = ref_time_s;
  opt_trj.cost = maximum_cost;

  //keep lane
  double inital_s = self.s;
  double intial_d = self.d;
  double target_s = next_waypoint_s;
  double target_d = self.lane_id*4 -2;

  auto search_optimal_trj = [&](){
    Trajectory best;
    best.cost = maximum_cost;
    for (int i = 0; i < D_SAMPLE_SIZE; i++) {
      double t_d = td_min + ref_time_d * i / D_SAMPLE_SIZE;
      for (int j = 0; j < S_SAMPLE_SIZE; j++) {
        double t_s = ts_min + ref_time_s * j / S_SAMPLE_SIZE;
        Trajectory candidate = generateTrajectory(inital_s, target_s,
            intial_d, target_d, t_s,
            ref_time_s, t_d, ref_time_d);

        if (candidate.cost < best.cost) {
          best = candidate;
        }
      }
    }
    return best;
  };
  do {
    opt_trj  = search_optimal_trj();
    //increase the time in case no valid trajectory is found
    ts_min += ref_time_s;
    td_min += ref_time_d;
  } while (opt_trj.cost == maximum_cost);

  //change left
  ts_min = ref_time_s;
  td_min = ref_time_d;
  if(self.lane_id<3){
    target_d = self.lane_id*4 +2;
    Trajectory best  = search_optimal_trj();
    if(best.cost<opt_trj.cost){
      opt_trj = best;
    }
  }
  ts_min = ref_time_s;
  td_min = ref_time_d;
  //change right
  if(self.lane_id>1){
    target_d = self.lane_id * 4 - 6;
    Trajectory best = search_optimal_trj();
    if (best.cost < opt_trj.cost) {
      opt_trj = best;
    }
  }

  ss.swap(opt_trj.s);
  dd.swap(opt_trj.d);

  return ;
}
Trajectory PathPlaner::generateTrajectory(double s_i, double s_f, double d_i, double d_f,
                                          double t_s, double ts_ref,double t_d,double td_ref)
{

  //generate trajectory for s
  Trajectory trj;

  //time serie
  vector<double> time_serie= {0,dt,-1};
  vector<double> ref_Ss ={s_i,help_s,s_f};
  vector<double> ref_Ds ={d_i,help_d,d_f};

  //generate spline
  time_serie[2] = t_s;
  tk::spline sp;
  sp.set_points(time_serie, ref_Ss);
  vector<double> s(N, 0);
  vector<double> s_deriv1(N, 0);
  vector<double> s_deriv2(N, 0);
  vector<double> s_deriv3(N, 0);
  for (int i = 0; i < N; i++) {
    s[i] = sp(dt * i);
    s_deriv1[i] = sp.deriv(1, dt * i);
    s_deriv2[i] = sp.deriv(2, dt * i);
    s_deriv3[i] = sp.deriv(3, dt * i);
  }

  time_serie[2] = t_d;
  sp.set_points(time_serie, ref_Ds);
  vector<double> d(N, 0);
  vector<double> d_deriv1(N, 0);
  vector<double> d_deriv2(N, 0);
  vector<double> d_deriv3(N, 0);

  for (int i = 0; i < N; i++) {
    d[i] = sp(dt * i);
    d_deriv1[i] = sp.deriv(1, dt * i);
    d_deriv2[i] = sp.deriv(2, dt * i);
    d_deriv3[i] = sp.deriv(3, dt * i);
  }

  //calculate cost and restrictions
  //restriction

  trj.Td_expected = td_ref;
  trj.Ts_expected = ts_ref;
  trj.D_expected = d_f;
  trj.S_expected = s_f;
  trj.Td = t_d;
  trj.d = d;
  trj.d_deriv1 = d_deriv1;
  trj.d_deriv2 = d_deriv2;
  trj.d_deriv3 = d_deriv3;
  trj.Ts = t_s;
  trj.s = s;
  trj.s_deriv1 = s_deriv1;
  trj.s_deriv2 = s_deriv2;
  trj.s_deriv3 = s_deriv3;

  trj.cost = calculateCost(trj);

  return trj;


}

bool PathPlaner::collisionFree(const Trajectory& trj) {
  int start_laneId = trj.d[0]/4 +1;
  int end_laneId = trj.d.back()/4 +1;

  //keep lane
  if(start_laneId == end_laneId){
    //end position of self
    double end_s = trj.s.back();
    if(env.front.exist){
      double front_car_end_s = env.front.s + env.front.v*dt*(N-1);
      //the predicted end position is beyond front car
      if(end_s > front_car_end_s)
      {
        return false;
      }
    }
  }

  //change lane
  Vehicle front_car_other_lane;
  Vehicle back_car_other_lane;
  Vehicle front_car_same_lane;

  front_car_same_lane = env.front;
  //change lane left
  if (end_laneId == (start_laneId + 1)) {
    front_car_other_lane = env.front_left;
    back_car_other_lane = env.back_left;
  }
  //change lane right
  else if (end_laneId == (start_laneId - 1)) {
    front_car_other_lane = env.front_right;
    back_car_other_lane = env.back_right;
  }
  //look for time point of lane change
  auto iter = find_if(
      trj.d.begin(), trj.d.end(),
      [&end_laneId](const double d) {return (d/4+1)==end_laneId;});
  if(iter == trj.d.end()){
    //vehicle has not yet reached other line
    return true;
  }
  size_t index = distance(trj.d.begin(), iter);
  double s_at_change = trj.s[index];
  //check collide with front car
  if (front_car_same_lane.exist) {
    double front_car_s_at_change = front_car_same_lane.s
        + front_car_same_lane.v * dt * index;
    if (front_car_s_at_change < s_at_change) {
      return false;
    }
  }
  // check collide with back_car in other lane
  if (back_car_other_lane.exist) {
    double back_car_s_at_change = back_car_other_lane.s
        + back_car_other_lane.v * dt * index;

    if (back_car_s_at_change > s_at_change) {
      return false;
    }
  }
  //check collision with front_car in other lane
  if (front_car_other_lane.exist) {
    double front_car_s_end = front_car_other_lane.s
        + front_car_other_lane.v * dt * (N - 1);
    if (front_car_s_end < trj.s.back()) {
      return false;
    }
  }

  return true;
}


double PathPlaner::calculateCost(const Trajectory& trj){

  double cost = maximum_cost;
  double total_jerk = 0;
  if (collisionFree(trj)) {
    for (size_t i = 0; i < trj.s.size(); i++) {
      double v_s = trj.s_deriv1[i];
      double v_d = trj.d_deriv1[i];

      double a_s = trj.s_deriv2[i];
      double a_d = trj.d_deriv2[i];

      double j_s = trj.s_deriv3[i];
      double j_d = trj.d_deriv3[i];

      if (sqrt(v_s * v_s + v_d * v_d) > maximum_speed
          || sqrt(a_s * a_s + a_d * a_d) > max_accel
          || sqrt(j_s * j_s + j_d * j_d) > max_jerk) {
        // overspeed, over acceleration, over jerk
        return cost;
      }
      total_jerk += (j_s * j_s + j_d * j_d) * dt;
    }
  }

  cost = total_jerk/trj.s.size();
  //time cost
  cost += trj.Ts_expected/trj.Ts;
  cost += trj.Td_expected/trj.Td;
  //distance cost
  cost += pow((trj.s.front() - trj.S_expected)/trj.S_expected,2);
  cost += pow((trj.d.front() - trj.D_expected)/trj.D_expected,2);

  return cost;
}
