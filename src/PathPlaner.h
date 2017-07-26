/*
 * PathPlaner.h
 *
 *  Created on: Jul 23, 2017
 *      Author: etendue
 */

#ifndef PATHPLANER_H_
#define PATHPLANER_H_

#include <vector>

struct InputData{
  double self_s;
  double self_d;
  double self_yaw;
  double self_speed;
  std::vector<double> previous_s;
  std::vector<double> previous_d;
  double end_s;
  double end_d;
  double waypoint_s;
  double waypoint_d;
  std::vector<std::vector<double> > sensor_data;
};

struct Vehicle{
	double x;
	double y;
	double s;
	double d;
	double vx;
	double vy;
	double v;
	double yaw;
	int lane_id;
	bool exist;
};
struct Trajectory{
	std::vector<double> s;
	std::vector<double> s_deriv1;
	std::vector<double> s_deriv2;
	std::vector<double> s_deriv3;
	std::vector<double> d;
	std::vector<double> d_deriv1;
	std::vector<double> d_deriv2;
	std::vector<double> d_deriv3;
	double Ts;
	double Td;
	double Ts_expected;
  double Td_expected;
  double S_expected;
  double D_expected;
	double cost;
};


class PathPlaner {
 public:
  PathPlaner();
  virtual ~PathPlaner();

 private:
  struct CarEnvironment{
  	Vehicle front_left;
  	Vehicle front_right;
  	Vehicle front;
  	Vehicle back_left;
  	Vehicle back_right;
  } env;
  Vehicle self;
  //help point for spline
  double help_s;
  double help_d;
  //reference time to reach target
  double ref_time_s;
  double ref_time_d;
  //
  double next_waypoint_s;
  double next_waypoint_d;

 public:
  void processingInputData(const InputData& data);
  void getBestTrajectory(std::vector<double>&ss,std::vector<double> &dd);

 private:
  Trajectory generateTrajectory(double s_i, double s_f, double d_i, double d_f,
                                double t_s, double ts_ref,double t_d,double td_ref);

  bool collisionFree(const Trajectory& trj);
  double calculateCost(const Trajectory& trj);
};

#endif /* PATHPLANER_H_ */
