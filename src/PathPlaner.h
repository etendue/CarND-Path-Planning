/*
 * PathPlaner.h
 *
 *  Created on: Jul 23, 2017
 *      Author: etendue
 */

#ifndef PATHPLANER_H_
#define PATHPLANER_H_

#include <vector>
#include <deque>
using namespace std;


struct FrenetSD{
  double s;
  double d;
};

struct Car{
  FrenetSD sd;
  double v;
  double a;
  int lane_id;
  bool exist;
};



struct InputData{
  Car ego_update;
  vector<vector<double> > sensor_data;
  int update_count;
};
struct Trajectory
{
  deque<double> s;
  deque<double> d;
};




class PathPlaner {
 public:
  PathPlaner();
  virtual ~PathPlaner();

 private:
  struct CarEnvironment{
    Car front_left;
    Car front_right;
    Car front;
    Car back_left;
    Car back_right;
  } env;


  Car ego;
  FrenetSD last_predict_sd;
  double last_predict_v;
  Trajectory last_prediction;
  double max_s;
  //

 public:
  void processInputData(const InputData& data);
  bool generatePrediction();
  Trajectory & getPrediction(){return last_prediction;};
  void setMaxS(const double s){ max_s = s;};

 private:

  void getKeepLaneTrajectory();
  void getChangeToLeftLaneTrajectory(bool left=true);
  void getChangeToRightLaneTrajectory();
  double getKeepLaneScore();
  double getChangeLeftScore();
  double getChangeRightScore();
  void getKeepLanePath(double dv,double d_target,vector<double>& s_path,vector<double>& d_path);
  void getChangeLanePath(double dv,double d_target,vector<double>& s_path,vector<double>& d_path);
};

#endif /* PATHPLANER_H_ */
