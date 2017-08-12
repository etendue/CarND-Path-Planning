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
#include <functional>
using namespace std;


struct FrenetSD{
  double s;
  double d;
};

struct Car{
  FrenetSD sd;
  double v;
  int lane_id;
  bool exist;
};



struct InputData{
  Car ego_update;
  vector<vector<double> > sensor_data;
};
struct Trajectory
{
  deque<double> x;
  deque<double> y;
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
  double last_x;
  double last_y;
  Trajectory last_prediction;
  double max_s;
  //
  function<vector<double>(double,double)> getXYFromSD;

 public:
  void processInputData(const InputData& data);
  bool generatePrediction();
  Trajectory & getPrediction(){return last_prediction;};
  void setMaxS(const double s){ max_s = s;};
  void setGetXYFunc(function<vector<double>(double,double)> func){ getXYFromSD = func;};

 private:

  void getKeepLaneTrajectory();
  void getChangeToLeftLaneTrajectory(bool left=true);
  void getChangeToRightLaneTrajectory();
  double getKeepLaneScore();
  double getChangeLeftScore();
  double getChangeRightScore();
  void generateKeepLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path);
  void generateChangeLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path);
};

#endif /* PATHPLANER_H_ */
