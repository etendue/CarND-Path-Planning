/*
 * PathPlaner.h
 *
 *  Created on: Jul 23, 2017
 *      Author: etendue
 */

#ifndef PATHPLANER_H_
#define PATHPLANER_H_

#include <vector>
#include "Eigen-3.3/Eigen/Dense"
using namespace std;
using namespace Eigen;

const int N =50;


struct PointState2D{
	  VectorXd s;
	  VectorXd d;
};

struct InputData{
  PointState2D self_state;
  vector<vector<double> > sensor_data;
};

struct Vehicle{
	PointState2D state;
	int lane_id;
	bool exist;
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

  struct CurveFlag{
    bool overspeed;
    bool underspeed;
    bool overaccel;
    bool overjerk;
  };

  Vehicle self;
  //

 public:
  void processingInputData(const InputData& data);
  void getBestTrajectory(std::vector<PointState2D>& trj);

 private:
  VectorXd JMT(const VectorXd& start,const VectorXd& target, double T);
  bool generateValidPath(const PointState2D& target, vector<PointState2D>& trj);
  double getBestJMP_KeepLane(std::vector<PointState2D>& path);
  double generatePathWithLimits(const PointState2D &target_state,vector<PointState2D>& path,bool distance_limit, bool jerk_limit=true);
  double generateSafePath(const PointState2D &target_state,vector<PointState2D>& path);
  double getFollow_KeepLane(vector<PointState2D>& path);
  vector<VectorXd> generateJMTPath(const PointState2D& start,const PointState2D& target, double T, CurveFlag& flags);
	void getCurve(const std::vector<VectorXd>& best_coeff,
			std::vector<PointState2D>& path);
};

#endif /* PATHPLANER_H_ */
