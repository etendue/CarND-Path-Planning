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


/****
 * the data to represent a Frenet coordinate
 ***/
struct FrenetSD{
  double s;/*!< s component along the car drive direction */
  double d;/*!< d component perpendicular to car drive direction */
};


/***
 * the data struct to store properties of car
 */
struct Car{
  FrenetSD sd; /*!< Prenet coordinates */
  double v;    /*!< velocity  */
  int lane_id; /*!< lane number start from 1, to 3 */
  bool exist;  /*!< field to check if car is there */
};


/***
 * data struct to encapsulate parameters from simulator
 */
struct InputData{
  Car ego_update; /*!< fields for ego car */
  vector<vector<double>> sensor_data; /*!< data fusion of other vehicles */
};

/***
 * struct to encapsulate a trajectory with component x and y
 */
struct Trajectory
{
  deque<double> x;
  deque<double> y;
};

/***
 * class with behavior planning, data fusion and path generation
 */
class PathPlaner {
 public:
  PathPlaner();
  virtual ~PathPlaner();

 private:
  //// stores information around the ego car
  struct NeighbourCars{
    Car front_left;
    Car front_right;
    Car front;
    Car back_left;
    Car back_right;
  } others;

  Car ego;/*!< ego car its self */
  FrenetSD last_predict_sd; /*!< frenet coordinate from last prediction */
  double last_predict_v;    /*!< velocity at the end of last prediction */
  double last_predict_x;    /*!< x value  at the end of last prediction */
  double last_predict_y;    /*!< y value  at the end of last prediction */
  Trajectory predition;     /*!< prediction of trajectory, this feeds to simulator */
  double max_s;             /*!< the trip length of simulator, used to correct s values */
  //! The function used for convert Frenet SD to XY.
  /*!
    \param s
    \param d
    \return x and y
  */
  function<vector<double>(double,double)> getXYFromSD;

 public:
  //! update the ego car information and detect surrounding cars.
  /*!
     \param InputData data : information sent from simulator
     \return none
  */
  void processInputData(const InputData& data);
  //! generate next trajectory based on data fusion.
  bool generatePrediction();
  //! get the prediction.
  Trajectory& getPrediction(){return predition;};
  //! set the trip length
  void setMaxS(const double s){ max_s = s;};
  //! set the help function, shall be called before to use it.
  /*!
       \param a function or functor, e.g. getXY2 from main.cpp.
       \return none
   */
  void setGetXYFunc(function<vector<double>(double,double)> func){ getXYFromSD = func;};

 private:

  //! handle keep lane related information and constraints and generate trajectory
  void getKeepLaneTrajectory();
  //! handle change to left lane related information and constraints and generate trajectory
  void getChangeToLeftLaneTrajectory(bool left=true);
  //! similar as to change left, with different surrounding cars information.
  void getChangeToRightLaneTrajectory();
  //! generate path with actions for keep lane
  /*!
        \param dv:  delta velocity between new predicted and last value
        \param d_target:  position of d component new prediction wants to reach
        \param x_path: container to store the predicted x values
        \param y_path: container to store the predicted y values
        \return none
    */
  void generateKeepLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path);
  //! generate path with actions for change a lane
  /*!
        same parameters as generateKeepLanePath
      */
  void generateChangeLanePath(double dv,double d_target,vector<double>& x_path,vector<double>& y_path);
  //! following functions are to get a score based surround cars and ego driving information
  double getKeepLaneScore();
  double getChangeScore(Car& front,Car& back);
  double getChangeLeftScore();
  double getChangeRightScore();
};

#endif /* PATHPLANER_H_ */
