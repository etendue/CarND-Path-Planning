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
const double MAX_VEL = 20;    //maximum velocity car is going to drive
const double MPH_50 = 22.35;  // 50mph
const double MAX_ACCEL = 10;  //10m/s²
const double MAX_JERK = 10;   //10m/s3
const double SAFT_DISTANCE = 30;  // the distance between front car or back car.
const double CRASH_DISTANCE = 10;  //if distance is small than this, change lane may crash
const double CAR_LENGTH = 4;       //minimum distance between ego car and front car
const double MAX_DISTANCE = 1.0e20; // maximum distance, used when car is not detected.
const double dT = 0.02;  //20ms time step

PathPlaner::PathPlaner() {
  ego.sd= {0,0};
  ego.v = 0;
  ego.exist = true;
  ego.lane_id =0;
  last_predict_v = 0;
  last_predict_sd.s = numeric_limits<double>::lowest();
  last_predict_sd.d = numeric_limits<double>::lowest();
  max_s = 6945.554;
  last_predict_x = 0;
  last_predict_y = 0;
}

PathPlaner::~PathPlaner() {
  // TODO Auto-generated destructor stub
}
void PathPlaner::processInputData(const InputData& data) {

  //update the states of ego
  ego = data.ego_update;

  //set the initial values
  if (last_predict_sd.s == numeric_limits<double>::lowest()) {
    last_predict_sd = ego.sd;
    auto xy = getXYFromSD(ego.sd.s, ego.sd.d);
    last_predict_x = xy[0];
    last_predict_y = xy[1];
  }

  //check other vehicles
  //reset the old data.

  others.front.exist = false;
  others.front.sd = {MAX_DISTANCE,MAX_DISTANCE};
  others.front_left.exist = false;
  others.front_left.sd = {MAX_DISTANCE,MAX_DISTANCE};
  others.front_right.exist = false;
  others.front_right.sd = {MAX_DISTANCE,MAX_DISTANCE};
  others.back_left.exist = false;
  others.back_left.sd = {-MAX_DISTANCE,-MAX_DISTANCE};
  others.back_right.exist = false;
  others.back_right.sd = {-MAX_DISTANCE,-MAX_DISTANCE};

  // iterate the data to find most near 5 cars
  for (int i = 0; i < data.sensor_data.size(); i++) {
    vector<double> other = data.sensor_data[i];
    Car car;
    car.exist = true;
    car.v = sqrt(pow(other[3], 2) + pow(other[4], 2));
    car.sd.s = other[5];
    car.sd.d = other[6];

    car.lane_id = ceil(other[6] / 4);

    //left side
    if (car.lane_id == (ego.lane_id - 1)) {
      //back
      if (car.sd.s < ego.sd.s) {
        //first detected or a more near vehicle is detected
        if (!others.back_left.exist || car.sd.s > others.back_left.sd.s) {
          others.back_left = car;
        }

      }
      //front
      else {
        if (!others.front_left.exist || car.sd.s < others.front_left.sd.s) {
          others.front_left = car;
        }
      }
    }
    //right side
    else if (car.lane_id == (ego.lane_id + 1)) {
      //back
      if (car.sd.s < ego.sd.s) {
        //first detected or a more near vehicle is detected
        if (!others.back_right.exist || car.sd.s > others.back_right.sd.s) {
          others.back_right = car;
        }

      }
      //front
      else {
        if (!others.front_right.exist || car.sd.s < others.front_right.sd.s) {
          others.front_right = car;
        }
      }
    }
    // same lane
    else if (car.lane_id == ego.lane_id) {
      //check only vehicle in the front
      if (car.sd.s > ego.sd.s) {
        //first detected
        if (!others.front.exist || car.sd.s < others.front.sd.s) {
          others.front = car;
        }
      }
    }
  }
}
bool PathPlaner::generatePrediction() {

  double score_keep = -99;
  double score_left = -99;
  double score_right = -99;

  // the car movement is based on the scores among three actions
  // Keep current lane, change to left and change to right lane


  // find where car will be at the end of last prediction
  // since we use perfect control, it is assumed the car goes exactly what it was told
  // that is why the last predict is used.
  int lane_id = ceil(last_predict_sd.d / 4);

  // get score of keep lane
  score_keep = getKeepLaneScore();
  if (lane_id > 1) {
    // if car is on lane 2 or 3, it check the score of change to left lane
    score_left = getChangeLeftScore();
  }
  if (lane_id < 3) {
    // if car is on lane 2 or 1, it check the score of change to right lane
    score_right = getChangeRightScore();
  }

  string movement;// help text to show car's next move
  string line_end;// end of line to display in console
  if (score_keep >= score_left && score_keep >= score_right) {
    //choose keep lane
    getKeepLaneTrajectory();
    movement = "↑↑";
    line_end = "\r";
  } else if (score_left > score_right) {
    getChangeToLeftLaneTrajectory();
    movement = "←←";
    line_end = "\n";
  } else {
    getChangeToRightLaneTrajectory();
    movement = "→→";
    line_end = "\n";
  }

  string car_pos = "--|   |ego|   |--";
  if (ego.lane_id < 2) {
         car_pos = "--|ego|   |   |--";
  } else if (ego.lane_id > 2) {
         car_pos = "--|   |   |ego|--";
  }

  cout << " " << car_pos;

  cout << "  V:" << setw(5) << setprecision(3) << left << last_predict_v;
  cout << "  Next Move:" << movement;
  cout << "  SCORES:(L:" << setw(4) << setprecision(3) << left << score_left;
  cout << " K:" << setw(4) << setprecision(3) << left << score_keep;
  cout << " R:" << setw(4) << setprecision(3) << left << score_right;
  cout << ") " << line_end<<flush;

  return true;

}
//change right is similar to change left except which surrounding cars to check
void PathPlaner::getChangeToRightLaneTrajectory() {
  return getChangeToLeftLaneTrajectory(false);
}
void PathPlaner::getChangeToLeftLaneTrajectory(bool left) {

  double d_target;
  bool slowDown = false;// flag is car needs to slow down or not
  if (left) {
    d_target = (ego.lane_id - 2) * 4 + 2;
    d_target = d_target < 2 ? 2 : d_target; // this helps even car drive off the road
    //when the front car on target lane is near and its's speed is slower than ego car
    if (others.front_left.exist && others.front_left.sd.s - ego.sd.s < SAFT_DISTANCE
        && others.front_left.v < last_predict_v) {
      slowDown = true;
    }
  } else {
    //check the right lane in case manuvor is to change to right
    d_target = (ego.lane_id) * 4 + 2;
    d_target = d_target > 10 ? 10 : d_target;
    if (others.front_right.exist && others.front_right.sd.s - ego.sd.s < SAFT_DISTANCE
        && others.front_right.v < last_predict_v) {
      slowDown = true;
    }
  }

  //slow down also if front car on current lane is near
  if (others.front.exist && others.front.sd.s - ego.sd.s < SAFT_DISTANCE
      && others.front.v < last_predict_v) {
    slowDown = true;
  }

  vector<double> x_path;
  vector<double> y_path;
  double delta_v;

  if (slowDown) {
    //try to decrease 2m/s if possible
    delta_v = -2;
    //further decrease if last predicted speed is bigger than front car
    //this is not guaranteed to be achieved
    if (last_predict_v > others.front.v) {
      delta_v -= last_predict_v - others.front.v;
    }
    //check the boundary, no minus speed.
    delta_v = (last_predict_v + delta_v) < 0 ? -fabs(last_predict_v) : delta_v;

  } else {
    //else try to increase velocity for maximum 2.5 m/s
    //2.5 m/s is calculated under max acceleration 10 m/s^2 within 1 second
    //by following a constant jerk value
    delta_v = (MAX_VEL - last_predict_v) > 2.5 ? 2.5 : (MAX_VEL - last_predict_v);
  }

  generateChangeLanePath(delta_v, d_target, x_path, y_path);

  for (int i = 0; i < x_path.size(); i++) {
    predition.x.push_back(x_path[i]);
    predition.y.push_back(y_path[i]);
  }

}
void PathPlaner::getKeepLaneTrajectory() {

  // here the logic is same as to change lane,
  // difference here only front car is checked.
  bool slowDown = false;
  if (others.front.exist && others.front.sd.s - ego.sd.s < SAFT_DISTANCE
      && others.front.v < last_predict_v) {
    slowDown = true;
  }

  double d_target = (ego.lane_id - 1) * 4 + 2;
  vector<double> x_path;
  vector<double> y_path;

  double delta_v;

  if (slowDown) {
    delta_v = -2;
    if (last_predict_v > others.front.v) {
      delta_v -= last_predict_v - others.front.v;
    }
    delta_v = (last_predict_v + delta_v) < 0 ? -fabs(last_predict_v) : delta_v;
  } else {
    delta_v = (MAX_VEL - last_predict_v) > 2.5 ? 2.5 : (MAX_VEL - last_predict_v);
  }

  generateKeepLanePath(delta_v, d_target, x_path, y_path);

  for (int i = 0; i < x_path.size(); i++) {
    predition.x.push_back(x_path[i]);
    predition.y.push_back(y_path[i]);
  }

}

void PathPlaner::generateKeepLanePath(double dv, double d_target,
                                      vector<double>& x_path,
                                      vector<double>& y_path) {
  /***
   * the strategy for keep lane is favor the speed control in s direction
   * the perpendicular direction d is secondary important
   * i.e. control speed in order to drive fast, avoid collision with front car
   * keep to lane center if possible, if not possible in one prediction, try next prediction
   */
  double t = 1.0;
  double v0 = last_predict_v;
  double s0 = last_predict_sd.s;
  double d0 = last_predict_sd.d;
  double delta_d = d_target - last_predict_sd.d;
  double jerk_s = 0;
  double jerk_d = 0;

  //these waypoints are used for interpolation of points
  //1. 5 points are chosen in Frenet coordinate
  //2. these 5 points are convert to X,Y coordinate
  //3. use spline to interpolate Xs, Ys against time
  //4. generate xs and ys
  vector<double> waypoints_t(5);//time series
  vector<double> waypoints_x(5);
  vector<double> waypoints_y(5);
  vector<double> waypoints_s(5);
  vector<double> waypoints_d(5, d0);

  bool overSpeed = false;// over speed flag
  do {
    jerk_s = MAX_JERK * copysign(1, dv); // jerk value depends on delta velocity to achieve
    //check if it hits the front car
    //if it hits the car then increase the time to reach target point,
    //so front car has more time to escape :)
    if (dv < 0) {
      double safe_distance = others.front.sd.s - s0 + others.front.v * t + CAR_LENGTH;
      double s_travel = v0 * t + jerk_s * pow(t / 2, 3);
      while (s_travel > safe_distance) {
        t += dT;
        safe_distance = others.front.sd.s - s0 + others.front.v * t + CAR_LENGTH;
        s_travel = v0 * t + jerk_s * pow(t / 2, 3);
      }
    }

    //check how much jerk can be allocated for s directin
    double jerk_s_needed = dv / pow(t / 2, 2);
    if (fabs(jerk_s_needed) < fabs(jerk_s)) {
      jerk_s = jerk_s_needed; // use sufficient value so the left can be used by d direction
    }

    // s directin curve is 3rd polynomial spline.
    // it moves from A to B by speed up and speed keep process
    // here 5 points are calculated at special positions
    // in fact it needs only 3 points, but conform to d direction movement, 5 points are selected
    // 2 of them are redundant.
    double delta_s1 = v0 * (t / 4) + jerk_s * pow(t / 4, 3) / 6;
    double delta_s2 = v0 * (t / 2) + jerk_s * pow(t / 2, 3) / 6;
    double delta_s3 = v0 * (t * 3 / 4) + jerk_s * pow(t, 3) * 25 / 384;
    double delta_s4 = v0 * (t) + jerk_s * pow(t / 2, 3);

    // now use the left jerk for d direction to keep ego car to lane center
    // if possible
    // the maximum distance the car can travel with given jerk equals jerk*(t/4)³*2 = d
    jerk_d = sqrt(MAX_JERK * MAX_JERK - jerk_s * jerk_s) * copysign(1, delta_d);

    //skip the calculation if ego car is really near at lane center
    if (fabs(delta_d) > 0.1) {
      double d_max = jerk_d * pow(t / 4, 3) * 2;
      if (fabs(delta_d) < fabs(d_max)) {
        jerk_d = delta_d / (pow(t / 4, 3) * 2);
      }
      double delta_d1 = jerk_d * pow(t / 4, 3) / 6;	//distance is third polynomial curve
      double delta_d2 = jerk_d * pow(t / 4, 3);
      waypoints_d = {d0, d0 + delta_d1, d0 + delta_d2, d0 + 2 * delta_d2 - delta_d1, d0 + 2 * delta_d2};
    }else
    {
      waypoints_d = {d0, d0, d0, d0, d0};
    }

    waypoints_t = {0, t / 4, t / 2, 3 * t / 4, t};
    waypoints_s = {s0, s0 + delta_s1, s0 + delta_s2,s0+delta_s3, s0 + delta_s4};
    //don't forget to correct if car travelled more than one lap.
    waypoints_s[0] = fmod(waypoints_s[0], max_s);
    waypoints_s[1] = fmod(waypoints_s[1], max_s);
    waypoints_s[2] = fmod(waypoints_s[2], max_s);
    waypoints_s[3] = fmod(waypoints_s[3], max_s);
    waypoints_s[4] = fmod(waypoints_s[4], max_s);

    //convert sd to xy
    for (size_t i = 0; i < 5; i++) {
      vector<double> xy = getXYFromSD(waypoints_s[i], waypoints_d[i]);
      waypoints_x[i] = xy[0];
      waypoints_y[i] = xy[1];
    }

    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(waypoints_t, waypoints_x);
    spline_y.set_points(waypoints_t, waypoints_y);

    x_path.clear();
    y_path.clear();
    double x_prev = last_predict_x;
    double y_prev = last_predict_y;

    overSpeed = false;

    // here check if prediction has overspeed.
    // if conversion from sd to xy is perfect, then this step is not necessary
    // the code is kept for debug purpose.
    for (size_t i = 1; i <= int(t / dT); i++) {
      x_path.push_back(spline_x(i * dT));
      y_path.push_back(spline_y(i * dT));
      double dx = x_path.back() - x_prev;
      double dy = y_path.back() - y_prev;

      //if over speed, then decrease the target velocity
      // and regenerate the trajectory.
      if (sqrt(dx * dx + dy * dy) > MPH_50 * dT) {
        dv -= 1;
        cout << "!!!Over speed!!! " << t << " dv:" << dv << endl;
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
  last_predict_x = x_path.back();
  last_predict_y = y_path.back();
}
void PathPlaner::generateChangeLanePath(double dv, double d_target,
                                        vector<double>& x_path,
                                        vector<double>& y_path) {
  /***
   * the change lane  path generation is same as generate keep lane
   * except it favors first to fulfill the movement in d direction.
   **/

  // magic 2.32 seconds will move car 4meter approximately
  // with jerk value at 8 m/s^3
  double t = 2.32;
  double v0 = last_predict_v;
  double s0 = last_predict_sd.s;
  double d0 = last_predict_sd.d;
  double delta_d = d_target - d0;

  double jerk_d = MAX_JERK * 0.8 * copysign(1, delta_d);
  double jerk_s = sqrt(MAX_JERK * MAX_JERK - jerk_d * jerk_d) * copysign(1, dv);

  vector<double> waypoints_t(5);
  vector<double> waypoints_x(5);
  vector<double> waypoints_y(5);
  vector<double> waypoints_s(5);
  vector<double> waypoints_d(5);

  // the maximum distance the car can travel with given jerk  j*(t/4)³*2 = d
  double d_max = fabs(jerk_d * pow(t / 4, 3) * 2);
  // if car does not reach the target d, increase the time
  while (d_max < fabs(delta_d)) {
    t += dT;
    d_max = fabs(jerk_d * pow(t / 4, 3) * 2);
  }

  bool overSpeed = false;

  do {

    //calculate the d direction first
    double delta_d1 = jerk_d * pow(t / 4, 3) / 6;
    double delta_d2 = jerk_d * pow(t / 4, 3);

    //remaining jerk used for speed control in s direction
    double jerk_s_needed = dv / pow(t / 2, 2);
    if (fabs(jerk_s_needed) < fabs(jerk_s)) {
      jerk_s = jerk_s_needed;
    }

    double delta_s1 = v0 * (t / 4) + jerk_s * pow(t / 4, 3) / 6;
    double delta_s2 = v0 * (t / 2) + jerk_s * pow(t / 2, 3) / 6;
    double delta_s3 = v0 * (t * 3 / 4) + jerk_s * pow(t, 3) * 25 / 384;
    double delta_s4 = v0 * (t) + jerk_s * pow(t / 2, 3);

    waypoints_t = {0, t / 4, t / 2, 3 * t / 4, t};
    waypoints_d = {d0, d0 + delta_d1, d0 + delta_d2, d0 + 2 * delta_d2 - delta_d1, d0 + 2 * delta_d2};
    waypoints_s = {s0, s0 + delta_s1, s0 + delta_s2,s0+delta_s3, s0 + delta_s4};
    waypoints_s[0] = fmod(waypoints_s[0], max_s);
    waypoints_s[1] = fmod(waypoints_s[1], max_s);
    waypoints_s[2] = fmod(waypoints_s[2], max_s);
    waypoints_s[3] = fmod(waypoints_s[3], max_s);
    waypoints_s[4] = fmod(waypoints_s[4], max_s);

    for (size_t i = 0; i < 5; i++) {
      vector<double> xy = getXYFromSD(waypoints_s[i], waypoints_d[i]);
      waypoints_x[i] = xy[0];
      waypoints_y[i] = xy[1];
    }

    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(waypoints_t, waypoints_x);
    spline_y.set_points(waypoints_t, waypoints_y);

    x_path.clear();
    y_path.clear();
    double x_prev = last_predict_x;
    double y_prev = last_predict_y;

    overSpeed = false;
    for (size_t i = 1; i <= int(t / dT); i++) {
      x_path.push_back(spline_x(i * dT));
      y_path.push_back(spline_y(i * dT));
      double dx = x_path.back() - x_prev;
      double dy = y_path.back() - y_prev;

      // the overspeed solution is a little bit different than keep lane case
      // here since most of jerk is allocated for d direction, change dv does not help much
      // instead decrease time so car change lane will not fast in oder to avoid overspeed.
      if (sqrt(dx * dx + dy * dy) > MPH_50 * dT) {
        cout << "!!!Over speed!!! " << t << " dv:" << dv << endl;
        if (dv > 0) {
          dv = 0;
        }
        t -= dT;
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
  last_predict_x = x_path.back();
  last_predict_y = y_path.back();
}

double PathPlaner::getKeepLaneScore() {
  double score = 0;
  //keep lane has per default 2 points more.
  score += 2;

  // get scored if no front car exist
  if (!others.front.exist) {
    score += 1;
    others.front.v = MAX_VEL;
    others.front.sd.s = MAX_DISTANCE;
  }

  //get scored if front car is far away
  if ((others.front.sd.s - ego.sd.s) / SAFT_DISTANCE > 10) {
    score += 10;
  } else {
    score += (others.front.sd.s - ego.sd.s) / SAFT_DISTANCE;
  }

  //get scored if front is has safe distance
  if ((others.front.sd.s - ego.sd.s) > SAFT_DISTANCE) {
    score += MAX_VEL;
  } else {
    // if not in safe distance then check who is faster
    score += others.front.v - ego.v;
  }

  return score;
}

double PathPlaner::getChangeScore(Car& front,Car& back){
  double score = 0;

  // get scored if no front car exist
  if (!front.exist) {
    score += 1;
    front.v = MAX_VEL;
    front.sd.s = MAX_DISTANCE;
  }

  //get scored if front car is far away
  if ((front.sd.s - ego.sd.s) / SAFT_DISTANCE > 10) {
    score += 10;
  } else {
    score += (front.sd.s - ego.sd.s) / SAFT_DISTANCE;
  }

  //if surrounding cars are near by, then penalty the score
  if ((front.sd.s - ego.sd.s) < CRASH_DISTANCE) {
    score -= 99;
  }

  //get scored if front is has safe distance
  if ((front.sd.s - ego.sd.s) > SAFT_DISTANCE) {
    score += MAX_VEL;
  } else {
    // if not in safe distance then check who is faster
    // ego car faster is penalted
    score += front.v - ego.v;
  }

  //if surrounding cars are near by, then penalty the score
  if (back.exist) {
    score -= 1;
    if ((ego.sd.s - back.sd.s) < SAFT_DISTANCE) {
      score -= 1;
      //check if it hits by back car
      if ((ego.sd.s - back.sd.s) < CRASH_DISTANCE) {
        score -= 99;
      }
    }
  }
  return score;
}
double PathPlaner::getChangeLeftScore(){
  return getChangeScore(others.front_left,others.back_left);
}
double PathPlaner::getChangeRightScore() {
  return getChangeScore(others.front_right,others.back_right);
}
