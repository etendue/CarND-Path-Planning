#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <fstream>
#include <string>
#include "PathPlaner.h"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}



int main() {
  uWS::Hub h;

  PathPlaner pp;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  /***
   * keep a variable containing last trajectory including
   * x,y, x',y',x'', y'', as for JMP, s,s',and s'' are needed for continuity.
   */
  tk::spline curve_smoother_x;
  tk::spline curve_smoother_y;
  curve_smoother_x.set_points(map_waypoints_s,map_waypoints_x);
  curve_smoother_y.set_points(map_waypoints_s,map_waypoints_y);

  int new_sample_count = floor(max_s)+1;
  map_waypoints_x.resize(new_sample_count);
  map_waypoints_y.resize(new_sample_count);
  map_waypoints_s.resize(new_sample_count);

//  ofstream log("spline.txt", std::ofstream::out);

  for(int i=0;i<new_sample_count-1;i++)
  {
    map_waypoints_s[i]=i;
    map_waypoints_x[i] = curve_smoother_x(map_waypoints_s[i]);
    map_waypoints_y[i] = curve_smoother_y(map_waypoints_s[i]);
//    log<<map_waypoints_s[i]<<","<<map_waypoints_x[i]<<","<<map_waypoints_y[i]<<endl;
  }
  //last point
  int i = new_sample_count-1;
  map_waypoints_s[i] = max_s;
  map_waypoints_x[i] = curve_smoother_x(map_waypoints_s[i]);
  map_waypoints_y[i] = curve_smoother_y(map_waypoints_s[i]);
/*  log<<map_waypoints_s[i]<<","<<map_waypoints_x[i]<<","<<map_waypoints_y[i]<<endl;
  log.close();*/
  vector<PointState2D> last_prediction(N);
  for(int i=0; i<N; i++)
  {
    last_prediction[i].s = VectorXd(3);
    last_prediction[i].d = VectorXd(3);
  }

  //pack data for path planer
  InputData d;
  d.self_state.s = VectorXd(3);
  d.self_state.d = VectorXd(3);

  double last_theta;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,max_s,
			   &map_waypoints_dy,&pp,&last_prediction,&last_theta,&d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];


          	double theta = deg2rad(car_yaw);

          	size_t size = previous_path_x.size();
          	if(size > 0){
          		size_t index = last_prediction.size() - size;
          		PointState2D next_point = last_prediction[index];
              double delta_theta = theta - last_theta;
          		d.self_state=next_point;
          		//d.self_state.s[1] = -next_point.d[1] * sin(delta_theta) + next_point.s[1] * cos(delta_theta);
          		//d.self_state.s[2] = -next_point.d[2] * sin(delta_theta) + next_point.s[2] * cos(delta_theta);
//              d.self_state.d[0] = car_d;
//          		d.self_state.d[1] = next_point.d[1];
//          		d.self_state.d[2] = next_point.d[2];
          	  //d.self_state.d[1] = next_point.d[1] * cos(delta_theta) + next_point.s[1] * sin(delta_theta);
          		//d.self_state.d[2] = next_point.d[2] * cos(delta_theta) + next_point.s[2] * sin(delta_theta);

          	}else
          	{
          		d.self_state.s << car_s,car_speed,0;
          		d.self_state.d << car_d,0,0;
          	}

//          	for(int i=0;i<N-size;i++)
//          	{
//
//          	  cout << last_prediction[i].s[0]<<" "<<last_prediction[i].d[0]<<" "<<endl;
//          	}
//          	static int cycle = 0;
//          	ofstream log("log.txt",std::ofstream::out | std::ofstream::app);
//
//          	for(int i = 0; i<N; i++)
//          	{
//          		log<<cycle<< " "<<last_prediction[i].s[0] << " "<< last_prediction[i].d[0] <<endl;
//          	}
//          	log.close();
//          	cycle ++;

          	last_theta = theta;

          	d.sensor_data.clear();
          	for(auto vehicle:sensor_fusion){
          	  d.sensor_data.push_back(vehicle);
          	}


          	pp.processingInputData(d);
          	pp.getBestTrajectory(last_prediction);


          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	for(int i=0;i<last_prediction.size();i++){
          	  //check if it runs over one whole circle.
          	  if(last_prediction[i].s[0]>max_s){
          	    last_prediction[i].s[0] -= max_s;
          	  }
          	  auto xy = getXY(last_prediction[i].s[0],last_prediction[i].d[0],map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	  //auto xy = getXY(last_prediction[i].s[0],6,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	  next_x_vals.push_back(xy[0]);
          	  next_y_vals.push_back(xy[1]);

          	}

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
          	//std::cout << msg << std::endl;
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































