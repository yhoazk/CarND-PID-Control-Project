#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <ncurses.h>


#define MAX_SPEED (.12f)
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


int main()
{
  uWS::Hub h;
  PID pid;
  PID pid_th;
  // TODO: Initialize the pid variable.
  //        P   I   D
  pid.Init(2.8,0.012, 0.25,   1.0,-1.0);
  //pid_th.Init(.5,0.0,.7,      0,1);
  //initscr();
  h.onMessage([&pid/*, &pid_th, &dp, &di, &dd*/](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    static double dp =3.0;
    //double dp =.0;
    static double di =0.012;
    //double di =0.0;
    static double dd =.735;
    static double throttle_value = 0.01;
   // double dd =2.35;
//    double best_cte;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
/*    switch (getch()) {
      case 'A':
        dp *= 1.1;
        break;
      case 'a':
        dp *= 0.9;
        break;
      case 'B':
        di *= 1.1;
        break;
      case 'b':
        di *= 0.9;
        break;
      case 'C':
        dd *= 1.1;
        break;
      case 'c':
        dd *= 0.9;
        break;
      default:
        break;
    }
    pid.update(dp,di,dd);*/
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if(fabs(cte) > .80 || fabs(steer_value) >0.6)
          {
            throttle_value *=0.9;
//            dp *=1.01;

            pid.update(dp,di,dd);

          }
          else{

            throttle_value =(throttle_value>=MAX_SPEED)?  MAX_SPEED:throttle_value +0.01;

          }

          pid.UpdateError(cte);
    //      pid_th.UpdateError(MAX_SPEED - speed);
          std::cout << "Throtle: " << throttle_value << " dp: " << dp << std::endl;
          steer_value = pid.Response();// * 1.5)/cte;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
         // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
