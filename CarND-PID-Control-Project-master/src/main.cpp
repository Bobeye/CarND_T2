#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
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

  PID steerpid;
  PID speedpid;
  // TODO: Initialize the pid variable.
  
  // Parameter set by experiment, following the tuning order of Kp, Kd, Ki
  double steer_kp = 6.0;
  double steer_ki = 0.003;
  double steer_kd = 1000.0;
  double speed_kp = 0.08;
  double speed_ki = 0.0001;
  double speed_kd = 0.001;
  steerpid.Init(steer_kp, steer_ki, steer_kd);
  speedpid.Init(speed_kp, speed_ki, speed_kd);


  int Twiddle_State = 0;
  double bestError = 100;
  int nstep = 0;

  h.onMessage([&steerpid, &speedpid, &nstep, &Twiddle_State, &bestError, &steer_kp, &steer_ki, &steer_kd, &speed_kp, &speed_ki, &speed_kd](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // steer control
          steerpid.UpdateError(cte);
          double steer_value = steerpid.Control();
          steer_value<-1?-1:steer_value;
          steer_value>1?1:steer_value;

          // speed control
          double speed_target = 10;
          speedpid.UpdateError(speed-speed_target);
          double throttle = speedpid.Control();
          throttle<-0.3?-0.3:throttle;
          throttle>0.3?0.3:throttle;

          nstep += 1;
          double steer_terror = steerpid.TotalError();
          if (abs(cte)<2.5) {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          // tuning if necessary 
          else if(abs(cte)>=2.5 && nstep>300) {
            if (Twiddle_State==0) {
              if (steer_terror/nstep < bestError) {
                bestError = steer_terror;
                steer_ki -= 0.0001;
              }
              else {
                steer_ki += 0.00005;
              }
              std::cout<<nstep<<"update ki: "<<steer_ki<<std::endl;
              steerpid.Init(steer_kp, steer_ki, steer_kd);
            } 

            // else if (Twiddle_State==1) {
            //   if (steer_terror/nstep < bestError) {
            //     bestError = steer_terror;
            //     steer_ki *= 1.1;
            //   }
            //   else {
            //     steer_ki *= 0.8;
            //   }
            //   std::cout<<nstep<<"update ki: "<<steer_ki<<std::endl;
            //   steerpid.Init(steer_kp, steer_ki, steer_kd);
            // } 
            
            // else if (Twiddle_State==2) {
            //   if (steer_terror/nstep < bestError) {
            //     bestError = steer_terror;
            //     steer_kd *= 1.1;
            //   }
            //   else {
            //     steer_kd *= 0.8;
            //   }
            //   std::cout<<nstep<<"update kd: "<<steer_kd<<std::endl;
            //   steerpid.Init(steer_kp, steer_ki, steer_kd);
            // } 

            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            std::cout<<"T error: "<<steer_terror/nstep<<std::endl;
            nstep = 0;
            // if (Twiddle_State<2) Twiddle_State+=1;
            // else Twiddle_State = 0;
          }
          else {
            ;
          }

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
