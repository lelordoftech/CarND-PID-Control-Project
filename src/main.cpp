#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <sys/time.h>

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

#ifndef MODE_NORMAL
void check_arguments(int argc, char* argv[])
{
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " Kp Ki Kd";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc < 4)
  {
    std::cerr << usage_instructions << std::endl;
  }
  else if (argc == 4)
  {
    has_valid_args = true;
  }
  else if (argc > 4)
  {
    std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
  }

  if (!has_valid_args)
  {
    exit(EXIT_FAILURE);
  }
}
#endif

const double steer_value_max = 1; // ~ deg2rad(57) ~ car 25deg
const double steer_value_min = -1; // ~ deg2rad(57) ~ car -25deg
const double throttle_max = 0.5;
struct timeval tp;
double pre_time = 0;

double p[3] = {0.389311, 0.0961499, 0.184516};
unsigned int it = 0; // Count number of test sample
unsigned int timeout = 0; // Count time in case Car cannot run
double err = 0;
double road = 0;
bool isConnected = false; // Check simulator status

#ifdef MODE_TWIDDLE
double dp[3] = {0.1, 0.001, 0.01};
unsigned int id = 0; // check p[] ~ Kp Ki Kd
unsigned int step = 0; // Jump step by step of twiddle
double best_err = 0;
bool isRun = true;
bool isUpdateFirstTime = true;
double threshold = 0.01;

void twiddle()
{
  if (isUpdateFirstTime == true)
  {
    isUpdateFirstTime = false;
    best_err = err/it;
  }

  if (dp[0]+dp[1]+dp[2] > threshold)
  {
    while (isRun == false)
    {
      if (step == 0)
      {
        p[id] += dp[id];
        step = 1;
        isRun = true;
      }
      else if (step == 1)
      {
        if (fabs(err/it) < fabs(best_err)) // There was some improvement
        {
          best_err = err/it;
          dp[id] *= 1.1;
          step = 0;
          id++;
          isRun = false;
        }
        else // There was no improvement
        {
          p[id] -= 2*dp[id]; // Go into the other direction
          step = 2;
          isRun = true;
        }
      }
      else if (step == 2)
      {
        if (fabs(err/it) < fabs(best_err)) // There was some improvement
        {
          best_err = err/it;
          dp[id] *= 1.05;
        }
        else // There was no improvement
        {
          p[id] += dp[id];
          dp[id] *= 0.95;
        }
        step = 0;
        id++;
        isRun = false;
      }

      if (id > sizeof(p)/sizeof(*p))
      {
        id = 0;
      }
    }
    isRun = false;
  }
}
#endif

int main(int argc, char* argv[])
{
#ifdef MODE_TWIDDLE
  std::cout << "TWIDDLE MODE" << std::endl;
#elif MODE_MANUAL
  std::cout << "MANUAL MODE" << std::endl;
#elif MODE_NORMAL
  std::cout << "NORMAL MODE" << std::endl;
#endif

  std::string out_file_name_ = "../output.txt";
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);
#ifndef MODE_NORMAL
  check_arguments(argc, argv);
#endif

#ifdef MODE_TWIDDLE
  out_file_ << "it BestError Tolerance Kp Ki Kd" << std::endl;
#elif MODE_MANUAL
  out_file_ << "it Road TotalError AverageError Kp Ki Kd" << std::endl;
#elif MODE_NORMAL
  out_file_ << "it CTE SteeringValue" << std::endl;
#endif

  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
#ifndef MODE_NORMAL
  double Kp = atof(argv[1]);
  double Ki = atof(argv[2]);
  double Kd = atof(argv[3]);
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
#endif
  pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid,&out_file_]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry" && isConnected == true)
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0;
          double throttle_value = 0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          gettimeofday(&tp, NULL);
          double current_time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
          double dt = current_time - pre_time;
          if (dt == 0 || dt > 30)
          {
            dt = 18;
          }
          pre_time = current_time;

          if (cte == pid.p_error)
          {
            timeout++;
          }
          else
          {
            timeout = 0;
          }

          if ((it > 100 && fabs(cte) > 3.5) // Car throw out lane after start 100 frame
#ifndef MODE_NORMAL
              || (road > 1200) // 1 lap : 1200m - max speed 50mph
#endif
              || (timeout > 50)) // Timeout case: car stop in a large time
          {
#ifdef MODE_TWIDDLE
            out_file_ << it << "\t" 
                      << best_err << "\t" 
                      << dp[0]+dp[1]+dp[2] << "\t" 
                      << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" 
                      << std::endl;
            std::cout << it << "\t" 
                      << "best_err " << best_err 
                      << " tolerance: " << dp[0]+dp[1]+dp[2]
                      << " Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] 
                      << " dKp: " << dp[0] << " dKi: " << dp[1] << " dKd: " << dp[2] 
                      << std::endl;
            twiddle();
#elif MODE_MANUAL
            out_file_ << it << "\t" 
                      << road << "\t" 
                      << err << "\t" 
                      << err/it << "\t" 
                      << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" 
                      << std::endl;
            std::cout << it
                      << " road " << road
                      << " total_err " << err 
                      << " average_err " << err/it 
                      << " Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] 
                      << std::endl;
            p[1] += 0.001; // Change this to update all p[]
#endif
            pid.Init(p[0], p[1], p[2]);

            // Reset
            it = 0;
            timeout = 0;
            err = 0;
            road = 0;
            isConnected = false;

            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
            it++;
            err += fabs(cte); // Collect total err
            road += speed*0.44704 * dt/1000; // Convert from miles per hour to meters per second
          }

          pid.UpdateError(0-cte, dt/1000); // Set point is 0 mean center lane
          steer_value = pid.TotalError();

          if (steer_value > steer_value_max)
          {
            steer_value = steer_value_max;
          }
          else if (steer_value < steer_value_min)
          {
            steer_value = steer_value_min;
          }

#ifdef MODE_NORMAL
          out_file_ << it << "\t" << cte << "\t" << steer_value << std::endl;
          // DEBUG
          std::cout << it << "\t" << cte << "\t" << steer_value << std::endl;
#endif

          if (fabs(steer_value) > 0.3 && speed > throttle_max*100*3/5)
          {
            throttle_value = -0.1; // Brake to decresse speed incase high steering angle
          }
          else
          {
            throttle_value = throttle_max;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value; // radian
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;  
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
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

  h.onConnection([&h,&pid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;

    isConnected = true;

    pid.Init(p[0], p[1], p[2]);
    // Reset
    it = 0;
    timeout = 0;
    err = 0;
    road = 0;

    // Get the first time point
    gettimeofday(&tp, NULL);
    pre_time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  });

  h.onDisconnection([&h,&out_file_](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;

    isConnected = false;
    // close files
    if (out_file_.is_open())
    {
      out_file_.close();
    }
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
