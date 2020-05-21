#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable: 4251)

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#ifndef WIN32
#include <unistd.h>
#include <time.h>

uint32_t GetTickCount() {
	struct timespec ts;
	unsigned theTick = 0U;
	clock_gettime(CLOCK_REALTIME, &ts);
	theTick = ts.tv_nsec / 1000000;
	theTick += ts.tv_sec * 1000;
	return theTick;
}
#endif

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

class Twiddle
{
public:
	Twiddle(bool _active)
	{
		active = _active;
		/* manual tuning, good for fixed 0.3 throttle:
		p[0] = 0.4;
		p[1] = 0.00;
		p[2] = 4; */
		/* result of 333 iteration with fixed 0.3 throttle: 
		p[0] = 1.22200785;
		p[1] = 0.00786800;
		p[2] = 29.69043041;*/
		// more refined with simple speed control:
		/*p[0] = 1.09100785;
		p[1] = 0.01686800;
		p[2] = 25.821;*/
		/* result of 60 mph target speed optimization: */
		/*p[0] = 1.01061975;
		p[1] = 0.01408510;
		p[2] = 23.62485219;*/
		/* result of 100 mph target speed, optimized for time: */
		p[0] = 0.98061975;
		p[1] = 0.01408510;
		p[2] = 23.62485219;
		dp[0] = 0.03;
		dp[1] = 0.003;
		dp[2] = 0.4;
	
		/*dp[0] = 0.1;
		dp[1] = 0.01;
		dp[2] = 1;*/
		iteration = 0;
		current_param = -1; // init
		current_param_direction = 1;
		best_err = -1;
		//best_frames = -1;
		best_time = 1000;
		total_time = 0;
		total_frames = -1;
		fLog = fopen("twiddle.log", "at");
	}
	FILE *fLog;
	double p[3];
	double dp[3]; // delta p, not added into p, easier to upkeep this way, and the p value for best_err is always available
	double best_err;
	int iteration;
	double total_err;
	double total_dist;
	int total_frames;
	double total_time;
	//int best_frames;
	double best_time;
	int current_param;
	int current_param_direction; // -1 or 1
	int last_tick_count = 0;
	int lap_start_tick_count = 0;
	bool active;

	bool finished()
	{
		return best_err < 0.02; // tolerance
	}
	void step_param()
	{
		current_param = (current_param + 1) % 3;
		current_param_direction = 1; // start with add pd
	}

	void failed_to_improve()
	{
		if (current_param_direction == -1)
		{
			dp[current_param] *= 0.9;
			step_param();
		}
		else
		{
			current_param_direction = -1;
		}
	}

	void successful_improvement()
	{
		printf("iter %d param %d %.8f->%.8f\n", iteration, current_param, p[current_param],
			p[current_param] + dp[current_param] * current_param_direction);
		fprintf(fLog, "iter %d param %d %.8f->%.8f\n", iteration, current_param, p[current_param],
			p[current_param] + dp[current_param] * current_param_direction);
		p[current_param] += dp[current_param] * current_param_direction;
		dp[current_param] *= 1.1;
		best_err = total_err;
		step_param();
	}

	bool record_cte(double cte, double speed)
	{
		int now = GetTickCount();
		double delta_t = (now - last_tick_count) / 1000.0; // in seconds
		last_tick_count = now;
		total_dist += speed * delta_t;

		if (!active)
		{
			// measure lap times only
			if (cte > 5)
			{
				//printf("went offroad: %.2f\n", cte);
				total_dist = 0;
				lap_start_tick_count = now;
				return false;
			}

			if (total_dist > 1147) // track length
			{
				total_time = (now - lap_start_tick_count) / 1000.0;
				printf("lap time: %.2f sec\n", total_time);
				if (best_time > total_time) best_time = total_time;
				total_dist = 0;
				lap_start_tick_count = now;
			}
			return true;
		}
		if (cte > 5)
		{
			printf("went offroad: %.2f\n", cte);
			failed_to_improve();
			return false;
		}
		total_err += cte * cte * delta_t;
		// time based: total_err += 1/40.0;
		
		total_frames++;
		if (total_dist > 1147) // track length
		//if (total_dist > 300) // for quick optimization
		{
			total_time = (now - lap_start_tick_count) / 1000.0;
			if (best_time > total_time) best_time = total_time;
			if (current_param == -1) // init
			{
				best_err = total_err;
				step_param();
			}
			else if (total_err < best_err)
			{
				successful_improvement();
			}
			else
			{
				failed_to_improve();
			}
			return false;
		}
		return true;
	}

	void start_next_run(PID &target_pid)
	{
		last_tick_count = GetTickCount();
		lap_start_tick_count = last_tick_count;
		total_dist = 0;

		if (!active) return;
		iteration++;
		printf("iter %d best %.4f (%.8f, %.8f, %.8f) dp(%.8f,%.8f,%.8f) param %d%c best %.2fs\n", iteration, best_err, p[0], p[1], p[2], dp[0], dp[1], dp[2], current_param, current_param_direction==1?'+':'-', best_time);
		fprintf(fLog, "iter %d best %.4f (%.8f, %.8f, %.8f) dp(%.8f,%.8f,%.8f) param %d%c last %.2fs best %.2fs\n", iteration, best_err, p[0], p[1], p[2], dp[0], dp[1], dp[2], current_param, current_param_direction == 1 ? '+' : '-', total_time, best_time);
		fflush(fLog);
		target_pid.Init(
			p[0] + dp[0] * current_param_direction * (current_param == 0),
			p[1] + dp[1] * current_param_direction * (current_param == 1),
			p[2] + dp[2] * current_param_direction * (current_param == 2));
		printf("trying params: (%.8f, %.8f, %.8f)\n", target_pid.Kp, target_pid.Ki, target_pid.Kd);
		total_err = 0;
		
		total_frames = 0;
		total_time = 0;
	}

};

int main() {
  uWS::Hub h;

  PID pid;
  PID pid_speed;
  pid_speed.Init(0.1, 0, 0.1);
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.98061975, 0.01408510, 23.62485219);
  int message_count = 0;
  //double distance = 0;
  Twiddle twiddle(false);
  
  twiddle.start_next_run(pid);
  int last_lap_time_start = 0;
  
  h.onMessage([&pid, &pid_speed, &message_count, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
		  double speed_mps = speed / 2.237;
		  if (!twiddle.record_cte(cte, speed_mps))
		  {
			  /*if (twiddle.finished())
			  {
				  printf("Twiddle finished, parameters: %.8f, %.8f,%.8f\n", twiddle.p[0], twiddle.p[1], twiddle.p[2]);
			  }
			  else*/
			  {
				  twiddle.start_next_run(pid);
			  }
			  std::string reset_msg = "42[\"reset\",{}]";
			  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
			  pid.Reset();
			  pid_speed.Reset();
			  message_count = 0;
			  //distancedistance = 0;
			  return;
		  }
		  message_count++;
		  //distance += speed_mps;
		  pid.UpdateError(cte);
		  if (speed < 0.1) steer_value = 0;
		  else
		  {
			  steer_value = -pid.TotalError() * 10 / speed;
		  }
          // DEBUG
          /*std::cout << message_count<<" dist "<<distance<<" CTE: " << cte << " Speed: "<<speed<<" Steering Value: " << steer_value 
                    << std::endl;*/

		  // from my behavior cloning drive.py:
		  double set_speed = 70;
		  double throttle;
		  double target_speed = set_speed / (1 + std::min(fabs(steer_value), 1.5));
		  /* old routine from drive.py:
		  if (target_speed > speed*1.2) throttle = 100;
		  else if (target_speed > speed) throttle = 0.3;
		  else if (target_speed > speed / 1.2) throttle = 0;
		  else throttle = -100;*/
		  double speed_diff = speed - target_speed; 
		  // for pid_speed testing: double speed_diff = speed - 30;
		  pid_speed.UpdateError(speed_diff);
		  throttle = -pid_speed.TotalError();
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}