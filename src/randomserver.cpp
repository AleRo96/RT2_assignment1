#include <inttypes.h>
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using namespace std;
using RandomPosition = rt2_assignment1_ros2::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1_ros2{
    class ros2_randomserver : public rclcpp::Node
	{
	public:
		explicit ros2_randomserver(const rclcpp::NodeOptions & options) : Node("ros2_random_position", options)
	{
		random_service = this->create_service<RandomPosition>(
				"ros2_random_service", std::bind(&ros2_randomserver::handle_service, this, _1, _2, _3));
	
	}
	
	private:

	double randMToN(double M, double N)
		{ return M + (rand() /( RAND_MAX / (N-M) ));}
		
		
		
	void handle_service(
		const std::shared_ptr<rmw_request_id_t> id,
	        const std::shared_ptr<RandomPosition::Request> req,
	        const std::shared_ptr<RandomPosition::Response> res)
	        
	        {
	        res->x = randMToN(req->x_min, req->x_max);
	        res->y = randMToN(req->y_min, req->y_max);
	        res->theta = randMToN(-3.14, 3.14);	
	        return;
	        }
              
       rclcpp::Service<RandomPosition>::SharedPtr random_service;

}; // class ros2_randomserver

}//namespace rt1_assignment1

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::ros2_randomserver)
