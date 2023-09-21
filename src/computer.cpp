
#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1_ros2/srv/command.hpp"
#include "rt2_assignment1_ros2/srv/position.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"
#include "rclcpp_components/register_node_macro.hpp"



using namespace std;
using RandomPosition = rt2_assignment1_ros2::srv::RandomPosition;
using Command = rt2_assignment1_ros2::srv::Command;
using Position = rt2_assignment1_ros2::srv::Position;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

bool start = false;
bool check = false;


namespace rt2_assignment1_ros2{

	class ros2_pos_client : public rclcpp::Node
{
	public:
		ros2_pos_client() : Node("Position_client"){
			
		//Definition of position client
                position_client = this->create_client<Position>("/go_to_point");
                while (!position_client->wait_for_service(std::chrono::seconds(1))){
                
	             if(!rclcpp::ok()){
		         RCLCPP_ERROR(this->get_logger(), "position client interrupted");
                         return;
	                                   }
	         }
		this->req_pos = std::make_shared<Position::Request>();
		this->res_pos = std::make_shared<Position::Response>();
			
	}
			
		void handle_position_service(){

			auto result_future= position_client->async_send_request(req_pos);
			if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_ERROR(this->get_logger(), "service call failed");
				}
			this->res_pos = result_future.get();
			
		}
		std::shared_ptr<Position::Request> req_pos;
		std::shared_ptr<Position::Response> res_pos; 
	
	
	private:
	
		rclcpp::Client<Position>::SharedPtr position_client;
	
	
}; // class ros2_pos_client



	class ros2_computer : public rclcpp::Node
 {
	public:
          ros2_computer(const rclcpp::NodeOptions & options) : Node("computer_machine", options)
   {
          //definition of a timer to handle the calls
          timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ros2_computer::timer_callback, this));
          
	  //initialization of the "control interface" service
          control_service = this->create_service<Command>("/control_interface", std::bind(&ros2_computer::handle_service, this, _1,_2,_3));
                            
	   //initialization of the random position client
          random_client = this->create_client<RandomPosition>("/random_position");
	  while (!random_client->wait_for_service(std::chrono::seconds(10))){
	        if(!rclcpp::ok()){
		    RCLCPP_ERROR(this->get_logger(), "random client interrupted");
                    return;
	            }
	      }	
    }
    
    
	private:
	
	     void handle_service(
	     const std::shared_ptr<rmw_request_id_t> id,
	     const std::shared_ptr<Command::Request> req_com,
	     const std::shared_ptr<Command::Response> res_com){
	     
	     
	       if (req_com->command =="start"){
	       
                   start = true;
                   res_com->ok = true;
                   }
                   
              else if (req_com->command == "stop"){
                  start = false;
                  res_com->ok = true;
                }
                res_com->ok = true;
  
  }
  
	void timer_callback()
	{
	
		auto random_position = std::make_shared<RandomPosition::Request>();
					
			if (check)
			{
				check=false;
			}
			else if (start == false)
			{ 
				}
			else if ( start == true)
			{
			
			random_position->x_max = 5.0;
			random_position->x_min = -5.0;
			random_position->y_max = 5.0;
			random_position->y_min = -5.0;
			
			using ServiceResponseFuture =rclcpp::Client<RandomPosition>::SharedFuture;
			
			
			auto response_received_callback= [this] (ServiceResponseFuture future) 
			{
				auto final_position = std::make_shared<ros2_pos_client>();
					final_position->req_pos->x = future.get()->x;
					final_position->req_pos->y = future.get()->y;
					final_position->req_pos->theta = future.get()->theta;
					std::cout << "\n Reaching the position: x= " << final_position->req_pos->x << " y= " <<final_position->req_pos->y << " theta = " <<final_position->req_pos->theta << std::endl;
					
					final_position->handle_position_service();
					std::cout << "Position reached" << std::endl;
			};
			auto future_result= random_client->async_send_request(random_position, response_received_callback);
			
			check = true;
			
			}		
	}
		
	rclcpp::Service<Command>::SharedPtr control_service;
	rclcpp::Client<RandomPosition>::SharedPtr random_client;
	rclcpp::TimerBase::SharedPtr timer;

}; //class ros2_computer

}//namespace

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::ros2_computer)
