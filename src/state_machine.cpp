/**
*
*
* \file state_machine.cpp
* \brief This file implement a state machine for the behaviour of the robot
* \author Roberta Alessi
* \version 0.1
* \date 5/06/2023
*
*
* \details
*
* Services : <BR>
*	° /user_interface
*
* Clients : <BR>
*	° /position_service
*
* Action Clients : <BR>
*	° /reaching_goal
* 
* Publishers : <BR>
*	° /action_status
*
*
*
*
* Description :
* In this node there are some services and clients implemented to control the robot.
* The user_interface server works with both position_server client and reaching_goal action-client.
* The position_server server is implemented in position_service.cpp file.
* When the user request to start the robot, the position_server is called to request a random position within the range of (-5, 5)
* and (-3.14, 3.14) for the orientation. Once the position_server sends its response, the action client
* reaching_goal takes those values to define the goal (in terms of x,y,theta) and makes the robot reach the goal.
*
*/







#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/planningAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/String.h"


/**
*
* \brief callback function of user_interface client to start and stop the robot's behaviour.
* 
* \param req.command string value that starts the robot. As response,it gives back a boolean value (ok)
* To understand more abouth the structure, look at the ros message Command.srv
*/
bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
*
* \brief Main function.
*
* Here the node is defined as "state_machine".
*
*/


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //client_pos action client
   actionlib::SimpleActionClient<rt2_assignment1::planningAction> client_pos("/reaching_goal", true);   
   client_pos.waitForServer();
   
   ros::Publisher status_pub = n.advertise<std_msgs::String>("action_status", 100);  
   std_msgs::String status;
   
   //random_position range value
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   while(ros::ok()){
   
   	ros::spinOnce();
   	if (start){
   	//call on position service for new random targets
   		client_rp.call(rp);
   		
   		//updating and reaching the target position received
   		rt2_assignment1::planningGoal goal;
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
  
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
   		//Waiting for the action getting result true 
   		client_pos.waitForResult();
		//call on client to ask for a new goal
   		client_pos.sendGoal(goal);
   		std::cout << "Position reached" << std::endl;
   	}
   	
   	ros::Duration(0.5).sleep();
   		
   	}
  
   return 0;
}
