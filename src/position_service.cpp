/**
* \file position_service.cpp
* \brief This file implement a position service for the robot to reach a random position.
* \author Roberta Alessi
* \version 0.1
* \date 5/06/2023
*
* \details
* 
*
* Services : <BR>
*	° /position_server
*
* Description :
*
* This node advertises a position service. When the service is required, we set the response,
* so that it can get a random position, predefined in the request within an limited range.
* 
*/






#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
* \brief Function to generate a random value within a range given by two paramenters M and N.
*
* \param M set the maximum value for the range
* \param N set the minimun value for the range
*
*/


double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
*
* \brief Service function used to get the random position
* 
* \param res.x get position x
* \param res.y get position y
* \param res.theta get angle theta
*
*/

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


/**
* \brief Main function
*
* Node defined as "random_position_server". 
*
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
