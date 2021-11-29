/**
 * Copyright (c) 2021 Rishabh Mukund
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <array>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"



ros::Publisher pub;

/**
 * @brief A function to check for obstacles using the lidar data
 *
 * @param: (sensor_msgs::LaserScan) Lidar data
 * @return: (bool) if or not there is an obstacle
 */
bool obstacle_detected(const sensor_msgs::LaserScan::ConstPtr& msg){
	// Checking for +20 degrees to -20 degrees
	std::array<int, 41> degrees = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 , 11, 12,
				13, 14, 15, 16, 17, 18, 19, 20, 340, 341, 342, 343, 344, 345, 346,
				347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359};

	// If obstacle is present before 0.4 meters return true
	for(auto i : degrees) {
		if(msg->ranges[i] < 0.4){
			ROS_INFO("Obstacle Detected");
			return true;
		}
	}
	ROS_INFO("Obstacle Not Detected");
	return false;
}

/**
 * @brief A callback function to print out the data received from lidar and
 * moving the robot to avoid obstacles
 *
 * @param: (sensor_msgs::LaserScan) Lidar data
 */
void walkerCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	geometry_msgs::Twist move_cmd;
	if(obstacle_detected(msg)){
		// Obstacle ahead so turn
		move_cmd.linear.x = 0.0;
		move_cmd.angular.z = 0.2;
	} else {
		// No obstacle ahead so go straight
		move_cmd.linear.x = 0.2;
		move_cmd.angular.z = 0.0;
	}
	pub.publish(move_cmd);
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "walker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 500,
		  walkerCallback);

  /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
  pub = n.advertise <geometry_msgs::Twist> ("/cmd_vel", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
