#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
using namespace std;
class Simple_pub{
	private:
		int range;
		
		
		ros::NodeHandle node;
		
	public:
		ros::Publisher pub = node.advertise<std_msgs::String> ("/randomAge", 100);
		Simple_pub(int);
		Simple_pub();

		int age(void){
			return (range);
		}


};
Simple_pub::Simple_pub(int rng){
	range = rng;
}

Simple_pub::Simple_pub(){
	range = 100;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Simple_pub");
	Simple_pub spub (120);

	while (ros::ok()){
		std_msgs::String msg;
		stringstream str;
		str<< "string to publish"<< endl;
		msg.data = str.str();
		spub.pub.publish(msg);
		ros::spinOnce();
	}
	
	return 10;

}