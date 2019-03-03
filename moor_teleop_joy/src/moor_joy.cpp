/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class MoorTeleop
{
public:
  MoorTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, light_0_axis_, light_1_axis_, skid_left_axis_, skid_right_axis_,increase_l_scale_, increase_a_scale_ , decrease_l_scale_, decrease_a_scale_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool skid_left_pressed_;
  bool light_0_pressed_;
  bool light_1_pressed_;
  bool skid_right_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

MoorTeleop::MoorTeleop():
  ph_("~"),
  linear_(1),
  angular_(0),
  skid_left_axis_(4),
  skid_right_axis_(5),
  light_0_axis_(6),
  light_1_axis_(7),
  increase_a_scale_(1),
  increase_l_scale_(0),
  decrease_a_scale_(3),
  decrease_l_scale_(2),
  l_scale_(0.2),
  a_scale_(0.2)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("button_increase_l_scale", increase_l_scale_, increase_l_scale_);
  ph_.param("button_increase_a_scale", increase_a_scale_, increase_a_scale_);
  ph_.param("button_decrease_l_scale", decrease_l_scale_, decrease_l_scale_);
  ph_.param("button_decrease_a_scale", decrease_a_scale_, decrease_a_scale_);
  ph_.param("axis_skid_left", skid_left_axis_, skid_left_axis_);
  ph_.param("axis_light_0", light_0_axis_, light_0_axis_);
  ph_.param("axis_light_1", light_1_axis_, light_1_axis_);
  ph_.param("axis_skid_right", skid_right_axis_, skid_right_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  skid_left_pressed_ = false;
  light_0_pressed_ = false;
  light_1_pressed_ = false;
  skid_right_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &MoorTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&MoorTeleop::publish, this));
}

void MoorTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  if (joy->buttons[increase_l_scale_] !=0)
	l_scale_=l_scale_+ 0.1;
  if (joy->buttons[increase_a_scale_] !=0)
	a_scale_=a_scale_+ 0.1;
  if (joy->buttons[decrease_l_scale_] !=0)
	l_scale_=l_scale_- 0.1;
  if (joy->buttons[decrease_a_scale_] !=0)
	a_scale_=a_scale_- 0.1;
  if (a_scale_<0)
	a_scale_=0;
  if (l_scale_<0)
	l_scale_=0;
  vel.angular.y=a_scale_;
  vel.linear.y=l_scale_;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
skid_left_pressed_ = joy->buttons[skid_left_axis_];
skid_right_pressed_ = joy->buttons[skid_right_axis_];
light_0_pressed_ = joy->buttons[light_0_axis_];
light_1_pressed_ = joy->buttons[light_1_axis_];
  if(skid_left_pressed_ )
  {
	vel.angular.z = a_scale_;
	vel.linear.x = l_scale_;
  }
  if(skid_right_pressed_ )
  {
	vel.angular.z = -1 * a_scale_;
	vel.linear.x = l_scale_;
  }
if(light_0_pressed_ )
  {
	vel.angular.z = a_scale_;
	vel.linear.x = -1 * l_scale_;
  }
if(light_1_pressed_ )
  {
	vel.angular.z = -1 * a_scale_;
	vel.linear.x = -1 * l_scale_;
  }
  last_published_ = vel;
  
}

void MoorTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  vel_pub_.publish(last_published_);
  zero_twist_published_=false;
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  MoorTeleop moor_teleop;

  ros::spin();
}
