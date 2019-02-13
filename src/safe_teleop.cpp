/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  linear_speed_(0.0),
  angular_speed_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("base_scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      geometry_msgs::Twist vel_msg;
      linear_vel_ = 0;
      angular_vel_ = 0.0;
      vel_msg.linear.x = linear_vel_;
      vel_msg.angular.z = angular_vel_;
      cmd_vel_pub_.publish(vel_msg);
      
      ROS_WARN_THROTTLE(1.0, "Timeout not implemented\r");
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      
      geometry_msgs::Twist vel_msg;

      if (!is_safe)
      {
        linear_vel_ = 0;
        ROS_WARN_THROTTLE(1.0, "ostabcle\r");
      }
      
      vel_msg.linear.x = linear_vel_;
      vel_msg.angular.z = angular_vel_;
      cmd_vel_pub_.publish(vel_msg);

      // ROS_WARN_THROTTLE(1.0, "command velocity publishing not implemented\r");
    }

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
  linear_vel_ = (double)linear_speed_;

  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::moveBackward()
{
  linear_vel_ = -(double)linear_speed_;

  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateClockwise()
{
  angular_vel_ = (double)angular_speed_;

  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateCounterClockwise()
{
  angular_vel_ = -(double)angular_speed_;

  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::stop()
{
  linear_vel_ = 0;
  angular_vel_ = 0.0;

  last_command_timestamp_ = ros::Time::now().toSec();
}


void SafeTeleop::increaseLinearSpeed()
{
  if (linear_speed_+linear_vel_increment_<=max_linear_vel_)
  {
    linear_speed_= linear_speed_ + linear_vel_increment_;

  }
  else
  {
    linear_speed_=max_linear_vel_;
  }
  last_command_timestamp_ = ros::Time::now().toSec();

  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  if (linear_speed_-linear_vel_increment_>0)
  {
    linear_speed_= linear_speed_ - linear_vel_increment_;
  }
  else
  {
    linear_speed_=0;
  }

  last_command_timestamp_ = ros::Time::now().toSec();
  
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  if (angular_speed_+angular_vel_increment_<=max_angular_vel_)
  {
    angular_speed_= angular_speed_ + angular_vel_increment_;
  }
  else
  {
    angular_speed_=max_angular_vel_;
  }

  last_command_timestamp_ = ros::Time::now().toSec();

  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  if (angular_speed_-angular_vel_increment_>=0)
  {
    angular_speed_= angular_speed_ - angular_vel_increment_;
  }
  else
  {
    angular_speed_=0;
  }

  last_command_timestamp_ = ros::Time::now().toSec();

  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();

  // ROS_INFO_THROTTLE(1.0, "print laser: %f,%f,%f\r", laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment);

  // for (int i=0; i<laser_scan.ranges.size(); i++)
  // {
  //   ROS_INFO("%d, %f\r", i, laser_scan.ranges[i]);
  // }

  if (linear_vel!=0)
  {
    int mid = laser_scan.ranges.size()/2;
    int idx_range = 15/(180/mid+1);
    // ROS_INFO("%d\r", idx_range);
  
  if ((linear_vel>0&&laser_scan.angle_min<0)||(linear_vel<0&&laser_scan.angle_min==0))
  {
    //forward
    for (int i=mid-1; i<mid+idx_range; i++)
    {
      if (laser_scan.ranges[i]<min_safety_distance_)
      { 
        ROS_INFO("%d, %f\r", i, laser_scan.ranges[i]);
        return false;
      }
        
    }
    for (int i=mid-1; i>mid-idx_range; i--)
    {
      if (laser_scan.ranges[i]<min_safety_distance_)
      {
        ROS_INFO("%d, %f\r", i, laser_scan.ranges[i]);
        return false;
      }
    }
    
  }

  else if ((linear_vel<0&&laser_scan.angle_min<0)||linear_vel>0&&laser_scan.angle_min==0)
  {
    // backward
    for (int i=0; i<5; i++)
    {
      if (laser_scan.ranges[i]<min_safety_distance_)
      {
        ROS_INFO("%d, %f\r", i, laser_scan.ranges[i]);
        return false;
      }
    }

    for (int i=laser_scan.ranges.size()-1; i>laser_scan.ranges.size()-5; i--)
    {
      if (laser_scan.ranges[i]<min_safety_distance_)
      {
        ROS_INFO("%d, %f\r", i, laser_scan.ranges[i]);
        return false;
      }
    }
  }
  }

  return true;

}

} // namespace safe_teleop_node


