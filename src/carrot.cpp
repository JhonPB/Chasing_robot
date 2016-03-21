#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include<geometry_msgs/Point.h>

ros::Publisher twist_pub;
geometry_msgs::Twist v;




double norm (double p)
    

{
    
    if (p > M_PI)
    {
        return p - 2*M_PI;
    }
    else if (p < -M_PI)
    {
        return p + 2*M_PI;
    }
    else
    {
        return p;
    }
}





void carrotcallback(const nav_msgs::Odometry::ConstPtr &z)
{
    geometry_msgs::Quaternion qt;
    qt = z->pose.pose.orientation;
    double yaw = tf::getYaw(qt);
    
    geometry_msgs::Point gt;
    gt = z->pose.pose.position;
    double x = gt.x;
    double y = gt.y;
    
    double xi = 4;
    double yi = 1.5;
    
    double xf = -3;
    double yf = 1.5;
    
    double xr = xi - x;
    double yr = yi - y;
    
    double j = yf - yi;
    double k = xf - xi;
    
    double l = y - yi;
    double m = x - xi;
    
    double t = atan2 (j,k);
    double tu = atan2(l,m);
    double b = t - tu;
    
    double ru = sqrt(pow(xr,2) + pow(yr,2));
    double r = sqrt(pow(ru,2) - pow(ru*sin(b),2));
    
    double xc = (r + 0.5)*cos(t) + xi;
    double yc = (r + 0.5)*sin(t) + yi;  
    
    double y_t = yf - y, x_t  = xf - x;
    
    double target = sqrt(pow(y_t,2) + pow(x_t,2)); 
                     
                     
    double xo = xc - x;
    double yo = yc - y;
    
    double ang = atan2(yo,xo) - yaw;
    double d = sqrt(pow(xo,2) + pow(yo,2));
    
    ang = norm(ang);
                     
      if (target > 0.2)
                     
{
    if (std::abs(ang) > 0.1)
    {
        
        v.linear.x = 0.2;
        v.angular.z = ang;   
        
    }
    else
    {
        v.linear.x = 0.2;
        v.angular.z = 0;
        
    }
      }
    else
    {
        v.linear.x = 0;
        v.angular.z = 0;
    }

                     
                     
                     
    
    twist_pub.publish(v);
    
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "carrot");
    
  ros::NodeHandle nh;
    
  ros::Subscriber sub = nh.subscribe("/vrep/vehicle/odometry", 1, carrotcallback);
    
  twist_pub = nh.advertise <geometry_msgs::Twist> ("gogo", 1);
    
   ros::spin();
    
    
}