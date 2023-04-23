/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/robot_gui_app/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui_app {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
    qRegisterMetaType<sensor_msgs::BatteryState>("sensor_msgs::BatteryState");
    qRegisterMetaType<QVector<int>>("QVector<int>");
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robot_gui_app");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    chatter_sub=n.subscribe("chatter",1000,&QNode::chatter_callback,this);
    cmdVel_sub = n.subscribe("odom",1000,&QNode::speedCallback, this);
    power_sub = n.subscribe("power",1000,&QNode::power_callback,this);
    env_humidity_sub = n.subscribe("env_sensors_humidity",1000,&QNode::env_humidity_callback,this);
    env_temperature_sub = n.subscribe("env_sensors_temperature",1000,&QNode::env_temperature_callback,this);
    env_CO2_sub = n.subscribe("env_sensors_CO2",1000,&QNode::env_CO2_callback,this);
    env_illuminance_sub = n.subscribe("env_sensors_illuminance",1000,&QNode::env_illuminance_callback,this);
    env_rainfall_sub = n.subscribe("env_sensors_rainfall",1000,&QNode::env_rainfall_callback,this);
    env_anemograph_sub = n.subscribe("env_sensors_anemograph",1000,&QNode::env_anemograph_callback,this);
    amcl_pose_sub=n.subscribe("amcl_pose",1000,&QNode::amcl_pose_callback,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_gui_app");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    chatter_sub=n.subscribe("chatter",1000,&QNode::chatter_callback,this);
    cmdVel_sub = n.subscribe("odom",1000,&QNode::speedCallback, this);
    power_sub = n.subscribe("power",1000,&QNode::power_callback,this);
    env_humidity_sub = n.subscribe("env_sensors_humidity",1000,&QNode::env_humidity_callback,this);
    env_temperature_sub = n.subscribe("env_sensors_temperature",1000,&QNode::env_temperature_callback,this);
    env_CO2_sub = n.subscribe("env_sensors_CO2",1000,&QNode::env_CO2_callback,this);
    env_illuminance_sub = n.subscribe("env_sensors_illuminance",1000,&QNode::env_illuminance_callback,this);
    env_rainfall_sub = n.subscribe("env_sensors_rainfall",1000,&QNode::env_rainfall_callback,this);
    env_anemograph_sub = n.subscribe("env_sensors_anemograph",1000,&QNode::env_anemograph_callback,this);
    amcl_pose_sub = n.subscribe("amcl_pose",1000,&QNode::amcl_pose_callback,this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
        ss << "连接正常 " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
        log(Info,std::string("sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
void QNode::set_cmd_vel(char k,float linear,float angular){
    std::map<char, std::vector<float>> moveBindings{
          {'i', {1, 0, 0, 0}},  {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
          {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
          {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}},
          {'I', {1, 0, 0, 0}},  {'J', {0, 1, 0, 0}},   {'L', {0, -1, 0, 0}},
          {'U', {1, 1, 0, 0}},  {'<', {-1, 0, 0, 0}},  {'>', {-1, -1, 0, 0}},
          {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}},   {'b', {0, 0, -1, 0}},
          {'k', {0, 0, 0, 0}},  {'K', {0, 0, 0, 0}}};
      char key = k;
      //计算是往哪个方向
      float x = moveBindings[key][0];
      float y = moveBindings[key][1];
      float z = moveBindings[key][2];
      float th = moveBindings[key][3];
      //计算线速度和角速度
      float speed = linear;
      float turn = angular;
      // Update the Twist message
      geometry_msgs::Twist twist;
      twist.linear.x = x * speed;
      twist.linear.y = y * speed;
      twist.linear.z = z * speed;

      twist.angular.x = 0;
      twist.angular.y = 0;
      twist.angular.z = th * turn;

      // Publish it and resolve any remaining callbacks
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
}
void QNode::chatter_callback(const std_msgs::String &msg){
    log(Info,"recive:"+msg.data);
}
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.angular.z);
}
void QNode::power_callback(const std_msgs::Float32 &msg){
    emit power_vel(msg.data);
}
void QNode::env_humidity_callback(const std_msgs::String::ConstPtr& msg){
    emit env_humidity(msg->data.data());
//    qDebug()<<"humidity_debug";
}
void QNode::env_temperature_callback(const std_msgs::String::ConstPtr& msg){
    emit env_temperature(msg->data.data());
//    qDebug()<<"temperature_debug";
}
void QNode::env_CO2_callback(const std_msgs::String::ConstPtr& msg){
    emit env_CO2(msg->data.data());
//    qDebug()<<"CO2_debug";
}
void QNode::env_illuminance_callback(const std_msgs::String::ConstPtr& msg){
    emit env_illuminance(msg->data.data());
//    qDebug()<<"illuminance_debug";
}
void QNode::env_rainfall_callback(const std_msgs::String::ConstPtr& msg){
    emit env_rainfall(msg->data.data());
//    qDebug()<<"rainfall_debug";
}
void QNode::env_anemograph_callback(const std_msgs::String::ConstPtr& msg){
    emit env_anemograph(msg->data.data());
//    qDebug()<<"anemograph_debug";
}

void QNode::sub_image(QString topic_name){
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);
    image_sub=it_.subscribe(topic_name.toStdString(),1000,&QNode::image_callback,this);
}
void QNode::image_callback(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,msg->encoding);
    QImage im=Mat2QImage(cv_ptr->image);
    emit image_val(im);
}
QImage QNode::Mat2QImage(cv::Mat const& src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}

void QNode::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg){
    emit position(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z);

}
void QNode::set_goal(double x,double y,double z){
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id="map";
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.orientation.z=z;
    goal_pub.publish(goal);
}

QMap<QString, QString> QNode::get_topic_list() {
  ros::master::V_TopicInfo topic_list;
  ros::master::getTopics(topic_list);
  QMap<QString, QString> res;
  for (auto topic : topic_list) {
    res.insert(QString::fromStdString(topic.name),
               QString::fromStdString(topic.datatype));
  }

  return res;
}

}  // namespace robot_gui_app
