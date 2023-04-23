/**
 * @file /include/robot_gui_app/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_gui_app_QNODE_HPP_
#define robot_gui_app_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QDebug>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <map>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QImage>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui_app {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void set_cmd_vel(char k,float linear,float angular);
    void sub_image(QString topic_name);
    void set_goal(double x,double y,double z);
    QMap<QString, QString> get_topic_list();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void speed_vel(float,float);
    void power_vel(float);
    void image_val(QImage);
    void position(double,double,double);
    void speed_x(double);
    void speed_y(double);
    void env_humidity(QString);
    void env_temperature(QString);
    void env_CO2(QString);
    void env_illuminance(QString);
    void env_rainfall(QString);
    void env_anemograph(QString);

private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    ros::Publisher cmd_vel_pub;
    ros::Publisher goal_pub;
    ros::Subscriber chatter_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber cmdVel_sub;
    ros::Subscriber power_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber env_humidity_sub;
    ros::Subscriber env_temperature_sub;
    ros::Subscriber env_CO2_sub;
    ros::Subscriber env_illuminance_sub;
    ros::Subscriber env_rainfall_sub;
    ros::Subscriber env_anemograph_sub;
    QStringListModel logging_model;
    image_transport::Subscriber image_sub;
    QString odom_topic;
    QImage Mat2QImage(cv::Mat const& src);
    void chatter_callback(const std_msgs::String &msg);
    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void power_callback(const std_msgs::Float32 &msg);
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
    void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void env_humidity_callback(const std_msgs::String::ConstPtr& msg);
    void env_temperature_callback(const std_msgs::String::ConstPtr& msg);
    void env_CO2_callback(const std_msgs::String::ConstPtr& msg);
    void env_illuminance_callback(const std_msgs::String::ConstPtr& msg);
    void env_rainfall_callback(const std_msgs::String::ConstPtr& msg);
    void env_anemograph_callback(const std_msgs::String::ConstPtr& msg);

};

}  // namespace robot_gui_app

#endif /* robot_gui_app_QNODE_HPP_ */
