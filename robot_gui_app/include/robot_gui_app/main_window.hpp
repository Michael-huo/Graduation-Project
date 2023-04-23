/**
 * @file /include/robot_gui_app/main_window.hpp
 *
 * @brief Qt based gui for robot_gui_app.
 *
 * @date November 2010
 **/
#ifndef robot_gui_app_MAIN_WINDOW_H
#define robot_gui_app_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "dashboard.h"
#include <QProcess>
#include <QDebug>
#include <QComboBox>
#include "qrviz.hpp"
#include <QSpinBox>
#include <QPointer>
#include <QStandardItemModel>
#include <sensor_msgs/BatteryState.h>

/*****************************************************************************
** Namespace
*****************************************************************************/
//class QProcess;
namespace robot_gui_app {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initTopicList();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state); 
    void refreashTopicList();
    void uiQrviz();
    void uiOther();
    void connections();
    void terminalInit();
    void rosbagInit();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linear_value_change(int);
    void slot_angular_value_change(int);
    void slot_pushbtn_click();
    void slot_speed_x(double);
    void slot_speed_yaw(double);
    void slot_batteryState(sensor_msgs::BatteryState msg);
    void slot_humidity(QString);
    void slot_temperature(QString);
    void slot_CO2(QString);
    void slot_illuminance(QString);
    void slot_rainfall(QString);
    void slot_anemograph(QString);
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_pointcloud2(int);
    void slot_display_RobotModel(int);
    void slot_display_Map(int);
    void slot_display_Path(int);
    void slot_set_start_pose();
    void slot_set_goal_pose();
    void slot_display_global_map(int state);
    void slot_display_local_map(int state);
    void slot_update_pose(double,double,double);
    void slot_set_return_pos();
    void slot_return();

private slots:
    void slot_pushButton_recordbag();
    void slot_pushButton_stop_record();
    void slot_pushButton_playbag();
    void slot_pushButton_deletebag();
    void slot_pushButton_baglist();
    void readoutput();
    void readerror();
    void on_lineEdit_terminal_returnPressed();
    void readBashStandardOutputInfo();
    void readBashStandardErrorInfo();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QProcess *terminal_cmd;
    QProcess *bag_process;
    QPointer<QProcess> m_process;
    qrviz* myrviz;
    QComboBox* fixed_box;
    QSpinBox* Cell_Count_Box;
    QComboBox* Grid_Color_Box;
    QComboBox* Laser_Topic_box;
    QComboBox* PointCloud2_Topic_box;
    QComboBox* Map_Topic_box;
    QComboBox* Map_Color_Scheme_box;
    QComboBox* Path_Topic_box;
    QComboBox* Path_Color_box;
    QComboBox* Global_CostMap_Topic_box;
    QComboBox* Global_Map_Color_Scheme_box;
    QComboBox* Global_Planner_Topic_box;
    QComboBox* Global_Planner_Color_box;
    QComboBox* Local_CostMap_Topic_box;
    QComboBox* Local_Map_Color_Scheme_box;
    QComboBox* Local_Planner_Topic_box;
    QComboBox* Local_Planner_Color_box;
    DashBoard *speedDashBoard;
};

}  // namespace robot_gui_app

#endif // robot_gui_app_MAIN_WINDOW_H
