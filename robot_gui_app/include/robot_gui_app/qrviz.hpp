#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>

class qrviz
{
public:
    qrviz(QVBoxLayout* layout);
    void Set_FixedFrame(QString Frame_name);
    void Display_Grid(int Cell_Count,QColor color,bool enable);
    void Display_TF(bool enable);
    void Display_LaserScan(QString laser_topic, bool enable);
    void Display_PointCloud2(QString pointcloud_topic, bool enable);
    void Display_RobotModel(bool enable);
    void Display_Map(QString topic,QString color_scheme,bool enable);
    void Display_Path(QString topic,QColor color,bool enable);
    void Set_Start_Pose();
    void Set_Goal_Pose();
    void Display_Global_Map(QString costmap_topic,QString costmap_color,QString path_topic,QColor path_color,bool enable);
    void Display_Local_Map(QString costmap_topic,QString costmap_color,QString path_topic,QColor path_color,bool enable);

private:
    rviz::RenderPanel* render_panel_;
    rviz::VisualizationManager* manager_;
    rviz::Display* Grid_=NULL;
    rviz::Display* TF_=NULL;
    rviz::Display* LaserScan_=NULL;
    rviz::Display* PointCloud2_=NULL;
    rviz::Display* RobotModel_=NULL;
    rviz::Display* Map_=NULL;
    rviz::Display* Path_=NULL;
    rviz::ToolManager* tool_manager_;
    rviz::Display* Global_Map_=NULL;
    rviz::Display* Global_Path_=NULL;
    rviz::Display* Local_Map_=NULL;
    rviz::Display* Local_Path_=NULL;

};

#endif // QRVIZ_HPP
