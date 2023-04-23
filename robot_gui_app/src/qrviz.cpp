#include "../include/robot_gui_app/qrviz.hpp"
#include <QDebug>

qrviz::qrviz(QVBoxLayout* layout)
{
    //creat rviz panel
    render_panel_ = new rviz::RenderPanel();
    //add to layout
    layout->addWidget(render_panel_);
    //creat rviz con_obj
    manager_=new rviz::VisualizationManager(render_panel_);
    tool_manager_=manager_->getToolManager();
    ROS_ASSERT(manager_!=NULL);
    //init render_panel
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->setFixedFrame("map");
    //init rviz con_obj
    manager_->initialize();
    manager_->startUpdate();
    manager_->removeAllDisplays();
}

void qrviz::Set_FixedFrame(QString Frame_name){
    manager_->setFixedFrame(Frame_name);
    qDebug()<<manager_->getFixedFrame();
}

void qrviz::Display_Grid(int Cell_Count,QColor color,bool enable){
    if(Grid_!=NULL){
        delete Grid_;
        Grid_=NULL;
    }
    Grid_=manager_->createDisplay("rviz/Grid","myGrid",enable);
    Grid_->subProp("Plane Cell Count")->setValue(Cell_Count);
    Grid_->subProp("Color")->setValue(color);
    ROS_ASSERT(Grid_!=NULL);
}
void qrviz::Display_TF(bool enable){
    if(TF_!=NULL){
        delete TF_;
        TF_=NULL;
    }
    TF_=manager_->createDisplay("rviz/TF","myTF",enable);
    ROS_ASSERT(TF_!=NULL);
}
void qrviz::Display_LaserScan(QString laser_topic, bool enable){
    if(LaserScan_!=NULL){
        delete LaserScan_;
        LaserScan_=NULL;
    }
    LaserScan_=manager_->createDisplay("rviz/LaserScan","myLaser",enable);
    LaserScan_->subProp("Topic")->setValue(laser_topic);
    ROS_ASSERT(LaserScan_!=NULL);
}
void qrviz::Display_PointCloud2(QString pointcloud_topic, bool enable){
    if(PointCloud2_!=NULL){
        delete PointCloud2_;
        PointCloud2_=NULL;
    }
    PointCloud2_=manager_->createDisplay("rviz/PointCloud2","myPointCloud2",enable);
    PointCloud2_->subProp("Topic")->setValue(pointcloud_topic);
    PointCloud2_->subProp("Color Transformer")->setValue("AxisColor");
    PointCloud2_->subProp("Size (m)")->setValue("0.03");
    ROS_ASSERT(PointCloud2_!=NULL);
}
void qrviz::Display_RobotModel(bool enable){
    if(RobotModel_!=NULL){
        delete RobotModel_;
        RobotModel_=NULL;
    }
    RobotModel_=manager_->createDisplay("rviz/RobotModel","myRobotModel",enable);
    ROS_ASSERT(RobotModel_!=NULL);
}
void qrviz::Display_Map(QString topic,QString color_scheme,bool enable){
    if(Map_!=NULL){
        delete Map_;
        Map_=NULL;
    }
    Map_=manager_->createDisplay("rviz/Map","myMap",enable);
    ROS_ASSERT(Map_!=NULL);
    Map_->subProp("Topic")->setValue(topic);
    Map_->subProp("Color Scheme")->setValue(color_scheme);
}
void qrviz::Display_Path(QString topic,QColor color,bool enable){
    if(Path_!=NULL){
        delete Path_;
        Path_=NULL;
    }
    Path_=manager_->createDisplay("rviz/Path","myPath",enable);
    ROS_ASSERT(Path_!=NULL);
    Path_->subProp("Topic")->setValue(topic);
    Path_->subProp("Color")->setValue(color);

}
void qrviz::Set_Start_Pose(){
    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetInitialPose");
    tool_manager_->setCurrentTool(current_tool_);
}
void qrviz::Set_Goal_Pose(){
    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetGoal");
    rviz::Property* pro=current_tool_->getPropertyContainer();
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    tool_manager_->setCurrentTool(current_tool_);
}
void qrviz::Display_Global_Map(QString costmap_topic,QString costmap_color,QString path_topic,QColor path_color,bool enable){
    if(Global_Map_!=NULL){
        delete Global_Map_;
        Global_Map_=NULL;
    }
    Global_Map_=manager_->createDisplay("rviz/Map","globalmap",enable);
    ROS_ASSERT(Global_Map_!=NULL);
    Global_Path_=manager_->createDisplay("rviz/Path","globalmap",enable);
    ROS_ASSERT(Global_Path_!=NULL);
    Global_Map_->subProp("Topic")->setValue(costmap_topic);
    Global_Map_->subProp("Color Scheme")->setValue(costmap_color);
    Global_Path_->subProp("Topic")->setValue(path_topic);
    Global_Path_->subProp("Color Scheme")->setValue(path_color);
}
void qrviz::Display_Local_Map(QString costmap_topic,QString costmap_color,QString path_topic,QColor path_color,bool enable){
    if(Local_Map_!=NULL){
        delete Local_Map_;
        Local_Map_=NULL;
    }
    Local_Map_=manager_->createDisplay("rviz/Map","localmap",enable);
    ROS_ASSERT(Local_Map_!=NULL);
    Local_Path_=manager_->createDisplay("rviz/Path","localmap",enable);
    ROS_ASSERT(Local_Path_!=NULL);
    Local_Map_->subProp("Topic")->setValue(costmap_topic);
    Local_Map_->subProp("Color Scheme")->setValue(costmap_color);
    Local_Path_->subProp("Topic")->setValue(path_topic);
    Local_Path_->subProp("Color")->setValue(path_color);
}

