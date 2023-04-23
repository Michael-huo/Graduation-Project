/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_gui_app/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui_app {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();

    setWindowIcon(QIcon("://images/AutoGo.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

//    on_button_connect_clicked(true);
    connections();
    uiQrviz();
    uiOther();
    terminalInit();
    rosbagInit();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void robot_gui_app::MainWindow::on_lineEdit_terminal_returnPressed()
{
    QString _str = ui.lineEdit_terminal->text();
    ui.textBrowser_terminal->append("Linux:~$ " + _str);
    m_process->write(ui.lineEdit_terminal->text().toLocal8Bit() + '\n');
    ui.lineEdit_terminal->clear();
}
void MainWindow::readBashStandardOutputInfo(){
    QByteArray _out = m_process->readAllStandardOutput();
    if(!_out.isEmpty())   ui.textBrowser_terminal->append(QString::fromLocal8Bit(_out));
}

void MainWindow::readBashStandardErrorInfo(){
    QByteArray _out = m_process->readAllStandardError();
    if(!_out.isEmpty())   ui.textBrowser_terminal->append(QString::fromLocal8Bit(_out));
}

void MainWindow::slot_linear_value_change(int value){
    ui.label_linear->setText(QString::number(value));
}
void MainWindow::slot_angular_value_change(int value){
    ui.label_angular->setText(QString::number(value));
}
void MainWindow::slot_pushbtn_click(){
    QPushButton* btn =qobject_cast<QPushButton*>(sender());
    qDebug()<<btn->text();
    char k = btn->text().toStdString()[0];
    bool is_all=ui.checkBox_is_ODW->isChecked();
    float linear = ui.label_linear->text().toFloat()*0.01;
    float angular = ui.label_angular->text().toFloat()*0.01;
    switch (k) {
        case 'u':
          qnode.set_cmd_vel(is_all ? 'U' : 'u', linear, angular);
          break;
        case 'i':
          qnode.set_cmd_vel(is_all ? 'I' : 'i', linear, angular);
          break;
        case 'o':
          qnode.set_cmd_vel(is_all ? 'O' : 'o', linear, angular);
          break;
        case 'j':
          qnode.set_cmd_vel(is_all ? 'J' : 'j', linear, angular);
          break;
        case 'l':
          qnode.set_cmd_vel(is_all ? 'L' : 'l', linear, angular);
          break;
        case 'm':
          qnode.set_cmd_vel(is_all ? 'M' : 'm', linear, angular);
          break;
        case ',':
          qnode.set_cmd_vel(is_all ? '<' : ',', linear, angular);
          break;
        case '.':
          qnode.set_cmd_vel(is_all ? '>' : '.', linear, angular);
          break;
    }
}
void MainWindow::slot_speed_x(double x) {
  speedDashBoard->set_speed(abs(x * 100));
  if (x > 0.001) {
    speedDashBoard->set_gear(DashBoard::kGear_D);
  } else if (x < -0.001) {
    speedDashBoard->set_gear(DashBoard::kGear_R);
  } else {
    speedDashBoard->set_gear(DashBoard::kGear_N);
  }

}
void MainWindow::slot_speed_yaw(double yaw) {
  if (yaw > 0.05) {
    ui.label_turnLeft->setPixmap(
        QPixmap::fromImage(QImage("://images/turnLeft_hl.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    ui.label_turnRight->setPixmap(
        QPixmap::fromImage(QImage("://images/turnRight_l.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
  } else if (yaw < -0.05) {
    ui.label_turnRight->setPixmap(
        QPixmap::fromImage(QImage("://images/turnRight_hl.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    ui.label_turnLeft->setPixmap(
        QPixmap::fromImage(QImage("://images/turnLeft_l.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
  } else {
    ui.label_turnLeft->setPixmap(
        QPixmap::fromImage(QImage("://images/turnLeft_l.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    ui.label_turnRight->setPixmap(
        QPixmap::fromImage(QImage("://images/turnRight_l.png").scaled(152, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
  }
  QString number = QString::number(abs(yaw),'f',3);
  ui.label_speed->setText(number);
}
void MainWindow::slot_batteryState(sensor_msgs::BatteryState msg) {
  ui.label_power_val->setText(QString::number(msg.voltage).mid(0, 5) + "V");
  double percentage = msg.percentage;
  speedDashBoard->set_oil(percentage);
  ui.progressBar_power->setValue(percentage > 100 ? 100 : percentage);
  if (percentage <= 20) {
    ui.progressBar_power->setStyleSheet(
        "QProgressBar::chunk {background-color: red;width: 20px;} QProgressBar "
        "{border: 2px solid grey;border-radius: 5px;text-align: center;}");
    // QMessageBox::warning(NULL, "电量不足", "电量不足，请及时充电！",
    // QMessageBox::Yes , QMessageBox::Yes);
  } else {
    ui.progressBar_power->setStyleSheet(
        "QProgressBar {border: 2px solid grey;border-radius: 5px;text-align: "
        "center;}");
  }
}
void MainWindow::slot_humidity(QString msg){
    ui.lcdNumber_humidity->setDigitCount(3);
    ui.lcdNumber_humidity->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_humidity->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_humidity->setPalette(lcdpat);
    ui.lcdNumber_humidity->display(msg.toInt());
}
void MainWindow::slot_temperature(QString msg){
    ui.lcdNumber_temperature->setDigitCount(3);
    ui.lcdNumber_temperature->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_temperature->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_temperature->setPalette(lcdpat);
    ui.lcdNumber_temperature->display(msg.toInt());
}
void MainWindow::slot_CO2(QString msg){
    ui.lcdNumber_CO2->setDigitCount(3);
    ui.lcdNumber_CO2->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_CO2->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_CO2->setPalette(lcdpat);
    ui.lcdNumber_CO2->display(msg.toInt());
}
void MainWindow::slot_illuminance(QString msg){
    ui.lcdNumber_illuminance->setDigitCount(3);
    ui.lcdNumber_illuminance->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_illuminance->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_illuminance->setPalette(lcdpat);
    ui.lcdNumber_illuminance->display(msg.toInt());
}
void MainWindow::slot_rainfall(QString msg){
    ui.lcdNumber_rainfall->setDigitCount(3);
    ui.lcdNumber_rainfall->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_rainfall->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_rainfall->setPalette(lcdpat);
    ui.lcdNumber_rainfall->display(msg.toInt());
}
void MainWindow::slot_anemograph(QString msg){
    ui.lcdNumber_anemograph->setDigitCount(3);
    ui.lcdNumber_anemograph->setSegmentStyle(QLCDNumber::Flat);
    QPalette lcdpat = ui.lcdNumber_anemograph->palette();
    lcdpat.setColor(QPalette::Normal,QPalette::WindowText,Qt::black);
    ui.lcdNumber_anemograph->setPalette(lcdpat);
    ui.lcdNumber_anemograph->display(msg.toInt());
}

void MainWindow::slot_update_image(QImage im){
    ui.label_image->setPixmap(QPixmap::fromImage(im));
}

void MainWindow::slot_sub_image(){
    qnode.sub_image(ui.lineEdit_image_topic->text());
}

void MainWindow::slot_treewidget_value_change(QString){
    myrviz->Set_FixedFrame(fixed_box->currentText());
}
void MainWindow::slot_display_grid(int state){
    bool enable=state>1?true:false;
    QStringList qli=Grid_Color_Box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Grid(Cell_Count_Box->text().toInt(),color,enable);
}
void MainWindow::slot_display_tf(int state){
    bool enable=state>1?true:false;
    myrviz->Display_TF(enable);
}
void MainWindow::slot_display_laser(int state){
    bool enable=state>1?true:false;
    myrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}
void MainWindow::slot_display_pointcloud2(int state){
    bool enable=state>1?true:false;
    myrviz->Display_PointCloud2(PointCloud2_Topic_box->currentText(),enable);
}
void MainWindow::slot_display_RobotModel(int state){
    bool enable=state>1?true:false;
    myrviz->Display_RobotModel(enable);
}
void MainWindow::slot_display_Map(int state){
    bool enable=state>1?true:false;
    myrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);
}
void MainWindow::slot_display_Path(int state){
    bool enable=state>1?true:false;
    QStringList qli=Path_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Path(Path_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_set_start_pose(){
    myrviz->Set_Start_Pose();
}
void MainWindow::slot_set_goal_pose(){
    myrviz->Set_Goal_Pose();
}
void MainWindow::slot_display_global_map(int state){
    bool enable=state>1?true:false;
    QStringList qli=Global_Planner_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Global_Map(Global_CostMap_Topic_box->currentText(),Global_Map_Color_Scheme_box->currentText(),Global_Planner_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_display_local_map(int state){
    bool enable=state>1?true:false;
    QStringList qli=Local_Planner_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Local_Map(Local_CostMap_Topic_box->currentText(),Local_Map_Color_Scheme_box->currentText(),Local_Planner_Topic_box->currentText(),color,enable);

}
void MainWindow::slot_update_pose(double x,double y,double z){
    ui.pos_x->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}
void MainWindow::slot_set_return_pos(){
    ui.return_x->setText(ui.pos_x->text());
    ui.return_y->setText(ui.pos_y->text());
    ui.return_z->setText(ui.pos_z->text());

}
void MainWindow::slot_return(){
    qnode.set_goal(ui.return_x->text().toDouble(),ui.return_y->text().toDouble(),ui.return_z->text().toDouble());
}

void MainWindow::initTopicList() {
    ui.topic_listWidget->clear();
    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name", "Type"));
    QMap<QString, QString> topic_list = qnode.get_topic_list();
    for (QMap<QString, QString>::iterator iter = topic_list.begin();
       iter != topic_list.end(); iter++) {
    ui.topic_listWidget->addItem(
        QString("%1   (%2)").arg(iter.key(), iter.value()));
  }
}
void MainWindow::refreashTopicList() {
    initTopicList();
}
void MainWindow::readoutput()
{
    ui.textEdit_bagoutput->append(bag_process->readAllStandardOutput().data());   //将输出信息读取到编辑框
}
void MainWindow::readerror()
{
    QMessageBox::information(0, "Error", bag_process->readAllStandardError().data());   //弹出信息框提示错误信息
}
void MainWindow::rosbagInit(){
    ui.lineEdit_username->setText("tianbot");//获取本机名
    bag_process = new QProcess(this);
    connect(bag_process , &QProcess::readyReadStandardOutput, this , &MainWindow::readoutput);
    connect(bag_process , &QProcess::readyReadStandardError, this , &MainWindow::readerror);
}
void MainWindow::terminalInit(){
    m_process = new QProcess;
    m_process->start("bash");
    m_process->waitForStarted();
    connect(m_process, SIGNAL(readyReadStandardOutput()), this, SLOT(readBashStandardOutputInfo()));
    connect(m_process, SIGNAL(readyReadStandardError()), this, SLOT(readBashStandardErrorInfo()));
}

//记录ros包
void MainWindow::slot_pushButton_recordbag()
{
    QString strget=ui.lineEdit_username->text();
    QString pathget=ui.lineEdit_bagpath->text();
    QString nameget=ui.lineEdit_bagname->text();
    QDate D;
    D=QDate::currentDate();
    QTime T;
    T=QTime::currentTime();
    QString bagname =QString("%1%2-%3-%4-%5:%6:%7.bag").arg(nameget).arg(D.year()).arg(D.month()).arg(D.day()).arg(T.hour()).arg(T.minute()).arg(T.second());//指定文件路径
    //qDebug() << bagname;
    ui.textEdit_bagoutput->append(QString("%1 $: 记录的包将被命名为：%2").arg(strget).arg(bagname));
    QString str4=QString("gnome-terminal -- bash -c '/home/%1/rosbash/bagrecord.sh %2%3'&").arg(strget).arg(strget).arg(pathget).arg(bagname);
    ui.textEdit_bagoutput->append(QString("%1 $: /home/%2/rosbash/bagrecord.sh %3").arg(strget).arg(strget).arg(bagname));
    char *n;
    QByteArray m=str4.toLatin1();
    n=m.data();
    system(n);
}
//停止记录
void MainWindow::slot_pushButton_stop_record(){
    QString str_cmd=QString("gnome-terminal -- bash -c 'rosnode kill '");
    char *n;
    QByteArray m=str_cmd.toLatin1();
    n=m.data();
    system(n);
}
//运行该bag文件
void MainWindow::slot_pushButton_playbag()
{
    QString strget=ui.lineEdit_username->text();
    QString pathget=ui.lineEdit_bagpath->text();
    QString getbag=ui.comboBox_bag->currentText();
    QString str=QString("/home/%1/rosbash/bagplay.sh %2%3 %4 \n").arg(strget).arg(strget).arg(pathget).arg(getbag);
    char *n;
    QByteArray m=str.toLatin1();
    ui.textEdit_bagoutput->append(QString("%1 $: %2").arg(strget).arg(str));
    n=m.data();
    bag_process->start("bash");      //启动终端(Windows下改为cmd)
    bag_process->waitForStarted();   //等待启动完成
    bag_process->write(n);           //向终端写入命令，注意尾部的“\n”不可省略
}
//删除该bag文件
void MainWindow::slot_pushButton_deletebag()
{
    QString strget=ui.lineEdit_username->text();
    QString pathget=ui.lineEdit_bagpath->text();
    QString getbag=ui.comboBox_bag->currentText();
    QString path =QString("/home/%1%2/%3").arg(strget).arg(pathget).arg(getbag);//指定文件路径
    QFile fileTemp(path);
    fileTemp.remove();
    ui.textEdit_bagoutput->append(QString("%1 $: 成功删除%2文件").arg(strget).arg(path));
    slot_pushButton_baglist();
}
//添加bag列表
void MainWindow::slot_pushButton_baglist()
{
    ui.comboBox_bag->clear();
    QString strget=ui.lineEdit_username->text();
    QString pathget=ui.lineEdit_bagpath->text();
    QString Path=QString("/home/%1%2").arg(strget).arg(pathget);//指定文件路径
    QDir dir(Path);
    //QDir dir(QDir::currentPath());//当前文件路径
    QString filtername = "*.bag";
    QStringList filter;
    filter << filtername;
    dir.setNameFilters(filter);
    QStringList Neuronindex = dir.entryList();
    ui.comboBox_bag->addItems(Neuronindex);//把列表加载到comboBox
    QString s = QString::number(Neuronindex.size());//txt文件个数
    ui.textEdit_bagoutput->append(QString("%1 $: 共有%2个bag文件").arg(strget).arg(s));
}

void MainWindow::uiQrviz(){
//    header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
//    Global Option ui
    QTreeWidgetItem* Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));
    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
//    FixFrame ui
    QTreeWidgetItem* Fixed_frame=new QTreeWidgetItem(QStringList()<<"Fixed_Frame");
    fixed_box=new QComboBox;
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
    Global->addChild(Fixed_frame);
    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);
//    Grid ui
    QTreeWidgetItem* Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/grid.png"));
    QCheckBox* Grid_Check=new QCheckBox();
    connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
    ui.treeWidget->addTopLevelItem(Grid);
    ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
    Grid->setExpanded(true);
    QTreeWidgetItem* Cell_Count=new QTreeWidgetItem(QStringList()<<"Plane Cell Count");
    Grid->addChild(Cell_Count);
    Cell_Count_Box=new QSpinBox;
    Cell_Count_Box->setValue(20);
    Cell_Count_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Cell_Count,1,Cell_Count_Box);
    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_Color);
    Grid_Color_Box=new QComboBox();
    Grid_Color_Box->addItem("160;160;160");
    Grid_Color_Box->setEditable(true);
    Grid_Color_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Grid_Color,1,Grid_Color_Box);
//    TF ui
    QTreeWidgetItem* TF=new QTreeWidgetItem(QStringList()<<"TF");
    TF->setIcon(0,QIcon("://images/classes/TF.png"));
    QCheckBox* TF_Check=new QCheckBox();
    connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
    ui.treeWidget->addTopLevelItem(TF);
    ui.treeWidget->setItemWidget(TF,1,TF_Check);
//    Laser ui
    QTreeWidgetItem* LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
    LaserScan->setIcon(0,QIcon("://images/classes/LaserScan.png"));
    QCheckBox* Laser_Check=new QCheckBox();
    connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
    ui.treeWidget->addTopLevelItem(LaserScan);
    ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
    LaserScan->setExpanded(true);
    QTreeWidgetItem* LaserTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Laser_Topic_box=new QComboBox;
    Laser_Topic_box->addItem("/scan");
    Laser_Topic_box->setEditable(true);
    Laser_Topic_box->setMaximumWidth(150);
    LaserScan->addChild(LaserTopic);
    ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);
//    PC2 ui
    QTreeWidgetItem* PointCloud2=new QTreeWidgetItem(QStringList()<<"PointCloud2");
    PointCloud2->setIcon(0,QIcon("://images/classes/PointCloud2.png"));
    QCheckBox* PointCloud2_Check=new QCheckBox();
    connect(PointCloud2_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_pointcloud2(int)));
    ui.treeWidget->addTopLevelItem(PointCloud2);
    ui.treeWidget->setItemWidget(PointCloud2,1,PointCloud2_Check);
    PointCloud2->setExpanded(true);
    QTreeWidgetItem* PointCloud2Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    PointCloud2_Topic_box=new QComboBox;
    PointCloud2_Topic_box->addItem("/pointcloud2");
    PointCloud2_Topic_box->setEditable(true);
    PointCloud2_Topic_box->setMaximumWidth(150);
    PointCloud2->addChild(PointCloud2Topic);
    ui.treeWidget->setItemWidget(PointCloud2Topic,1,PointCloud2_Topic_box);
//    RobotModel ui
    QTreeWidgetItem* RobotModel=new QTreeWidgetItem(QStringList()<<"RobotModel");
    TF->setIcon(0,QIcon("://images/classes/RobotModel.png"));
    QCheckBox* RobotModel_Check=new QCheckBox;
    connect(RobotModel_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_RobotModel(int)));
    ui.treeWidget->addTopLevelItem(RobotModel);
    ui.treeWidget->setItemWidget(RobotModel,1,RobotModel_Check);
//    Map ui
    QTreeWidgetItem* Map=new QTreeWidgetItem(QStringList()<<"Map");
    Map->setIcon(0,QIcon("://images/classes/Map.png"));
    QCheckBox* Map_Check=new QCheckBox();
    connect(Map_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Map(int)));
    ui.treeWidget->addTopLevelItem(Map);
    ui.treeWidget->setItemWidget(Map,1,Map_Check);
    Map->setExpanded(true);
    QTreeWidgetItem* MapTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Map_Topic_box=new QComboBox();
    Map_Topic_box->addItem("/map");
    Map_Topic_box->setEditable(true);
    Map_Topic_box->setMaximumWidth(150);
    Map->addChild(MapTopic);
    ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
//   Map color scheme
    QTreeWidgetItem* MapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map_Color_Scheme_box=new QComboBox();
    Map_Color_Scheme_box->addItem("/map");
    Map_Color_Scheme_box->addItem("/costmap");
    Map_Color_Scheme_box->addItem("/raw");
    Map_Color_Scheme_box->setMaximumWidth(150);
    Map->addChild(MapColorScheme);
    ui.treeWidget->setItemWidget(MapColorScheme,1,Map_Color_Scheme_box);
//    Path ui
    QTreeWidgetItem* Path=new QTreeWidgetItem(QStringList()<<"Path");
    Path->setIcon(0,QIcon("://images/classes/Path.png"));
    QCheckBox* Path_Check=new QCheckBox();
    connect(Path_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Path(int)));
    ui.treeWidget->addTopLevelItem(Path);
    ui.treeWidget->setItemWidget(Path,1,Path_Check);
    QTreeWidgetItem* PathTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Path_Topic_box=new QComboBox();
    Path_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Path_Topic_box->setEditable(true);
    Path_Topic_box->setMaximumWidth(150);
    Map->addChild(PathTopic);
    ui.treeWidget->setItemWidget(PathTopic,1,Path_Topic_box);
//    Path color scheme
    QTreeWidgetItem* PathColorScheme=new QTreeWidgetItem(QStringList()<<"Color");
    Path_Color_box=new QComboBox();
    Path_Color_box->addItem("0;12;255");
    Path_Color_box->setEditable(true);
    Path_Color_box->setMaximumWidth(150);
    Map->addChild(PathColorScheme);
    ui.treeWidget->setItemWidget(PathColorScheme,1,Path_Color_box);
//    Golobal Map
    QTreeWidgetItem* GlobalMap=new QTreeWidgetItem(QStringList()<<"Global Map");
    GlobalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* GlobalMap_Check=new QCheckBox();
    connect(GlobalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_global_map(int)));
    ui.treeWidget->addTopLevelItem(GlobalMap);
    ui.treeWidget->setItemWidget(GlobalMap,1,GlobalMap_Check);
    GlobalMap->setExpanded(true);
//    Golobal CostMap
    QTreeWidgetItem* Global_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    Global_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    GlobalMap->addChild(Global_CostMap);
    Global_CostMap->setExpanded(true);
//    GolobalMap Topic
    QTreeWidgetItem* Global_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_CostMap_Topic_box=new QComboBox();
    Global_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
    Global_CostMap_Topic_box->setEditable(true);
    Global_CostMap_Topic_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_CostMap_Topic);
    ui.treeWidget->setItemWidget(Global_CostMap_Topic,1,Global_CostMap_Topic_box);
//    GolobalMap Color Scheme
    QTreeWidgetItem* Global_Map_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Global_Map_Color_Scheme_box=new QComboBox();
    Global_Map_Color_Scheme_box->addItem("costmap");
    Global_Map_Color_Scheme_box->addItem("map");
    Global_Map_Color_Scheme_box->addItem("raw");
    Global_Map_Color_Scheme_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_Map_Color_Scheme);
    ui.treeWidget->setItemWidget(Global_Map_Color_Scheme,1,Global_Map_Color_Scheme_box);
//    Golobal Planner
    QTreeWidgetItem* Global_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    Global_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    GlobalMap->addChild(Global_Planner);
    Global_Planner->setExpanded(true);
//    GolobalPath Topic
    QTreeWidgetItem* Global_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_Planner_Topic_box=new QComboBox();
    Global_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/global_plan");
    Global_Planner_Topic_box->setEditable(true);
    Global_Planner_Topic_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Topic);
    ui.treeWidget->setItemWidget(Global_Planner_Topic,1,Global_Planner_Topic_box);
//    GolobalPath Color Scheme
    QTreeWidgetItem* Global_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Global_Planner_Color_box=new QComboBox();
    Global_Planner_Color_box->addItem("255;0;0");
    Global_Planner_Color_box->setEditable(true);
    Global_Planner_Color_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Global_Planner_Color_Scheme,1,Global_Planner_Color_box);
//    Local Map
    QTreeWidgetItem* LocalMap=new QTreeWidgetItem(QStringList()<<"Local Map");
    LocalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* LocalMap_Check=new QCheckBox();
    connect(LocalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_local_map(int)));
    ui.treeWidget->addTopLevelItem(LocalMap);
    ui.treeWidget->setItemWidget(LocalMap,1,LocalMap_Check);
    LocalMap->setExpanded(true);
//    Local CostMap
    QTreeWidgetItem* Local_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    Local_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    LocalMap->addChild(Local_CostMap);
    Local_CostMap->setExpanded(true);
//    LocalMap Topic
    QTreeWidgetItem* Local_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_CostMap_Topic_box=new QComboBox();
    Local_CostMap_Topic_box->addItem("/move_base/local_costmap/costmap");
    Local_CostMap_Topic_box->setEditable(true);
    Local_CostMap_Topic_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_CostMap_Topic);
    ui.treeWidget->setItemWidget(Local_CostMap_Topic,1,Local_CostMap_Topic_box);
//    LocalMap Color Scheme
    QTreeWidgetItem* Local_Map_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Local_Map_Color_Scheme_box=new QComboBox();
    Local_Map_Color_Scheme_box->addItem("costmap");
    Local_Map_Color_Scheme_box->addItem("map");
    Local_Map_Color_Scheme_box->addItem("raw");
    Local_Map_Color_Scheme_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_Map_Color_Scheme);
    ui.treeWidget->setItemWidget(Local_Map_Color_Scheme,1,Local_Map_Color_Scheme_box);
//    Local Planner
    QTreeWidgetItem* Local_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    Local_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    LocalMap->addChild(Local_Planner);
    Local_Planner->setExpanded(true);
//    LocalPath Topic
    QTreeWidgetItem* Local_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_Planner_Topic_box=new QComboBox();
    Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Local_Planner_Topic_box->setEditable(true);
    Local_Planner_Topic_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Topic);
    ui.treeWidget->setItemWidget(Local_Planner_Topic,1,Local_Planner_Topic_box);
//    LocalPath Color Scheme
    QTreeWidgetItem* Local_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Local_Planner_Color_box=new QComboBox();
    Local_Planner_Color_box->addItem("0;12;255");
    Local_Planner_Color_box->setEditable(true);
    Local_Planner_Color_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Local_Planner_Color_Scheme,1,Local_Planner_Color_box);
}
void MainWindow::uiOther(){
        speedDashBoard = new DashBoard(ui.widget_dashboard);
        ui.horizontalSlider_linear->setValue(20);
        ui.horizontalSlider_angular->setValue(20);
        ui.progressBar_power->setValue(0);
}
void MainWindow::connections(){
//    Control connect
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_change(int)));
    connect(ui.horizontalSlider_angular,SIGNAL(valueChanged(int)),this,SLOT(slot_angular_value_change(int)));
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_co,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_st,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
//    rosbag connect
    connect(ui.pushButton_recordbag,SIGNAL(clicked()),this,SLOT(slot_pushButton_recordbag()));
    connect(ui.pushButton_stoprecord,SIGNAL(clicked()),this,SLOT(slot_pushButton_stop_record()));
    connect(ui.pushButton_playbag,SIGNAL(clicked()),this,SLOT(slot_pushButton_playbag()));
    connect(ui.pushButton_deletebag,SIGNAL(clicked()),this,SLOT(slot_pushButton_deletebag()));
    connect(ui.pushButton_baglist,SIGNAL(clicked()),this,SLOT(slot_pushButton_baglist()));
//    Other connect
    connect(&qnode, SIGNAL(batteryState(sensor_msgs::BatteryState)), this,SLOT(slot_batteryState(sensor_msgs::BatteryState)));
    connect(&qnode, SIGNAL(env_humidity(QString)), this, SLOT(slot_humidity(QString)));
    connect(&qnode, SIGNAL(env_temperature(QString)), this, SLOT(slot_temperature(QString)));
    connect(&qnode, SIGNAL(env_CO2(QString)), this, SLOT(slot_CO2(QString)));
    connect(&qnode, SIGNAL(env_illuminance(QString)), this, SLOT(slot_illuminance(QString)));
    connect(&qnode, SIGNAL(env_rainfall(QString)), this, SLOT(slot_rainfall(QString)));
    connect(&qnode, SIGNAL(env_anemograph(QString)), this, SLOT(slot_anemograph(QString)));
    connect(&qnode, SIGNAL(speed_x(double)), this, SLOT(slot_speed_x(double)));
    connect(&qnode, SIGNAL(speed_y(double)), this, SLOT(slot_speed_yaw(double)));
    connect(&qnode,SIGNAL(image_val(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(&qnode,SIGNAL(position(double,double,double)),this,SLOT(slot_update_pose(double,double,double)));
    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));
    connect(ui.set_start_btn,SIGNAL(clicked()),this,SLOT(slot_set_start_pose()));
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_goal_pose()));
    connect(ui.set_return_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_return_pos()));
    connect(ui.return_btn,SIGNAL(clicked()),this,SLOT(slot_return()));
    connect(ui.refreash_topic_btn, SIGNAL(clicked()),this,SLOT(refreashTopicList()));
}

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
            ui.treeWidget->setEnabled(false);
		} else {
			ui.button_connect->setEnabled(false);
            ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.layout_rviz);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.layout_rviz);

		}
	}
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Robot GUI APP v1.0</h2><p>Copyright Michael Huo</p><p>Ubuntu18.04--ROS Melodic--Qt5.9.9</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_gui_app");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_gui_app");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
//    if(bag_process)
//    {
//          bag_process->close();
//          bag_process->waitForFinished();
//    }
    if(m_process)
    {
          m_process->close();
          m_process->waitForFinished();
    }
	QMainWindow::closeEvent(event);
}

}  // namespace robot_gui_app



