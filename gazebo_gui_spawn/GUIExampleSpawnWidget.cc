/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIExampleSpawnWidget.hh"


using namespace gazebo;


MyDialog::MyDialog(QWidget* parent) : QDialog(parent) {
  // 设置对话框标题
  this->setWindowTitle("优化参数修改");

  // 设置对话框窗口大小
  this->resize(320, 400);

  // 创建一个垂直布局
  QVBoxLayout* layout = new QVBoxLayout;

  // 添加标题标签
  QLabel* titleLabel = new QLabel("设定修改");
  titleLabel->setAlignment(Qt::AlignHCenter);
  layout->addWidget(titleLabel);

  // 添加组件选择
  QHBoxLayout* checkBoxLayout1 = new QHBoxLayout;
  QCheckBox* checkBox1 = new QCheckBox("覆盖范围");
  QCheckBox* checkBox2 = new QCheckBox("冗余度");
  // 设置文字颜色为白色
  checkBox1->setStyleSheet("QCheckBox { color: white; }");
  checkBox2->setStyleSheet("QCheckBox { color: white; }");

  // 设置文字字体为粗体
  checkBox1->setFont(QFont("Arial", 10, QFont::Bold));
  checkBox2->setFont(QFont("Arial", 10, QFont::Bold));
  checkBoxLayout1->addWidget(checkBox1);
  checkBoxLayout1->addWidget(checkBox2);
  layout->addLayout(checkBoxLayout1);

  // 添加输入框和说明文字
  QLineEdit* lineEdit1 = new QLineEdit;
  QLabel* label1 = new QLabel("优化轮数:");
  lineEdit1->setText("30");
  layout->addWidget(label1);
  layout->addWidget(lineEdit1);

  // 添加输入框和说明文字
  QLineEdit* lineEdit2 = new QLineEdit;
  QLabel* label2 = new QLabel("解集数量:");
  lineEdit2->setText("30");
  layout->addWidget(label2);
  layout->addWidget(lineEdit2);

  QLabel* titlelabel2 = new QLabel("遗传算法参数");
  titlelabel2->setAlignment(Qt::AlignHCenter);
  layout->addWidget(titlelabel2);

  QLineEdit* lineEdit3 = new QLineEdit;
  QLabel* label3 = new QLabel("交叉概率:");
  lineEdit3->setText("0.7");
  layout->addWidget(label3);
  layout->addWidget(lineEdit3);

  QLineEdit* lineEdit4 = new QLineEdit;
  QLabel* label4 = new QLabel("变异概率:");
  lineEdit4->setText("0.01");
  layout->addWidget(label4);
  layout->addWidget(lineEdit4);

  // 创建一个按钮
  QPushButton* button = new QPushButton("保存参数");
  layout->addWidget(button);

  // 连接按钮点击事件到槽函数
  connect(button, &QPushButton::clicked, this, &MyDialog::onButtonClicked);

  // 设置布局到对话框
  this->setLayout(layout);


  // Some init for elems
  this->opt_turns = 30;
  this->n_solu = 30;
  this->pro_cross = 0.7;
  this->pro_mut = 0.01;
  this->cover = true;
  this->redun = true;
  this->new_save = false;
}


void MyDialog::onButtonClicked() {
      if (dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(0)->widget())->isChecked()==false
       && dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(1)->widget())->isChecked()==false){

        std::cout<<"Please choose at least one metric for optimization!"<<std::endl;
        return ;

      }else if(dynamic_cast<QLineEdit*>(this->layout()->itemAt(8)->widget())->text().toDouble()>1 ||
      dynamic_cast<QLineEdit*>(this->layout()->itemAt(10)->widget())->text().toDouble()>1){

        std::cout<<"Probability should be below 1!"<<std::endl;
        return ;
      }

      
      // Save the params
      this->opt_turns = dynamic_cast<QLineEdit*>(this->layout()->itemAt(3)->widget())->text().toInt();
      this->n_solu = dynamic_cast<QLineEdit*>(this->layout()->itemAt(5)->widget())->text().toInt();
      this->pro_cross = dynamic_cast<QLineEdit*>(this->layout()->itemAt(8)->widget())->text().toDouble();
      this->pro_mut = dynamic_cast<QLineEdit*>(this->layout()->itemAt(10)->widget())->text().toDouble();

      // 获取覆盖范围框的勾选状态
      this->cover = dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(0)->widget())->isChecked();
      // 获取冗余度框的勾选状态
      this->redun = dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(1)->widget())->isChecked();
      
      // Output saved params
      std::cout << "覆盖范围框是否被勾选: " << (this->cover ? "是" : "否") << std::endl;
      std::cout << "冗余度框是否被勾选: " << (this->redun ? "是" : "否") << std::endl;
      std::cout << "优化轮数: " << this->opt_turns << std::endl;
      std::cout << "解集数量: " << this->n_solu << std::endl;
      std::cout << "交叉概率: " << this->pro_cross << std::endl;
      std::cout << "变异概率: " << this->pro_mut << std::endl;
      
      this->new_save = true;
      // 关闭对话框
      this->accept();
      
}

bool MyDialog::if_cover(){
  return this->cover;
}

bool MyDialog::if_redun(){
  return this->redun;
}

int MyDialog::NumSolutions(){
  return this->n_solu;
}

int MyDialog::NumOptTurns(){
  return this->opt_turns;
}







// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIExampleSpawnWidget)

/////////////////////////////////////////////////
GUIExampleSpawnWidget::GUIExampleSpawnWidget()
  : GUIPlugin()
{
  this->counter = 0;
  this->opt_turns = 30;
  this->n_solu = 30;
  this->cover = true;
  this->redun = true;
  this->ifsub = true;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button1 = new QPushButton(tr("传感器信息"));
  QPushButton *button2 = new QPushButton(tr("优化参数修改"));
  QPushButton *button3 = new QPushButton(tr("优化传感器配置"));
  QPushButton *button4 = new QPushButton(tr("标定板设置"));
  connect(button1, SIGNAL(clicked()), this, SLOT(OnButton1()));
  connect(button2, SIGNAL(clicked()), this, SLOT(OnButton2()));
  connect(button3, SIGNAL(clicked()), this, SLOT(OnButton3()));
  connect(button4, SIGNAL(clicked()), this, SLOT(OnButton4()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button1);
  frameLayout->addWidget(button2);
  frameLayout->addWidget(button3);
  frameLayout->addWidget(button4);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(140, 160);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->factoryPub = this->node->Advertise<msgs::Param_V>("~/mytopic");
  this->statsSub = this->node->Subscribe("~/world_stats",
      &GUIExampleSpawnWidget::OnStats, this);
  this->statsSub2 = this->node->Subscribe("~/mytopic",
      &GUIExampleSpawnWidget::Subtopic, this);

}

/////////////////////////////////////////////////
GUIExampleSpawnWidget::~GUIExampleSpawnWidget()
{
}

/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton1()
{
  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr factoryPub =
  node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;

  msg.set_sdf_filename("model://PLUGIN_print_sensor_info");

  // Pose to initialize the model to
  msgs::Set(msg.mutable_pose(),
      ignition::math::Pose3d(
        ignition::math::Vector3d(100, 100, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  // Send the message
  factoryPub->Publish(msg);
}

/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton2()
{
  MyDialog dialog;
  dialog.exec();
  if (dialog.new_save){
    dialog.new_save = false;
    this->ifsub = true;
    this->opt_turns = dialog.NumOptTurns();
    this->n_solu = dialog.NumSolutions();
    this->cover = dialog.if_cover();
    this->redun = dialog.if_redun();
  }
}

/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton3()
{
  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr factoryPub =
  node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;

  msg.set_sdf_filename("model://a");

  // Pose to initialize the model to
  msgs::Set(msg.mutable_pose(),
      ignition::math::Pose3d(
        ignition::math::Vector3d(100, 100, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  // Send the message
  factoryPub->Publish(msg);
}

void GUIExampleSpawnWidget::OnButton4()
{
  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr factoryPub =
  node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;

  msg.set_sdf_filename("model://PLUGIN_board_placement");

  // Pose to initialize the model to
  msgs::Set(msg.mutable_pose(),
      ignition::math::Pose3d(
        ignition::math::Vector3d(100, 100, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  // Send the message
  factoryPub->Publish(msg);
}

void GUIExampleSpawnWidget::Subtopic(ConstParam_VPtr &_msg){
  if (this->ifsub){
    this->ifsub = false;
    for (int i=0; i<_msg->param_size();i++){
      const ::gazebo::msgs::Param& p = _msg->param(i);
      std::cout<<p.name()<<std::endl;
      const ::gazebo::msgs::Any& a = p.value();
      if (a.has_int_value()){
        std::cout<<a.int_value()<<std::endl;
      }else if (a.has_bool_value()){
        std::cout<<a.bool_value()<<std::endl;
      }
    }
  }
}

void GUIExampleSpawnWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  // 创建一个 Gazebo 消息
  gazebo::msgs::Param_V msg;

  
  gazebo::msgs::Param *cover = msg.add_param();
  cover->set_name("cover");
  msgs::Any *v_c = new msgs::Any(msgs::ConvertAny(this->cover));
  cover->set_allocated_value(v_c);

  gazebo::msgs::Param *redun = msg.add_param();
  redun->set_name("redun");
  msgs::Any *v_r = new msgs::Any(msgs::ConvertAny(this->redun));
  redun->set_allocated_value(v_r);

  gazebo::msgs::Param *NOT = msg.add_param();
  NOT->set_name("num_turns");
  msgs::Any *v_t = new msgs::Any(msgs::ConvertAny(this->opt_turns));
  NOT->set_allocated_value(v_t);

  gazebo::msgs::Param *NS = msg.add_param();
  NS->set_name("num_solutions");
  msgs::Any *v_s = new msgs::Any(msgs::ConvertAny(this->n_solu));
  NS->set_allocated_value(v_s);

  this->factoryPub->Publish(msg);
}
