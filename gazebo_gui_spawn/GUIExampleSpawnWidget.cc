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
  
  this->setWindowTitle("优化参数修改");

  
  this->resize(320, 400);

  // Some init for elems
  this->opt_turns = 30;
  this->n_solu = 30;
  this->pro_cross = 0.7;
  this->pro_mut = 0.01;
  this->adap_factor = 0.01;
  this->cover = true;
  this->redun = true;
  this->new_save = false;
  this->init = false;

  
  QVBoxLayout* layout = new QVBoxLayout;

  
  QLabel* titleLabel = new QLabel("设定修改");
  titleLabel->setAlignment(Qt::AlignHCenter);
  layout->addWidget(titleLabel);

  
  QHBoxLayout* checkBoxLayout1 = new QHBoxLayout;
  QCheckBox* checkBox1 = new QCheckBox("覆盖范围");
  QCheckBox* checkBox2 = new QCheckBox("冗余度");
  
  checkBox1->setStyleSheet("QCheckBox { color: white; }");
  checkBox2->setStyleSheet("QCheckBox { color: white; }");

  
  checkBox1->setFont(QFont("Arial", 10, QFont::Bold));
  checkBox2->setFont(QFont("Arial", 10, QFont::Bold));
  checkBox1->setChecked(true);
  checkBoxLayout1->addWidget(checkBox1);
  checkBoxLayout1->addWidget(checkBox2);
  layout->addLayout(checkBoxLayout1);

  QIntValidator *validator = new QIntValidator(1, 500, this);


  QLineEdit* lineEdit1 = new QLineEdit;
  lineEdit1->setValidator(validator);

  QLabel* label1 = new QLabel("优化轮数:");
  lineEdit1->setText("50");
  layout->addWidget(label1);
  layout->addWidget(lineEdit1);

  
  QLineEdit* lineEdit2 = new QLineEdit;
  lineEdit2->setValidator(validator);
  QLabel* label2 = new QLabel("解集数量:");
  lineEdit2->setText("70");
  layout->addWidget(label2);
  layout->addWidget(lineEdit2);

  QLabel* titlelabel2 = new QLabel("遗传算法参数");
  titlelabel2->setAlignment(Qt::AlignHCenter);
  layout->addWidget(titlelabel2);

  QHBoxLayout *sliderlayout1 = new QHBoxLayout();
  QSlider *slider1 = new QSlider(Qt::Horizontal);
  slider1->setRange(0, 100); 
  slider1->setValue(70); 
  QLabel* label3 = new QLabel("交叉概率:");
  QLabel *valueLabel1 = new QLabel("0.70");
  layout->addLayout(sliderlayout1);
  sliderlayout1->addWidget(label3);
  QSpacerItem *spacer1 = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
  sliderlayout1->addSpacerItem(spacer1);
  sliderlayout1->addWidget(valueLabel1);
  layout->addWidget(slider1);

  QObject::connect(slider1, &QSlider::valueChanged, [=](int value) {
      double doubleValue = value / 100.0;
      valueLabel1->setText(QString::number(doubleValue, 'f', 2));
  });

  QHBoxLayout *sliderlayout2 = new QHBoxLayout();
  QSlider *slider2 = new QSlider(Qt::Horizontal);
  slider2->setRange(0, 100); 
  slider2->setValue(1); 
  QLabel* label4 = new QLabel("变异概率:");
  QLabel *valueLabel2 = new QLabel("0.01");
  layout->addLayout(sliderlayout2);
  sliderlayout2->addWidget(label4);
  QSpacerItem *spacer2 = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
  sliderlayout2->addSpacerItem(spacer2);
  sliderlayout2->addWidget(valueLabel2);
  layout->addWidget(slider2);

  QObject::connect(slider2, &QSlider::valueChanged, [=](int value) {
      double doubleValue = value / 100.0;
      valueLabel2->setText(QString::number(doubleValue, 'f', 2));
  });

  QHBoxLayout *sliderlayout3 = new QHBoxLayout();
  QSlider *slider3 = new QSlider(Qt::Horizontal);
  slider3->setRange(0, 100); 
  slider3->setValue(1); 
  QLabel* label5 = new QLabel("自适应因子:");
  QLabel *valueLabel3 = new QLabel("0.01");
  layout->addLayout(sliderlayout3);
  sliderlayout3->addWidget(label5);
  QSpacerItem *spacer3 = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
  sliderlayout3->addSpacerItem(spacer3);
  sliderlayout3->addWidget(valueLabel3);
  layout->addWidget(slider3);

  QObject::connect(slider3, &QSlider::valueChanged, [=](int value) {
      double doubleValue = value / 100.0;
      valueLabel3->setText(QString::number(doubleValue, 'f', 2));
  });

  QHBoxLayout *sliderlayout4 = new QHBoxLayout();
  QSlider *slider4 = new QSlider(Qt::Horizontal);
  slider4->setRange(0, 200); 
  slider4->setValue(100); 
  QLabel* label6 = new QLabel("初始化偏转角间隔最小值:");
  QLabel *valueLabel4 = new QLabel("1.0*PI");
  layout->addLayout(sliderlayout4);
  sliderlayout4->addWidget(label6);
  QSpacerItem *spacer4 = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
  sliderlayout4->addSpacerItem(spacer4);
  sliderlayout4->addWidget(valueLabel4);
  layout->addWidget(slider4);

  QObject::connect(slider4, &QSlider::valueChanged, [=](int value) {
      double doubleValue = value / 100.0;
      valueLabel4->setText(QString::number(doubleValue, 'f', 2)+"*PI");
  });

  QHBoxLayout *sliderlayout5 = new QHBoxLayout();
  QSlider *slider5 = new QSlider(Qt::Horizontal);
  slider5->setRange(0, 200); 
  slider5->setValue(140); 
  QLabel* label7 = new QLabel("初始化偏转角间隔最大值:");
  QLabel *valueLabel5 = new QLabel("1.4*PI");
  layout->addLayout(sliderlayout5);
  sliderlayout5->addWidget(label7);
  QSpacerItem *spacer5 = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
  sliderlayout5->addSpacerItem(spacer5);
  sliderlayout5->addWidget(valueLabel5);
  layout->addWidget(slider5);

  QObject::connect(slider5, &QSlider::valueChanged, [=](int value) {
      double doubleValue = value / 100.0;
      valueLabel5->setText(QString::number(doubleValue, 'f', 2)+"*PI");
  });

  // 创建一个按钮
  QPushButton* button = new QPushButton("保存参数");
  layout->addWidget(button);

  // 连接按钮点击事件到槽函数
  connect(button, &QPushButton::clicked, this, &MyDialog::onButtonClicked);

  // 设置布局到对话框
  this->setLayout(layout);


  
}


void MyDialog::onButtonClicked() {
      if (dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(0)->widget())->isChecked()==false
       && dynamic_cast<QCheckBox*>(this->layout()->itemAt(1)->layout()->itemAt(1)->widget())->isChecked()==false){

        std::cout<<"Please choose at least one metric for optimization!"<<std::endl;
        return ;
      }

      
      // Save the params
      this->opt_turns = dynamic_cast<QLineEdit*>(this->layout()->itemAt(3)->widget())->text().toInt();
      this->n_solu = dynamic_cast<QLineEdit*>(this->layout()->itemAt(5)->widget())->text().toInt();
      this->pro_cross = dynamic_cast<QSlider*>(this->layout()->itemAt(8)->widget())->value()/100.0;
      this->pro_mut = dynamic_cast<QSlider*>(this->layout()->itemAt(10)->widget())->value()/100.0;
      this->adap_factor = dynamic_cast<QSlider*>(this->layout()->itemAt(12)->widget())->value()/100.0;
      this->min_angle = dynamic_cast<QSlider*>(this->layout()->itemAt(14)->widget())->value()/100.0*3.1415926;
      this->max_angle = dynamic_cast<QSlider*>(this->layout()->itemAt(16)->widget())->value()/100.0*3.1415926;

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
      std::cout << "自适应因子: " << this->adap_factor << std::endl;
      std::cout << "初始化偏转角间隔最小值: " << this->min_angle <<" rad "<< std::endl;
      std::cout << "初始化偏转角间隔最大值: " << this->max_angle <<" rad "<< std::endl;
      
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
  this->pro_cross = 0.7;
  this->pro_mut = 0.01;

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
  QPushButton *button4 = new QPushButton(tr("标定板自动设置"));
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

  gazebo::msgs::Param *pro_c = msg.add_param();
  pro_c->set_name("proba_cross");
  msgs::Any *v_cro = new msgs::Any(msgs::ConvertAny(this->pro_cross));
  pro_c->set_allocated_value(v_cro);

  gazebo::msgs::Param *pro_m = msg.add_param();
  pro_m->set_name("proba_mut");
  msgs::Any *v_mut = new msgs::Any(msgs::ConvertAny(this->pro_mut));
  pro_m->set_allocated_value(v_mut);

  // this->factoryPub->Publish(msg);
}
