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
using namespace ignition;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIExampleSpawnWidget)

/////////////////////////////////////////////////
GUIExampleSpawnWidget::GUIExampleSpawnWidget()
  : GUIPlugin()
{
  this->counter = 0;

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
  QPushButton *button = new QPushButton(tr("Spawn Sphere"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

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
  this->resize(120, 40);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
GUIExampleSpawnWidget::~GUIExampleSpawnWidget()
{
}

/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton()
{
  // 获取当前世界指针
      std::cout << '0' << std::endl;
      // std::map<std::string, std::string> list = GetModels();
      // std::map<std::string, std::string>::iterator iter;
      // for (iter = list.begin();iter!=list.end();iter++){
      //   std::cout << iter->first<< ' '<<iter->second<<std::endl;
      // }
  std::cout<<"If Worlds running:"<<physics::worlds_running()<<std::endl;
  if (physics::has_world("default")){
      physics::WorldPtr world = physics::get_world("default");
      std::cout << '1' << std::endl;
      // 获取世界中的所有模型
      physics::Model_V models = world->Models();
      std::cout << '2' << std::endl;
      // 遍历每个模型
      for (auto model : models)
      {
        // 获取模型名称
        std::string modelName = model->GetName();

        // 获取模型类型
        std::string modelType = "";
        
        modelType = model->GetType();


        // 获取模型当前姿态
        math::Pose3d modelPose = model->WorldPose();

        // 输出模型信息
        std::cout << "Model Name: " << modelName << std::endl;
        std::cout << "Model Type: " << modelType << std::endl;
        std::cout << "Model Pose: " << modelPose << std::endl;
      }
  }else{
    std::cout<< "NO world named default"<<std::endl;
  }
}
