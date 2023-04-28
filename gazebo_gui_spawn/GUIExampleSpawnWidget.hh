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
#ifndef _GUI_EXAMPLE_SPAWN_WIDGET_HH_
#define _GUI_EXAMPLE_SPAWN_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/common/common.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/GuiEvents.hh>
#endif

class MyDialog : public QDialog {
    Q_OBJECT

public:
    // 构造函数
    MyDialog(QWidget* parent = nullptr);

    bool if_cover();

    bool if_redun();

    int NumSolutions();

    int NumOptTurns();

    public: bool new_save;

public slots:
    // 槽函数：处理按钮点击事件
    void onButtonClicked();


    private:
        bool cover;
        bool redun;
        int n_solu;
        int opt_turns;
        double pro_cross;
        double pro_mut;
};

namespace gazebo
{
    class GAZEBO_VISIBLE GUIExampleSpawnWidget : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: GUIExampleSpawnWidget();

      /// \brief Destructor
      public: virtual ~GUIExampleSpawnWidget();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnButton1();

      protected slots: void OnButton2();

      protected slots: void OnButton3();

      protected slots: void OnButton4();

      public: void OnStats(ConstWorldStatisticsPtr &_msg);

      public: void Subtopic(ConstParam_VPtr &_msg);

      public: bool ifsub;

      /// \brief Counter used to create unique model names
      private: unsigned int counter;

      /// \brief Node used to establish communication with gzserver.
      private: transport::NodePtr node;

      /// \brief Publisher of factory messages.
      private: transport::PublisherPtr factoryPub;

      private: transport::SubscriberPtr statsSub;

      private: transport::SubscriberPtr statsSub2;

      private: event::ConnectionPtr updateTimer; // 定时器

      private:
        bool cover;
        bool redun;
        int n_solu;
        int opt_turns;
        double pro_cross;
        double pro_mut;
    };
}
#endif
