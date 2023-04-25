
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class MyGuiPlugin : public GuiPlugin
  {
    public: MyGuiPlugin() : GuiPlugin()
    {
      // 订阅 Gui 事件
      this->connections.push_back(
        gui::Events::ConnectButtonClicked(
          boost::bind(&MyGuiPlugin::OnButtonClick, this)));
    }

    public: void OnButtonClick()
    {
      // 获取当前世界指针
      physics::WorldPtr world = physics::get_world();

      // 获取世界中的所有模型
      physics::Model_V models = world->Models();

      // 遍历每个模型
      for (auto model : models)
      {
        // 获取模型名称
        std::string modelName = model->GetName();

        // 获取模型类型
        std::string modelType = "";
        if (model->HasType())
        {
          modelType = model->GetType();
        }

        // 获取模型当前姿态
        math::Pose modelPose = model->WorldPose();

        // 输出模型信息
        std::cout << "Model Name: " << modelName << std::endl;
        std::cout << "Model Type: " << modelType << std::endl;
        std::cout << "Model Pose: " << modelPose << std::endl;
      }
    }
  };

  GZ_REGISTER_GUI_PLUGIN(MyGuiPlugin)
}