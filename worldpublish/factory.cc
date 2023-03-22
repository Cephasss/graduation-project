
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <stdio.h>
#include <time.h>

namespace gazebo
{
class Factory : public WorldPlugin
{

  private: physics::WorldPtr wld;
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    std::cout<< _parent->Name() << std::endl;
    this->wld = physics::get_world(_parent->Name());
    std::cout<<"loaded"<<std::endl;
    time_t start;
    while(0){
        start = time(NULL);
        physics::Model_V models = wld->Models();
    // 遍历每个模型
    for (auto model : models)
    {
      // 获取模型名称
        std::string modelName = model->GetName();

        // 获取模型类型
        std::string modelType = "";
        
        modelType = model->GetType();


        // 获取模型当前姿态
        ignition::math::Pose3d modelPose = model->WorldPose();

        // 输出模型信息
        std::cout << "Model Name: " << modelName << std::endl;
        std::cout << "Model Type: " << modelType << std::endl;
        std::cout << "Model Pose: " << std::endl;
        while(1){
            if (time(NULL)-start>1){
                break;
            }
        }
    }
  }
  }

  public: void Update(){
    physics::Model_V models = wld->Models();
    // 遍历每个模型
    for (auto model : models)
    {
      // 获取模型名称
        std::string modelName = model->GetName();

        // 获取模型类型
        std::string modelType = "";
        
        modelType = model->GetType();


        // 获取模型当前姿态
        ignition::math::Pose3d modelPose = model->WorldPose();

        // 输出模型信息
        std::cout << "Model Name: " << modelName << std::endl;
        std::cout << "Model Type: " << modelType << std::endl;
        std::cout << "Model Pose: " << std::endl;

    
    }
    

    
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}