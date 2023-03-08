#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string.h>

namespace gazebo
{
  class ginfo : public ModelPlugin
  {
    public: 
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
      {
        // Store the pointer to the model
        this->model = _parent;

        // Get the world pointer
        this->world = this->model->GetWorld();

        // Initialize random number generator
        std::srand(std::time(nullptr));

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ginfo::OnUpdate, this));
        this->count = 0;
      }

      void OnUpdate()
      { 
        if (count!=100){
          count++;
          return ;
        }
        count = 0;
        // Get all the models in the world
        auto models = this->world->Models();
        std::string grd = "ground_plane";
        // Loop through each model and update its x coordinate
        for (auto const& model : models)
        {
          if (model != this->model)
          { 
            if (grd==model->GetName()){
              continue;
            }
            std::string modelName = model->GetName();

            // 获取模型类型
            std::string modelType = "";
            
            modelType = model->GetType();

            // 获取模型当前姿态
            auto pose = model->WorldPose();

            // 输出模型信息
            std::cout << "Model Name: " << modelName << " ";
            std::cout << "Model Type: " << modelType << " ";
            std::cout << "Model Pose: " << pose << std::endl;
            auto rand_x = -0.1 + 0.2 * (static_cast<double>(std::rand()) / RAND_MAX);
            auto rand_y = -0.1 + 0.2 * (static_cast<double>(std::rand()) / RAND_MAX);
            auto rand_z = -0.1 + 0.2 * (static_cast<double>(std::rand()) / RAND_MAX);
            pose.Pos().X() += rand_x;
            pose.Pos().Y() += rand_y;
            model->SetWorldPose(pose);


            std::cout<<std::endl;
        }
      }
      }

    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      int count;
  };

  GZ_REGISTER_MODEL_PLUGIN(ginfo)
}