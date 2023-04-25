#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/rendering.hh>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string.h>

namespace gazebo
{
  class print_sensor_info : public ModelPlugin
  {
    public: 
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
      {
        int cam_num = 0;
        int ray_num = 0;
        // Store the pointer to the model
        this->model = _parent;

        // Get the world pointer
        this->world = this->model->GetWorld();

        // Initialize random number generator
        std::srand(std::time(nullptr));

        auto sm = sensors::SensorManager::Instance();
        std::vector< std::string > st;
        auto sensor_list = sm->GetSensors();
        sensors::CameraSensorPtr camera;
        int flag = 0;
        rendering::CameraPtr cam;

        for (auto sen : sensor_list){
            if (sen->Type()=="camera"){
              cam_num++;
              camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sen);

              // Check which sensor matches the model, then input the parameters of the sensor
              // (Building pyramid model for the camera, then calculate the normals of 4 planes)
 
              cam = camera->Camera();

              std::cout << "A camera belongs to \"" << camera->Name() << "\" found, whose parameters are:" << std::endl;
              std::cout << "  Image width: " << cam->ImageWidth() << std::endl;
              std::cout << "  Image height: " << cam->ImageHeight() << std::endl;
              std::cout << "  Cam horizontal FOV: " << cam->HFOV() << std::endl;
              std::cout << "  Nearest dist " << cam->NearClip() << std::endl;
              std::cout << "  Farest dist: " << cam->FarClip() << std::endl;


            }else if(sen->Type()=="ray"){
              ray_num++;
              sensors::RaySensorPtr rs = std::dynamic_pointer_cast<sensors::RaySensor>(sen);
              ignition::math::Angle HFOV_max = rs->AngleMax();
              ignition::math::Angle HFOV_min = rs->AngleMin();
              ignition::math::Angle VFOV_max = rs->VerticalAngleMax();
              ignition::math::Angle VFOV_min = rs->VerticalAngleMin();
              double ray_range_max = rs->RangeMax();
              double ray_range_min = rs->RangeMin();
              std::cout << "A ray sensor belongs to \"" << rs->Name() << "\" found, whose parameters are:" << std::endl;
              std::cout << "  Horizontal FOV: " << HFOV_max-HFOV_min << std::endl;
              if (VFOV_max.Degree()-VFOV_min.Degree()) std::cout << "  Vertical FOV: " << VFOV_max-VFOV_min << std::endl;
              std::cout << "  Nearest dist " << ray_range_min << std::endl;
              std::cout << "  Farest dissensort: " << ray_range_max << std::endl;
            }
          }
        std::cout<<"Find in total "<<sensor_list.size()<< " sensors, with "<<cam_num<< " cameras, "<<ray_num<<" lidars."<<std::endl;
        std::cout<<std::endl;
        //this->world->RemoveModel(this->model);
      }

    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      int count;
  };

  GZ_REGISTER_MODEL_PLUGIN(print_sensor_info)
}