#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string.h>


namespace gazebo
{
  class sensor_info : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;
      this->msdf = model->GetSDF();
      sc = model->Scale();
      std::cout << "Model loaded: " << model->GetName() << std::endl;
      std::cout << "Scale: " << sc <<  std::endl;
      std::cout << "Sensor number: " << model->GetSensorCount() << std::endl;
      // Get the list of cameras in the model
      std::string des;
      std::string val;
      std::cout << "att count: "  << msdf->GetAttributeCount() << std::endl;
      std::cout << "des count: "  << msdf->GetElementDescriptionCount() << std::endl;
      std::cout << "Element width: "  << msdf->HasElement("width") << std::endl;
      std::cout << "Element h_fov: "  << msdf->HasElement("horizontal_fov") << std::endl;
      auto links = model->GetLinks();

      for (auto link : links)
      {
        std::cout << "Link name: " << link->GetName() << std::endl;
        std::cout << "Child count: " << link->GetChildCount() << std::endl;
      }
      auto sm = sensors::SensorManager::Instance();
      std::vector< std::string > st;
      auto sensor_list = sm->GetSensors();
      for (auto s : sensor_list){
        std::cout<< "Name: "<<s->Name()<<std::endl;
        std::cout<< "Type: "<<s->Type()<<std::endl;
        if (s->Type()=="camera"){
          sensors::CameraSensorPtr camera = std::dynamic_pointer_cast<sensors::CameraSensor>(s);
          std::cout << "Camera " << camera->Name() << " parameters:" << std::endl;
          std::cout << "  Image width: " << camera->ImageWidth() << std::endl;
          std::cout << "  Image height: " << camera->ImageHeight() << std::endl;
        }
      }
    }
    
    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      ignition::math::Vector3d sc;
      sdf::ElementPtr msdf;
      int count;
  };

  GZ_REGISTER_MODEL_PLUGIN(sensor_info)
}