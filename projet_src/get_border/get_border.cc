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
  class get_border : public ModelPlugin
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
            std::bind(&get_border::OnUpdate, this));
        this->count = 0;
        ignition::math::Vector3d init;
        init.X() = 20;
        init.Y() = 20;
        init.Z() = 20;
        this->model->SetWorldPose(init);
      }

      void OnUpdate()
      { 
        if (this->count){
          return ;
        }
        auto p = this->model->WorldPose();
        double z = 0;
        double x = 0;
        double y = 0;
        this->model->GetWorld()->Physics()->InitForThread();
        physics::RayShapePtr distray = boost::dynamic_pointer_cast<physics::RayShape>(
          this->model->GetWorld()->Physics()->CreateShape("ray", physics::CollisionPtr()));
        ignition::math::Vector3d start;
        ignition::math::Vector3d end = start;
        std::string near_e;
        double dist = 0;
        // look from top
        x = -10;
        y = -10;
        z = 100;
        start.Z() = z;
        end.Z() = -z;
        while (x<=10){
          start.X() = x;
          end.X() = x;
          while (y<=10){
            start.Y() = y;
            end.Y() = y;
            distray->SetPoints(start, end);
            distray->GetIntersection(dist, near_e);
            y+=0.1;
            if (dist!=100 && dist!=1000 && dist>0){
              std::cout << "Nearest entity below is named: " << near_e << ", distance: " << dist <<std::endl;
            }
          }
          y=-10;
          x+=0.1;
        }

        // look from front
        x = 100;
        y = -10;
        z = 0.1;
        start.X() = x;
        end.X() = -x;
        while (z<=10){
          start.Z() = z;
          end.Z() = z;
          while (y<=10){
            start.Y() = y;
            end.Y() = y;
            distray->SetPoints(start, end);
            distray->GetIntersection(dist, near_e);
            y+=0.1;
            if (dist!=1000 && dist>0){
              std::cout << "Nearest entity behind is named: " << near_e << ", distance: " << dist <<std::endl;
            }
          }
          y=-10;
          z+=0.1;
        }

        // look from behind
        x = -100;
        y = -10;
        z = 0.1;
        start.X() = x;
        end.X() = -x;
        while (z<=10){
          start.Z() = z;
          end.Z() = z;
          while (y<=10){
            start.Y() = y;
            end.Y() = y;
            distray->SetPoints(start, end);
            distray->GetIntersection(dist, near_e);
            y+=0.1;
            if (dist!=1000 && dist>0){
              std::cout << "Nearest entity front is named: " << near_e << ", distance: " << dist <<std::endl;
            }
          }
          y=-10;
          z+=0.1;
        }

        // look from left
        x = -10;
        y = 100;
        z = 0.1;
        start.X() = x;
        end.X() = -x;
        while (z<=10){
          start.Z() = z;
          end.Z() = z;
          while (x<=10){
            start.X() = x;
            end.X() = x;
            distray->SetPoints(start, end);
            distray->GetIntersection(dist, near_e);
            x+=0.1;
            if (dist!=1000 && dist>0){
              std::cout << "Nearest entity right is named: " << near_e << ", distance: " << dist <<std::endl;
            }
          }
          x=-10;
          z+=0.1;
        }

        // look from right
        x = -10;
        y = -100;
        z = 0.1;
        start.Y() = y;
        end.Y() = -y;
        while (z<=10){
          start.Z() = z;
          end.Z() = z;
          while (x<=10){
            start.X() = x;
            end.X() = x;
            distray->SetPoints(start, end);
            distray->GetIntersection(dist, near_e);
            x+=0.1;
            if (dist!=1000 && dist>0){
              std::cout << "Nearest entity left is named: " << near_e << ", distance: " << dist <<std::endl;
            }
          }
          x=-10;
          z+=0.1;
        }
        
        this->count = 1;
      }

    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      int count;
  };

  GZ_REGISTER_MODEL_PLUGIN(get_border)
}