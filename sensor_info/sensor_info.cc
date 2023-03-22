#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string.h>
#include <vector>
#include <time.h>
#include <random>
#include <math.h>

#define SOLUTION_NUM 50


struct s_info
{
  std::string model_name;
  std::string type;
  gazebo::physics::ModelPtr mdl;
  gazebo::sensors::SensorPtr sen;
  ignition::math::Pose3d model_pose;
  ignition::math::Vector3d TopLeft;
  ignition::math::Vector3d TopRight; 
  ignition::math::Vector3d BottomLeft;
  ignition::math::Vector3d BottomRight;
};


std::vector<ignition::math::Vector3d> get_border(gazebo::physics::WorldPtr world, gazebo::physics::ModelPtr model){
  std::vector<ignition::math::Vector3d> res;
  ignition::math::Vector3d point;
  auto p = model->WorldPose();
      double z = 0;
      double x = 0;
      double y = 0;
      world->Physics()->InitForThread();
      gazebo::physics::RayShapePtr distray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        model->GetWorld()->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
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
            point.X()=x;
            point.Y()=y;
            point.Z()=z-dist;
            res.push_back(point);
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
            point.X()=x-dist;
            point.Y()=y;
            point.Z()=z;
            res.push_back(point);
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
            point.X()=x+dist;
            point.Y()=y;
            point.Z()=z;
            res.push_back(point);
          }
        }
        y=-10;
        z+=0.1;
      }

      // look from left
      x = -10;
      y = 100;
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
            point.X()=x;
            point.Y()=y-dist;
            point.Z()=z;
            res.push_back(point);
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
            point.X()=x;
            point.Y()=y+dist;
            point.Z()=z;
            res.push_back(point);
          }
        }
        x=-10;
        z+=0.1;
      }
  return res;
}


namespace gazebo
{
  class sensor_info : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    { 
      // Variables
      int flag = 0;
      int sensor_model_num = 0;
      int init = 20;
      int rand_index = 0;
      std::srand(std::time(nullptr));
      std::vector<s_info*>* solution_list[SOLUTION_NUM];
      std::vector<ignition::math::Vector3d> border_points;
      ignition::math::Vector3d point;
      ignition::math::Pose3d tmp_pose;

      // Get sensor pointers
      auto sm = sensors::SensorManager::Instance();
      std::vector< std::string > st;
      auto sensor_list = sm->GetSensors();

      this->model = _parent;
      std::cout << "Model loaded: " << model->GetName() << std::endl;
      std::cout << "Scale: " << sc <<  std::endl;
      std::cout << "Sensor number: " << model->GetSensorCount() << std::endl;

      // Get the world and the number of sensors
      physics::WorldPtr world = model->GetWorld();
      physics::Model_V models = world->Models();
      for (int i=0; i<SOLUTION_NUM; i++) {
        std::vector<s_info*>* one_solu = new std::vector<s_info*>();
        solution_list[i] = one_solu;
      }
      for (auto model:models){
        if (model->GetSensorCount()){
          sensor_model_num++;
          tmp_pose = model->WorldPose();
          tmp_pose.Pos().X() = init;
          tmp_pose.Pos().Y() = init;
          tmp_pose.Pos().Z() = init;
          init++;
          model->SetWorldPose(tmp_pose);
          model->SetStatic(true);
          model->SetGravityMode(false);

          // Initiate the solution list
          for (int i=0; i<SOLUTION_NUM; i++){
            s_info* one = new s_info();
            one->model_name = model->GetName();
            one->mdl = model;
            one->model_pose = tmp_pose;
            solution_list[i]->push_back(one);
          }
        }
      }
      this->model->SetWorldPose(tmp_pose);

      // Get border points of the car model
      border_points = get_border(world, this->model);
      std::cout<<"border points size: "<< border_points.size()<<std::endl;
      std::cout<<"solution numbers: "<< SOLUTION_NUM<<std::endl;
      std::cout<<"sensor number in solution: "<< solution_list[0]->size()<<std::endl;

      // Set random values to the solution according to the restriction
      for (int i=0; i<SOLUTION_NUM; i++){
        std::vector<s_info*>* solution = solution_list[i];
        for (auto s:*solution){
          rand_index = border_points.size() * rand()/RAND_MAX;
          s->model_pose.Pos() = border_points.at(rand_index);
          ignition::math::Vector3d rot_vec;
          rot_vec.X() = 2*M_PI * rand()/RAND_MAX - M_PI;
          rot_vec.Y() = 2*M_PI * rand()/RAND_MAX - M_PI;
          rot_vec.Z() = 2*M_PI * rand()/RAND_MAX - M_PI;
          s->model_pose.Rot().Euler(rot_vec);
          for (auto sen : sensor_list){
            if (sen->Type()=="camera"){
              sensors::CameraSensorPtr camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sen);
              if (s->model_name + "::link" == camera->ParentName()){
                s->sen = sen;
                rendering::CameraPtr cam = camera->Camera();
                std::cout << "A camera belongs to \"" << s->model_name << "\" found, whose parameters are:" << std::endl;
                std::cout << "  Image width: " << cam->ImageWidth() << std::endl;
                std::cout << "  Image height: " << cam->ImageHeight() << std::endl;
                std::cout << "  Cam horizontal FOV: " << cam->HFOV() << std::endl;
                std::cout << "  Nearest dist " << cam->NearClip() << std::endl;
                std::cout << "  Farest dist: " << cam->FarClip() << std::endl;

                // Get the camera parameters
                double fov = cam->HFOV().Radian();
                double aspectRatio = cam->ImageWidth() / camera->ImageHeight();
                double nearClip = cam->NearClip();
                double farClip = cam->FarClip();
                // Compute the camera pose and direction             
                ignition::math::Pose3d cameraPose = s->model_pose;
                cameraPose.Pos()+=camera->Pose().Pos();
                ignition::math::Vector3d cameraPos = cameraPose.Pos();
                ignition::math::Quaterniond cameraRot = cameraPose.Rot();
                ignition::math::Vector3d cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));

                // Compute the camera frustum vectors
                double halfFovTan = tan(fov / 2.0);
                ignition::math::Vector3d leftVec = cameraRot.RotateVector(ignition::math::Vector3d(0,1,0));
                ignition::math::Vector3d topVec = cameraDir.Cross(leftVec);
                topVec.Normalize();
                leftVec = topVec.Cross(cameraDir);
                leftVec.Normalize();
                std::cout<<"  Camera pose: "<<cameraPose<<std::endl;
                ignition::math::Vector3d TopLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
                ignition::math::Vector3d TopRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
                ignition::math::Vector3d BottomLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
                ignition::math::Vector3d BottomRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;

                std::cout<<"  The pyramid vectors: "<<std::endl;
                std::cout<<"    Top left: "<<TopLeft<<std::endl;
                std::cout<<"    Top right: "<<TopRight<<std::endl;
                std::cout<<"    Bottom left: "<<BottomLeft<<std::endl;
                std::cout<<"    Bottom right: "<<BottomRight<<std::endl;
              }
            }else if(sen->Type()=="ray"){
              continue; //TO DO
            }
          }
        }
      }
      for (int i=0; i<SOLUTION_NUM; i++){
        std::vector<s_info*>* solution = solution_list[i];
        for (auto s:*solution){
          std::cout<<s->model_name<<": "<<s->model_pose<<std::endl;
        }
        break;
      }
      std::cout<<std::endl;

      

      for (auto s : sensor_list){
        if (s->Type()=="camera"){
          int c_note = 0;
          sensors::CameraSensorPtr camera = std::dynamic_pointer_cast<sensors::CameraSensor>(s);
          std::cout<<"A camera" << std::endl;
          std::cout<< "Name: "<<s->Name()<<std::endl;
          std::cout<< "Type: "<<s->Type()<<std::endl;
          rendering::CameraPtr cam = camera->Camera();
          std::cout << "Camera " << camera->Name() << " parameters:" << std::endl;
          std::cout << "  Image width: " << cam->ImageWidth() << std::endl;
          std::cout << "  Image height: " << cam->ImageHeight() << std::endl;
          std::cout << "  Cam horizontal FOV: " << cam->HFOV() << std::endl;
          std::cout << "  Nearest dist " << cam->NearClip() << std::endl;
          std::cout << "  Farest dist: " << cam->FarClip() << std::endl;

          // Get the camera parameters
          double fov = cam->HFOV().Radian();
          double aspectRatio = cam->ImageWidth() / camera->ImageHeight();
          double nearClip = cam->NearClip();
          double farClip = cam->FarClip();
          // Compute the camera pose and direction
          ignition::math::Pose3d cameraPose = camera->Pose();
          ignition::math::Vector3d cameraPos = cameraPose.Pos();
          ignition::math::Quaterniond cameraRot = cameraPose.Rot();
          ignition::math::Vector3d cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));
          std::cout<<camera->ParentName()<<std::endl;
          auto models = world->Models();
          for (auto const& model : models){
            
            if (model->GetSensorCount()!=1){
              continue;
            }
            auto links = model->GetLinks();
            for (auto const& link : links){
              std::cout<<link->GetId()<<std::endl;
              if (link->GetId()==camera->ParentId()){
                std::cout<<"match! "<<std::endl;
                flag = 1;
                cameraPose = model->WorldPose();
                cameraPos = cameraPose.Pos();
                cameraRot = cameraPose.Rot();
                cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));
              }
            }
            if (flag){
              flag = 0;
              break;
            }
          }

          // Compute the camera frustum vectors
          double halfFovTan = tan(fov / 2.0);
          ignition::math::Vector3d leftVec = cameraRot.RotateVector(ignition::math::Vector3d(0,1,0));
          ignition::math::Vector3d topVec = cameraDir.Cross(leftVec);
          topVec.Normalize();
          leftVec = topVec.Cross(cameraDir);
          leftVec.Normalize();
          std::cout<<cameraPose<<std::endl;
          // ignition::math::Vector3d nearTopLeft = cameraPos + nearClip * cameraDir + halfFovTan * nearClip * leftVec + halfFovTan * nearClip / aspectRatio * topVec;
          // ignition::math::Vector3d nearTopRight = cameraPos + nearClip * cameraDir - halfFovTan * nearClip * leftVec + halfFovTan * nearClip / aspectRatio * topVec;
          // ignition::math::Vector3d nearBottomLeft = cameraPos + nearClip * cameraDir + halfFovTan * nearClip * leftVec - halfFovTan * nearClip / aspectRatio * topVec;
          // ignition::math::Vector3d nearBottomRight = cameraPos + nearClip * cameraDir - halfFovTan * nearClip * leftVec - halfFovTan * nearClip / aspectRatio * topVec;

          // ignition::math::Vector3d farTopLeft = cameraPos + farClip * cameraDir + halfFovTan * farClip * leftVec + halfFovTan * farClip / aspectRatio * topVec;
          // ignition::math::Vector3d farTopRight = cameraPos + farClip * cameraDir - halfFovTan * farClip * leftVec + halfFovTan * farClip / aspectRatio * topVec;
          // ignition::math::Vector3d farBottomLeft = cameraPos + farClip * cameraDir + halfFovTan * farClip * leftVec - halfFovTan * farClip / aspectRatio * topVec;
          // ignition::math::Vector3d farBottomRight = cameraPos + farClip * cameraDir - halfFovTan * farClip * leftVec - halfFovTan * farClip / aspectRatio * topVec;

          ignition::math::Vector3d TopLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
          ignition::math::Vector3d TopRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
          ignition::math::Vector3d BottomLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
          ignition::math::Vector3d BottomRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;


          std::cout<<"Top left: "<<TopLeft<<std::endl;
          std::cout<<"Top right: "<<TopRight<<std::endl;
          std::cout<<"Bottom left: "<<BottomLeft<<std::endl;
          std::cout<<"Bottom right: "<<BottomRight<<std::endl;
          // std::cout<<"Top left: "<<nearTopLeft.X()<<", "<<nearTopLeft.Y()<<", "<<nearTopLeft.Z()<<std::endl;
          // // Visualize the frustum
          // rendering::ScenePtr scene = cam->GetScene();
          // scene->DrawLine(nearTopLeft,farTopLeft,"TopLeft");
          // scene->DrawLine(nearTopRight,farTopRight,"TopRight");
          // scene->DrawLine(nearBottomLeft,farBottomLeft,"BottomLeft");
          // scene->DrawLine(nearBottomRight,farBottomRight,"BottomRight");
          TopLeft.Normalize();
          TopRight.Normalize();
          BottomLeft.Normalize();
          BottomRight.Normalize();
          ignition::math::Vector3d c1 = TopLeft.Cross(TopRight);
          ignition::math::Vector3d c2 = TopRight.Cross(BottomRight);
          ignition::math::Vector3d c3 = BottomRight.Cross(BottomLeft);
          ignition::math::Vector3d c4 = BottomLeft.Cross(TopLeft);
          std::cout<<c1<<std::endl;
          std::cout<<c2<<std::endl;
          std::cout<<c3<<std::endl;
          std::cout<<c4<<std::endl;

          for (int sx = -100; sx<100; sx++){
            for (int sy = -100; sy<100; sy++){
              for (int sz = 0; sz<10; sz++){
                ignition::math::Vector3d sVec;
                sVec.X() = sx;
                sVec.Y() = sy;
                sVec.Z() = sz;
                sVec = sVec-cameraPos;
                //std::cout<<sVec<<std::endl;
                if (sVec.Length()<nearClip || sVec.Length()>farClip) continue;
                if (sVec.Dot(c1)<=0) continue;
                if (sVec.Dot(c2)<=0) continue;
                if (sVec.Dot(c3)<=0) continue;
                if (sVec.Dot(c4)<=0) continue;
                c_note++;
              }
            }
          }
          std::cout<<c_note<<std::endl;
          std::cout<<std::endl;
        }


        if (s->Type()=="ray"){
          sensors::RaySensorPtr laser = std::dynamic_pointer_cast<sensors::RaySensor>(s);
          std::cout<<"A lidar" << std::endl;
          std::cout<< "Name: "<<s->Name()<<std::endl;
          std::cout<< "Type: "<<s->Type()<<std::endl;
          std::cout << "  Range max: " << laser->RangeMax() << " m" << std::endl;
          std::cout << "  Range min: " << laser->RangeMin() << " m" << std::endl;
          std::cout << "  Range count: " << laser->RangeCount() << std::endl;
          std::cout << "  Resolution: " << laser->RangeResolution() << " m" << std::endl;
          std::cout << "  H angle max: " << laser->AngleMax().Degree() << " degrees" << std::endl;
          std::cout << "  H angle min: " << laser->AngleMin().Degree() << " degrees" << std::endl;
          std::cout << "  V angle max: " << laser->VerticalAngleMax().Degree() << " degrees" << std::endl;
          std::cout << "  V angle min: " << laser->VerticalAngleMin().Degree() << " degrees" << std::endl;
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