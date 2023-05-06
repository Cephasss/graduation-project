#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <string.h>
#include <vector>
#include <time.h>
#include <random>
#include <math.h>


#define SOLUTION_NUM 1
#define DEBUG false                 // Set true to print some intermediate infos


struct s_info{
  std::string model_name;
  std::string type;
  // std::string ray_type;  //TODO
  gazebo::sensors::SensorPtr sen;
  ignition::math::Pose3d model_pose;
  ignition::math::Vector3d nor1;
  ignition::math::Vector3d nor2; 
  ignition::math::Vector3d nor3;
  ignition::math::Vector3d nor4;
  ignition::math::Angle lidar_fov;
  int near;
  int far;
};

int power(int base, int pow);

std::vector<ignition::math::Vector3d> get_normals(double fov, double aspectRatio, double nearClip, double farClip, ignition::math::Pose3d sensorPose);

namespace gazebo
{
  class calculate : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    { 
      // Variables------------------------------------------------------------------------------------------------------------------------------------------
        // simple (temporary) params
      int flag = 0;
      int sensor_model_num = 0;
      int init = 20;
      double fov;
      double aspectRatio;
      double nearClip;
      double farClip;
      sensors::CameraSensorPtr camera;
      rendering::CameraPtr cam;
      ignition::math::Pose3d sensorPose;
      std::vector<ignition::math::Vector3d> normals;
      sensors::RaySensorPtr rs;
      int rand_index = 0;
      double rand_num_a = 0;
      double rand_num_b = 0;
      int select_factor = 2;
      ignition::math::Vector3d point;
      ignition::math::Pose3d tmp_pose;
      std::vector<s_info*>* solution;
      std::srand(std::time(0));
        // For optimization process
      int turn = 1;
      double cover_ratio = 0;
      double score_solutions[SOLUTION_NUM];
      int score_rank[SOLUTION_NUM];
      double sel_proba[SOLUTION_NUM];
      std::vector<s_info> tmp_solu_list[SOLUTION_NUM];
      double tmp_score_solutions[SOLUTION_NUM];
      double tmp_sel_proba[SOLUTION_NUM];
      double score_sum = 0;
      int tmp_solu_index = 0;
      int multi_count = 0;

        // The base lists
      std::vector<s_info*>* solution_list[SOLUTION_NUM];
      std::vector<ignition::math::Vector3d> border_points;
      std::vector<std::vector<ignition::math::Vector3d>> border_points_type;
      ignition::math::Vector3d type;


      ignition::math::Vector3d rot_vec;

      // Get sensor pointers-------------------------------------------------------------------------------------------------------------------------------
      auto sm = sensors::SensorManager::Instance();
      std::vector< std::string > st;
      auto sensor_list = sm->GetSensors();

      this->model = _parent;

      // Get the world and the number of sensor models (not sensors because a model may contain several sensors)-------------------------------------------
      physics::WorldPtr world = model->GetWorld();
      physics::Model_V models = world->Models();
      for (int i=0; i<SOLUTION_NUM; i++) {
        std::vector<s_info*>* one_solu = new std::vector<s_info*>;
        solution_list[i] = one_solu;
      }
      for (auto model:models){
        if (model->GetSensorCount()){
          sensor_model_num++;

          // Initiate the solution list
          for (int i=0; i<SOLUTION_NUM; i++){
            s_info* one = new struct s_info;
            one->model_name = model->GetName();
            one->model_pose = model->WorldPose();
            solution_list[i]->push_back(one);
          }
        }
      }
      if (1){
        std::cout<<"border points size: "<< border_points.size()<<std::endl;
        std::cout<<"solution numbers: "<< SOLUTION_NUM<<std::endl;
        std::cout<<"sensor number in solution: "<< solution_list[0]->size()<<std::endl;
        solution = solution_list[0];
        // Calculate camera of lidar parameters------------------------------------------------------------------------------------------------------------
        for (auto s:*solution){
          //std::cout << "  -1 " << std::endl;
          for (auto sen : sensor_list){
            // if a camera
            if (sen->Type()=="camera"){
              camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sen);
              // Check which sensor matches the model, then input the parameters of the sensor
              // (Building pyramid model for the camera, then calculate the normals of 4 planes)
              for (auto link : world->ModelByName(s->model_name)->GetLinks()){
                if (s->model_name + "::" + link->GetName()==camera->ParentName()){
                  flag = 1;
                  break;
                }
              }
              if (flag){
                flag = 0;
                s->sen = sen;
                s->type = "camera";
                cam = camera->Camera();
                if (DEBUG){
                  std::cout << "A camera belongs to \"" << s->model_name << "\" found, whose parameters are:" << std::endl;
                  std::cout << "  Image width: " << cam->ImageWidth() << std::endl;
                  std::cout << "  Image height: " << cam->ImageHeight() << std::endl;
                  std::cout << "  Cam horizontal FOV: " << cam->HFOV() << std::endl;
                  std::cout << "  Nearest dist " << cam->NearClip() << std::endl;
                  std::cout << "  Farest dist: " << cam->FarClip() << std::endl;
                }
                // Get the camera parameters
                fov = cam->HFOV().Radian();
                aspectRatio = cam->ImageWidth() / camera->ImageHeight();
                nearClip = cam->NearClip();
                farClip = cam->FarClip();
                s->near = nearClip;
                s->far = farClip;
                // Compute the camera pose and direction             
                sensorPose = s->model_pose;
                sensorPose.Pos()+=camera->Pose().Pos();
                s->model_pose = sensorPose;
                normals = get_normals(fov, aspectRatio, nearClip, farClip, sensorPose);
                s->nor1 = normals[0];
                s->nor2 = normals[1];
                s->nor3 = normals[2];
                s->nor4 = normals[3];
              }

            // if a lidar
            }else if(sen->Type()=="ray"){
              rs = std::dynamic_pointer_cast<sensors::RaySensor>(sen);
              ignition::math::Angle HFOV_max = rs->AngleMax();
              ignition::math::Angle HFOV_min = rs->AngleMin();
              ignition::math::Angle VFOV_max = rs->VerticalAngleMax();
              ignition::math::Angle VFOV_min = rs->VerticalAngleMin();
              double ray_range_max = rs->RangeMax();
              double ray_range_min = rs->RangeMin();
              // Check which sensor matches the model, then input the parameters of the sensor
              // (Building different model for the ray)
              for (auto link : world->ModelByName(s->model_name)->GetLinks()){
                if (s->model_name + "::" + link->GetName()==rs->ParentName()){
                  flag = 1;
                  break;
                }
              }
              if (flag){
                flag = 0;
                s->sen = sen;
                s->type = "ray";
                if (DEBUG){
                  std::cout << "A ray sensor belongs to \"" << s->model_name << "\" found, whose parameters are:" << std::endl;
                  std::cout << "  Horizontal FOV: " << HFOV_max-HFOV_min << std::endl;
                  if (VFOV_max.Degree()-VFOV_min.Degree()) std::cout << "  Vertical FOV: " << VFOV_max-VFOV_min << std::endl;
                  std::cout << "  Nearest dist " << ray_range_min << std::endl;
                  std::cout << "  Farest dist: " << ray_range_max << std::endl;
                }
                // Now only build the model for mid70-like lidar(Circle FOV)
                s->near = ray_range_min;
                s->far = ray_range_max;
                // Compute the ray pose and direction             
                sensorPose = s->model_pose;
                sensorPose.Pos()+=rs->Pose().Pos();
                s->model_pose = sensorPose;
                // Get the direction vector
                s->nor1 = sensorPose.Rot().RotateVector(ignition::math::Vector3d(1,0,0));
                s->lidar_fov = VFOV_max;
              }
            }
          }
          
        }
        //std::cout << "  Initialize: "<< i << std::endl;
      }
        double redun_ratio = 0;
        int redun_flag = 0;
          int all_area = 0;
      // Print values for debug
      if (1){
        for (int i=0; i<SOLUTION_NUM; i++){
          solution = solution_list[i];
          std::cout<<"FOR THE "<<i<<" -th:"<<std::endl;
          for (auto s:*solution){
            std::cout<<s->model_name<<": "<<s->model_pose<<std::endl;
          }
        }
      }
      std::cout<<std::endl;
      
      // The optimization process--------------------------------------------------------------------------------------------------------------------------
      ignition::math::Vector3d sVec;
      ignition::math::Vector3d sVec_c;
        // Initiate the varaibles
        score_sum = 0;
        tmp_solu_index = 0;
        ignition::math::Quaterniond qtheta;
        memset(score_solutions, 0, sizeof(score_solutions));
        memset(score_rank, 0, sizeof(score_rank));
        memset(sel_proba, 0, sizeof(sel_proba));
        memset(tmp_score_solutions, 0, sizeof(tmp_score_solutions));
        memset(tmp_sel_proba, 0, sizeof(tmp_sel_proba));
        // Calculate the score for each solution in the list and store them in a list----------------------------------------------------------------------
        for (int i=0; i<SOLUTION_NUM; i++){
          solution = solution_list[i];
          cover_ratio = 0;
          
          for (int sz = 0; sz<=16; sz++){
            // std::cout<<sx<<std::endl;
            for (int sr = 1; sr<=36; sr++){
              // if (sr>=30 && sr<=50) continue;
              for (int stheta = -78; stheta<79; stheta++){
                all_area++;
                rot_vec.X() = 0;
                rot_vec.Y() = 0;
                rot_vec.Z() = (double)stheta/25.0;
                qtheta.Euler(rot_vec);
                sVec_c.X() = (double)sr/3.0;
                sVec_c.Y() = 0.0;
                sVec_c.Z() = (double)sz/4.0;
                sVec_c = qtheta.RotateVector(sVec_c);
                // Skip if too close or inside the car
                // if (sVec.Z()<max_z && sVec.X()>min_x && sVec.X()<max_x && sVec.Y()>min_y && sVec.Y()<max_y) continue;
                // Initiate a space point
                multi_count = 0;
                redun_flag = 0;
                for (auto s:*solution){
                    sVec = sVec_c;
                  sVec -= s->model_pose.Pos();
                  if (sVec.Length()<s->near || sVec.Length()>s->far) continue;
                  if (s->type=="camera"){
                    if (sVec.Dot(s->nor1)<=0) continue;
                    if (sVec.Dot(s->nor2)<=0) continue;
                    if (sVec.Dot(s->nor3)<=0) continue;
                    if (sVec.Dot(s->nor4)<=0) continue;
                    
                    // Redundancy check
                    if (multi_count){
                      score_solutions[i]-=1/power(2,multi_count);
                    }else{
                      score_solutions[i]++;
                      cover_ratio++;
                    }
                    // s->score++;
                    multi_count++;
                  }else if(s->type=="ray"){
                    //std::cout<<acos(sVec.Dot(s->nor1)/sVec.Length()) <<std::endl;

                    if (acos(sVec.Dot(s->nor1)/sVec.Length()) > s->lidar_fov.Radian()) continue;
                    // Redundancy check
                    if (multi_count){
                        if (!redun_flag) redun_ratio++;
                        redun_flag=1;
                      score_solutions[i]+=1/power(2,multi_count);
                    }else{
                      if (stheta<4.1) score_solutions[i]+=0.1;
                      score_solutions[i]++;
                      cover_ratio++;
                    }
                    multi_count++;
                  }
                }
              }
            }
          }
          cover_ratio = cover_ratio / all_area;
          redun_ratio = redun_ratio / all_area;
          for (auto s1:*solution){
            for (auto s2:*solution){
              if(s1->model_name==s2->model_name) continue;
              if ((s1->model_pose.Pos()-s2->model_pose.Pos()).Length()>0.15) continue;
              score_solutions[i]-=2000;
            }
          }
        }
        std::cout<<cover_ratio<<std::endl;
        std::cout<<redun_ratio<<std::endl;

      
      // Refresh the pointers
      for (int i=0; i<SOLUTION_NUM; i++){
        for (int c=0; c<sensor_model_num; c++){
          delete solution_list[i]->at(c);
          solution_list[i]->at(c) = nullptr;
        }
        delete solution_list[i];
        solution_list[i] = nullptr;
      }

    }

    
    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      ignition::math::Vector3d sc;
      sdf::ElementPtr msdf;
      int count;
  };

  GZ_REGISTER_MODEL_PLUGIN(calculate)
}

int power(int base, int pow){
  int res = base;
  int p = pow;
  if (p==0) return 1;
  if (p<0){
    p=-p;
    res = 1/res;
  }
  while (p>0){
    p--;
    res*=res;
  }
  return res;
}
std::vector<ignition::math::Vector3d> get_normals(double fov, double aspectRatio, double nearClip, double farClip, ignition::math::Pose3d sensorPose){
  std::vector<ignition::math::Vector3d> normals;
  ignition::math::Vector3d cameraPos = sensorPose.Pos();
  ignition::math::Quaterniond cameraRot = sensorPose.Rot();
  ignition::math::Vector3d cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));

  // Compute the camera frustum vectors
  double halfFovTan = tan(fov / 2.0);
  ignition::math::Vector3d leftVec = cameraRot.RotateVector(ignition::math::Vector3d(0,1,0));
  ignition::math::Vector3d topVec = cameraDir.Cross(leftVec);
  topVec.Normalize();
  leftVec = topVec.Cross(cameraDir);
  leftVec.Normalize();
  if (DEBUG) std::cout<<"  Camera pose: "<<sensorPose<<std::endl<<std::endl;
  ignition::math::Vector3d TopLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
  ignition::math::Vector3d TopRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec + halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
  ignition::math::Vector3d BottomLeft = (farClip-nearClip) * cameraDir + halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
  ignition::math::Vector3d BottomRight = (farClip-nearClip) * cameraDir - halfFovTan * (farClip-nearClip) * leftVec - halfFovTan * (farClip-nearClip) / aspectRatio * topVec;
  TopLeft.Normalize();
  TopRight.Normalize();
  BottomLeft.Normalize();
  BottomRight.Normalize();
  normals.push_back(TopLeft.Cross(TopRight));
  normals.push_back(TopRight.Cross(BottomRight));
  normals.push_back(BottomRight.Cross(BottomLeft));
  normals.push_back(BottomLeft.Cross(TopLeft));
  if (0){
    std::cout<<"  The pyramid vectors: "<<std::endl;
    std::cout<<"    Top left: "<<TopLeft<<std::endl;
    std::cout<<"    Top right: "<<TopRight<<std::endl;
    std::cout<<"    Bottom left: "<<BottomLeft<<std::endl;
    std::cout<<"    Bottom right: "<<BottomRight<<std::endl;
  }
  return normals;
}

