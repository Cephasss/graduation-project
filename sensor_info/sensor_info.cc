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

#define SOLUTION_NUM 50
#define DEBUG false                 // Set true to print some intermediate infos
#define MAX_TURN 50

#define ELITE_NUM 3

#define GET_BEHIND 0

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

ignition::math::Vector3d gen_random_angle(ignition::math::Vector3d type);

int angle_punish(std::vector<s_info*>* solution, int num_sen, double mina, double maxa);

bool check_angle(std::vector<s_info*>* solution,int num_sen, double mina, double maxa);

bool check_angle(std::vector<s_info> solution,int num_sen, double mina, double maxa);

int get_cross_index(std::vector<s_info> solution_a,std::vector<s_info> solution_b,int num_sen, int cro_pos);

std::vector<std::vector<ignition::math::Vector3d>> get_border(gazebo::physics::WorldPtr world);

std::vector<ignition::math::Vector3d> get_normals(double fov, double aspectRatio, double nearClip, double farClip, ignition::math::Pose3d sensorPose);

void get_solution_rank(double* score_list, int* rank);

namespace gazebo
{
  class sensor_info : public ModelPlugin
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
      std::vector<s_info> solution_a(SOLUTION_NUM);
      std::vector<s_info> solution_b(SOLUTION_NUM);
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
      int last_highest_score = -500000;
      int unchanged_turns = 0;
        // Genetic parameters
      double CRO_PROBA = 0.7;
      double MUT_PROBA = 0.1;
      int num_cro = 0;
      int num_mut = 0;
        // The base lists
      std::vector<s_info*>* solution_list[SOLUTION_NUM];
      std::vector<ignition::math::Vector3d> border_points;
      std::vector<std::vector<ignition::math::Vector3d>> border_points_type;
      ignition::math::Vector3d type;
      double min_angle = 0.2*M_PI;
      double max_angle = 0.4*M_PI;

      std::cout<<"Solution numbers: 50"<<std::endl;
      std::cout<<"Maximum generations: 50"<<std::endl;
      std::cout<<"Init cross proba: 0.7"<<std::endl;
      std::cout<<"Init mutation proba: 0.1"<<std::endl;
      std::cout<<"Adapt factor: 0.01"<<std::endl;
      std::cout<<"Min yaw angle interval: "<< min_angle <<" rad"<<std::endl;
      std::cout<<"Max yaw angle interval: "<< max_angle <<" rad"<<std::endl;


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
            s_info* one = new struct s_info;
            one->model_name = model->GetName();
            one->model_pose = model->WorldPose();
            solution_list[i]->push_back(one);
          }
        }
      }

      this->model->SetWorldPose(tmp_pose);        // move the plugin model to avoid interference

      // Get border points of the car model---------------------------------------------------------------------------------------------------------------
      border_points_type = get_border(world);
      border_points = border_points_type.at(0);
      int count_type = 0;
      for (int i=0; i<border_points_type.at(1).size(); i++){
        if ((int)border_points_type.at(1).at(i).X()==1) count_type++;
      }
      std::cout<<"front: "<< count_type<<std::endl;
      count_type = 0;
      for (int i=0; i<border_points_type.at(1).size(); i++){
        if ((int)border_points_type.at(1).at(i).Y()==1) count_type++;
      }
      std::cout<<"left: "<< count_type<<std::endl;
      count_type = 0;
      for (int i=0; i<border_points_type.at(1).size(); i++){
        if ((int)border_points_type.at(1).at(i).Y()==-1) count_type++;
      }
      std::cout<<"right: "<< count_type<<std::endl;
      count_type = 0;
      for (int i=0; i<border_points_type.at(1).size(); i++){
        if ((int)border_points_type.at(1).at(i).Z()==1) count_type++;
      }
      std::cout<<"top: "<< count_type<<std::endl;

      double min_x, max_x, min_y, max_y, max_z;
      min_x = border_points.at(0).X();
      max_x = min_x;
      min_y = border_points.at(0).Y();
      max_y = min_y;
      max_z = border_points.at(0).Z();
      for (auto point:border_points){             // also get the limit for some use
        if (point.X()<min_x) min_x = point.X();
        if (point.X()>max_x) max_x = point.X();
        if (point.Y()<min_y) min_y = point.Y();
        if (point.Y()>max_y) max_y = point.Y();
        if (point.Z()>max_z) max_z = point.Z();
      }
      if (1){
        std::cout<<"border points size: "<< border_points.size()<<std::endl;
        std::cout<<"solution numbers: "<< SOLUTION_NUM<<std::endl;
        std::cout<<"sensor number in solution: "<< solution_list[0]->size()<<std::endl;
        std::cout<<"max z: "<< max_z <<std::endl;
        std::cout<<"max x: "<< max_x <<std::endl;
        std::cout<<"max y: "<< max_y <<std::endl;
        std::cout<<"min x: "<< min_x <<std::endl;
        std::cout<<"min y: "<< min_y <<std::endl;
      }

      // Initialize and get the sensor info----------------------------------------------------------------------------------------------------------------
      ignition::math::Vector3d rot_vec;
      for (int i=0; i<SOLUTION_NUM; i++){
        solution = solution_list[i];
        while(1){
          for (auto s:*solution){
            //std::cout << "  -3 " << std::endl;
            // Set random values to the solution according to the restriction------------------------------------------------------------------------------
              // for coordinates
            while (1){
              rand_index = border_points.size() * rand()/RAND_MAX;
              if ((border_points.at(rand_index).Z()>=max_z*0.91 && border_points.at(rand_index).X()>=max_x*0.34)
              || (border_points.at(rand_index).Z()>=max_z*0.85 && border_points.at(rand_index).X()>=max_x*0.1
              && border_points.at(rand_index).Y() >=0.75 * max_y && border_points.at(rand_index).Y() <=0.75 * min_y)
              || (border_points.at(rand_index).Z()>=max_z*0.56 && border_points.at(rand_index).X()>=max_x*0.6)
              || (border_points.at(rand_index).Z()>=max_z*0.3 && border_points.at(rand_index).X()>=max_x*0.95
              && border_points.at(rand_index).Y() <= 0.7 * max_y && border_points.at(rand_index).Y() >= 0.7 * min_y)
              ) break;
            }
            s->model_pose.Pos() = border_points.at(rand_index);
            //std::cout << "  -2 " << std::endl;
              // for pose angle
            while(1){
              type = border_points_type[1].at(rand_index);
              if ((int)type.Z()){
                rot_vec.X() = 0;
                rot_vec.Y() = -M_PI/6 * rand()/RAND_MAX + M_PI/12;
                if(border_points.at(rand_index).Z()>=max_z*0.9){
                  if(border_points.at(rand_index).Y()>=0.6*max_y){
                    if (border_points.at(rand_index).Y()>=0.75*max_y){
                      rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/3;
                    }else{
                      rot_vec.Z() = M_PI/2 * rand()/RAND_MAX;
                    }
                  }else if(border_points.at(rand_index).Y()<=0.6*min_y){
                    if (border_points.at(rand_index).Y()<=0.75*min_y){
                      rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/3;
                    }else{
                      rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX;
                    }
                  }else{
                    rot_vec.Z() =  M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                  }
                }else{
                  if(border_points.at(rand_index).Y()>=0){
                    rot_vec.Z() = M_PI/3 * rand()/RAND_MAX;
                  }else{
                    rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX;
                  }
                }
              }else if((int)type.X()==-1){
                rot_vec.X() = 0;
                rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                rot_vec.Z() = M_PI/3*2 * rand()/RAND_MAX + M_PI/3*5;
              }else if((int)type.X()==1){
                rot_vec.X() = 0;
                rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                if (border_points.at(rand_index).Y()>0){
                  if (border_points.at(rand_index).Y()<=0.75*max_y){
                    rot_vec.Z() = M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                  }else{
                    rot_vec.Z() = M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                  }
                }else{
                  if (border_points.at(rand_index).Y()>=0.75*min_y){
                    rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                  }else{
                    rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                  }
                }
              }else if((int)type.Y()==1){
                if (border_points.at(rand_index).Y()<=0.75*max_y){
                  rot_vec.X() = 0;
                  rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                  rot_vec.Z() = M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                }else{
                  rot_vec.X() = 0;
                  rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                  rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/12;
                }
              }else if((int)type.Y()==-1){
                if (border_points.at(rand_index).Y()>=0.75*min_y){
                  rot_vec.X() = 0;
                  rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                  rot_vec.Z() = -M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                }else{
                  rot_vec.X() = 0;
                  rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                  rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/12;
                }
              }
              s->model_pose.Rot().Euler(rot_vec);
              //if (check_angle(world, this->model, s->model_pose)) break;
              break;
            }
          }
          if (check_angle(solution, sensor_model_num, min_angle, max_angle)) break;
          // break;
        }
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
      std::cout<<"Starting optimization process..."<<std::endl;
      while (turn <= MAX_TURN){
        
        std::cout<<"  For the turn "<<turn<<": "<<std::endl;
        turn++;

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
          if (i==SOLUTION_NUM/3) std::cout<<"    33.3%..."<<std::endl;
          if (i==SOLUTION_NUM/3*2) std::cout<<"    66.7%..."<<std::endl;
          for (int sz = 0; sz<=16; sz++){
            // std::cout<<sx<<std::endl;
            for (int sr = 1; sr<=36; sr++){
              // if (sr>=30 && sr<=50) continue;
              for (int stheta = -59; stheta<59; stheta++){
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
                      // continue;
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
          // cover_ratio = cover_ratio / (45*13*121);
          for (auto s1:*solution){
            for (auto s2:*solution){
              if(s1->model_name==s2->model_name) continue;
              if ((s1->model_pose.Pos()-s2->model_pose.Pos()).Length()>0.15) continue;
              score_solutions[i]-=2000;
            }
          }
          score_solutions[i]-=angle_punish(solution, sensor_model_num, min_angle, max_angle);
          // std::cout<<score_solutions[i]<<std::endl;
          // score_solutions[i] = (0.5+0.5*cover_ratio) * score_solutions[i];
        }
        // std::cout<<cover_ratio<<std::endl;
        std::cout<<"    100%..."<<std::endl;
        for (int i=0; i<SOLUTION_NUM; i++) score_sum+=score_solutions[i];
        get_solution_rank(score_solutions, score_rank);
        // for (int i=0; i<SOLUTION_NUM; i++) std::cout<<score_rank[i]<<" ";
        std::cout<<"    The score of the top 10 solutions this turn: "<<std::endl;
        for (int i=0; i<10; i++){
          for (int j=0; j<SOLUTION_NUM; j++){
            if (i+1==score_rank[j]){
              std::cout<<"      "<<j<<", "<<score_solutions[j]<<std::endl;
              break;
            }
          }
        }

        // Protect the elite ones
        for (int i=0; i<ELITE_NUM; i++){
          for (int j=0; j<SOLUTION_NUM; j++){
            if (i+1==score_rank[j]){
              for (int k=0; k<sensor_model_num; k++) solution_a[k] = (*solution_list[j]->at(k));
              rand_num_a = score_rank[j];
              score_rank[j] = score_rank[i];
              score_rank[i] = rand_num_a;
              solution = solution_list[j];
              rand_num_a = score_solutions[j];
              solution_list[j] = solution_list[i];
              score_solutions[j] = score_solutions[i];
              solution_list[i] = solution;
              score_solutions[i] = rand_num_a;
              tmp_score_solutions[tmp_solu_index] = score_solutions[j];
              tmp_solu_list[tmp_solu_index++] = solution_a;        
              break;
            }
          }
        }

        // Select proba
        sel_proba[0] = score_solutions[0]/score_sum;
        for (int i=0; i<ELITE_NUM; i++) sel_proba[i] = 0;
        for (int i=ELITE_NUM; i<SOLUTION_NUM; i++) sel_proba[i] = sel_proba[i-1]+score_solutions[i]/score_sum;

        // Update the parameters depending on scores--------------------------------------------------------------------------------------------------------
        if (last_highest_score!=-500000){
          if (abs(last_highest_score-score_solutions[0])<1){
            unchanged_turns++;
            MUT_PROBA+=(1-MUT_PROBA)*unchanged_turns*0.01 * (MAX_TURN-turn)/MAX_TURN;
            CRO_PROBA+=(1-CRO_PROBA)*unchanged_turns*0.01 * (MAX_TURN-turn)/MAX_TURN;
            if (MUT_PROBA>=1) MUT_PROBA=1;
            if (CRO_PROBA>=1) CRO_PROBA=1;
          }else if(last_highest_score<score_solutions[0]-1.1){
            MUT_PROBA = MUT_PROBA*0.3;
            CRO_PROBA = CRO_PROBA*0.8;
            if (MUT_PROBA<0.01) MUT_PROBA=0.01;
            if (CRO_PROBA<0.3) CRO_PROBA=0.3;
            unchanged_turns = 0;
            last_highest_score = score_solutions[0];
          }
        }else{
          last_highest_score = score_solutions[0];
        }
        int check_count = 0;

        // When converge locally, additional mutatant process:
        if (unchanged_turns>5){
          for (int mut_turn=0; mut_turn<5; mut_turn++){
            num_mut++;
            rand_num_a = floor((SOLUTION_NUM-0.0001)*rand()/double(RAND_MAX)); 
            for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[(int)rand_num_a]->at(j));
            }
            while (1){
              for (int i=0; i<sensor_model_num; i++){
                while (1){
                  rand_index = border_points.size() * rand()/RAND_MAX;
                  if ((border_points.at(rand_index).Z()>=max_z*0.91 && border_points.at(rand_index).X()>=max_x*0.34)
                  || (border_points.at(rand_index).Z()>=max_z*0.85 && border_points.at(rand_index).X()>=max_x*0.1
                  && border_points.at(rand_index).Y() >=0.75 * max_y && border_points.at(rand_index).Y() <=0.75 * min_y)
                  || (border_points.at(rand_index).Z()>=max_z*0.56 && border_points.at(rand_index).X()>=max_x*0.6)
                  || (border_points.at(rand_index).Z()>=max_z*0.3 && border_points.at(rand_index).X()>=max_x*0.95
                  && border_points.at(rand_index).Y() <= 0.7 * max_y && border_points.at(rand_index).Y() >= 0.7 * min_y)
                  ) break;
                }
                solution_a.at(i).model_pose.Pos() = border_points.at(rand_index);
                ignition::math::Vector3d rot_vec;
                while(1){
                  type = border_points_type[1].at(rand_index);
                  if ((int)type.Z()){
                    rot_vec.X() = 0;
                    rot_vec.Y() = -M_PI/6 * rand()/RAND_MAX + M_PI/12;
                    if(border_points.at(rand_index).Z()>=max_z*0.9){
                      if(border_points.at(rand_index).Y()>=0.6*max_y){
                        if (border_points.at(rand_index).Y()>=0.75*max_y){
                          rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/3;
                        }else{
                          rot_vec.Z() = M_PI/2 * rand()/RAND_MAX;
                        }
                      }else if(border_points.at(rand_index).Y()<=0.6*min_y){
                        if (border_points.at(rand_index).Y()<=0.75*min_y){
                          rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/3;
                        }else{
                          rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX;
                        }
                      }else{
                        rot_vec.Z() =  M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }else{
                      if(border_points.at(rand_index).Y()>=0){
                        rot_vec.Z() = M_PI/3 * rand()/RAND_MAX;
                      }else{
                        rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX;
                      }
                    }
                  }else if((int)type.X()==-1){
                    rot_vec.X() = 0;
                    rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                    rot_vec.Z() = M_PI/3*2 * rand()/RAND_MAX + M_PI/3*5;
                  }else if((int)type.X()==1){
                    rot_vec.X() = 0;
                    rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                    if (border_points.at(rand_index).Y()>0){
                      if (border_points.at(rand_index).Y()<=0.75*max_y){
                        rot_vec.Z() = M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }else{
                        rot_vec.Z() = M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }else{
                      if (border_points.at(rand_index).Y()>=0.75*min_y){
                        rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }else{
                        rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }
                  }else if((int)type.Y()==1){
                    if (border_points.at(rand_index).Y()<=0.75*max_y){
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                    }else{
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/12;
                    }
                  }else if((int)type.Y()==-1){
                    if (border_points.at(rand_index).Y()>=0.75*min_y){
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = -M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                    }else{
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/12;
                    }
                  }
                  solution_a.at(i).model_pose.Rot().Euler(rot_vec);
                  //if (check_angle(world, this->model, solution_a.at((int)rand_num_b).model_pose)) break;
                  break;
                }
                
                
              }
              if (check_angle(solution_a, sensor_model_num, min_angle, max_angle)) break;
              // break;
            }
            tmp_score_solutions[tmp_solu_index] = 1;
            tmp_solu_list[tmp_solu_index++] = solution_a;
          }
        }

        // Determine the next generation
        int repete_flag=0;
        int select_one = 0;
        int select_two = 0;
        while (tmp_solu_index<SOLUTION_NUM){

          // std::cout<<tmp_solu_index<<std::endl;
          // Do selection process
          // if (select_factor){
            // select_factor--;
          rand_num_a = (double)rand()/double(RAND_MAX);
          for (int i=0; i<SOLUTION_NUM; i++){
            if(rand_num_a<=sel_proba[i]){
              select_one = i;
              break;
            }
          }
          rand_num_a = (double)rand()/double(RAND_MAX);
          for (int i=0; i<SOLUTION_NUM; i++){
            if(rand_num_a<=sel_proba[i]){
              select_two = i;
              break;
            }
          }
          // }else{
          //   select_factor+=2;
          // }
          if (tmp_solu_index==SOLUTION_NUM) break;                      // Each time after a process check the solution number limit

 
          // Do cross process if possible
          rand_num_a = rand()/double(RAND_MAX);
          int cro_pos_b = 0;
          if (rand_num_a<=CRO_PROBA){
            // std::cout<<"    Cross happens"<<std::endl;
            rand_num_a = (SOLUTION_NUM-0.0001)*rand()/RAND_MAX;      // For equal proba we do some operations
            rand_num_b = (SOLUTION_NUM-0.0001)*rand()/RAND_MAX;
            for (int j=0; j<sensor_model_num; j++){
                // solution_a[j] = (*solution_list[(int)floor(rand_num_a)]->at(j));
                // solution_b[j] = (*solution_list[(int)floor(rand_num_b)]->at(j));
                solution_a[j] = (*solution_list[select_one]->at(j));
                solution_b[j] = (*solution_list[select_two]->at(j));
            }
            // std::cout<<" between "<<(int)floor(rand_num_a)+1<<" and "<<(int)floor(rand_num_b)+1<<std::endl;
            if (tmp_solu_index<=SOLUTION_NUM-2){
              repete_flag=0;
              
              tmp_score_solutions[tmp_solu_index] = rand_num_a*1000+rand_num_b*100+rand_num_b;
              tmp_score_solutions[tmp_solu_index+1] = rand_num_a*100+rand_num_b*1000+rand_num_b;
              for (int m=0; m<tmp_solu_index; m++){
                if ((int)tmp_score_solutions[m]==(int)tmp_score_solutions[tmp_solu_index]) repete_flag=1;
              }
              if (!repete_flag){
                num_cro++;
                num_cro++;
                rand_num_a = (int)floor((sensor_model_num-0.0001)*rand()/RAND_MAX);
                std::vector<s_info> solution_cross_a = solution_a;
                std::vector<s_info> solution_cross_b = solution_b;
                for (int cro_pos = rand_num_a; cro_pos<sensor_model_num; cro_pos++){
                  //cro_pos_b = get_cross_index(solution_a,solution_b,sensor_model_num,cro_pos);
                  //solution_cross_a.at(cro_pos) = solution_b.at(cro_pos_b);
                  //solution_cross_b.at(cro_pos_b) = solution_a.at(cro_pos);
                  solution_cross_a.at(cro_pos) = solution_b.at(cro_pos);
                  solution_cross_b.at(cro_pos) = solution_a.at(cro_pos);
                }
                tmp_solu_list[tmp_solu_index++] = solution_cross_a;
                tmp_solu_list[tmp_solu_index++] = solution_cross_b;
              }
            }else if(tmp_solu_index==SOLUTION_NUM-1){
              repete_flag=0;
              cro_pos_b = (int)floor((sensor_model_num-0.0001)*rand()/RAND_MAX);
              std::vector<s_info> solution_cross_a = solution_a;
              for (int cro_pos = 0; cro_pos<rand_num_a; cro_pos++) solution_cross_a.push_back(solution_a.at(cro_pos));
              for (int cro_pos = rand_num_a; cro_pos<sensor_model_num; cro_pos++) {
                //cro_pos_b = get_cross_index(solution_a,solution_b,sensor_model_num,cro_pos);
                //solution_cross_a.push_back(solution_b.at(cro_pos_b));
                solution_cross_a.push_back(solution_b.at(cro_pos));
              }
              tmp_score_solutions[tmp_solu_index] = rand_num_a*1000+rand_num_b*100+rand_num_b;
                for (int m=0; m<tmp_solu_index; m++){
                if ((int)tmp_score_solutions[m]==(int)tmp_score_solutions[tmp_solu_index]) repete_flag=1;
              }
              if (!repete_flag){
                num_cro++;
                tmp_solu_list[tmp_solu_index++] = solution_cross_a;
              }
            }
          }else{
            repete_flag=0;
            for (int m=0; m<tmp_solu_index; m++){
              if ((int)tmp_score_solutions[m]==(int)score_solutions[select_one]) repete_flag=1;
            }
            if (!repete_flag){
              // std::cout<<"    Select the "<<i+1<<" -th solution"<<std::endl;
              for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[select_one]->at(j));
              }
              tmp_score_solutions[tmp_solu_index] = score_solutions[select_one];
              tmp_solu_list[tmp_solu_index++] = solution_a;         // Insert the selected one into the new list for next generation
            }
            if (tmp_solu_index==SOLUTION_NUM) break;  
            repete_flag=0;
            for (int m=0; m<tmp_solu_index; m++){
              if ((int)tmp_score_solutions[m]==(int)score_solutions[select_two]) repete_flag=1;
            }
            if (!repete_flag){
              // std::cout<<"    Select the "<<i+1<<" -th solution"<<std::endl;
              for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[select_two]->at(j));
              }
              tmp_score_solutions[tmp_solu_index] = score_solutions[select_two];
              tmp_solu_list[tmp_solu_index++] = solution_a;         // Insert the selected one into the new list for next generation
            }
          }
          // std::cout<<"    Cross over"<<std::endl;
          if (tmp_solu_index==SOLUTION_NUM) break;                      // Each time after a process check the solution number limit


          // Do mutation process
          check_count = 0;
          rand_num_a = rand()/double(RAND_MAX);
          if (rand_num_a<=MUT_PROBA){
            // std::cout<<"    Mut happens"<<std::endl;
            num_mut++;
            rand_num_a = floor((SOLUTION_NUM-0.0001)*rand()/double(RAND_MAX));      // Choose the mutant solution
            for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[(int)rand_num_a]->at(j));
            }
            rand_num_a = floor((sensor_model_num-0.0001)*rand()/double(RAND_MAX));   // Choose how many sensors to mutate(may repete)
            
            for (int i=0; i<(int)rand_num_a; i++){
              check_count = 0;
              rand_num_b = floor((sensor_model_num-0.0001)*rand()/double(RAND_MAX));   // Choose which sensor in the solution to mutate
              while (1){
                check_count++;
                while (1){
                  rand_index = border_points.size() * rand()/RAND_MAX;
                  if ((border_points.at(rand_index).Z()>=max_z*0.91 && border_points.at(rand_index).X()>=max_x*0.34)
                  || (border_points.at(rand_index).Z()>=max_z*0.85 && border_points.at(rand_index).X()>=max_x*0.1
                  && border_points.at(rand_index).Y() >=0.75 * max_y && border_points.at(rand_index).Y() <=0.75 * min_y)
                  || (border_points.at(rand_index).Z()>=max_z*0.56 && border_points.at(rand_index).X()>=max_x*0.6)
                  || (border_points.at(rand_index).Z()>=max_z*0.3 && border_points.at(rand_index).X()>=max_x*0.95
                  && border_points.at(rand_index).Y() <= 0.7 * max_y && border_points.at(rand_index).Y() >= 0.7 * min_y)
                  ) break;
                }
                solution_a.at((int)rand_num_b).model_pose.Pos() = border_points.at(rand_index);
                ignition::math::Vector3d rot_vec;
                while(1){
                  type = border_points_type[1].at(rand_index);
                  if ((int)type.Z()){
                    rot_vec.X() = 0;
                    rot_vec.Y() = -M_PI/6 * rand()/RAND_MAX + M_PI/12;
                    if(border_points.at(rand_index).Z()>=max_z*0.9){
                      if(border_points.at(rand_index).Y()>=0.6*max_y){
                        if (border_points.at(rand_index).Y()>=0.75*max_y){
                          rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/3;
                        }else{
                          rot_vec.Z() = M_PI/2 * rand()/RAND_MAX;
                        }
                      }else if(border_points.at(rand_index).Y()<=0.6*min_y){
                        if (border_points.at(rand_index).Y()<=0.75*min_y){
                          rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/3;
                        }else{
                          rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX;
                        }
                      }else{
                        rot_vec.Z() =  M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }else{
                      if(border_points.at(rand_index).Y()>=0){
                        rot_vec.Z() = M_PI/3 * rand()/RAND_MAX;
                      }else{
                        rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX;
                      }
                    }
                  }else if((int)type.X()==-1){
                    rot_vec.X() = 0;
                    rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                    rot_vec.Z() = M_PI/3*2 * rand()/RAND_MAX + M_PI/3*5;
                  }else if((int)type.X()==1){
                    rot_vec.X() = 0;
                    rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                    if (border_points.at(rand_index).Y()>0){
                      if (border_points.at(rand_index).Y()<=0.75*max_y){
                        rot_vec.Z() = M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }else{
                        rot_vec.Z() = M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }else{
                      if (border_points.at(rand_index).Y()>=0.75*min_y){
                        rot_vec.Z() = -M_PI/3 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }else{
                        rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX  + M_PI/3*border_points.at(rand_index).Y()/(max_y-min_y)/2;
                      }
                    }
                  }else if((int)type.Y()==1){
                    if (border_points.at(rand_index).Y()<=0.75*max_y){
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                    }else{
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = M_PI/2 * rand()/RAND_MAX + M_PI/12;
                    }
                  }else if((int)type.Y()==-1){
                    if (border_points.at(rand_index).Y()>=0.75*min_y){
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = -M_PI/18 * rand()/RAND_MAX + M_PI/6 * border_points.at(rand_index).Y()/(max_y-min_y)/2;
                    }else{
                      rot_vec.X() = 0;
                      rot_vec.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
                      rot_vec.Z() = -M_PI/2 * rand()/RAND_MAX - M_PI/12;
                    }
                  }
                  solution_a.at((int)rand_num_b).model_pose.Rot().Euler(rot_vec);
                  //if (check_angle(world, this->model, solution_a.at((int)rand_num_b).model_pose)) break;
                  break;
                }
                if (check_angle(solution_a, sensor_model_num, min_angle, max_angle)||check_count==1000) break;    // Avoid infinite loop
                // break;
                
              }
            }
            tmp_score_solutions[tmp_solu_index] = 1;
            tmp_solu_list[tmp_solu_index++] = solution_a;
          }
        }
        std::cout<<"In this turn, cross has happened "<< num_cro << " times, mutation has happened "<< num_mut <<" times."<<std::endl;
        num_cro = 0;
        num_mut = 0;

        // Update the sensor parameter changes for these mutations------------------------------------------------------------------------------------------
        for (int i=0; i<SOLUTION_NUM; i++){
          solution = solution_list[i];
          for (int j=0; j<sensor_model_num; j++){
            solution->at(j)->model_pose = tmp_solu_list[i][j].model_pose;
            solution->at(j)->sen = tmp_solu_list[i][j].sen;
            solution->at(j)->near = tmp_solu_list[i][j].near;
            solution->at(j)->far = tmp_solu_list[i][j].far;
            
            if (tmp_solu_list[i][j].type=="camera"){
              sensors::CameraSensorPtr camera = std::dynamic_pointer_cast<sensors::CameraSensor>(solution->at(j)->sen);
              rendering::CameraPtr cam = camera->Camera();

              // Get the camera parameters
              fov = cam->HFOV().Radian();
              aspectRatio = cam->ImageWidth() / camera->ImageHeight();

              // Compute the camera pose and direction             
              std::vector<ignition::math::Vector3d> normals = get_normals(fov, aspectRatio, solution->at(j)->near, solution->at(j)->far, solution->at(j)->model_pose);
              solution->at(j)->nor1 = normals[0];
              solution->at(j)->nor2 = normals[1];
              solution->at(j)->nor3 = normals[2];
              solution->at(j)->nor4 = normals[3];
            }else if(tmp_solu_list[i][j].type=="ray"){
              rs = std::dynamic_pointer_cast<sensors::RaySensor>(solution->at(j)->sen);
              sensorPose = tmp_solu_list[i][j].model_pose;
              sensorPose.Pos()+=rs->Pose().Pos();
              solution->at(j)->model_pose = sensorPose;
              // Get the direction vector
              solution->at(j)->nor1 = sensorPose.Rot().RotateVector(ignition::math::Vector3d(1,0,0));
            }
          }
        }
      std::cout<<std::endl;
      }
      
      // Print the results and update the poses of the sensors in gazebo-----------------------------------------------------------------------------------
      std::cout<<"The final score of the top 10 solutions: "<<std::endl;
      get_solution_rank(score_solutions, score_rank);
      for (int i=0; i<10; i++){
        for (int j=0; j<SOLUTION_NUM; j++){
          if (i+1==score_rank[j]){
            std::cout<<j<<", "<<score_solutions[j]<<std::endl;
            if (i>ELITE_NUM-1) break;
            std::cout<<"FOR THE "<<j<<" -th, which is in the top 3 solutions:"<<std::endl;
            solution = solution_list[j];
            for (auto s:*solution){
              for (auto model:models){
                if (model->GetName()==s->model_name){
                  model->SetWorldPose(s->model_pose);
                }
              }
              std::cout<<"  "<<s->model_name<<": "<<s->model_pose<<std::endl;
            }
            break;
          }
        }
      }
      for (int i=0; i<10; i++){
        for (int j=0; j<SOLUTION_NUM; j++){
          if (i+1==score_rank[j]){
            if (i>0) break;
            solution = solution_list[j];
            for (auto s:*solution){
              for (auto model:models){
                if (model->GetName()==s->model_name){
                  model->SetWorldPose(s->model_pose);
                }
              }
            }
            
            cover_ratio = 0;
            double redun_ratio = 0;
            int space_points = 0;
            int redun_flag = 0;
            ignition::math::Quaterniond qtheta;
            for (int sz = 0; sz<=16; sz++){
              // std::cout<<sx<<std::endl;
              for (int sr = 1; sr<=36; sr++){
                // if (sr>=30 && sr<=50) continue;
                for (int stheta = -59; stheta<59; stheta++){
                  space_points++;
                  redun_flag = 0;
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
                        score_solutions[i]+=1/power(2,multi_count);
                        redun_ratio++;
                      }else{
                        score_solutions[i]++;
                        cover_ratio++;
                      }
                      // s->score++;
                      multi_count++;
                    }else if(s->type=="ray"){
                      if (acos(sVec.Dot(s->nor1)/sVec.Length()) > s->lidar_fov.Radian()) continue;
                      // Redundancy check
                      if (multi_count){
                        //score_solutions[i]+=1/power(2,multi_count);
                        if (!redun_flag) redun_ratio++;
                        redun_flag=1;
                      }else{
                        if (stheta<4.1) score_solutions[i]+=0.1;
                        //score_solutions[i]++;
                        cover_ratio++;
                      }
                      multi_count++;
                    }
                  }
                }
              }
            }
            redun_ratio = redun_ratio / cover_ratio;
            cover_ratio = cover_ratio / space_points;
            std::cout<<" Top solution's cover ratio: "<<cover_ratio<<std::endl;
            std::cout<<" Top solution's redundancy ratio: "<<redun_ratio<<std::endl;
            break;
          }
        }
      }

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

  GZ_REGISTER_MODEL_PLUGIN(sensor_info)
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

ignition::math::Vector3d gen_random_angle(ignition::math::Vector3d type){
  ignition::math::Vector3d res;
  std::srand(std::time(nullptr));
  if (type.Z()){
    res.X() = 0;
    res.Y() = M_PI/6 * rand()/RAND_MAX;
    res.Z() = M_PI * rand()/RAND_MAX - M_PI/2;
  }else if(type.X()==-1){
    res.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
    res.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
    res.Z() = M_PI/3*2 * rand()/RAND_MAX + M_PI/3*5;
  }else if(type.X()==1){
    res.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
    res.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
    res.Z() = M_PI/3*2 * rand()/RAND_MAX - M_PI/3;
  }else if(type.Y()==1){
    res.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
    res.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
    res.Z() = M_PI/3*2 * rand()/RAND_MAX;
  }else if(type.Y()==-1){
    res.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
    res.Y() = M_PI/6 * rand()/RAND_MAX - M_PI/12;
    res.Z() = -M_PI/3*2 * rand()/RAND_MAX;
  }
  return res;
}

int angle_punish(std::vector<s_info*>* solution, int num_sen, double mina, double maxa){
  if (num_sen==1) return 0;
  double min_yaw = 10;
  double yaw_list[20];
  double ranked_yaw_list[20];
  int index = 0;
  int min_index = 0;
  int turn = 1;
  memset(yaw_list, 10.0, sizeof(yaw_list));
  memset(ranked_yaw_list, 10.0, sizeof(ranked_yaw_list));
  for (auto s:*solution) yaw_list[index++] = s->model_pose.Yaw();
  index = 0;
  while (turn<=num_sen){
    for (int i=0; i<num_sen; i++){
      if (yaw_list[i]<min_yaw){
        min_index = i;
        min_yaw = yaw_list[i];
      }
    }
    // std::cout<<"Index "<<max_index<<" with rank "<<turn<<std::endl;
    ranked_yaw_list[index++] = min_yaw;
    yaw_list[min_index] = 10.0;
    min_yaw = 10.0;
    turn++;
  }
  for (int i=1; i<index; i++){
    // std::cout<<ranked_yaw_list[i]-ranked_yaw_list[i-1]<<std::endl;
    if (ranked_yaw_list[i]-ranked_yaw_list[i-1] < mina ||
    ranked_yaw_list[i]-ranked_yaw_list[i-1] > maxa) return 5000;    // Make sure all the sensors will have the seperate yaw to ensure cover ration
  }
  return 0;
}

bool check_angle(std::vector<s_info*>* solution, int num_sen, double mina, double maxa){
  if (num_sen==1) return true;
  double min_yaw = 10;
  double yaw_list[20];
  double ranked_yaw_list[20];
  int index = 0;
  int min_index = 0;
  int turn = 1;
  memset(yaw_list, 10.0, sizeof(yaw_list));
  memset(ranked_yaw_list, 10.0, sizeof(ranked_yaw_list));
  for (auto s:*solution) yaw_list[index++] = s->model_pose.Yaw();
  index = 0;
  while (turn<=num_sen){
    for (int i=0; i<num_sen; i++){
      if (yaw_list[i]<min_yaw){
        min_index = i;
        min_yaw = yaw_list[i];
      }
    }
    // std::cout<<"Index "<<max_index<<" with rank "<<turn<<std::endl;
    ranked_yaw_list[index++] = min_yaw;
    yaw_list[min_index] = 10.0;
    min_yaw = 10.0;
    turn++;
  }
  for (int i=1; i<index; i++){
    // std::cout<<ranked_yaw_list[i]-ranked_yaw_list[i-1]<<std::endl;
    if (ranked_yaw_list[i]-ranked_yaw_list[i-1] < mina
    || ranked_yaw_list[i]-ranked_yaw_list[i-1] > maxa) return false;    // Make sure all the sensors will have the seperate yaw to ensure cover ration
  }
  
  return true;
}

bool check_angle(std::vector<s_info> solution, int num_sen, double mina, double maxa){
  if (num_sen==1) return true;
  double min_yaw = 10;
  double yaw_list[20];
  double ranked_yaw_list[20];
  int index = 0;
  int min_index = 0;
  int turn = 1;
  memset(yaw_list, 10.0, sizeof(yaw_list));
  memset(ranked_yaw_list, 10.0, sizeof(ranked_yaw_list));
  for (auto s:solution) yaw_list[index++] = s.model_pose.Yaw();
  index = 0;
  while (turn<=num_sen){
    for (int i=0; i<num_sen; i++){
      if (yaw_list[i]<min_yaw){
        min_index = i;
        min_yaw = yaw_list[i];
      }
    }
    ranked_yaw_list[index++] = min_yaw;
    yaw_list[min_index] = 10.0;
    min_yaw = 10.0;
    turn++;
  }
  for (int i=1; i<index; i++){
    //std::cout<<ranked_yaw_list[i]-ranked_yaw_list[i-1]<<std::endl;
    if (ranked_yaw_list[i]-ranked_yaw_list[i-1] < mina
    || ranked_yaw_list[i]-ranked_yaw_list[i-1] > maxa) return false;    // Make sure all the sensors will have the seperate yaw to ensure cover ration
  }
  return true;
}

int get_cross_index(std::vector<s_info> solution_a,std::vector<s_info> solution_b,int num_sen, int cro_pos){
  if (num_sen==1) return true;
  double min_yaw = 10;
  double yaw_list[20];
  int ranked_yaw_list_a[20];
  int ranked_yaw_list_b[20];
  int index = 0;
  int min_index = 0;
  int turn = 0;
  memset(ranked_yaw_list_a, 0, sizeof(ranked_yaw_list_a));
  memset(ranked_yaw_list_b, 0, sizeof(ranked_yaw_list_b));
  memset(yaw_list, 10.0, sizeof(yaw_list));
  for (auto s:solution_a) yaw_list[index++] = s.model_pose.Yaw();
  while (turn<num_sen){
    for (int i=0; i<num_sen; i++){
      if (yaw_list[i]<min_yaw){
        min_index = i;
        min_yaw = yaw_list[i];
      }
    }
    ranked_yaw_list_a[min_index] = turn;
    yaw_list[min_index] = 10.0;
    min_yaw = 10.0;
    turn++;
  }
  turn = 0;
  index = 0;
  memset(yaw_list, 10.0, sizeof(yaw_list));
  for (auto s:solution_b) yaw_list[index++] = s.model_pose.Yaw();
  while (turn<num_sen){
    for (int i=0; i<num_sen; i++){
      if (yaw_list[i]<min_yaw){
        min_index = i;
        min_yaw = yaw_list[i];
      }
    }
    ranked_yaw_list_b[min_index] = turn;
    yaw_list[min_index] = 10.0;
    min_yaw = 10.0;
    turn++;
  }
  for (int i=0; i<num_sen; i++){
    if (ranked_yaw_list_b[i] == ranked_yaw_list_a[cro_pos]) return i;
  }
  std::cout<<"Error when cross, but continue"<<std::endl;
  return cro_pos;
}

std::vector<std::vector<ignition::math::Vector3d>> get_border(gazebo::physics::WorldPtr world){
  std::vector<std::vector<ignition::math::Vector3d>> R;
  std::vector<ignition::math::Vector3d> res;
  std::vector<ignition::math::Vector3d> res_type;
  ignition::math::Vector3d point;
  double z = 0;
  double x = 0;
  double y = 0;
  world->Physics()->InitForThread();
  gazebo::physics::RayShapePtr distray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
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
      y+=0.05;
      if (dist!=100 && dist!=1000 && dist>0){
        point.X()=x;
        point.Y()=y;
        point.Z()=z-dist+0.006;
        res.push_back(point);
        point.X()=0;
        point.Y()=0;
        point.Z()=1;
        res_type.push_back(point);
      }
    }
    y=-10;
    x+=0.05;
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
      y+=0.05;
      if (dist!=1000 && dist>0){
        point.X()=x-dist+0.006;
        point.Y()=y;
        point.Z()=z;
        res.push_back(point);
        point.X()=1;
        point.Y()=0;
        point.Z()=0;
        res_type.push_back(point);
      }
    }
    y=-10;
    z+=0.05;
  }
  if (GET_BEHIND){
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
        y+=0.05;
        if (dist!=1000 && dist>0){
          point.X()=x+dist-0.006;
          point.Y()=y;
          point.Z()=z;
          res.push_back(point);
          point.X()=-1;
          point.Y()=0;
          point.Z()=0;
          res_type.push_back(point);
        }
      }
      y=-10;
      z+=0.05;
    }
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
      x+=0.05;
      if (dist!=1000 && dist>0){
        point.X()=x;
        point.Y()=y-dist+0.006;
        point.Z()=z;
        res.push_back(point);
        point.X()=0;
        point.Y()=1;
        point.Z()=0;
        res_type.push_back(point);
      }
    }
    x=-10;
    z+=0.05;
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
      x+=0.05;
      if (dist!=1000 && dist>0){
        point.X()=x;
        point.Y()=y+dist-0.006;
        point.Z()=z;
        res.push_back(point);
        point.X()=0;
        point.Y()=-1;
        point.Z()=0;
        res_type.push_back(point);
      }
    }
    x=-10;
    z+=0.05;
  }
  R.push_back(res);
  R.push_back(res_type);
  return R;
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

void get_solution_rank(double* score_list, int* rank){
  double max_score = -100000;
  int max_index = 0;
  double tmp_list[SOLUTION_NUM];
  int turn = 1;
  for (int i=0; i<SOLUTION_NUM; i++) tmp_list[i] = score_list[i];

  while (turn<=SOLUTION_NUM){
    for (int i=0; i<SOLUTION_NUM; i++){
      if (tmp_list[i]>max_score){
        max_index = i;
        max_score = tmp_list[i];
      }
    }
    // std::cout<<"Index "<<max_index<<" with rank "<<turn<<std::endl;
    rank[max_index] = turn;
    tmp_list[max_index] = -100001;
    max_score = -100000;
    turn++;
  }
}