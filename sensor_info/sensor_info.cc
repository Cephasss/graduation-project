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

#define SOLUTION_NUM 70
#define DEBUG false                  // Set true to print some intermediate infos
#define MAX_TURN 30

#define CRO_PROBA 0.7
#define MUT_PROBA 0.01
#define ELITE_NUM 1

struct s_info
{
  std::string model_name;
  std::string type;
  gazebo::physics::ModelPtr mdl;
  gazebo::sensors::SensorPtr sen;
  ignition::math::Pose3d model_pose;
  ignition::math::Vector3d nor1;
  ignition::math::Vector3d nor2; 
  ignition::math::Vector3d nor3;
  ignition::math::Vector3d nor4;
  int near;
  int far;
  int score;
};

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

bool check_angle(gazebo::physics::WorldPtr world, gazebo::physics::ModelPtr model, ignition::math::Pose3d camera_pose){
  ignition::math::Vector3d point;
  world->Physics()->InitForThread();
  gazebo::physics::RayShapePtr distray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    model->GetWorld()->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
  ignition::math::Vector3d start = camera_pose.Pos();
  ignition::math::Vector3d end = start;
  ignition::math::Quaterniond cameraRot = camera_pose.Rot();
  ignition::math::Vector3d cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));
  std::string near_e;
  double dist = 0;
  cameraDir.Normalize();
  end += cameraDir;
  distray->SetPoints(start, end);
  distray->GetIntersection(dist, near_e);
  // std::cout<<near_e<<" "<<dist<<std::endl;
  if (dist<1) return false;
  return true;
}

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
      y+=0.4;
      if (dist!=100 && dist!=1000 && dist>0){
        point.X()=x;
        point.Y()=y;
        point.Z()=z-dist+0.003;
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
        point.X()=x-dist+0.003;
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
        point.X()=x+dist-0.003;
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
        point.Y()=y-dist+0.003;
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
        point.Y()=y+dist-0.003;
        point.Z()=z;
        res.push_back(point);
      }
    }
    x=-10;
    z+=0.1;
  }
  return res;
}

std::vector<ignition::math::Vector3d> get_normals(double fov, double aspectRatio, double nearClip, double farClip, ignition::math::Pose3d camera_pose){
  std::vector<ignition::math::Vector3d> normals;
  ignition::math::Vector3d cameraPos = camera_pose.Pos();
  ignition::math::Quaterniond cameraRot = camera_pose.Rot();
  ignition::math::Vector3d cameraDir = cameraRot.RotateVector(ignition::math::Vector3d(1,0,0));

  // Compute the camera frustum vectors
  double halfFovTan = tan(fov / 2.0);
  ignition::math::Vector3d leftVec = cameraRot.RotateVector(ignition::math::Vector3d(0,1,0));
  ignition::math::Vector3d topVec = cameraDir.Cross(leftVec);
  topVec.Normalize();
  leftVec = topVec.Cross(cameraDir);
  leftVec.Normalize();
  if (DEBUG) std::cout<<"  Camera pose: "<<camera_pose<<std::endl<<std::endl;
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
  double max_score = 0;
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
    tmp_list[max_index] = -1;
    max_score = 0;
    turn++;
  }
}

namespace gazebo
{
  class sensor_info : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    { 
      // Variables
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
      ignition::math::Pose3d cameraPose;
      std::vector<ignition::math::Vector3d> normals;
      int rand_index = 0;
      double rand_num_a = 0;
      double rand_num_b = 0;
      int select_factor = 2;
      ignition::math::Vector3d point;
      ignition::math::Pose3d tmp_pose;
      std::vector<s_info*>* solution;
      std::vector<s_info> solution_a(SOLUTION_NUM);
      std::vector<s_info> solution_b(SOLUTION_NUM);
      std::srand(std::time(nullptr));
        // For optimization process
      int turn = 1;
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


      // Get sensor pointers
      auto sm = sensors::SensorManager::Instance();
      std::vector< std::string > st;
      auto sensor_list = sm->GetSensors();

      this->model = _parent;

      // Get the world and the number of sensor models (not sensors because a model may contain several sensors)
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
            one->mdl = model;
            one->model_pose = model->WorldPose();
            solution_list[i]->push_back(one);
          }
        }
      }

      this->model->SetWorldPose(tmp_pose);        // move the plugin model to avoid interference

      // Get border points of the car model
      border_points = get_border(world, this->model);
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
      if (DEBUG){
        std::cout<<"border points size: "<< border_points.size()<<std::endl;
        std::cout<<"solution numbers: "<< SOLUTION_NUM<<std::endl;
        std::cout<<"sensor number in solution: "<< solution_list[0]->size()<<std::endl;
      }

      // Set random values to the solution according to the restriction
      ignition::math::Vector3d rot_vec;
      for (int i=0; i<SOLUTION_NUM; i++){
        solution = solution_list[i];
        for (auto s:*solution){
          while (1){
            rand_index = border_points.size() * rand()/RAND_MAX;
            if (border_points.at(rand_index).Z()>max_z*0.7 && border_points.at(rand_index).X()>max_x-0.4*(max_x-min_x)) break;
          }
          s->model_pose.Pos() = border_points.at(rand_index);
          while(1){
            rot_vec.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
            rot_vec.Y() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
            rot_vec.Z() = M_PI * rand()/RAND_MAX - M_PI/2;
            s->model_pose.Rot().Euler(rot_vec);
            if (check_angle(world, this->model, s->model_pose)) break;
          }
          for (auto sen : sensor_list){
            if (sen->Type()=="camera"){
              camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sen);

              // Check which sensor matches the model, then input the parameters of the sensor
              // (Building pyramid model for the camera, then calculate the normals of 4 planes)
              auto links = s->mdl->GetLinks();
              for (auto link : links){
                if (s->model_name + "::" + link->GetName()==camera->ParentName()){
                  flag = 1;
                  break;
                }
              }
              if (flag){
                flag = 0;
                s->sen = sen;
                s->type = "camera";
                s->score = 0;
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
                cameraPose = s->model_pose;
                cameraPose.Pos()+=camera->Pose().Pos();
                s->model_pose = cameraPose;
                normals = get_normals(fov, aspectRatio, nearClip, farClip, cameraPose);
                s->nor1 = normals[0];
                s->nor2 = normals[1];
                s->nor3 = normals[2];
                s->nor4 = normals[3];
                
              }
            }else if(sen->Type()=="ray"){
              continue; //TO DO
            }
          }
        }
      }

      // Print values for debug
      if (DEBUG){
        for (int i=0; i<SOLUTION_NUM; i++){
          solution = solution_list[i];
          std::cout<<"FOR THE "<<i<<" -th:"<<std::endl;
          for (auto s:*solution){
            std::cout<<s->model_name<<": "<<s->model_pose<<std::endl;
          }
        }
      }
      std::cout<<std::endl;



      if (0){
        return;
      }
      
      // The optimization process--------------------------------------------------
      ignition::math::Vector3d sVec;
      std::cout<<"Starting optimization process..."<<std::endl;
      while (turn <= MAX_TURN){
        
        std::cout<<"  For the turn "<<turn<<": "<<std::endl;
        turn++;

        // Initiate the varaibles
        score_sum = 0;
        tmp_solu_index = 0;
        memset(score_solutions, 0, sizeof(score_solutions));
        memset(score_rank, 0, sizeof(score_rank));
        memset(sel_proba, 0, sizeof(sel_proba));
        memset(tmp_score_solutions, 0, sizeof(tmp_score_solutions));
        memset(tmp_sel_proba, 0, sizeof(tmp_sel_proba));

        // Calculate the score for each solution in the list and store them in a list
        for (int i=0; i<SOLUTION_NUM; i++){
          solution = solution_list[i];
          if (i==SOLUTION_NUM/3) std::cout<<"    33.3%..."<<std::endl;
          if (i==SOLUTION_NUM/3*2) std::cout<<"    66.7%..."<<std::endl;
          for (int sx = -100; sx<100; sx++){
            for (int sy = -100; sy<100; sy++){
              for (int sz = 0; sz<10; sz++){
                
                sVec.X() = sx;
                sVec.Y() = sy;
                sVec.Z() = sz;
                multi_count = 0;
                for (auto s:*solution){
                  sVec -= s->model_pose.Pos();
                  if (s->type=="camera"){
                    if (sVec.Length()<s->near || sVec.Length()>s->far) continue;
                    if (sVec.Dot(s->nor1)<=0) continue;
                    if (sVec.Dot(s->nor2)<=0) continue;
                    if (sVec.Dot(s->nor3)<=0) continue;
                    if (sVec.Dot(s->nor4)<=0) continue;
                    // Skip if too close or inside the car
                    if (sz<max_z && sx>min_x && sx<max_x && sy>min_y && sy<max_y){
                      
                      continue;
                    } 
                    if (multi_count){
                      score_solutions[i]+=1/power(2,multi_count);
                    }else{
                      score_solutions[i]++;
                    }
                    // s->score++;
                    multi_count++;
                  }                               // TODO for ray sensor
                }
                //solution_list[i] = solution;
              }
            }
          }
          for (int sx = min_x; sx<max_x; sx++){
            for (int sy = min_y; sy<max_y; sy++){
              for (int sz = 0; sz<max_z; sz++){
                for (int bias = 1; bias<6; bias++){
                  sVec.X() = sx+bias/5;
                  sVec.Y() = sy+bias/5;
                  sVec.Z() = sz+bias/5;
                  for (auto s:*solution){
                    sVec -= s->model_pose.Pos();
                    if (s->type=="camera"){
                      if (sVec.Length()<s->near || sVec.Length()>s->far) continue;
                      if (sVec.Dot(s->nor1)<=0) continue;
                      if (sVec.Dot(s->nor2)<=0) continue;
                      if (sVec.Dot(s->nor3)<=0) continue;
                      if (sVec.Dot(s->nor4)<=0) continue;
                      score_solutions[i]-=1000;
                    }
                  }
                }
              }
            }
          }
        }

        std::cout<<"    100%..."<<std::endl;
        for (int i=0; i<SOLUTION_NUM; i++) score_sum+=score_solutions[i];
        get_solution_rank(score_solutions, score_rank);
        // for (int i=0; i<SOLUTION_NUM; i++) std::cout<<score_rank[i]<<" ";
        std::cout<<std::endl<<"    The score of the top 10 solutions this turn: "<<std::endl;
        for (int i=0; i<10; i++){
          for (int j=0; j<SOLUTION_NUM; j++){
            if (i+1==score_rank[j]){
              std::cout<<"      "<<j<<", "<<score_solutions[j]<<std::endl;
              break;
            }
          }
        }
        for (int i=0; i<10; i++){
          for (int j=0; j<SOLUTION_NUM; j++){
            if (i+1==score_rank[j]){
              if (i>=ELITE_NUM) break;
              for (int k=0; k<sensor_model_num; k++){
                solution_a[k] = (*solution_list[j]->at(k));
              }
              rand_num_a = score_rank[j];
              score_rank[j] = score_rank[i];
              score_rank[i] = rand_num_a;
              solution = solution_list[j];
              rand_num_a = score_solutions[j];
              solution_list[j] = solution_list[i];
              score_solutions[j] = score_solutions[i];
              solution_list[i] = solution;
              score_solutions[i] = rand_num_a;
              tmp_solu_list[tmp_solu_index++] = solution_a;        // Protect the elite ones
              break;
            }
          }
        }
        sel_proba[0] = score_solutions[0]/score_sum;
        for (int i=1; i<SOLUTION_NUM; i++) sel_proba[i] = sel_proba[i-1]+score_solutions[i]/score_sum;

        // Determine the next generation
        while (tmp_solu_index<SOLUTION_NUM){

          // Do selection process
          if (select_factor){
            select_factor--;
            rand_num_a = rand()/double(RAND_MAX);
            for (int i=ELITE_NUM; i<SOLUTION_NUM; i++){
              if(rand_num_a<=sel_proba[i]){
                // std::cout<<"    Select the "<<i+1<<" -th solution"<<std::endl;
                for (int j=0; j<sensor_model_num; j++){
                  solution_a[j] = (*solution_list[i]->at(j));
                }
                tmp_solu_list[tmp_solu_index++] = solution_a;         // Insert the selected one into the new list for next generation
                break;
              }
            }
          }else{
            select_factor+=2;
          }
          if (tmp_solu_index==SOLUTION_NUM) break;                      // Each time after a process check the solution number limit

 
          // Do cross process if possible
          rand_num_a = rand()/double(RAND_MAX);
          if (rand_num_a<=CRO_PROBA){
            // std::cout<<"    Cross happens";
            rand_num_a = (SOLUTION_NUM-0.0001)*rand()/RAND_MAX;      // For equal proba we do some operations
            rand_num_b = (SOLUTION_NUM-0.0001)*rand()/RAND_MAX;
            for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[(int)floor(rand_num_a)]->at(j));
                solution_b[j] = (*solution_list[(int)floor(rand_num_b)]->at(j));
            }
            // std::cout<<" between "<<(int)floor(rand_num_a)+1<<" and "<<(int)floor(rand_num_b)+1<<std::endl;
            if (tmp_solu_index<=SOLUTION_NUM-2){
              rand_num_a = (int)floor((sensor_model_num-0.0001)*rand()/RAND_MAX);
              std::vector<s_info> solution_cross_a = solution_a;
              std::vector<s_info> solution_cross_b = solution_b;
              for (int cro_pos = rand_num_a; cro_pos<sensor_model_num; cro_pos++){
                solution_cross_a.at(cro_pos) = solution_b.at(cro_pos);
                solution_cross_b.at(cro_pos) = solution_a.at(cro_pos);
              }
              tmp_solu_list[tmp_solu_index++] = solution_cross_a;
              tmp_solu_list[tmp_solu_index++] = solution_cross_b;
            }else if(tmp_solu_index==SOLUTION_NUM-1){
              rand_num_a = (int)floor((sensor_model_num-0.0001)*rand()/RAND_MAX);
              std::vector<s_info> solution_cross_a = solution_a;
              for (int cro_pos = 0; cro_pos<rand_num_a; cro_pos++) solution_cross_a.push_back(solution_a.at(cro_pos));
              for (int cro_pos = rand_num_a; cro_pos<sensor_model_num; cro_pos++) solution_cross_a.push_back(solution_b.at(cro_pos));
              tmp_solu_list[tmp_solu_index++] = solution_cross_a;
            }
          }
          if (tmp_solu_index==SOLUTION_NUM) break;                      // Each time after a process check the solution number limit


          // Do mutation process
          rand_num_a = rand()/double(RAND_MAX);
          if (rand_num_a<=MUT_PROBA){
            std::cout<<"    Wow, mutation happens"<<std::endl;
            rand_num_a = floor((SOLUTION_NUM-0.0001)*rand()/double(RAND_MAX));      // Choose the mutant solution

            rand_num_b = floor((sensor_model_num-0.0001)*rand()/double(RAND_MAX));   // Choose which sensor in the solution to mutate
            for (int j=0; j<sensor_model_num; j++){
                solution_a[j] = (*solution_list[(int)rand_num_a]->at(j));
            }
            while (1){
              rand_index = border_points.size() * rand()/RAND_MAX;
              if (border_points.at(rand_index).Z()>max_z*0.7 && border_points.at(rand_index).X()>max_x-0.4*(max_x-min_x)) break;
            }
            solution_a.at((int)rand_num_b).model_pose.Pos() = border_points.at(rand_index);
            ignition::math::Vector3d rot_vec;
            while(1){
              rot_vec.X() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
              rot_vec.Y() = M_PI/3 * rand()/RAND_MAX - M_PI/6;
              rot_vec.Z() = M_PI * rand()/RAND_MAX - M_PI/2;
              solution_a.at((int)rand_num_b).model_pose.Rot().Euler(rot_vec);
              if (check_angle(world, this->model, solution_a.at((int)rand_num_b).model_pose)) break;
            }
            tmp_solu_list[tmp_solu_index++] = solution_a;
          }
        }


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
            }
            
          }

        }



      }
      
      std::cout<<"The final score of the top 10 solutions: "<<std::endl;
      get_solution_rank(score_solutions, score_rank);
      for (int i=0; i<10; i++){
        for (int j=0; j<SOLUTION_NUM; j++){
          if (i+1==score_rank[j]){
            std::cout<<j<<", "<<score_solutions[j]<<std::endl;
            if (i>0) break;
            std::cout<<"FOR THE "<<j<<" -th, which is the top solution:"<<std::endl;
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

        // TO DO
        // if (s->Type()=="ray"){
        //   sensors::RaySensorPtr laser = std::dynamic_pointer_cast<sensors::RaySensor>(s);
        //   std::cout<<"A lidar" << std::endl;
        //   std::cout<< "Name: "<<s->Name()<<std::endl;
        //   std::cout<< "Type: "<<s->Type()<<std::endl;
        //   std::cout << "  Range max: " << laser->RangeMax() << " m" << std::endl;
        //   std::cout << "  Range min: " << laser->RangeMin() << " m" << std::endl;
        //   std::cout << "  Range count: " << laser->RangeCount() << std::endl;
        //   std::cout << "  Resolution: " << laser->RangeResolution() << " m" << std::endl;
        //   std::cout << "  H angle max: " << laser->AngleMax().Degree() << " degrees" << std::endl;
        //   std::cout << "  H angle min: " << laser->AngleMin().Degree() << " degrees" << std::endl;
        //   std::cout << "  V angle max: " << laser->VerticalAngleMax().Degree() << " degrees" << std::endl;
        //   std::cout << "  V angle min: " << laser->VerticalAngleMin().Degree() << " degrees" << std::endl;
        // }
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