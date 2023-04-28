#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string.h>
#include <vector>
#include <math.h>

namespace gazebo
{
  class board_placement : public ModelPlugin
  { 
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    { 
        int sensor_count = 0;
        ignition::math::Vector3d board_cor_list[20];
        ignition::math::Pose3d model_pose;
        ignition::math::Vector3d vec_dir;

        ignition::math::Vector3d vec_front;
        ignition::math::Vector3d vec_left;
        ignition::math::Vector3d vec_right;
        ignition::math::Vector3d vec_behind;

        ignition::math::Pose3d my_pose;

        int flag = 0;  // 1 for front, 2 for left, 3 for right, 4 for behind, 
        double min_len = 30.0;

        physics::ModelPtr boardptr;

        this->world = _parent->GetWorld();
        this->model = _parent;
        for (auto model:this->world->Models()){
            if (model->GetSensorCount()){
                sensor_count++;
                model_pose = model->WorldPose();
                vec_dir = model_pose.Rot().RotateVector(ignition::math::Vector3d(1,0,0));

                vec_front.Z() = 0;
                vec_left.Z() = 0;
                vec_right.Z() = 0;
                vec_behind.Z() = 0;

                vec_front.X() = 7.5-model_pose.Pos().X();
                vec_left.X() = 0;
                vec_right.X() = 0;
                vec_behind.X() = -7.5-model_pose.Pos().X();

                vec_front.Y() = 0;
                vec_left.Y() = 5.0-model_pose.Pos().Y();
                vec_right.Y() = -5.0-model_pose.Pos().Y();
                vec_behind.Y() = 0;

                if (vec_front.Dot(vec_dir)>0){
                    flag = 1;
                    min_len = vec_front.Length()/(vec_front.Dot(vec_dir)/vec_front.Length());
                }

                if (vec_left.Dot(vec_dir)>0){
                    if (vec_left.Length()/(vec_left.Dot(vec_dir)/vec_left.Length())<min_len){
                        min_len = vec_left.Length()/(vec_left.Dot(vec_dir)/vec_left.Length());
                        flag = 2;
                    }
                }

                if (vec_right.Dot(vec_dir)>0){
                    if (vec_right.Length()/(vec_right.Dot(vec_dir)/vec_right.Length())<min_len){
                        min_len = vec_right.Length()/(vec_right.Dot(vec_dir)/vec_right.Length());
                        flag = 3;
                    }
                }

                if (vec_behind.Dot(vec_dir)>0){
                    if (vec_behind.Length()/(vec_behind.Dot(vec_dir)/vec_behind.Length())<min_len){
                        min_len = vec_behind.Length()/(vec_behind.Dot(vec_dir)/vec_behind.Length());
                        flag = 4;
                    }
                }

                board_cor_list[sensor_count-1] = model_pose.Pos()+ignition::math::Vector3d(vec_dir.X()*min_len, vec_dir.Y()*min_len, vec_dir.Z()*min_len);

                transport::NodePtr node(new transport::Node());
                node->Init(this->world->Name());
                transport::PublisherPtr factoryPub =
                node->Advertise<msgs::Factory>("~/factory");
                msgs::Model m;
                // // 生成一个唯一的模型名称
                std::string unique_model_name = "calibration_plane_" + std::to_string(sensor_count);
                m.set_name(unique_model_name);

                msgs::Factory c;
                c.set_sdf_filename("model://calibration_plane");
                // msg.set_allow_renaming(true);
                // msg.set_edit_name(unique_model_name);



                switch (flag)
                {
                case 1:
                    msgs::Set(m.mutable_pose(), ignition::math::Pose3d(board_cor_list[sensor_count-1],
                        ignition::math::Quaterniond(0, M_PI/2, M_PI)));

                    break;
                case 2:
                    msgs::Set(m.mutable_pose(), ignition::math::Pose3d(board_cor_list[sensor_count-1],
                        ignition::math::Quaterniond(0, M_PI/2, -M_PI/2)));
                    break;
                case 3:
                    msgs::Set(m.mutable_pose(), ignition::math::Pose3d(board_cor_list[sensor_count-1],
                        ignition::math::Quaterniond(0, M_PI/2, M_PI/2)));
                    break;
                case 4:
                    msgs::Set(m.mutable_pose(), ignition::math::Pose3d(board_cor_list[sensor_count-1],
                        ignition::math::Quaterniond(0, M_PI/2, 0)));
                    break;
                
                default:
                    break;
                }
                const double mass = 1.0;
                ignition::math::Vector3d size(2,2,0.1);
                msgs::AddBoxLink(m, mass, size);
                std::ostringstream newModelStr;
                newModelStr << "<sdf version='" << SDF_VERSION << "'>"
                    << msgs::ModelToSDF(m)->ToString("")
                    << "</sdf>";
                msgs::Factory msg;
                msg.set_sdf(newModelStr.str());
                factoryPub->Publish(msg);
                flag=0;
                //this->world->InsertModelSDF(cplane);
                //boardptr = this->world->ModelByName("calibration_plane");
                //boardptr->SetName("calibration_plane"+std::to_string(sensor_count));

            }
        }
        if (!sensor_count){
            std::cout<<"NO sensor found"<<std::endl;
        }
        for (int i=0; i<sensor_count; i++){
            std::cout<<board_cor_list[i]<<std::endl;
        }
    }

    
    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      sdf::ElementPtr msdf;
  };

  GZ_REGISTER_MODEL_PLUGIN(board_placement)
}