#include <cstdlib>
#include <string>

#include <gazebo/gazebo.hh>

int main(int argc, char** argv)
{
  // 初始化Gazebo
  gazebo::setupClient(argc, argv);

  // 创建并加载世界文件
  gazebo::runWorld(argv[1]);

  // 清理Gazebo
  gazebo::shutdown();

  return EXIT_SUCCESS;
}