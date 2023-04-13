#include <cstdlib>

int main()
{
    system("gnome-terminal -- bash -c 'cd build && export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH\
     && export GAZEBO_RENDERING_QUALITY=1 && gazebo 1.world --verbose; exec bash'");
    return 0;
}