#include "Stopper.h"

int main(int argc, char **argv){
    //Initiaate new ROS node named Stopper
    ros::init( argc, argv, "Stopper");

    //Create a new stopper object
    Stopper stopper;

    //Start the movement
    stopper.startMoving();

    return 0;
}