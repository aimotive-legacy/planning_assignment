# Gazebo setup instructions 

## Linux 

Gazebo can be most easily ran on Linux (Debian/Ubuntu is the preferred distribution).  

 - Please follow the setup instructions on the Gazebo website (http://gazebosim.org/tutorials?tut=install_ubuntu) 

 - Clone this repository 

 - Change to the world directory and run gazebo 

        cd /path/to/this/dir/world
        export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:`pwd`
        gazebo --verbose interview_city.world 

 - Optionally compile and run the sample client 
        
        cd build
        cmake ..
        make 
        ./client_controller
