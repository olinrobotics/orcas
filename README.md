# RoboSys CV project

### Notes

`docker pull ros:kinetic`
`docker run -it --name ros_workspace ros:kinetic`

1. Use `docker-compose build && docker-compose up` to get a working environment.
1. Have a volume mounted to the catkin_ws however you want.
1. Exec into the container (bash) and source the /opt setup script.
1. `cd ~/catkin_ws && catkin_make`
1. Use an editor of your choice on the host system to edit files within the bind mount.
1. Make sure that `echo $ROS_PACKAGE_PATH` shows your current src directory (unsure of how to do this)
