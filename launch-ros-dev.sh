# NOTE(danny): allow connections from docker to MacOS XQuartz
xhost + ${hostname}
export HOSTNAME=`hostname`
pushd catkin_ws
docker-compose up -d
popd

docker exec -it robosyscv_app_1 /bin/bash
