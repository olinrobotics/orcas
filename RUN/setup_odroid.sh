#!/bin/bash
# NOTE(danny): not fully working yet
set -e

die() {
    echo >&2 "$@"
    exit 1
}

[ "$#" -eq 3 ] || die "Usage: ./setup_odroid.sh <hostname> <wifi ssid> <wifi password>"

NEW_HOSTNAME=$1
WIFI_SSID=$2
WIFI_PASSWORD=$3

NEW_USERNAME="robolab"
NEW_PASSWORD="robolab"

function add_robot_user {
    egrep "^$NEW_USERNAME" /etc/passwd >/dev/null
    if [ $? -eq 0 ]; then
        echo "$NEW_USERNAME exists!"
        return 0
    fi

    pass=$(perl -e 'print crypt($ARGV[0], "password")' $NEW_PASSWORD)
    useradd -m -p $pass $NEW_USERNAME
    if [ $? -ne 0 ]; then
        echo "Couldn't make $NEW_USERNAME user!"
        return 1
    fi
}

function add_docker {
    echo "Setting up required packages for installing docker"
    apt-get update && apt-get install -y curl vim git apt-transport-https
    echo "Adding docker public key"
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
    echo "Adding docker apt repository"
    add-apt-repository "deb [arch=armhf] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    apt-get update
    echo "Installing docker"
    apt-get install -y docker-ce
}

add_docker()
add_robot_user()
if [ add_robot_user() -eq 0 ]; then
    echo "Setting up docker so $NEW_USERNAME can use it"
    usermod -aG docker $NEW_USERNAME
else
    echo "Can't set up docker group!"
    exit 1
fi

echo "Enabling docker on boot"
# sudo systemctl enable docker

echo "Connecting to $WIFI_SSID with password $WIFI_PASSWORD"
echo "nmcli d wifi connect '$WIFI_SSID' password '$WIFI_PASSWORD'" >

echo "Setting up avahi to advertise ssh access"
sudo mkdir -p /etc/avahi/services
tee /etc/avahi/services/ssh.service <<EOF
<?xml version="1.0" standalone='no'?><!--*-nxml-*-->
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">

<service-group>

  <name replace-wildcards="yes">%h</name>

  <service>
    <type>_ssh._tcp</type>
    <port>22</port>
  </service>

</service-group>

EOF

tee /etc/systemd/system/robo-wifi.service <<EOF
multi-user.target

EOF

echo "Changing hostname from odroid to $NEW_HOSTNAME (not checking if current is odroid)"
sudo sed -i 's/$(hostname)/$NEW_HOSTNAME/g' /etc/hosts
sudo sed -i 's/$(hostname)/$NEW_HOSTNAME/g' /etc/hostname

echo "Cloning orcas repo into home directory"
cd $HOME
git clone https://github.com/olinrobotics/orcas.git

echo "Ok. Done! Reboot to apply changes"
echo "Find the device on the network by running ``"
