zumy_ID=$1

if [[ -n "$zumy_ID" ]]; then
    echo "Setting up your Zumy"
    echo "Pinging Zumy $zumy_ID ..."
else
    echo "Error: Enter Zumy ID as first argument."
    exit 1
fi


timeout 3s ping "zumy$zumy_ID".local 
echo "SSHing..."
while true; do
    read -p "First time using this one? (y) " yn
    case $yn in
        [Yy]* ) ssh zumy@zumy"$zumy_ID".local; sh-keygen -t rsa -f ˜/.ssh/id_rsa; ssh-copy-id zumy@zumy"$zumy_ID".local; break;;
        * ) break;;
    esac
done

echo "Running load_ros.sh..."
# sh ~/ee106_zumy_support/load_ros.sh zumy"$zumy_ID"

echo "Trying to SSH again..."
ssh zumy@zumy"$zumy_ID".local

echo "Launching nodes onboard the Zumy..."
enter_zumy_workspace
cs
roslaunch odroid_machine remote_zumy.launch mname:=zumy"$zumy_ID"


