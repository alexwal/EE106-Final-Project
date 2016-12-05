#! /bin/bash

result="0"
counter=0
count_time=3
host="m92p-3.local"
#host="veloped.local"

#step 1: let's make sure that avahi is up.
up="1"
while [ "$up" != "0" ]; do
  echo "Avahi is not up"
  avahi-daemon --check
  up=$? #magic icantation to get the last exit status code in this script, which in this case, is the one for avahi-daemon 
  sleep 0.05

done

echo "Avahi is up"

while [ "$result" != "1" ]; do
  sleep 0.5
  result=$(nmap "$host" -sP | grep -o -P '..host up'| awk '{print $1}')
  #echo "$result"
  counter=$((counter+1))
  echo "$counter"
  
  if [ "$counter" = "$count_time" ]; then
    printf "\nGiving up and rebooting...\n"
    python /home/zumy/zumy_workspace/src/zumy_ros/src/ping_light.py
    exit 0
    # echo odroid | sudo -S reboot
  fi
  if [ "$result" = "1" ]; then
     printf "\nWireless network found\n"
  else
     printf "\nNo wireless connection\nTrying to reconnect...\nSoftware disabling & re-enabling wifi\n\n"
     echo odroid | sudo -S ifdown wlan0
     echo odroid | sudo -S ifup wlan0
  fi



done

python home/zumy/zumy_workspace/src/zumy_ros/src/ping_light.py