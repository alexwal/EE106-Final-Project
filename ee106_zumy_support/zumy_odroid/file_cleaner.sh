#!/bin/bash

#to remove extraneous files on the zumy
NAME="$1"

echo "Removing Extraneous files"
shopt -s extglob 
rm -r !(*.txt|network_test.sh|zumy_workspace)
echo "Done"
