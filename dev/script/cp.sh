#!/bin/bash

install_dir_actprocess="/home/pi/pi/robi/lib/actprocess/install"
install_dir_cortex="/home/pi/pi/robi/share/cortex/install"
install_dir_visualprocess="/home/pi/pi/robi/share/visualprocess/install"
install_dir_audioprocess="/home/pi/pi/robi/lib/audioprocess/install"
install_dir_sensorprocess="/home/pi/pi/robi/share/sensorprocess/install"

src_dir_actprocess="/home/pi/toby_ws/src/actprocess/src"
src_dir_cortex="/home/pi/toby_ws/src/cortex/src"
src_dir_visualprocess="/home/pi/toby_ws/src/visualprocess/src"
src_dir_audioprocess="/home/pi/toby_ws/src/audioprocess/src"
src_dir_sensorprocess="/home/pi/toby_ws/src/sensorprocess/src"


function copy_py_back()
{
  filelist=$(ls $1)
  destFilelist=$(ls $2)
  for name in ${filelist[*]}
  do
    if [ -f $1/$name ]; then
      # diff $1/name $2/name
      # Status = $?
      if [ -e $2/$name ]; then
        if [ $1/$name -nt $2/$name ]; then 
          echo "Info:copying file $name to $2"
          cp -f $1/$name  $2/$name
        else
          #echo "file $1/$name has no change. Ignored. "
          echo -n "."
        fi
      else
        echo -n "~"              #if src folder don't have the same file
      fi
    fi

  done

  echo " Done!"

}

copy_py_back $install_dir_actprocess $src_dir_actprocess
copy_py_back $install_dir_cortex $src_dir_cortex
copy_py_back $install_dir_visualprocess $src_dir_visualprocess
copy_py_back $install_dir_audioprocess $src_dir_audioprocess
copy_py_back $install_dir_sensorprocess $src_dir_sensorprocess
