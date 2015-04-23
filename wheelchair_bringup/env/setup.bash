env_dir=$(dirname $BASH_SOURCE)
dir=$(pwd)
cd $env_dir

source vars.bash

# find the workspace's setup file
while [ ! \( -e src -a -e src/CMakeLists.txt \) -a "$(pwd)" != "/" ] ; do
  cd ..
done

if [ "$(pwd)" = "/" ]; then
  echo "Not in catkin workspace" 1>&2
else
	source $(pwd)/devel/setup.bash
fi

cd $dir

