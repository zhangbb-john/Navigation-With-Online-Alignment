clear
start_time=$(date +%s)
dir="$(pwd)"

cd .. 
rm -rf CMakeLists.txt
cd ..
rm -rf ./devel ./build
sleep 3
catkin_make -DCATKIN_WHITELIST_PACKAGES="auv_nav_msg"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
echo "$(pwd)"
end_time=$(date +%s)
cost_time=$[ $end_time-$start_time ]
echo "build time is $(($cost_time/60))min $(($cost_time%60))s"

if [ -e log ];
then
echo "folder log exists"
else
mkdir log 
fi
if [ -e bag ];
then
echo "folder bag exists"
else
mkdir bag 
fi
cd log 
if [ -e log ];
then
echo "folder log exists"
else
mkdir log 
fi
cd $dir
