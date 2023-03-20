clear
source ../../devel/setup.bash
# bash ./kill.sh
for i in {1..3};do
echo $i
sleep 1;
roslaunch ./ukf_localization/launch/sim_localization.launch
done