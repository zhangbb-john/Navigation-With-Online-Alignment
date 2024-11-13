clear
source ../../devel/setup.bash
# bash ./kill.sh
for i in {1..3};do
echo $i
echo "wait for 20s"
sleep 20;
roslaunch ./ukf_localization/launch/sim_localization_proposed.launch
echo "wait for 10s"
sleep 10;
done