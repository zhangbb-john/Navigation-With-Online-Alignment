source ../../devel/setup.bash
# source /home/ubuntu/Integrated_navigation/devel/setup.bash
# echo "What do you want to do"
case "$1" in
"rqt")
echo "Now open rqt"
sleep 3
rqt
;;
"rqt_plot")
echo "Now open rqt_plot"
sleep 3
rqt_plot
;;
"rqt_bag")
echo "Now open rqt_bag"
sleep 3
rqt_bag
;;
esac
# rqt_bag