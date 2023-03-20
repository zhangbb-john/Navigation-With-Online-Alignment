# rosnode kill -a

for i in {1..3};do
sleep 1;
echo $i
date;
rosnode kill -a;
sleep 0.1;
ps -ef|grep pub|grep -v grep| cut -c 9-15|xargs kill -s 9;
sleep 0.1;
ps -ef|grep rqt|grep -v grep| cut -c 9-15|xargs kill -s 9;
sleep 0.1;
ps -ef|grep sim_localization|grep -v grep| cut -c 9-15|xargs kill -s 9
done