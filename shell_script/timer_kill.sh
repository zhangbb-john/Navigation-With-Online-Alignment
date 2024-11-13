# rosnode kill -a

for i in {1..3};do
echo "i=$i"
for j in {1..610}; do 
sleep 10;
date;
echo "now total sleep time is $((j*10)) seconds"
done
rosnode kill -a;
sleep 0.1;
ps -ef|grep rqt|grep -v grep| cut -c 9-15|xargs kill -s 9;
sleep 0.1;
ps -ef|grep pub|grep -v grep| cut -c 9-15|xargs kill -s 9;
sleep 0.1;
ps -ef|grep sim_localization|grep -v grep| cut -c 9-15|xargs kill -s 9
sleep 0.1;
done