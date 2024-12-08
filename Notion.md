# Todo
## we need to keep some trajectory ellipse
15 * 15 * 10 maybe



## 

## allow gps position production for reproducing other methods in simulation?

## increase the frequency

# Issue
## How can we avoid PF degenerte, i.e., small covariance not reflecting truth

# done
## add disp trial index
## restore parameters
## add time cost
## remove warning
## change main_test to main
## add beacon localization error.
## how to accomadate 
function [dx, initState, y, y_Q, groundTruth] = generateData(params,dynModel,measModel)

function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    ekf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,dt, groundTruth, makeplots)
R is multiple 


function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    ukf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,dt, groundTruth)

R is multiple covariance matrix


function [traj_max,traj_mean,traj_std,P_mean,traj_sample_iwmax] = ...
    lsUkf(initialize, dynModel,measModel,measurements,...
    x0_nonLin,Q0,Q,R,dt, groundTruth)

R is multiple covariance matrix