
function [M,P] = ekf_predict1(M,P,A,Q,a,dt)
  M = a(M,dt);
  P = A * P * A' + Q;
end
