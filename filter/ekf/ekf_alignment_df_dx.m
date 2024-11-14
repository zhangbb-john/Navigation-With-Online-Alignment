function Jac_func = ekf_alignment_df_dx(dynModel)
% Define all variables, including dt as symbolic for use in dynModel
syms x y z r p yaw vx vy vz offsetr offsetp offsety beaconx beacony beaconz dt real  

% Define state vector as a column vector
xn = [x y z r p yaw vx vy vz offsetr offsetp offsety beaconx beacony beaconz]';

% Call the system dynamic model function
pred = dynModel(xn, dt);

% Compute the Jacobian of the prediction with respect to the state vector
Jac = jacobian(pred, xn);

% Convert the Jacobian into a MATLAB function handle
Jac_func = matlabFunction(Jac, 'Vars', {xn, dt});
end
