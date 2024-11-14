

function [M,P,K,MU,S] = ekf_update1(M,P,y,H,R,h, state_angle_idx, meas_angle_idx)
	NormalizeAngle = @(angle)(mod(angle + pi, 2 * pi) + (mod(angle + pi, 2 * pi) < 0) * 2 * pi) - pi;

	MU = h(M);
	%
	% update step
	%  
	S = (R + H*P*H');
	K = P*H'/S;
	diff_pred = y - MU;
	[cS,~] = chol(S,'lower');
	normalized_diff = cS \ diff_pred;
	diff_pred(meas_angle_idx) = NormalizeAngle(diff_pred(meas_angle_idx));
	M = M + K * diff_pred;
	M(state_angle_idx) = NormalizeAngle(M(state_angle_idx));
	P = P - K*S*K';
end

