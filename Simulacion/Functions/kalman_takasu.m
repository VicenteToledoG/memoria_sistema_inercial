function [x, P, chi2] = kalman_takasu(x, P, dz, R, H)
%KALMAN_TAKASU Kalman Filter equations from T. Takasu
%
% Jan Zwiener (jan@zwiener.org)
%
% x (n x 1) A priori state vector (size=n) at epoch k
% P (n x n) Covariance matrix of state vector x at epoch k
% dz (m x 1) Measurement difference vector (size=m) at epoch k
%           dz = measurement - predicted measurement
%           dz = z - H*x
% R (m x m) Covariance matrix of measurement vector z
% H (m x n) Observation matrix so that y = H*x
%
% Return value:
% x (n x 1) A posteriori state vector at epoch k (corrected by measurements)
% P (n x n) A posteriori covariance of x at epoch k
assert(size(P,1)==size(P,2), 'P must be a square matrix');
assert(size(P,1)==size(x,1), 'P and x must have the same number of rows');
assert(size(x,2)==1, 'x must be a column vector');
assert(size(dz,2)==1, 'dz must be a column vector');
assert(size(dz,1)==size(R,1), 'dz and R must have the same number of rows');
assert(size(R,1)==size(R,2), 'R must be a square matrix');
assert(size(H,1)==size(dz,1), 'H and dz must have the same number of rows');
assert(size(H,2)==size(x,1), 'H and x must have consistent dimensions');
assert(nargin == 5, 'kalman_takasu requires 5 input arguments');

D = P*H';
S = H*D + R; % S = H*D*H' + R
U = chol(S);
E = D/U; % trsm E=D*U^1 solve E*U = D for E
K = E/(U'); % trsm  K*U' = E, solve for K
dx = K*dz;
x = x + dx;
P = P - E*E'; % dsyrk
P = 0.5*(P+P');

% Optional chi2 test, see [1] section 8.3.1.2 "Detecting anomalous Sensor Data"
y = (dz'/U)';
chi2 = (y'*y) / length(dz);
% [1] Grewal, Weill, Andrews (2001): Global positioning systems, inertial
% navigation, and integration. 1st Ed. John Wiley & Sons, New York.

end
