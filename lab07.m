% Lab 7
% Pendulum on a cart - supervisory control example
%% Linearization around the upper (unstable) position

g = 9.81;
L = 1;

% x = [theta; dtheta]

A = [0 1;g/L 0];
B = [0; 1/L];
C = eye(2);
D = 0;

plant = ss(A, B, C, D);

Q = eye(2);
R = 1;
N = zeros(2,1);

% The system is observable, therefore we can use LQR
[K, S, e] = lqr(plant, Q, R, N);

ctrl = ss(0, zeros(1,2), 0, K);
ctrlplant = feedback(plant, ctrl);
initial(ctrlplant, [pi/6, 0, 0])

%% Original (nonlinear system)

T = 5;
Ts = 0.05;
t = 0:Ts:T;
N = numel(t);
X = zeros(2, N);
U = zeros(1, N);
X(:, 1) = [pi / 2; 0];
for k = 2:N
    u = -K * X(:, k-1);
    [~, y] = ode45(@(~, x) pendcartODE(t(k), x, u, [g, L]), [0, Ts], X(:, k-1));
    X(:, k) = y(end, :)';
end

plot(t, X(1,:))


function xdot = pendcartODE(t, x, u, params)
    theta = x(1);
    dtheta = x(2);
    
    g = params(1);
    L = params(2);
    
    xdot = zeros(2,1);
    
    xdot(1) = dtheta;
    xdot(2) = g/L * sin(theta) + 1 / L * cos(theta) * u;
end