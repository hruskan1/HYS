% Lab 7
% Pendulum on a cart - supervisory control example
%% Linearization around the upper (unstable) position

m = 1;  % [ kg ]
g = 9.81;   % [ m s^2 ]
L = 1;  % [ m ]

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

%% Original (nonlinear system) with nonlinear supervisory control

T = 7;
Ts = 0.05;
t = 0:Ts:T;
N = numel(t);
X = zeros(2, N);
U = zeros(1, N);
X(:, 1) = [0 ; 10];
Eref = m * g * L;
mode = NaN(1, N);
P = 1;

umax = 20;

for k = 2:N
    theta = X(1, k-1);
    dtheta = X(2, k-1);
%     Supervisory control, local/global controller selection
    if abs(wrapToPi(theta)) < pi / 6 && abs(dtheta) < 2
        u = max([-umax, min([umax, -K * wrapToPi(X(:, k-1))])]);
        mode(k) = 1;
    else
        Etilde = Eref - m * L^2 * dtheta^2 / 2 - m * g * L * cos(theta);
        u = max([-umax, min([umax, P * dtheta * cos(theta) * Etilde])]);
        mode(k) = 0;
    end
    
    U(k) = u;
    [~, y] = ode45(@(~, x) pendcartODE(t(k), x, u, [g, L]), [0, Ts], X(:, k-1));
    X(:, k) = y(end, :)';
end

plotstates(t, X, U, mode)

function plotstates(t, X, U, mode)
    subplot 311
    plot(t, X(1,:))
    ylabel('\theta (rad)')
    subplot 312
    plot(t, X(2,:))
    ylabel('d\theta (rad/s)')
    subplot 313
    scatter(t(mode == 1), U(mode == 1), 'b')
    hold on
    scatter(t(mode == 0), U(mode == 0), 'r')
    ylabel('u (m s^{-2})')
    xlabel('Time (s)')
end

function xdot = pendcartODE(t, x, u, params)
    theta = x(1);
    dtheta = x(2);
    
    g = params(1);
    L = params(2);
    
    xdot = zeros(2,1);
    
    xdot(1) = dtheta;
    xdot(2) = g/L * sin(theta) + 1 / L * cos(theta) * u;
end