% Definition of the model
bb = MyBouncingBall();

% Initial conditions
x0 = [4, 0];

% Time spans
tspan = [0, 20];
jspan = [0, 30];

% Specify solver options.
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7);

% Compute solution
sol = bb.solve(x0, tspan, jspan, config);

%% Plot the solution
plotFlows(sol)