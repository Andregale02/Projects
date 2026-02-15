%% Bayesian optimization

%% Cleanup %%

clear;
close all;
clc;

addpath("robotics-toolbox-matlab-master\")
addpath("spatialmath-matlab-master\")
%% Constants  definition %% 
Ts = 1e-3;         % sampling time (s). We assume measurements are available at this rate
Tsim = 4;         % simulation length (s)

%% robot properties

n_DoFs = 7; % dof 

friction = [2 2 2 2 2 2 2]; % friction coefficient of each dof

Robot = panda_robot(); % carica il robot

%% Motion Reference

t0 = 0; % [s]
tf = 4.; % [s]
time = t0:Ts:Tsim; % [s]
freq = 1; % [Hz]

q0 = [-0.7160   -0.5850    0.3504   -1.5666    0.2241   -2.1201   -2.8398];

for ii=1:n_DoFs
    q_r(:,ii) = q0(ii) + 10*pi/180*sin(time*2*pi*freq);
    dq_r(:,ii) = 2*pi*freq*10*pi/180*cos(time*2*pi*freq);
    ddq_r(:,ii) = -(2*pi*freq)^2*10*pi/180*sin(time*2*pi*freq);
end

clc
disp('going to optimization...');

%crea una stuct di 2 matrici,la prima matrice(prima riga) x posizione, seconda x velocità, terza x accelerazione
r.q_r = q_r;  
r.dq_r = dq_r;
r.ddq_r = ddq_r;

toll_qerr = 20*pi/180; %imposto un valore di tolleranza per l'errore POSSIBILE PARAMETRO DA MODIFICARE

%% Global parameters to be used in the BO
% all constants in a structure for convenience 

% valori già assegnati prima, adesso raggruppati in const
const.Ts = Ts; % inner loop sampling time
const.Tsim = Tsim; 
const.time = time;

const.n_DoFs = n_DoFs;

const.r = r;

const.Robot = Robot;
const.Robot_friction = friction;
const.q_0 = q0;
const.toll_qerr = toll_qerr;

%% Bayesian optimization of controller parameters 
% Define bounds on the optimization variables  %

opt_vars = [];

bound_min_Kp = 0.;
bound_max_Kp = 10000;

bound_min_Kd = 0;
bound_max_Kd = 500;

bound_min_Ki = 0.;
bound_max_Ki = 1000;

%%
% PID parameters, 1x21 struct, ogni elemento è PID parameter, con nome,range...

opt_vars = [opt_vars optimizableVariable('Kp1', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki1', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd1', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp2', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki2', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd2', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp3', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki3', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd3', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp4', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki4', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd4', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp5', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki5', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd5', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp6', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki6', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd6', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative
opt_vars = [opt_vars optimizableVariable('Kp7', [bound_min_Kp, bound_max_Kp],'Type','real')]; % PID proportional
opt_vars = [opt_vars optimizableVariable('Ki7', [bound_min_Ki, bound_max_Ki],'Type','real')]; % PID integral
opt_vars = [opt_vars optimizableVariable('Kd7', [bound_min_Kd, bound_max_Kd],'Type','real')]; % PID derivative

%%
% Define objective function as a function of the optimization variables
% opt_vars only, to be passed to the optimizer

func =  @(x_vars)(obj_PID_panda(x_vars,const));
initial_X = []; % array2table(rho_init, 'VariableNames', opt_var_names);

clear objBO; % just to reset the function inner counter

%%
% Perform bayesian optimization 

results = bayesopt(func,opt_vars,...
    'Verbose',0,...
    'AcquisitionFunctionName','expected-improvement',... %-plus',...
    'IsObjectiveDeterministic', true,... % simulations with noise --> objective function is not deterministic
    'MaxObjectiveEvaluations', 2,...
    'MaxTime', inf,...
    'NumCoupledConstraints',0, ...
    'NumSeedPoint',10,...
    'GPActiveSetSize', 300,...
    'PlotFcn',{@plotMinObjective,@plotObjectiveEvaluationTime});

best_vars = results.bestPoint;

%%
% Evaluate performance of the optimal design
close all

obj_PID_panda(best_vars,const);

%%

const.plot_fig = true;
const.Tsim = 10;
N_mc = 1;
OBJ_VEC = zeros(N_mc,1);
for i=1:N_mc
    OBJ_VEC(i) = obj_PID_panda(best_vars,const);
    disp(OBJ_VEC(i));
end

%%

const.Tsim = 10;
idx_min = results.IndexOfMinimumTrace(end);
results.XTrace(idx_min,:)
clear objBO;
obj_val=obj_PID_panda(results.XTrace(idx_min,:), const);

%%

idx_min = results.IndexOfMinimumTrace(end);
N = length(results.ObjectiveTrace);
iteration = 1:N;

figure %1
plot(iteration, results.ObjectiveTrace, 'k*')
hold on;
plot(results.ObjectiveMinimumTrace, 'r', 'LineWidth', 2)
plot(iteration(idx_min), results.ObjectiveMinimumTrace(idx_min), 'MarkerEdgeColor','black',...
    'MarkerFaceColor','gree', 'Marker', 'square', 'MarkerSize',10);
h=xlabel('Iteration index $i$ (-)');
set(h,'Interpreter', 'Latex');
h=ylabel('Performance cost  $\tilde J$ (-)');
set(h,'Interpreter', 'Latex');
legend('Current point', 'Current best point', 'Overall best point');
grid('on');

%%

clear t
t = time;

Kp = diag([results.bestPoint.Kp1, results.bestPoint.Kp2, results.bestPoint.Kp3, results.bestPoint.Kp4, results.bestPoint.Kp5, results.bestPoint.Kp6, results.bestPoint.Kp7]);
Ki = diag([results.bestPoint.Ki1, results.bestPoint.Ki2, results.bestPoint.Ki3, results.bestPoint.Ki4, results.bestPoint.Ki5, results.bestPoint.Ki6, results.bestPoint.Ki7]);
Kd = diag([results.bestPoint.Kd1, results.bestPoint.Kd2, results.bestPoint.Kd3, results.bestPoint.Kd4, results.bestPoint.Kd5, results.bestPoint.Kd6, results.bestPoint.Kd7]);

B               = zeros(n_DoFs,n_DoFs,length(t));
g               = zeros(length(t),n_DoFs);
tau_l           = zeros(length(t),n_DoFs);
q_msr           = zeros(length(t),n_DoFs);
dq_msr          = zeros(length(t),n_DoFs);
ddq_msr         = zeros(length(t),n_DoFs);
tau_PID         = zeros(length(t),n_DoFs);
tau_comp        = zeros(length(t),n_DoFs);

q_msr(1,:) = q_r(1,:);

B(:,:,1)  = Robot.inertia(q_r(1,:));
g(1,:)    = Robot.gravload(q_r(1,:));

tau_l(1,:) = g(1,:)' + friction*dq_msr(1,:)';

q_err = 0;

wb = waitbar(0,'Please wait...');

ierr_m(1,:) = [0 0 0 0 0 0 0];

for jj=2:length(t)
    
    B(:,:,jj)  = Robot.inertia(q_msr(jj-1,:));
    g(jj,:)    = Robot.gravload(q_msr(jj-1,:));
    
    tau_l(jj,:)    = g(jj,:)' + friction*dq_msr(jj,:)';
    tau_PID(jj,:)  = B(:,:,jj)*(Kp * (q_r(jj,:) - q_msr(jj-1,:))' - Kd * dq_msr(jj-1,:)' + Ki * ierr_m(jj-1,:)');
    tau_comp(jj,:) = g(jj,:)' + friction*dq_msr(jj,:)';
    
    Beq = B(:,:,jj);
    
    ddq_msr(jj,:) = (Beq)\( -tau_l(jj,:)' + tau_PID(jj,:)' + tau_comp(jj,:)' );
    dq_msr(jj,:) = dq_msr(jj-1,:) + ddq_msr(jj,:)*Ts;
    q_msr(jj,:) = q_msr(jj-1,:) + dq_msr(jj,:)*Ts;
    
    ierr_m(jj,:) = ierr_m(jj-1,:) + (q_r(jj,:) - q_msr(jj,:)) * Ts;
    
    q_err = q_err + abs(q_r(jj,:) - q_msr(jj,:));
    
    waitbar(jj/length(t),wb);
    
end

J_err = sum(q_err)/length(t);

close(wb)

step = 100;

figure %2
for jj=1:step:length(t)
    plot(Robot,q_msr(jj,:));
end


max_error_joints=[];
figure %3 TRAJECTORY ERRORS
for i = 1:7
    subplot(7,1,i)
    plot(t, (q_r(:,i) - q_msr(:,i)) * 180/pi)
    ylabel(['Joint ' num2str(i)])
    if i == 1
        title(' Trajectory tracking errors [deg]')
    end
    if i == 7
        xlabel('time [s]')
    end
    grid on
    max_error_joints=[max_error_joints, max((q_r(:,i) - q_msr(:,i))* 180/pi)];
end

MAX_error_idx=  max_error_joints ==max( max_error_joints);
MAX_error_value= max_error_joints(MAX_error_idx);

disp(['Maximum trajectory error =', num2str(MAX_error_value), ' [deg];     At joint n°:', num2str(find(MAX_error_idx))]);
%VERSIONE SENZA SUBPLOTS
% figure
% plot(t,(q_r-q_msr)*180/pi);
% xlabel('time [s]');
% ylabel('error [deg]');
% legend('eq1','eq2','eq3','eq4','eq5','eq6','eq7')
% title('Absolute Postion Error');
% grid

figure %4
for i = 1:7
    subplot(7,1,i)
    plot(t, q_r(:,i)* 180/pi, 'LineWidth', 1.2)
    hold on
    plot(t, q_msr(:,i)* 180/pi, '--', 'LineWidth', 1.2)
    ylabel(['Joint ' num2str(i)])
    if i == 1
        title('Reference vs Measured Trajectories [deg]')
    end
    if i == 7
        xlabel('time [s]')
    end
    legend(['q ref' num2str(i)], ['q msr' num2str(i)])
    grid on
end
%VERSIONE SENZA SUBPLOTS
% figure %4
% plot(t,q_r);
% hold on
% plot(t,q_msr);
% xlabel('time [s]');
% ylabel('[rad]');
% legend('q1','q2','q3','q4','q5','q6','q7','qmsr1','qmsr2','qmsr3','qmsr4','qmsr5','qmsr6','qmsr7')
% grid



figure %5
for i = 1:7
    subplot(7,1,i)
    plot(t, dq_r(:,i)* 180/pi, 'LineWidth', 1.2)
    hold on
    plot(t, dq_msr(:,i)* 180/pi, '--', 'LineWidth', 1.2)
    ylabel(['Joint ' num2str(i)])
    if i == 1
        title('Reference vs Measured Velocity [deg/s]')
    end
    if i == 7
        xlabel('time [s]')
    end
    legend(['dq ref' num2str(i)], ['dq msr' num2str(i)])
    grid on
end

%VERSIONE SENZA SUBPLOTS
% figure
% plot(t,dq_r);
% hold on
% plot(t,dq_msr);
% xlabel('time [s]');
% ylabel('dq [rad/s]');
% legend('dq1','dq2','dq3','dq4','dq5','dq6','dq7','dqmsr1','dqmsr2','dqmsr3','dqmsr4','dqmsr5','dqmsr6','dqmsr7')
% grid

figure
for i = 1:7
    subplot(7,1,i)
    plot(t, tau_PID(:,i), 'LineWidth', 1.2)
    ylabel(['\tau ctrl, ' num2str(i)])
    grid on

    if i == 1
        title('Control Torque for Each Joint [Nm]')
    end
    if i == 7
        xlabel('time [s]')
    end
end
%VERSIONE SENZA SUBPLOT
% figure
% plot(t,tau_PID);
% xlabel('time [s]');
% ylabel('tau_{ctrl} [Nm]');
% legend('tau_{ctrl,1}','tau_{ctrl,2}','tau_{ctrl,3}','tau_{ctrl,4}','tau_{ctrl,5}','tau_{ctrl,6}','tau_{ctrl,7}')

%%

OBJ_VEC = [OBJ_VEC; obj_val];

save results_panda_PID results
