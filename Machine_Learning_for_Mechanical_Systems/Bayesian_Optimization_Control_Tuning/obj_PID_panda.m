%function J = objMPC(x_var,const)
%
%   Evaluates the objective function to be minimized using bayesian
%   optimization
%   Inputs: 
%       X_VAR - variables to be optimized (in a table-like
%       data structure)
%       CONST - structure with constant values 

function J = obj_PID_panda(x_var, const)

    
    %% Count the number of function evaluations %%
    persistent idx_sim;
    if isempty(idx_sim)
        idx_sim = 1;
    end
    %% assign all constant values %%
    
    sim_var = const;
    % assign bayesian optimization variables (overwrite default constants
    % if there exist a field with the same name )
    assert(size(x_var,1) == 1)
    x_var_struct = table2struct(x_var(1,:));
    fname = fieldnames(x_var_struct);
    for idx_fn = 1:length(fname)
       sim_var.(fname{idx_fn}) = x_var_struct.(fname{idx_fn});
    end
    
    %% Simulation settings - inner loop and outer loop configuration %%
    %% 
    % Setup PID object (inner loop controller)
    Kp = diag([sim_var.Kp1, sim_var.Kp2, sim_var.Kp3, sim_var.Kp4, sim_var.Kp5, sim_var.Kp6, sim_var.Kp7]);
    Ki = diag([sim_var.Ki1, sim_var.Ki2, sim_var.Ki3, sim_var.Ki4, sim_var.Ki5, sim_var.Ki6, sim_var.Ki7]);
    Kd = diag([sim_var.Kd1, sim_var.Kd2, sim_var.Kd3, sim_var.Kd4, sim_var.Kd5, sim_var.Kd6, sim_var.Kd7,]);
    
    Robot_eval = panda_robot();
    
    %%
    
    toll_qerr = const.toll_qerr;
    Ts = const.Ts;
    t = const.time;
    n_DoFs = const.n_DoFs;
    Robot = const.Robot;
    q_r = const.r.q_r;
    dq_r = const.r.dq_r;
    ddq_r = const.r.ddq_r;
    f = const.Robot_friction;
        
    %%
    % Run the simulation

    %INITIALIZE the dynamic parameters of the rotor
    g_eval = zeros(length(t),n_DoFs);
    B_eval = zeros(n_DoFs,n_DoFs,length(t));
    
    B            = zeros(n_DoFs,n_DoFs,length(t));
    g            = zeros(length(t),n_DoFs);
    tau_l        = zeros(length(t),n_DoFs);
    q_msr        = zeros(length(t),n_DoFs);
    qerr         = zeros(length(t),n_DoFs); %errori pos
    dqerr        = zeros(length(t),n_DoFs); %errori vel
    ddqerr       = zeros(length(t),n_DoFs); %errori acc
    dq_msr       = zeros(length(t),n_DoFs);
    ddq_msr      = zeros(length(t),n_DoFs);
    tau_PID      = zeros(length(t),n_DoFs);
    tau_comp     = zeros(length(t),n_DoFs);
    
    q_msr(1,:) = q_r(1,:); % popolato con la posizione iniziale per ogni dof
    
    B(:,:,1)  = Robot.inertia(q_r(1,:)); % inerzia matrix per ogni instante
    g(1,:)    = Robot.gravload(q_r(1,:)); % gravitational torque per ogni joint
    
    %popolata come i due vettori sopra
    B_eval(:,:,1) = Robot_eval.inertia(q_r(1,:)); % inerzia matrix per ogni instante
    g_eval(1,:)   = Robot_eval.gravload(q_r(1,:)); % gravitational torque per ogni joint
    
    wb = waitbar(0,'Please wait...');
    
    ierr_m(1,:) = [0 0 0 0 0 0 0];
    dierr_m(1,:) = [0 0 0 0 0 0 0];

    jj = 1;
    exit_flag = false;

    % penalty = [0 0 0 0 0 0 0]; #FORSE UTILE PER OTTIMIZZAZIONE SEPARATA
    penalty=0;
    
    length(t)
    
    J=[];

    % Dentro questo ciclo si simula il movimento
    while ( jj<length(t) && exit_flag==false) 
        
        jj=jj+1; % instante jj
       
        B(:,:,jj)  = Robot.inertia(q_msr(jj-1,:)); % calcola nuove inerize valutando jj-1
        g(jj,:)    = Robot.gravload(q_msr(jj-1,:)); % calcola nuova gravitational torque
        
        B_eval(:,:,jj) = Robot_eval.inertia(q_msr(jj-1,:));
        g_eval(jj,:)   = Robot_eval.gravload(q_msr(jj-1,:));
        
        tau_l(jj,:)    = f*dq_msr(jj-1,:)' + g(jj,:)' ; %torque 
        tau_PID(jj,:)  = B_eval(:,:,jj)*(Kp * (q_r(jj,:) - q_msr(jj-1,:))' - Kd * dq_msr(jj-1,:)' + Ki * ierr_m(jj-1,:)'); % B_eval(:,:,jj) * (ddq_r(jj,:)'
        tau_comp(jj,:) = f*dq_msr(jj-1,:)' + g_eval(jj,:)';
        
        Beq = B(:,:,jj);
        
        ddq_msr(jj,:) = (Beq)\( - tau_l(jj,:)' + tau_PID(jj,:)' + tau_comp(jj,:)');  %- tau_l(jj,:)+ tau_comp(jj,:) = 0 SE INERZIE SONO CONSTANTI
        dq_msr(jj,:) = dq_msr(jj-1,:) + ddq_msr(jj,:)*Ts;
        q_msr(jj,:) = q_msr(jj-1,:) + dq_msr(jj,:)*Ts;
        
        ierr_m(jj,:) = ierr_m(jj-1,:) + (q_r(jj,:) - q_msr(jj,:)) * Ts; % errore cumulato posizione
        dierr_m(jj,:) = dierr_m(jj-1,:) + (dq_r(jj,:) - dq_msr(jj,:)) * Ts; % errore cumulato velocità

        qerr(jj,:)   = q_r(jj,:) - q_msr(jj,:);
        dqerr(jj,:)  = dq_r(jj,:) - dq_msr(jj,:);
        ddqerr(jj,:) = ddq_r(jj,:) - ddq_msr(jj,:);
        
        %cost function J (idea: check penalty for instability)
        toll_qerr = 20*pi/180;
        
        if sum(qerr(jj,:) > toll_qerr) > 0 
            penalty = 100*exp(t(jj)/t(end)); %MANCA IL MENO? COSI' PENALIZZA PIù LE INTERRUZIONI VERSO LA FINE
            exit_flag = true;  
        end
        

        waitbar(jj/length(t),wb);
        
       
    end
  
    %cost function J (idea: define KPIs for its implementation), J deve essere scalare, ricordarsi di cumulare gli errori
    J = sum(max(abs(qerr)))+sum(max(abs(dqerr)))+sum(ierr_m(end,:))/t(end)+sum(dierr_m(end,:))/t(end)+penalty; % normalizzazione degli errori potrebbe essere fatta sulla media della reference
    close(wb)
    
    % disp J terms
    disp('penalty: ')
    
    fprintf('Function evaluation %.0f: final cost: %12.8f \n', idx_sim, J)
    fprintf('-------------------------------\n')
    idx_sim = idx_sim + 1;
    
end