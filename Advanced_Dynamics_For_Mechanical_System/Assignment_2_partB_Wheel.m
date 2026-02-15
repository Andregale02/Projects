close all
clearvars
clc


%% Parameters definition

n_res=2;
rangewidth_num=100;
rangewidth_exp=150;

%% Data loading 

load("ADMS_Assignment1_PartB\Data.mat")

n_frf=size(frf,2);
df=freq(2)-freq(1);


%% natural frequencies identification
%average between tf
frf_mean=mean(abs(frf),2);

%estimation of the natural frequencies
[~,locs_exp]=findpeaks(abs(frf_mean),MinPeakHeight=1e-1,MinPeakDistance=150/df); 
nat_f_exp=freq(locs_exp); % NOTA BENE: nat_f_exp sono in Hz
number_resonances=size(nat_f_exp,1);
nat_f=nat_f_exp*ones(1,size(frf,2));

%% plotting

for ii=1:n_frf
figure(ii)
subplot(3,1,1)
semilogy(freq,abs(frf(:,ii)),LineWidth=2)
hold on
% semilogy(freq,frf_mean,'k',linewidth=3)
% hold on
grid on
subplot(3,1,2)
plot(freq,angle(frf(:,ii)),LineWidth=2)
grid on
hold on
subplot(3,1,3)
plot(freq,cohe(:,ii),LineWidth=2)
xlabel('frequency [Hz]')
ylabel('coherence [-]')
grid on
%sgtitle(strcat("Accelerometer ",num2str(ii))) % NUMERO DELLA FIGURA =
%NUMERO ACCELEROMETERO
subplot(3,1,1)
xline(nat_f_exp)

end



%% Plot settings

label=cell(1,2*size(frf,2));
for l=1:size(frf,2)
    label(l)={strcat('numerical FRF #',num2str(l))};
end
for l=size(frf,2)+1:2*size(frf,2)
    label(l)={strcat('experimental FRF #',num2str(l-size(frf,2)))};
end
% label_exp_num= [strcat(label,' Exp'),strcat(label,' Num')];

%% Initial guess estimation

x0_new=initial_guess_calculator(nat_f,frf,locs_exp,df);

%% Error minimisation

opt_3D=zeros(n_frf,5,n_res);

for j=1:n_res
    x0=[x0_new(:,j,1), x0_new(:,j,2), x0_new(:,j,3), x0_new(:,j,4), x0_new(:,j,5)];
    [opt_mat,frf_num(:,:,j)] = error_minimisation(x0,frf,rangewidth_num,locs_exp(j),df);
    
    opt_3D(:,:,j)=opt_mat;

    plotwidth_f_num (:,j)= freq((locs_exp(j)-rangewidth_num):(locs_exp(j)+rangewidth_num));
    plotwidth_f_exp = freq((locs_exp(j)-rangewidth_exp):(locs_exp(j)+rangewidth_exp));

    % figure()
    % subplot(2,1,1)
    % semilogy(plotwidth_f_num,abs(frf_num),'o',plotwidth_f_exp,abs(frf((locs_exp(j)-rangewidth_exp):(locs_exp(j)+rangewidth_exp),:)))
    % grid on
    % legend(label)
    % subplot(2,1,2)
    % plot(plotwidth_f_num,angle(frf_num),'o',plotwidth_f_exp,angle(frf((locs_exp(j)-rangewidth_exp):(locs_exp(j)+rangewidth_exp),:)))
    % grid on
    % legend(label)
end



for ii=1:n_frf
    for jj=1:n_res
        %if max(abs(frf(locs_exp(jj),ii)))>1e-1
        figure(ii)
        subplot(3,1,1)
        semilogy(plotwidth_f_num (:,jj), abs(frf_num(ii,:,jj)), 'o',Markersize=2,LineWidth=0.3)%, plotwidth_f_num, abs(Gjk_num_range_j(x0)), '*'
        grid on
        hold on
        xlabel('frequency [Hz]')
        ylabel('magnitude [(m/(s^2))/N]')
       

        subplot(3,1,2)
        plot(plotwidth_f_num (:,jj), angle(frf_num(ii,:,jj)), 'o',Markersize=2,LineWidth=0.3)
        grid on
        hold on
        xlabel('frequency [Hz]')
        ylabel('phase [rad]')

        %end
    end
end

%% mode shapes computation

modes=mode_shapes(opt_3D,n_frf,n_res);

% parameters definition
theta_pos = 0:pi/12:11/12*pi;
theta_neg = -theta_pos;

line_undeformed = ['--', 'k'];
line_identified = ['-o', 'r'];
line_simmetry = 'b';

modes=9*ones(12,n_res)+modes;

figure('Position', [100, 100, 800, 400]) 


subplot(1,2,1)
polarplot(0:pi/12:2*pi, 9*ones(1, 25), line_undeformed, theta_pos, modes(:, 1), line_identified, theta_neg, modes(:, 1), line_simmetry, 'linewidth', 1.5)
ax = gca;
ax.ThetaZeroLocation = 'bottom';
ax.ThetaTick = 0:15:360;
%ax.RLim = [0 3];
ax.RTickLabel = {};
title('First axial mode')

subplot(1,2,2)
polarplot(0:pi/12:2*pi, 9*ones(1, 25), line_undeformed, theta_pos, modes(:, 2), line_identified, theta_neg, modes(:, 2), line_simmetry, 'linewidth', 1.5)
ax = gca;
ax.ThetaZeroLocation = 'bottom';
ax.ThetaTick = 0:15:360;
%ax.RLim = [0 3];
ax.RTickLabel = {};
title('Second axial mode')

legend('Undeformed', 'Axial mode shape (identified)', 'Axial mode shape (symmetry)', 'Location', 'northoutside')


%% interpolazione

pos_interp=0:pi/72:11*pi/12;

mode1=spline(theta_pos,modes(:,1),pos_interp);
mode2=spline(theta_pos,modes(:,2),pos_interp);


figure('Position', [100, 100, 800, 400]) 


subplot(1,2,1)
polarplot(0:pi/12:2*pi, 9*ones(1, 25), line_undeformed,theta_pos, modes(:, 1), 'o', pos_interp, mode1, '-r', -pos_interp, mode1, line_simmetry, 'linewidth', 1.5)
ax = gca;
ax.ThetaZeroLocation = 'bottom';
ax.ThetaTick = 0:15:360;
%ax.RLim = [0 3];
ax.RTickLabel = {};
title('First axial mode')

subplot(1,2,2)
polarplot(0:pi/12:2*pi, 9*ones(1, 25), line_undeformed,theta_pos, modes(:, 2), 'o', pos_interp, mode2, '-r', -pos_interp, mode2, line_simmetry, 'linewidth', 1.5)
ax = gca;
ax.ThetaZeroLocation = 'bottom';
ax.ThetaTick = 0:15:360;
%ax.RLim = [0 3];
ax.RTickLabel = {};
title('Second axial mode')

legend('Undeformed', 'Axial mode shape (identified)', 'Axial mode shape (interpolated)','Axial mode shape (symmetry)', 'Location', 'northoutside')


