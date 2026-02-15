clc
clearvars
close all

%% data characterisation

load('danno_1.mat')
time_tot_1=size(y,1)*(1/fsamp);

fs_1=fsamp;
euv_1=euv(1);
y=[y(:,1),y(:,3),y(:,4),y(:,2)];
y_1=y./euv_1;
clear euv
clear y


load('danno_2.mat')
time_tot_2=size(y,1)*(1/fsamp);

fs_2=fsamp;
euv_2=euv(1);
y=[y(:,1),y(:,3),y(:,4),y(:,2)];
y_2=y./euv_2;
clear euv
clear y

load('sana.mat')
time_tot_s=size(y,1)*(1/fsamp);
y=[y(:,1),y(:,3),y(:,4),y(:,2)];
fs_s=fsamp;
euv_s=euv(1);
y_s=y./euv_s;

clear y

%% data plot
pos=[5,20,50,90]; % accelerometers positions

fs=fs_1;
dt=1/fs;
dt_s=1/fs_s;
t_vect=0:dt:(time_tot_1-dt);
t_vect_s=0:dt_s:(time_tot_s-dt_s);

for ii=1:4

    figure()
    subplot(3,1,1)
    plot(t_vect./60,y_1(:,ii))
    grid on
    ylabel('acceleration [m/s^2]')
    xlabel('time [min]')
    title('damage 1')

    subplot(3,1,2)
    plot(t_vect./60,y_2(:,ii))
    grid on
    ylabel('acceleration [m/s^2]')
    xlabel('time [min]')
    title('damage 2')

    subplot(3,1,3)
    plot(t_vect_s./60,y_s(:,ii))
    grid on
    ylabel('acceleration [m/s^2]')
    xlabel('time [min]')
    title('healty')


    sgtitle(strcat(num2str(pos(ii)),'% L'))
    
end

%% split in multiple records
n_records=3;
n_points_rec=round(size(y_s,1)/n_records);
y_s_new=zeros(n_points_rec,size(y_s,2),n_records);

for ii=1:n_records
    if ii==1
        y_s_new(:,:,ii)=y_s(1:n_points_rec,:);
    else
        y_s_new(:,:,ii)=y_s((ii-1)*n_points_rec+1:ii*n_points_rec,:);
    end
end

%% window 
for ii=1:n_records
    t_wind=40; % seconds
    N_w=round(t_wind*fs_s); % #points in the window
    N_overlap=floor(0.50*N_w); % #overlapping points
    win=hanning(N_w);

%% mode shapes computations 

    [psi,S,G,U,V,f_vect]=cross_spect_svd(y_s_new(:,:,ii),fs_s,N_w,N_overlap,win);

end
%% eigenfrequencies identification
    df=fs_s/N_w;

    [~,locs]=findpeaks(exp(S(1,:)), 'MinPeakDdistance', round(10/ds), 'MinPeakHeight',5e-8);

eigenfreq=f_vect(locs);

%% restriction to eigenmodes 

psi=real(psi(:,locs))';

%% plot
x_vect=[0,5,25,50,90,100];
mode=[zeros(4,1),psi,zeros(4,1)];
figure
plot(x_vect,mode)
legend('1','2','3','4')
