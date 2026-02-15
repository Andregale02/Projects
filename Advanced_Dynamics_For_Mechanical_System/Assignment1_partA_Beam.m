clearvars
close all
clc
tic


%% DATA

L=1.2; %length [m]
h=8e-3; %thickness [m]
b=40e-3; %width [m]
rho=2700; %density [Kg/m3]
E=68e9; %young's modulus [Pa]

m=rho*b*h; %distributed mass
J=(b*h^3)/12; %moment of inertia
EJ=E*J; %rotational stiffness

%% Discrete domains

df=1e-2;
fmax=200; % max frequency, tbc
f_vect=0:df:fmax;

dx=1e-3;
x=0:dx:L;

omega=f_vect*2*pi;

%% boundary conditions

% gamma=sqrt(omega).*(EJ/m).^(-1/4)

H=@(omega) [1   0   1   0;
            0   1   0   1;
            -cos(L.*sqrt(omega).*(EJ/m).^(-1/4)) -sin(L.*sqrt(omega).*(EJ/m).^(-1/4))   cosh(L.*sqrt(omega).*(EJ/m).^(-1/4))    sinh(L.*sqrt(omega).*(EJ/m).^(-1/4));
            sin(L.*sqrt(omega).*(EJ/m).^(-1/4)) -cos(L.*sqrt(omega).*(EJ/m).^(-1/4))    sinh(L.*sqrt(omega).*(EJ/m).^(-1/4))    cosh(L.*sqrt(omega).*(EJ/m).^(-1/4))
            ];

dets=zeros(length(omega),1);
for j=1:length(omega)
    dets(j)=det(H(omega(j)));
end

figure(1)
semilogy(f_vect,abs(dets))
hold on
grid on

locs=find(islocalmin(abs(dets)));
nat_f=omega(locs);

%[~,locs]=findpeaks(-abs(dets),"MinPeakDistance",2,"MinPeakHeight",-0.1);
%nat_f=omega(locs);

plot(f_vect(locs),abs(dets(locs)),'o')
title('Determinant')

z=ones(4);
for j=1:length(nat_f)
    HH=H(nat_f(j));
    z(2:end,j)=HH(2:end,2:end)\-HH(2:end,1);
end

%% eigenmodes computation

PHI=zeros(length(x),length(nat_f));

for j=1:length(nat_f)
    phi=@(x) z(1,j)*cos(sqrt(nat_f(j)).*((EJ/m).^(-1/4)).*x) + z(2,j)*sin(sqrt(nat_f(j)).*((EJ/m).^(-1/4)).*x) + z(3,j)*cosh(sqrt(nat_f(j)).*((EJ/m).^(-1/4)).*x) + z(4,j)*sinh(sqrt(nat_f(j)).*((EJ/m).^(-1/4)).*x);
    PHI(:,j)=phi(x);
end

PHI=PHI./2;
normal=ones(size(PHI,1),1)*PHI(size(PHI,1),:);
PHI=PHI./normal;

color=[0.8500 0.3250 0.0980;0.9290 0.6940 0.1250; 0.4940 0.1840 0.5560;0.4660 0.6740 0.1880];
color_modes=color(1:size(PHI,2),:);

figure(2)
plot(x,PHI(:,1),x,PHI(:,2),x,PHI(:,3),x,PHI(:,4),linewidth=1)
title('Mode shapes')
xlabel('[m]')
ylabel('[m]')
grid on
ax.ColorOrder = color_modes;

%%
x_j=[0.2 0.4 0.6 1.2]; %response measurment points non usare 0.7
x_k= 1.2*ones(1,length(x_j)); %application of the force  
Gjk_mat=zeros(length(omega),length(x_k));

%% visualizzazione grafico
% 
label_modes=cell(1,2*length(nat_f));
for l=1:length(nat_f)
     label_modes(l)={strcat("Mode ",num2str(l))};
end
for l=1+length(nat_f):2*length(nat_f)
     label_modes(l)={strcat("Mode ",num2str(l-length(nat_f))," num")};
end
% 
label=cell(1,length(x_j));
for l=1:length(x_j)
    label(l)={strcat('x_j=',num2str(x_j(l)),'m x_k=',num2str(x_k(l)),'m')};
end
label_exp_num= [strcat(label,' Exp'),strcat(label,' Num')];

 

%% FRF analitiche
for ii=1:length(x_j)


pos_xj=1+x_j(ii)/dx;
pos_xk=1+x_k(ii)/dx;

xi=0.01; % adimensional damping

Gjk=@(omega) 0;
m_i=zeros(length(nat_f),1);

for j=1:length(nat_f)
    m_i(j)=m.*trapz(x,PHI(:,j).^2);
    Gjk=@(omegavar) Gjk(omegavar) + PHI(pos_xj,j).*PHI(pos_xk,j)./(m_i(j).*((-omegavar.^2) + 1i.*2.*xi.*omegavar.*nat_f(j) + nat_f(j).^2));
    Gjk_mat(:,ii)=Gjk(omega);
end
% il for precedente secondo me è una porcata (Ale). 1- Nonn ha senso
% definire la funzione nel for. 2- Si può eliminare il for e rendere tutto
% matriciale 

%%attiva il seguente codice per vedere i plot delle FRF exp per ciascuna coppia separatamente
figure(ii+2)    
subplot(2,1,2)
plot(f_vect,angle(Gjk_mat(:,ii)),linewidth=1.5)%,DisplayName=strcat('x_j = ',num2str(x_j(ii)),' x_k = ',num2str(x_k(ii))))
title('FRF phase')
xlabel('Frequency [Hz]')
ylabel('[rad]')
grid on
hold on
% legend(label(ii))


subplot(2,1,1)
semilogy(f_vect,abs(Gjk_mat(:,ii)),linewidth=1.5)%,DisplayName=strcat('x_j = ',num2str(x_j(ii)),' x_k = ',num2str(x_k(ii))))
title('FRF magnitude')
xlabel('Frequency [Hz]')
ylabel('[m/N]')
grid on
hold on
% legend(label(ii))
%sgtitle(strcat("Input in x_k = ",num2str(x_k(ii))," m, output in x_j = ",num2str(x_j(ii))," m"))
end

%% plotta le FRF di ogni coppia tutte insieme  

figure()    
subplot(2,1,2)
plot(f_vect,unwrap(angle(Gjk_mat)),linewidth=1.5)%,DisplayName=strcat('x_j = ',num2str(x_j(ii)),' x_k = ',num2str(x_k(ii))))
title('FRF phase')
xlabel('Frequency [Hz]')
ylabel('[rad]')
grid on
hold on
legend(label)



subplot(2,1,1)
semilogy(f_vect,abs(Gjk_mat),linewidth=1.5)%,DisplayName=strcat('x_j = ',num2str(x_j(ii)),' x_k = ',num2str(x_k(ii))))
title('FRF magnitude')
xlabel('Frequency [Hz]')
ylabel('[m/N]')
grid on
hold on
legend(label)
sgtitle("Experimental FRFs",'Fontweight', 'bold')

%% EXPERIMENTAL APPROACH - simplified approach
Gjk_exp=Gjk_mat;

for ii=1:length(x_k)
%estimation of the natural frequencies
[~,locs_exp]=findpeaks(abs(Gjk_exp(:,ii))); 

%per escludere i finti picchi quando eccitiamo/misuriamo nei nodi, andiamo a valutare anche la fase 
for j=1:length(locs_exp)
    delta_phase=(angle(Gjk_exp(locs_exp(j)+1,ii))-angle(Gjk_exp(locs_exp(j)-1,ii)));
    if delta_phase>=0
        locs_exp(j)=0;
    end

end

locs_exp=locs_exp(find(locs_exp)); % eliminazione dei finti picchi
nat_f_exp(:,ii)=omega(locs_exp);
number_resonances=size(nat_f_exp,1);


%estimation of non-dimensional damping and Ar
Ar=zeros(number_resonances,1);
xi_exp=zeros(number_resonances,1);
for j=1:number_resonances
    dphase=(angle(Gjk_exp(locs_exp(j)+1))-angle(Gjk_exp(locs_exp(j)-1)))./(2.*df.*2.*pi); 
    xi_exp(j)=1./(abs(dphase).*nat_f_exp(j));
    Ar(j)=Gjk_exp(locs_exp(j),ii).*(-nat_f_exp(j,ii).^2 + 1i.*2.*xi_exp(j).*nat_f_exp(j,ii).*nat_f_exp(j,ii) + nat_f_exp(j,ii).^2);
end
Ar0(:,ii)=real(Ar)+1e-7i;
xi_exp0(:,ii)=xi_exp;
end

% initial guess residuals
Rl0=1e-5*Ar0;
Rh0=1e-5*Ar0;


%% Parameters estimation
opt_3D=zeros(length(x_k),5,number_resonances); %  modifica 3 in 5

% definizione range nat freq j
rangewidth_num=2.5/df; %range indici frequency range (5000 = 0.5Hz)
rangewidth_exp=4/df;

for j=1:number_resonances

approx_range_locs_num = (locs_exp(j)-rangewidth_num):(locs_exp(j)+rangewidth_num); % indici frequeny range numerical
plotwidth_f_exp = f_vect((locs_exp(j)-rangewidth_exp):(locs_exp(j)+rangewidth_exp)); % frequeny range experimental
plotwidth_f_num (:,j) = f_vect(approx_range_locs_num); % frequeny range numerical

Gjk_exp_range_j = Gjk_exp(approx_range_locs_num,:); % Gjk exp valuatata in range j (diventa vettore)
omega_range_j = approx_range_locs_num*df*2*pi; % omega range j
omega_range_mat = ones(length(x_k),1)*omega_range_j;

%Gjk_num_range_j = @(var) var(3)./(-omega_range_j.^2 + 1i.*2.*var(2).*var(1).*omega_range_j + var(1).^2)+var(4)./(omega_range_j.^2)+var(5);
Gjk_num_range_j = @(var) var(:,3)./(-omega_range_mat.^2 + 1i.*2.*var(:,2).*var(:,1).*omega_range_mat + var(:,1).^2)+var(:,4)./(omega_range_mat.^2);%+var(:,5);%; % numerical FRF 
% var = [natff, xii, Arr, Rll, Rhh] vettore di variabili da approssimare

%% least square error minimisation

Err=@(var) sum(sum(real(Gjk_exp_range_j-Gjk_num_range_j(var)').^2)+imag(Gjk_exp_range_j-Gjk_num_range_j(var)').^2); % Error def
%Err=@(var) sum(sum(abs(Gjk_exp_range_j-Gjk_num_range_j(var)').^2)); % Error def

x0=[nat_f_exp(j,:)', xi_exp0(j,:)', Ar0(j,:)', Rl0(j,:)', Rh0(j,:)']; % inital trial vector


OPTIONS = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt','MaxFunctionEvaluations',1e4,'Display', 'off'); 
opt_mat=lsqnonlin(Err, x0, [], [], OPTIONS); % minimisation
Gjk_final(:,:,j)=Gjk_num_range_j(opt_mat);

% creazione matrice 3D dei parametri ottimizzati colonne: righe: coppie, [natff, xii, Arr, Rll, Rhh], spessore: risonanze
opt_3D(:,:,j)=opt_mat;


% figure(3)
% subplot(2,1,1)
% semilogy(plotwidth_f_num, abs(Gjk_num_range_j(opt_mat)), '.')%, plotwidth_f_num, abs(Gjk_num_range_j(x0)), '*')
% legend('E1','E2','E3','N1','N2','N3')
% grid on
% %legend('Experimental','Numerical')
% title('Magnitude');
% ylabel('m/N')
% xlabel('frequency [Hz]')
% subplot(2,1,2)
% plot(plotwidth_f_num, unwrap(angle(Gjk_num_range_j(opt_mat))), 'o')
% grid on
% legend('E1','E2','E3','N1','N2','N3')
% %legend('Experimental','Numerical')
% title('Phase');
% ylabel('rad')
% xlabel('frequency [Hz]')


end
toc
%% plot


for ii=1:length(x_j)
for jj=1:number_resonances

% if ii~=3 || jj~=3
figure(ii+2)
subplot(2,1,1)
semilogy(plotwidth_f_num (:,jj), abs(Gjk_final(ii,:,jj)), 'o',Markersize=3,LineWidth=0.3)%, plotwidth_f_num, abs(Gjk_num_range_j(x0)), '*'
grid on
hold on
%title('Magnitude');
%ylabel('m/N')
%xlabel('frequency [Hz]')
%legend(label_exp_num)
%ax = gca; 
%ax.ColorOrder = color_frf;


subplot(2,1,2)
plot(plotwidth_f_num (:,jj), angle(Gjk_final(ii,:,jj)), 'o',Markersize=3,LineWidth=0.3)
grid on
hold on
%title('Phase');
%ylabel('rad')
%xlabel('frequency [Hz]')
%sgtitle(strcat("Resonance mode ",num2str(j)),'Fontweight', 'bold')

%legend(label_exp_num)
% end
end
end
%% calcolo mode shapes numeriche

figure(2)
hold on
mode=zeros(length(x_j),number_resonances);

for i=1:number_resonances
    Gjk_resonance=@(var) var(:,3)./(-opt_3D(:,1,i).^2 + 1i.*2.*var(:,2).*var(:,1).*opt_3D(:,1,i) + var(:,1).^2)+var(:,4)./(opt_3D(:,1,i).^2)+var(:,5); % numerical FRF pair 1
    mode(:,i)=-imag(Gjk_resonance(opt_3D(:,:,i)));
end
normalizzazione=ones(length(x_k),1)*mode(length(x_k),:);
mode=mode./normalizzazione;
plot(x_j,mode,'*',Linewidth=1.5)
ax = gca; 
ax.ColorOrder = color_modes;
legend(label_modes)
