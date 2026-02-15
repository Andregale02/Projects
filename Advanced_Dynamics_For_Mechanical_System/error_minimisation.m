function [opt_mat,Gjk_final] = error_minimisation(x0,Gjk_exp,rangewidth_num,locs_exp,df)
%[opt_mat,Gjk_final] = error_minimisation(x0,Gjk_exp,rangewidth_num,locs_exp,df)
%Inputs:
%opt_mat parameter optimizer in the form: [natff, xii, Arr, Rll, Rhh] for a certain resonance
%Gjk_final: numerical transfer function approximated in the range
%Outputs:
%x0:[natff, xii, Arr, Rll, Rhh] for a certain resonance
%Gjk_exp: experimental t.f.
%rangewidth_num: length of the numerical approximation
%locs_exp: locs of the resonance peaks 
%df: freq. resolution

approx_range_locs_num = (locs_exp-rangewidth_num):(locs_exp+rangewidth_num); % indici frequeny range numerical
Gjk_exp_range_j = Gjk_exp(approx_range_locs_num,:); % Gjk exp valuatata in range j (diventa vettore)
omega_range_j = approx_range_locs_num*df*2*pi; % omega range j
omega_range_mat = ones(size(Gjk_exp,2),1)*omega_range_j;

% var = [natff, xii, Arr, Rll, Rhh] vettore di variabili da approssimare
Gjk_num_range_j = @(var) var(:,3)./(-omega_range_mat.^2 + 1i.*2.*var(:,2).*var(:,1).*omega_range_mat + var(:,1).^2)+var(:,4)./(omega_range_mat.^2)+var(:,5); % numerical FRF
%scalar version:Gjk_num_range_j = @(var) var(3)./(-omega_range_j.^2 + 1i.*2.*var(2).*var(1).*omega_range_j + var(1).^2)+var(4)./(omega_range_j.^2)+var(5); 

%% least square error minimisation

Err=@(var) sum(sum(real(Gjk_exp_range_j-Gjk_num_range_j(var)').^2)+imag(Gjk_exp_range_j-Gjk_num_range_j(var)').^2); % Error def

OPTIONS = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt','MaxFunctionEvaluations',1e4,'Display', 'off'); 
opt_mat=lsqnonlin(Err, x0, [], [], OPTIONS); % minimisation
Gjk_final=Gjk_num_range_j(opt_mat);

end