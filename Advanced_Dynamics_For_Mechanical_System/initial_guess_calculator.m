function [x0]=initial_guess_calculator(nat_f,frf,locs,df)

nat_f=nat_f.*2.*pi;

number_resonances=size(nat_f,1);
Ar0=zeros(number_resonances,size(frf,2));
xi_exp0=zeros(number_resonances,size(frf,2));
x0=zeros(size(frf,2),number_resonances,5);


for ii=1:size(frf,2)
%estimation of non-dimensional damping and Ar
Ar=zeros(number_resonances,1);
xi_exp=zeros(number_resonances,1);
for j=1:number_resonances
    dphase=(angle(frf(locs(j)+1,ii))-angle(frf(locs(j)-1,ii)))./(2.*df.*2.*pi); 
    xi_exp(j)=1./(abs(dphase).*nat_f(j,ii));
    Ar(j)=frf(locs(j),ii).*(-nat_f(j,ii).^2 + 1i.*2.*xi_exp(j).*nat_f(j,ii).*nat_f(j,ii) + nat_f(j,ii).^2);
end
Ar0(:,ii)=Ar;
xi_exp0(:,ii)=xi_exp;
end

% initial guess residuals
Rl0=0*Ar0;
Rh0=0*Ar0;

x0(:,:,1)=nat_f';
x0(:,:,2)=xi_exp0';
x0(:,:,3)=Ar0';
x0(:,:,4)=Rl0';
x0(:,:,5)=Rh0';

%x0=[nat_f(j,:)', xi_exp0(j,:)', Ar0(j,:)', Rl0(j,:)', Rh0(j,:)'];

end