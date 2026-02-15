function mode = mode_shapes(opt_3D,n_frf,n_res)

mode=zeros(n_frf,2);
for i=1:n_res
    Gjk_resonance=@(var) var(:,3)./(-opt_3D(:,1,i).^2 + 1i.*2.*var(:,2).*var(:,1).*opt_3D(:,1,i) + var(:,1).^2)+var(:,4)./(opt_3D(:,1,i).^2)+var(:,5); % numerical FRF range 1
    mode(:,i)=-imag(Gjk_resonance(opt_3D(:,:,i)));
end
normalizzazione=ones(n_frf,1)*mode(n_frf,:);
mode=mode./normalizzazione;
end