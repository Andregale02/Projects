function [psi,S,G,U,V,f_vect]=cross_spect_svd(signal_matrix,fs,N_w,N_overlap,win)
% [psi,S,G,U,V]=cross_spect_svd(signal_matrix,fs,N_w,N_overlap,win)
% computes cross spectrum outut matrix and performs the SVD of it
%
% INPUT:
% *signal matrix: signal to be analysed [N x n_channels]
% *fs: sampling frequency
% *N_w: number of points of the window
% *N_overlap: number of overlapping points
% *win: window vector
%
% OUTPUT:
% *psi: mode shapes matrix
% *S: matrix having singular values on each row 
% *G: cross spectrum matrix
% *U,V: singular value decomposition matrices
% *f_vect: frequency vector

%% SVD

channels=size(signal_matrix,2);
G=[];
for r=1:channels
    for c=1:channels
        [g, f_vect]=autocross(signal_matrix(:,r),signal_matrix(:,c),fs,N_w,N_overlap,win);
        G(r,c,:)=g; %cross spectrum matrix
    end
end

psi=[];
n_f=size(G,3);
S=zeros(channels,n_f);

for p=1:n_f
    [U,s,V] = svd(G(:,:,p));
    S(:,p)=diag(s); % Matrix having singular values on each row

    psi=[psi,U(:,1)]; % mode shapes matrix
end

figure() 
semilogy(f_vect,S)
title('Singular values')
legend('s.v. 1','s.v. 2','s.v. 3','s.v. 4')
grid on
xlabel('frequency [Hz]')

end