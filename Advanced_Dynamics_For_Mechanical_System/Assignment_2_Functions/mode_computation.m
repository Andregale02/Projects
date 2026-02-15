function [Q]=mode_computation(modes,incidenze,l,gamma,pos,F)

n_gdl=size(modes,1);

Q=[];

% looping on the finite elements
for k=15:17
% building the nodal displacements vector of each element in the global
% reference frame
    csi=[];
    Q_i=[];
    %looping through modes

    % chose the points in the chosen beem
    for ii=1:length(pos)
        if pos(ii)>=(k-15)*l(k) && pos(ii)<(k-14)*l(k)
            csi=[csi,pos(ii)-(k-15)*l(k)]; %position inside the element
        end
    end

    for jj=1:size(modes,2)

        mode=modes(:,jj);

        for iri=1:6
            if incidenze(k,iri) <= n_gdl
            xkG(iri,1)=mode(incidenze(k,iri));
            else
            xkG(iri,1)=0.;
            end
        end

        % Global to Local reference frame rotation
        lambda = [ cos(gamma(k)) sin(gamma(k)) 0. 
                  -sin(gamma(k)) cos(gamma(k)) 0.
	                    0.      0.     1. ] ;
        Lambda = [ lambda     zeros(3)
                  zeros(3)   lambda      ] ;
        xkL=Lambda*xkG;
    
        % Computing the axial (u) and transversal (w) displacements by means of
        % shape functions        
    
        fu=zeros(6,length(csi));
        fu(1,:)=1-csi/l(k);
        fu(4,:)=csi/l(k);
        u=(fu'*xkL)';
        fw=zeros(6,length(csi));
        fw(2,:)=2*(csi/l(k)).^3-3*(csi/l(k)).^2+1;
        fw(3,:)=l(k)*((csi/l(k)).^3-2*(csi/l(k)).^2+csi/l(k));
        fw(5,:)=-2*(csi/l(k)).^3+3*(csi/l(k)).^2;
        fw(6,:)=l(k)*((csi/l(k)).^3-(csi/l(k)).^2);
        w=(fw'*xkL)';
        % Local to global transformation of the element's deformation
        xyG=lambda(1:2,1:2)'*[u+csi;w];
        yG=xyG(2,:);

        % compute Q for all points at a specific mode
        Q_i=[Q_i;yG.*F];
    end
    % modes on rows, points on columns
    Q=[Q,Q_i];
end
end
