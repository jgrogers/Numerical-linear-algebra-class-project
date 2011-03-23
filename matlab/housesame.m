function [Q,R] = housesame(A)
%I looked at a book that I have "Scientific Computing" by 
% Michael T. Heath to help me with this algorithm
[m,n] = size(A);
Q = eye (m, m);
R = A;
for k=1:n
    alpha = sign(R(k,k)) * sqrt(R(k:end,k)'*R(k:end,k));
    
    vk = R(:,k);
    vk(1:k-1) = 0;
    vk(k) = vk(k) + alpha;
    betak  = vk'*vk;
    if (betak == 0)

    else
        for j = k:n
            gamma_j = vk'*R(:,j);
            R(:,j)= R(:,j)-2*(gamma_j/betak)*vk;
        end
        Pk = eye(m,m) - (2*vk*vk')./betak;
        Q = Q*Pk;
    end
    
end
