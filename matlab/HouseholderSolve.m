function [R,QTb] = HouseholderSolve(A,b)
%I looked at a book that I have "Scientific Computing" by 
% Michael T. Heath to help me with this algorithm
%This is made more efficient by updating b in place and not
%accumulating Q

[m,n] = size(A);
R = A;
QTb = b;
for k=1:n
    alpha = sign(R(k,k)) * sqrt(R(k:end,k)'*R(k:end,k));
    
    vk = R(:,k);
    vk(1:k-1) = 0;
    vk(k) = vk(k) + alpha;
    betak  = vk'*vk;
    if (betak == 0)
        %Rank def, skip this col
    else
        for j = k:n
            gamma_j = vk'*R(:,j);
            R(:,j)= R(:,j)-2*(gamma_j/betak)*vk;
        end
        gamma_j = vk' * QTb;
        QTb = QTb - 2*(gamma_j / betak) * vk;
    end
    
end
