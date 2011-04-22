function [R, QTb,P] = GivensSolveSparse(A,b,usecolamd)
if usecolamd
    P = colamd(A);
    R = [A(:,P) b];
else
    R = [A b];
    P = zeros(size(A,1));
end
R = R';
[m,n] = size(R);
%figure
%spy(R);
nu = 0;
den = 0;
gv= tic;
for j=1:min(m,n)
    [col_j,col_i,col_s] = find(R(j,j+1:end));
    for ind = 1:size(col_i,2)
        i = col_i(ind) + j;
        a1 = full(R(j,j));
        a2 = full(R(j,i));%col_s(ind);%full(R(i,j));
        [c,s] = cmp_giv(a1,a2);
        ei = find(R(j:end,i));
        ej = find(R(j:end,j));
        co = union(ei,ej);
        nu = nu + 1;
        den = den + size(co,2);
        R(j-1+co,[j,i]) = R(j-1+co,[j,i])*[c s; -s c];
        R(j,i) = 0;
    end
end
% for j=1:min(m,n)
%     [col_i,col_j,col_s] = find(R(j+1:end,j));
%     
%     
%     for ind = 1:size(col_i)
%         i = col_i(ind) + j;
%         a1 = full(R(j,j));
%         a2 = col_s(ind);%full(R(i,j));
%         [c,s] = cmp_giv(a1,a2);
%         ei = find(R(i, j:end));
%         ej = find(R(j, j:end));
%         co = union(ei,ej);
%         nu = nu + 1;
%         den = den + size(co,2);
%         R([j, i],j-1+co) = [c -s; s c]*R([j,i],j-1+co);
%         R(i,j) = 0;
%     end
% end
R = R';
toc(gv);
QTb = R(:,m);
R = R(:,1:m-1);
sprintf('Average number of entries to update per row %f out of %d\n', den/nu, n)
function [c,s] = cmp_giv(a,b)
if(a == 0)
    f = 1;
end

    if (b == 0)
       c = 1; s = 0;
    else
        if (abs(b) > abs(a))
            tau = -a /b;
            s = 1/sqrt(1+tau*tau);
            c = s * tau;
        else 
            tau = -b/a;
            c = 1/sqrt(1+tau*tau);
            s = c * tau;
        end
    end
    