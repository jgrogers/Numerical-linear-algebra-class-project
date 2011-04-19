function [R, QTb,P] = GivensSolveSparse(A,b,usecolamd)
if usecolamd
    P = colamd(A);
    R = [A(:,P) b];
else
    R = [A b];
    P = zeros(size(A,1));
end
[m,n] = size(R);
%figure
%spy(R);
nu = 0;
den = 0;
gv= tic;
for j=1:min(m,n)
    a = find(R(j+1:end,j));
    
    for (ind = 1:size(a))
        i = a(ind) + j;
        a1 = full(R(j,j));
        a2 = full(R(i,j));
        [c,s] = cmp_giv(a1,a2);
        ei = find(R(i, j:end));
        ej = find(R(j, j:end));
        co = union(ei,ej);
        nu = nu + 1;
        den = den + size(co,2);
        for (ind2 = 1:size(co,2))
           f = co(ind2)+j-1;
           rj = full(R(j,f));
           ri = full(R(i,f));
            x = c * rj - s *ri;
            y = s * rj + c* ri;
            R(j,f) = x;
            R(i,f) = y;
 %       close
 %      spy(R);

        end
%        R([j, i],j:n) = [c -s; s c]*R([j,i],j:n);
        R(i,j) = 0;
    end
end
toc(gv);
QTb = R(:,n);
R = R(:,1:n-1);
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
    