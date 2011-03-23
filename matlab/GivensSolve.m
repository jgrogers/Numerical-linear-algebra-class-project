function [R, QTb] = GivensSolve(A,b)

R = [A b];
[m,n] = size(R);

for j=1:min(m,n)
    
    for i = m:-1:j+1
        a1 = R(j,j);
        a2 = R(i,j);
        [c,s] = cmp_giv(a1,a2);
        R([j,i],j:n)
        R([j, i],j:n) = [c -s; s c]*R([j,i],j:n);
        R([j,i],j:n)
    end
end
QTb = R(:,n);
R = R(:,1:n-1);

function [c,s] = cmp_giv(a,b)
a
b

    if (b == 0)
       c = 1; s = 0;
    else
        if (b > a)
            tau = -a /b;
            s = 1/sqrt(1+tau*tau);
            c = s * tau;
        else 
            tau = -b/a;
            c = 1/sqrt(1+tau*tau);
            s = c * tau;
        end
    end
    