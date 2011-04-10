function X = linear_qr_solve_sparse(A, b)

[R, QTb] = GivensSolveSparse(A,b);

X = tri_solve(R, QTb);



function x = tri_solve(R, b)
%Assuming R is upper triangular
m = size(b,1);
n = size(R,2);
x = zeros(n,1);
for i = n:-1:1
    for j = i+1:n
       b(i) = b(i) - R(i,j)*x(j);
    end
    x(i) = b(i) / R(i,i);
end