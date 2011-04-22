function test_sparseqr(siz1,siz2)

A = sprand(siz1,siz2,0.7);
b = rand(siz1,1);

x1 = linear_qr_solve_sparse(A,b,true);
[q,r] = qr(A);

x2 = inv(r'*r)*r'*q'*b;
norm(x1-x2)