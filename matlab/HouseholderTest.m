%Unit test for HouseholderSolve.m
function HouseholderTest
testit(100,50)
testit(50,50)
testit(50,100)
testit(100,100)
testit(10000,100)

function err = testit(rows, cols)
A = rand(rows, cols);
b = rand(rows,1);
tic
[R, QTb] = HouseholderSolve(A,b);
toc
x1 = pinv(R'*R)*R'*QTb;
tic
[q,r] = qr(A);
toc
x2 = pinv(r'*r)*r'*q'*b;
err = norm(x1 - x2);

