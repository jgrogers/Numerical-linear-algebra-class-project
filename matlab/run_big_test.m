function run_big_test()
sparr = [];
dearr = [];
densityarr = [];
szarr = [];
for i =(1:5)
    [sp, de, density, sz] = execute_slam_both(20,15+i*5,2,20)
    sparr(i) = sp;
    dearr(i) = de;
    densityarr(i) = density;
    szarr(i) = sz;
end

plot(sparr);
hold on
plot(dearr,'r');

hold off

figure
plot(sparr, densityarr, 'bx');
hold on
plot(dearr, densityarr, 'r+');
