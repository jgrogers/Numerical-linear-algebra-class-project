function run_big_test()
sparr = [];
dearr = [];
densityarr = [];
szarr = [];
for i =(1:20)
    [sp, de, density, sz] = execute_slam_both(20,1500,0.05*i,20)
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
