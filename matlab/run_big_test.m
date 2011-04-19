function run_big_test()

range = [];
for i =(1:10)
    rang = 0.1 * i
    result = execute_slam_both(22,0.4,rang,20, i < 6);
    results(i) = result;
    range(i) = rang;
end
plot(range(1:5), [results(1:5).spplain]);
hold on
plot(range, [results(:).spar], 'g');
hold on
plot(range, [results(:).dens],'r');
legend('sparse', 'sparse+COLAMD','dense');
xlabel('Sensor range in meters');
ylabel('Run time in seconds');
title('Comparison of performance of sparse with and without COLAMD reording vs dense');

hold off

figure
plot([results(:).density],[results(:).speedup]);
xlabel('Size of measurement matrix m*n');
ylabel('Speedup as % of dense run time');
title('Speedup for sparse method over dense compared to problem size');