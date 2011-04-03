function lm = gen_rand_landmarks(num,range)
lm = struct('x',{},'y',{});
for i = 1:num
    lm(i).x = rand(1,1) * range - range/2;
    lm(i).y = rand(1,1) * range - range/2;

end
