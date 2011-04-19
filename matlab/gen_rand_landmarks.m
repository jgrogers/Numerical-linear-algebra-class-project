function lm = gen_rand_landmarks(res,range)
lm = struct('x',{},'y',{});
ind = 1;
for i = -range:res:range
    for j = -range:res:range
        lm(ind).x = i;
        lm(ind).y = j;
        ind = ind + 1;
    end
end

%for i = 1:num
%    lm(i).x = rand(1,1) * range - range/2;
%    lm(i).y = rand(1,1) * range - range/2;
%end
