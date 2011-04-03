function [z, da] = get_measurements(lm, posearr, range)
z = {};
da = {};
for i = 1:size(posearr,2)
    [zi, dai] = get_measurement(lm, posearr(i), range);
    z{i} = zi;
    da{i} = dai;
end



function [z,da] = get_measurement(lm, pose, range)
cnt = 1;
z = struct('range',{},'bearing',{});
da = zeros(0,0);
for i = 1:size(lm,2)
    lm_local = transform_to(pose, lm(i));
    if (inrange(lm_local, range))
        z(cnt) = measure(lm_local);
        n = gen_noise(2,0.01);
        z(cnt).range = z(cnt).range + n(1);
        z(cnt).bearing = z(cnt).bearing + n(2);
        da(cnt) = i;
        cnt = cnt + 1;
    end
end


function n = gen_noise(sz,scale)
n = zeros(sz,1);
n(1) = (rand(1,1) - 0.5)*scale;
n(2) = (rand(1,1) - 0.5)*scale;

function lm_out = transform_to(pose, lm)
dx = lm.x - pose.x;
dy = lm.y - pose.y;
lm_out.x = dx* cos(pose.theta) - dy * sin(pose.theta);
lm_out.y = dx* sin(pose.theta) + dy * cos(pose.theta);

function t = inrange(lm_local, range)
dist = sqrt(lm_local.x^2 + lm_local.y^2);
t = dist < range;

function m = measure(lm_local)
m.range = sqrt(lm_local.x^2 + lm_local.y^2);
m.bearing = atan2(lm_local.y, lm_local.x);
