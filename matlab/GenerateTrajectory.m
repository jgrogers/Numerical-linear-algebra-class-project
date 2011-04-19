function traj = GenerateTrajectory(num_steps)
p.x = 0;
p.y = 0;
p.theta =0;
traj = struct('x',{},'y',{},'theta',{});
v.x = 0.5;%(rand(1,1)-0.5)*1.0;
v.y = 0;
v.theta = 0.314;%1.0;%(rand(1,1)-0.5)*1.0;
traj(1) = p;
for i = 2:num_steps
    p.x = p.x + v.x*cos(p.theta+v.theta/2);
    p.y = p.y + v.x * sin(p.theta +v.theta/2);
    p.theta = p.theta + v.theta;
    %v = rand_acc(v,0.2);
    traj(i) = p;
end


function v = rand_acc(vin, mag)
a.x = (rand(1,1) - 0.5) * mag;
a.y = 0;
a.theta = (rand(1,1)-0.5) * mag;

v.x = vin.x + a.x;
v.y = vin.y + a.y;
v.theta = vin.theta + a.theta;

v.theta = thresh(v.theta, 0.4);
v.x = thresh(v.x, 4.5);
if (v.x < 0) 
    v.x = 0;
end
v.y = thresh(v.y, 4.5);
if (v.y < 0) 
    v.y = 0;
end

function b = thresh(bin, val)
b = bin;
if (bin > val) 
    b = val;
end
if (bin < -val)
    b = -val;
end

