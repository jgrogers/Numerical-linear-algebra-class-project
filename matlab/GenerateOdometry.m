function odo = GenerateOdometry(traj_true)
odo=  [];
for i = 2:size(traj_true,2) 
    
    n = gen_noise(3,0.01);
    rel_odo = GetRelPose(traj_true(i-1), traj_true(i));
    odo(i-1).x = rel_odo.x + n(1);
    odo(i-1).y = rel_odo.y + n(2);
    odo(i-1).theta = rel_odo.theta + n(3);
end

function rel_pose = GetRelPose(p1, p2)
c1 = cos(p1.theta);
c2 = cos(p2.theta);
s1 = sin(p1.theta);
s2 = sin(p2.theta);

c = c1*c2 + s1*s2;
s = -s1*c2+ c1*s2;
R = atan2(s,c);
dx = p2.x - p1.x;
dy = p2.y - p1.y;
xrel = c1*dx + s1*dy;
yrel = -s1*dx + c1*dy;
rel_pose.x = xrel;
rel_pose.y = yrel;
rel_pose.theta = R;
%rel_pose.x = p2.x - p1.x;
%rel_pose.y = p2.y - p1.y;
%rel_pose.theta = wrapToPi(p2.theta - p1.theta);



function n = gen_noise(sz,scale)
n = zeros(sz,1);
n(1) = (rand(1,1) - 0.5)*scale;
n(2) = (rand(1,1) - 0.5)*scale;