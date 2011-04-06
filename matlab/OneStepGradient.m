function [trajout, lmout,resid,dx] = OneStepGradient(traj, lm, odo, z, da)

%assume I have range bearing measurements on some landmarks
%da is ground truth data association
ind = 1;


numposeidx = 3 * size(traj,2);
lmidx_map = containers.Map('KeyType','int32','ValueType','int32');
landmark_index_counter = 1;
total_meas = 0;
for i = 1:size(traj,2)
    total_meas = total_meas + size(da{i},2);
    for j=1:size(da{i},2)
        lmnum = da{i}(j);
        if (lmidx_map.isKey(lmnum))
        else
            lmidx_map(lmnum) = landmark_index_counter;
            landmark_index_counter = landmark_index_counter + 2;
        end
    end
end
odo_meas = size(odo,2);
A = zeros(total_meas*2+3 + odo_meas*3, numposeidx + landmark_index_counter-1);
H = zeros(total_meas*2+3+odo_meas*3,1);
A(1:3,1:3) = -eye(3,3) * 10000;
zeropose.x = 0;
zeropose.y = 0;
zeropose.theta = 0;
H(1:3,1) = 10000 * getodoerr(zeropose, traj(1), zeropose)';
for i = 2:size(traj,2)
   %odometry
   [odoerr, dododx1, dododx2] = getodoerr(traj(i-1), traj(i), odo(i-1));
   H(3+(i-2)*3+1:3+(i-1)*3,1) = odoerr;
   A(3+(i-2)*3+1:3+(i-1)*3,3+(i-3)*3+1:3+(i-2)*3) = dododx1;
   A(3+(i-2)*3+1:3+(i-1)*3,3+(i-2)*3+1:3+(i-1)*3) = dododx2;
   
end
measidx = 4 + size(odo,2)*3;
for i = 1:size(traj,2)
    for j=1:size(da{i},2)
        lmnum = da{i}(j);
        [h,dhdx,dhdlm] = gethpt(traj(i), lm(lmnum), z{i}(j));
        %dhdx = getdhdx(traj(i), lm(lmnum));
        %dhdlm = getdhdlm(traj(i), lm(lmnum));
        poseidx = (i-1)*3 + 1;
        lmidx = numposeidx + lmidx_map(lmnum);
        H(measidx:measidx+1,1) = -h';
        A(measidx:measidx+1,poseidx:poseidx+2) = dhdx;
        A(measidx:measidx+1,lmidx:lmidx+1) = dhdlm;
        measidx = measidx + 2;
    end
end
A;
H;
%solve for the correction for error
dx = linear_qr_solve(A, H);
%[q,r] = qr(A);
%dx = inv(r'*r)*r'*q'*H;
%dx = tri_solve(r, q'*H)
%Now, apply the correction to the trajectory and the landmarks
trajout = traj;
lmout = lm;
resid = norm(H);
for i= 1:size(traj,2)
    trajout(i).x = traj(i).x + dx((i-1)*3+1,1);
    trajout(i).y = traj(i).y + dx((i-1)*3+2,1);
    trajout(i).theta = traj(i).theta + dx((i-1)*3+3,1);
end
ks = cell2mat(lmidx_map.keys);
for i = 1:size(lmidx_map.keys,2)
    idx = ks(i);
    lmout(idx).x = lm(idx).x + dx(numposeidx + lmidx_map(idx));
    lmout(idx).y = lm(idx).y + dx(numposeidx + lmidx_map(idx)+1);
end
    

function x = tri_solve(R, b)
%Assuming R is upper triangular
m = size(b,1);
n = size(R,2);
x = zeros(n,1);
for i = n:-1:1
    for j = i+1:n
       b(i) = b(i) - R(i,j)*x(j);
    end
    x(i) = b(i) / R(i,i);
end

function Xo = GetXo(pose, lm)
dx = lm.x - pose.x;
dy = lm.y - pose.y;

Xo(1) = dx * cos(pose.theta)+ dy*sin(pose.theta);
Xo(2) = -dx * sin(pose.theta) + dy * cos(pose.theta);

function Jeo = GetJeo(pose,lm)
Jeo = eye(2,2);
Xo = GetXo(pose,lm);
dx = Xo(1);
dy = Xo(2);
d2 = dx*dx + dy*dy;
d = sqrt(d2);
Jeo(1,1) =  dx / d;
Jeo(1,2) =  dy / d;
Jeo(2,1) = -dy / d2;
Jeo(2,2) =  dx / d2;
% 
function [h,dhdx, dhdlm] = gethpt(pose, lm, z)
%ran = z.range;
%bear = z.bearing;
%z.range = ran * cos(bear);
%z.bearing = ran* sin(bear);
pred = transform_to(pose, lm);
h = struct2array(z) - struct2array(pred);
[pose]
[lm]
[z]
h
%h(2) = wrapToPi(h(2));
Jor = eye(2,3);
dx = lm.x - pose.x;
dy = lm.y - pose.y;
Jor(1,1) = cos(pose.theta);
Jor(1,2) = sin(pose.theta);
Jor(1,3) = dx*sin(pose.theta)-dy*cos(pose.theta);
Jor(2,1) = -sin(pose.theta);
Jor(2,2) = cos(pose.theta);
Jor(2,3) = dx*cos(pose.theta)+dy*sin(pose.theta);
% Jor(1,1) = -cos(pose.theta);
% Jor(1,2) = -sin(pose.theta);
% Jor(1,3) = dy*cos(pose.theta) - dx * sin(pose.theta);
% Jor(2,1) = sin(pose.theta);
% Jor(2,2) = -cos(pose.theta);
% Jor(2,3) = -dx*cos(pose.theta) - dy * sin(pose.theta);
Jeo = eye(2,2);
dhdx = Jeo * Jor;
Jof = eye(2,2);
Jof(1,1) = -cos(pose.theta);
Jof(1,2) = -sin(pose.theta);
Jof(2,1) = sin(pose.theta);
Jof(2,2) = -cos(pose.theta);
dhdlm = Jeo * Jof;

function [h,dhdx, dhdlm] = gethbr(pose, lm, z)
pred = measure(transform_to(pose, lm));
h = struct2array(z) - struct2array(pred);
[pose] 
[lm]
[z]
h
h(2) = wrapToPi(h(2));
Jor = eye(2,3);
dx = lm.x - pose.x;
dy = lm.y - pose.y;
Jor(1,1) = -cos(pose.theta);
Jor(1,2) = -sin(pose.theta);
Jor(1,3) = dy*cos(pose.theta) - dx * sin(pose.theta);
Jor(2,1) = sin(pose.theta);
Jor(2,2) = -cos(pose.theta);
Jor(2,3) = -dx*cos(pose.theta) - dy * sin(pose.theta);
Jeo = GetJeo(pose,lm);
dhdx = Jeo * Jor;
Jof = eye(2,2);
Jof(1,1) = cos(pose.theta);
Jof(1,2) = sin(pose.theta);
Jof(2,1) = -sin(pose.theta);
Jof(2,2) = cos(pose.theta);
dhdlm = Jeo * Jof;

% function dhdx = getdhdx(pose, lm)
% Jor = eye(2,3);
% dx = lm.x - pose.x;
% dy = lm.y - pose.y;
% Jor(1,1) = -cos(pose.theta);
% Jor(1,2) = -sin(pose.theta);
% Jor(1,3) = dy*cos(pose.theta) - dx * sin(pose.theta);
% Jor(2,1) = sin(pose.theta);
% Jor(2,2) = -cos(pose.theta);
% Jor(2,3) = -dx*cos(pose.theta) - dx * sin(pose.theta);
% Jeo = GetJeo(pose,lm);
% dhdx = Jeo * Jor;
% 
% function dhdlm = getdhdlm(pose, lm)
% Jof = eye(2,2);
% Jeo = GetJeo(pose,lm);
% Jof(1,1) = cos(pose.theta);
% Jof(1,2) = sin(pose.theta);
% Jof(2,1) = -sin(pose.theta);
% Jof(2,2) = cos(pose.theta);
% dhdlm = Jeo * Jof;

function lm_out = transform_to(pose, lm)
dx = lm.x - pose.x;
dy = lm.y - pose.y;
lm_out.x = dx* cos(pose.theta) + dy * sin(pose.theta);
lm_out.y = -dx* sin(pose.theta) + dy * cos(pose.theta);

function m = measure(lm_local)
m.range = sqrt(lm_local.x^2 + lm_local.y^2);
m.bearing = atan2(lm_local.y, lm_local.x);

% function [odoerr, dododx1, dododx2] = getodoerr(x1, x2, z) 
% s1 = sin(x1.theta);
% c1 = cos(x1.theta);
% s2 = sin(x2.theta);
% c2 = cos(x2.theta);
% 
% c = c1*c2 + s1*s2;
% s = -s1*c2+ c1*s2;
% R = atan2(s,c);
% dx = x2.x - x1.x;
% dy = x2.y - x1.y;
% xrel = c1*dx + s1*dy;
% yrel = -s1*dx + c1*dy;
% [xrel yrel R]'
% odoerr = struct2array(z)'- [xrel yrel R]';
% odoerr(3) = wrapToPi(odoerr(3));
% odoerr
% dt1 = -s1 * dx + c1 * dy;
% dt2 = -c1 * dx - s1 * dy;
% dododx1 = [-c1, -s1, dt1;
%             s1, -c1, dt2;
%             0, 0, -1];
% dododx2 = eye(3,3);
% 

function [odoerr, dododx1, dododx2] = getodoerr(x1, x2, z) 
thetamid = x1.theta + z.theta/2.0;

f.x = x1.x + z.x * cos(thetamid);% - z.y * sin(thetamid);
f.y = x1.y + z.x * sin(thetamid);% + z.y * cos(thetamid);
f.theta = x1.theta + z.theta;

odoerr = struct2array(x2)'- [f.x f.y f.theta]';
odoerr(3) = wrapToPi(odoerr(3));
dododx1 = [1, 0, -z.x*sin(thetamid);% - z.y*cos(thetamid);
           0, 1, z.x*cos(thetamid) ;%- z.y*sin(thetamid);
            0, 0, 1];
dododx2 = -eye(3,3);

