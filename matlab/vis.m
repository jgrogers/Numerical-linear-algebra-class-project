close all;
landmark_density = 0.4;
num_poses = 20;
range = 1.2;
max_steps = 20;
lim_abs_err = 1e-2;
lim_rel_err = 1e-2;
lm_true = gen_rand_landmarks(landmark_density,3);
traj_true = GenerateTrajectory(num_poses);
odo = GenerateOdometry(traj_true);
[z,da,meascnt] = get_measurements(lm_true, traj_true, range);
probsz = meascnt + size(odo,2);
%figure
for i = 1:size(traj_true,2)
   xini(i) = traj_true(i);
   xini(i).x = xini(i).x + rand(1,1)*0.5-0.25;
   xini(i).y = xini(i).y + rand(1,1)*0.5-0.25;
   xini(i).theta = xini(i).theta + rand(1,1)*0.1-0.05;
end
xini(1).x = 0;
xini(1).y = 0;
xini(1).theta = 0;
%xopt = traj_true;
lm_ini = lm_true; 
for i = 1:size(lm_ini,2)
   lm_ini(i).x = lm_ini(i).x + rand(1,1)*2 -0.7; 
   lm_ini(i).y = lm_ini(i).y + rand(1,1)*2 -0.7; 
end

lm_opt = lm_ini;
xopt = xini;

prior_norm_dx = 1;
dispstate(xopt, lm_opt, da,traj_true, lm_true);

d2 = 0;
A = OneStepGradientSparse(xopt, lm_opt, odo, z, da);
close all;
figure
spy(A);
[m,n] = size(A);
[q,r] = qr(A);
figure
spy(r(1:n,1:n));
p = colamd(A);
[q2,r2] = qr(A(:,p));
figure
spy(r2(1:n, 1:n));

for (i = 1:0)
    [xopt, lm_opt,resid,dx,d2] = OneStepGradient(xopt, lm_opt, odo, z, da);
    err(i) = resid;
    delta(:,i)= dx;
    figure
    dispstate(xopt, lm_opt, da,traj_true, lm_true);
    ndx = norm(dx);
    if ndx < lim_abs_err || ndx / prior_norm_dx < lim_rel_err
        break
    end
    prior_norm_dx = ndx;
end
dens = toc(stime);
