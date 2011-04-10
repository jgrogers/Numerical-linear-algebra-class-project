function [spar, dens, density, probsz] = execute_slam_both(num_poses, num_landmarks, range, max_steps)
lim_abs_err = 1e-10;
lim_rel_err = 1e-10;
lm_true = gen_rand_landmarks(num_landmarks,10);
traj_true = GenerateTrajectory(num_poses);
odo = GenerateOdometry(traj_true);
[z,da,meascnt] = get_measurements(lm_true, traj_true, range);
probsz = meascnt + size(odo,2);
%figure
for i = 1:size(traj_true,2)
   [traj_true(i)]
   xini(i) = traj_true(i);
   xini(i).x = rand(1,1)*0.5-0.25;
   xini(i).y = rand(1,1)*0.5-0.25;
   xini(i).theta = rand(1,1)*0.1-0.05;
end
xini(1).x = 0;
xini(1).y = 0;
xini(1).theta = 0;
%xopt = traj_true;
lm_ini = lm_true; 
for (i = 1:size(lm_ini,2))
   lm_ini(i).x = rand(1,1)*15 -0.7; 
   lm_ini(i).y = rand(1,1)*15 -0.7; 
end

lm_opt = lm_ini;
xopt = xini;
dispstate(xopt, lm_opt, da,traj_true,lm_true);
prior_norm_dx = 1;
density = 0;
tic

for (i = 1:max_steps)
    [xopt, lm_opt,resid,dx,density] = OneStepGradientSparse(xopt, lm_opt, odo, z, da);
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
spar = toc
lm_opt = lm_ini;
xopt = xini;
close all
tic
d2 = 0;
for (i = 1:max_steps)
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
dens = toc
close all
sprintf('Sparse was %f, dense was %f, sparse was %f faster, problem density %f\n',spar, dens, (dens-spar) / dens, density)
        