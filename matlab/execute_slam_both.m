function result = execute_slam_both(num_poses, landmark_density, range, max_steps, do_nocolamd)
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
dispstate(xopt, lm_opt, da,traj_true,lm_true);
prior_norm_dx = 1;
density = 0;
stime = tic;
if (do_nocolamd)
    for (i = 1:max_steps)
        [xopt, lm_opt,resid,dx,density] = OneStepGradientSparse(xopt, lm_opt, odo, z, da,false);
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
end
spplain = toc(stime);
lm_opt = lm_ini;
xopt = xini;
close all

stime = tic;

for (i = 1:max_steps)
    [xopt, lm_opt,resid,dx,density] = OneStepGradientSparse(xopt, lm_opt, odo, z, da,true);
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
spar = toc(stime);
lm_opt = lm_ini;
xopt = xini;
close all
stime = tic;
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
dens = toc(stime);
close all
speedup = (dens-spar)/dens;
sprintf('Sparse was %f, dense was %f, sparse was %f faster, problem density %f\n',spar, dens, (dens-spar) / dens, density)
result.spplain = spplain;
result.spar=spar;
result.dens=dens;
result.speedup=speedup;
result.density=density;
result.probsz = probsz;