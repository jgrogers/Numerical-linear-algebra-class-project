function execute_slam_test(num_poses, num_landmarks, range, max_steps)
lim_abs_err = 1e-10;
lim_rel_err = 1e-10;
lm_true = gen_rand_landmarks(num_landmarks,10);
traj_true = GenerateTrajectory(num_poses);
odo = GenerateOdometry(traj_true);
[z,da] = get_measurements(lm_true, traj_true, range);
%figure
for i = 1:size(traj_true,2)
   [traj_true(i)]
   xopt(i) = traj_true(i);
   xopt(i).x = rand(1,1)*0.5-0.25;
   xopt(i).y = rand(1,1)*0.5-0.25;
   xopt(i).theta = rand(1,1)*0.1-0.05;
end
xopt(1).x = 0;
xopt(1).y = 0;
xopt(1).theta = 0;
%xopt = traj_true;
lm_opt = lm_true; 
for (i = 1:size(lm_opt,2))
   lm_opt(i).x = rand(1,1)*15 -0.7; 
   lm_opt(i).y = rand(1,1)*15 -0.7; 
end
dispstate(xopt, lm_opt, da,traj_true,lm_true);
prior_norm_dx = 1;
for (i = 1:max_steps)
    [xopt, lm_opt,resid,dx] = OneStepGradient(xopt, lm_opt, odo, z, da);
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
figure 
plot(err);
err
delta
