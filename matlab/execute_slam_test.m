function execute_slam_test(num_poses, num_landmarks, range, steps)

lm_true = gen_rand_landmarks(num_landmarks,10);
traj_true = GenerateTrajectory(num_poses);
odo = GenerateOdometry(traj_true);
[z,da] = get_measurements(lm_true, traj_true, range);
%figure
for i = 1:size(traj_true,2)
   [traj_true(i)]
   xopt(i) = traj_true(i);
   xopt(i).x = traj_true(i).x + rand(1,1)*0.5-0.25;
   xopt(i).y = traj_true(i).y + rand(1,1)*0.5-0.25;
   xopt(i).theta = traj_true(i).theta + rand(1,1)*0.1-0.05;
end
xopt(1).x = 0;
xopt(1).y = 0;
xopt(1).theta = 0;
%xopt = traj_true;
lm_opt = lm_true;    
dispstate(xopt, lm_opt, da,traj_true,lm_true);
for (i = 1:steps)
    [xopt, lm_opt] = OneStepGradient(xopt, lm_opt, odo, z, da);
    figure
    dispstate(xopt, lm_opt, da,traj_true, lm_true);

end
