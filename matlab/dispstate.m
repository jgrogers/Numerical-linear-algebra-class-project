function dispstate(traj, landmarks, da,truetraj,truelm)
hold on


for i = 1:size(da,2)
    for j = 1:size(da{i},2)
        ind = da{i}(j);
        lm = landmarks(ind);
        plot([traj(i).x, lm.x], [traj(i).y, lm.y])
        plot([lm.x], [lm.y],'bx');
        lmtrue = truelm(ind);
        plot([lmtrue.x], [lmtrue.y],'gx');

    end
end


plot([traj.x], [traj.y],'r');
plot([truetraj.x], [truetraj.y],'g');
