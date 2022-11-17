d= load('walking_motion.txt');

start_pose = [0.0, 0.0, -0.24, 0.6, -0.36, 0.0, ...
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0, ...
            0.0, 0.0, 0.0, ...
            0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0, ...
            0.0, 0.0, ...
            -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0];

cycle_start_idx = 31112;
cycle_end_idx = 34712;

data = zeros(cycle_end_idx-cycle_start_idx, 1 + size(start_pose,2));

data(1:cycle_end_idx-cycle_start_idx, 1) = (0:cycle_end_idx-cycle_start_idx-1)/2000.0;
data(1:cycle_end_idx-cycle_start_idx, 2:13) = d(cycle_start_idx:cycle_end_idx-1, 1:12);
data(1:cycle_end_idx-cycle_start_idx, 14:34) = repmat(start_pose(13:33), cycle_end_idx-cycle_start_idx , 1);

save('processed_data_tocabi_walk.txt', 'data', '-ascii', '-double', '-tabs')