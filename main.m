% clear; 
clc;

%%
scan_dir = '/media/user/888aec20-5827-4647-942b-6f0b7f024fc3/data/KITTI360/KITTI-360/data_3d_raw/2013_05_28_drive_0009_sync/velodyne_points/data/';
scan_names = listdir(scan_dir);
num_scans = length(scan_names);

%%
cam_poses = readmatrix('/media/user/888aec20-5827-4647-942b-6f0b7f024fc3/data/KITTI360/KITTI-360/data_poses/2013_05_28_drive_0009_sync/cam0_to_world.txt');
cam_poses_idxes = cam_poses(:, 1);
figure(1); clf;
pcshow(cam_poses(:, [5,9,13]));

%
cam2lidar = [0.04307104361 -0.08829286498 0.995162929 0.8043914418;
            -0.999004371 0.007784614041 0.04392796942 0.2993489574;
            -0.01162548558 -0.9960641394 -0.08786966659 -0.1770225824;
            0, 0, 0, 1];


%%
is_map_made = 0;
map_downsize = 0.5; % meter 

is_skip = 10;
for ii=1:num_scans
    
    [where, exist] = find(ii == cam_poses_idxes);
    if(isempty(exist))
        continue;
    end
    
    if(rem(ii, is_skip) ~= 0 )
%         disp(skip_counter)
%         disp("skip")
            continue;
    end
        
    scan_path = fullfile(scan_dir, scan_names{ii});
    scan = readBin(scan_path);
    
%     scan_pose = [reshape(lidar_poses(where, 2:end), 4, 3)'; 0,0,0,1];
    scan_pose = reshape(cam_poses(where, 2:end), 4, 4)';
    scan_global = local2global(scan, inv(cam2lidar), scan_pose);
    if(~is_map_made)
        global_map = scan_global;
        is_map_made = 1;
    else
        global_map = pcmerge(global_map, scan_global, map_downsize);
    end
    
    disp("scan " + num2str(ii) + " is stacked in the global frame");
    
%     if(ii > 2000)
%         break
%     end
end

figure(2); clf; 
pcshow(global_map);
caxis([0, 0.6])
