%% 主程序：机械臂动态避障仿真流程
clc; clear; close all;

% === 步骤1：生成场景 ===
scene = generate_static_scene();    % 生成静态场景
% scene = generate_dynamic_scene(); % 生成动态场景（取消注释以启用动态障碍物）
% 调试：检查scene.camera.intrinsics.ImageSize
disp('scene.camera.intrinsics.ImageSize:');
disp(scene.camera.intrinsics.ImageSize);  % 应输出 [480, 848]


% === 步骤2：模拟深度相机输出 ===
[depthImg, colorImg, ptCloud] = depth_camera_sim(scene);

% === 步骤3：处理点云 ===
[downsampledCloud, groundPlane, obstacleClusters] = pointcloud_processing(ptCloud);

% === 步骤4：可视化结果 ===
% 显示深度图和RGB图
figure;
subplot(1,2,1); imshow(depthImg, []); title('深度图');
subplot(1,2,2); imshow(colorImg); title('RGB图');

% 显示原始点云和地面分割结果
figure;
pcshow(ptCloud); title('原始点云');
hold on;
pcshow(groundPlane, 'r'); title('地面（红色）');

% 显示障碍物聚类
figure;
pcshow(downsampledCloud); 
hold on;
for i = 1:length(obstacleClusters)
    pcshow(obstacleClusters{i}, rand(1,3)); % 随机颜色显示不同聚类
end
title('障碍物聚类');