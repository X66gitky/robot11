function [depthImg, colorImg, ptCloud] = depth_camera_sim(scene)
    % 输入：scene（包含障碍物和机械臂的3D场景）
    % 输出：depthImg（带噪声的深度图）、colorImg（RGB图）、ptCloud（有序点云）
 % 输入验证
    if ~isfield(scene, 'camera') || ~isfield(scene.camera, 'intrinsics') || ~isfield(scene, 'obstacles')
        error('scene结构体缺少必要字段：camera.intrinsics或obstacles');
    end
    %% 1. 从场景渲染深度图和RGB图
    [depthRaw, colorImg] = render_scene(scene);

    %% 2. 添加传感器噪声
    % 高斯噪声（σ=1.5cm）
    depthNoisy = depthRaw + 0.015 * randn(size(depthRaw));

     % 脉冲噪声（限制在有效测距范围内）
    impulseMask = rand(size(depthRaw)) < 0.003;
    depthNoisy(impulseMask) = max(scene.camera.intrinsics.depth_range(1), ...
                                  min(scene.camera.intrinsics.depth_range(2), ...
                                      depthNoisy(impulseMask) + 0.1 * randn(nnz(impulseMask), 1)));

    %% 3. 运动模糊（动态场景）
    if scene.isDynamic
        h = fspecial('motion', 15, 30);  % 模糊核（长度15像素，角度30度）
        depthNoisy = imfilter(depthNoisy, h, 'replicate');
    end

     %% 4. 生成点云（过滤无效深度）
    validDepthMask = depthNoisy >= scene.camera.intrinsics.depth_range(1) & ...
                     depthNoisy <= scene.camera.intrinsics.depth_range(2);
    [u, v] = meshgrid(1:scene.camera.intrinsics.ImageSize(2), 1:scene.camera.intrinsics.ImageSize(1));
    points = scene.camera.intrinsics.deproject([u(validDepthMask), v(validDepthMask), depthNoisy(validDepthMask)]);
    ptCloud = pointCloud(points, 'Color', reshape(colorImg(validDepthMask), [], 3));

    %% 5. 返回结果
    depthImg = depthNoisy;
end


%% 在仿真环境中生成与真实D435i相机一致的深度图像和RGB图像，模拟真实传感器的输出，为后续的避障算法提供输入数据。
% 模拟D435i的深度噪声（高斯噪声 + 脉冲噪声）
% 动态障碍物的运动模糊效果
% 生成有序点云（保留像素空间关系）