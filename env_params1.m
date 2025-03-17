function env = env_params1()
% 珞石ER3机械臂动态避障仿真环境参数配置
% 核心参数来源：Intel RealSense D435i技术手册、VINS-Mono标定方案[[1]][[3]]、OpenRooms光照模型
% 最后更新：2025-03-16

%% ========================= 基础场景参数 ========================
env.scene_size = [5.0, 5.0, 3.0];   % 场景尺寸[x,y,z] (单位：米)
env.obstacle_density = 0.3;          % 障碍物分布密度（0-1）


% env_params.m - 环境参数定义（障碍物、相机、仿真设置）
% env_params.m
obstacles = struct();
obstacles.obs1 = struct();
obstacles.obs1.type = 'sphere';
obstacles.obs1.radius = 0.1;
% 其余参数...


%% ====================== 深度相机参数(D435i) =====================

env.d435i = struct(...
    'resolution',    [848, 480], ...  % 深度分辨率（像素）
    'fps',          98, ...          % 最大帧率（Hz）
    'hfov',         87.3, ...        % 水平视场角（度）±0.5°误差
    'depth_range',  [0.2, 5.0], ...  % 有效测距范围（米）
    'baseline',     0.05, ...        % IR相机基线距离（米）
    'min_depth',    0.1, ...         % 物理最小探测距离（米）
    'projection_model', 'rectilinear', ... % 投影模型
    'noise_model', struct(...
        'sigma', 0.015, ...
        'motion_blur', struct('enable', true, 'prob', 0.003) ...
    ), ...  % 闭合 noise_model 结构体
    'intrinsics', struct(...         % 内参矩阵（左IR相机）
        'K', [617.37, 0, 421.55; 0, 617.37, 237.27; 0, 0, 1], ...
        'distortion', [0.12, -0.25, 0.001, 0.003, 0] ...
    ) ...  % 闭合 intrinsics 结构体
);  % 闭合整个 env.d435i 结构体




%% 
% 内参矩阵（左IR相机）
env.d435i.intrinsics = struct(...
    'K', [617.37, 0,       421.55;   % [fx, 0, cx; 0, fy, cy; 0, 0, 1]
          0,       617.37, 237.27;
          0,       0,       1],...
    'distortion', [0.12, -0.25, 0.001, 0.003, 0]... % [k1, k2, p1, p2, k3]
);

%% ====================== RGB相机参数 ========================
env.rgb_cam = struct(...
    'resolution',       [1280, 720],...   % RGB分辨率
    'fps',              30,...            % 帧率（Hz）
    'intrinsics',       struct(...        % 内参
        'K', [920.0, 0,   640.5;
              0,    920.0, 360.5;
              0,    0,     1],...
        'distortion', [0.09, -0.18, 0.001, 0.002, 0]...
    ),...
    'exposure_mode',    'auto',...        % 曝光模式(auto/manual)
    'white_balance',    4500....           % 色温(K)
);

%% ======================= IMU参数 ========================
% env.imu = struct(...
%     'accel_range',      4,        % 加速度计量程(±4 g)
%     'gyro_range';       2000;     % 陀螺仪量程(±2000 dps)
%     'noise_density';    struct(...% 噪声密度
%         'accel',        0.0003,  % m/s²/√Hz
%         'gyro';         0.008 ...% rad/s/√Hz
%     ),...
%     'bias_instability'; struct(...% 零偏不稳定性
%         'accel',        0.0012,  % m/s²
%         'gyro';         0.00015 ...% rad/s
%     ),...
%     'update_rate';      200 ...  % 采样率(Hz)
% );

%% ======================= 标定参数 ======================
env.calibration = struct(...
    'T_imu_cam',    [0.011, -0.002, 0.017],... % IMU到相机的平移(m)
    'R_imu_cam',    eul2rotm([0.012, -0.004, 0.009], 'XYZ'),... % 欧拉角转旋转矩阵
    'time_offset',  0.0,...       % 时间延迟(s)
    'estimate_mode',struct(...    % 在线标定配置
        'estimateExtrinsic', true,... % 是否优化外参
        'estimateTd',        true ... % 是否估计时间偏移
    )...
);

%% ====================== 动态场景配置 =====================
env.dynamic_scene = struct(...
    'moving_obstacles', struct(...% 动态障碍物参数
        'speed_range',   [0.1, 1.5],... % 运动速度范围(m/s)
        'trajectory_type','sinusoidal'...% 运动轨迹类型
    ),...
    'lighting', struct(...        % 环境光照
        'ambient',       0.6,...         % 环境光强度
        'directional',  [0.5, -0.3, 0.8],...% 定向光源方向（单位向量）
        'env_map',      struct(...       % 环境贴图参数
            'path',     'imenv_default.hdr',...% HDR贴图路径
            'resolution',[1920, 5120]...% 贴图分辨率
        )...
    ),...
    'motion_blur', struct(...     % 运动模糊参数
        'kernel_size',   5,...    % 模糊核尺寸
        'sigma',        1.5 ...  % 高斯核标准差
    )...
);

%% ====================== 高级优化参数 =====================
env.optimization = struct(...
    'parallel_workers',  4,...    % 并行计算线程数
    'cache_enabled',     true,...% 启用环境缓存
    'collision_check',   struct(...% 碰撞检测参数
        'safety_margin', 0.15,...% 安全距离(m)
        'voxel_size',    0.05 ...% 障碍物体素尺寸
    )...
);

%% ====================== 硬件接口配置 =====================
env.hardware_interface = struct(...
    'ros_master_uri',   'http://192.168.1.100:11311',...% ROS主机地址
    'depth_topic',      '/camera/depth/image_rect_raw',...% 深度话题
    'imu_topic',        '/camera/imu'...% IMU话题
);

%% ====================== 动态障碍物 ======================
env.obstacles.obs1 = struct(...
    'type', 'sphere',...
    'radius', 0.1,...
    'initialPosition', [-0.3, 0, 0.5],...
    'motion', 'sinusoidal',...
    'amplitude', 0.3,...
    'frequency', 0.5...
);

env.obstacles.obs2 = struct(...
    'type', 'cylinder',...
    'radius', 0.1,...
    'height', 0.3,...
    'center', [0, 0, 0.5],...
    'radiusTrajectory', 0.4,...
    'angularVelocity', 0.8...
);
%% 仿真环境参数
env.simTime = 10;                      % 仿真总时长（秒）
env.timeStep = 0.01;                   % 仿真步长（秒）
env.gravity = [0, 0, -9.81];           % 重力加速度（m/s²）
env.floorSize = [2, 2];                % 地面尺寸 [长, 宽]（米）



end











%% 使用示例：在仿真或控制脚本中调用参数：
% % 加载环境参数
% env_params;
% 
% % 示例：获取障碍物1的实时位置
% t = 2.5;  % 当前时间（秒）
% obs1_pos = obstacles.obs1.initialPosition + ...
%            [obstacles.obs1.amplitude * sin(2*pi*obstacles.obs1.frequency*t), 0, 0];
% 
% % 示例：生成D435i相机的虚拟深度图
% depthImg = simulateDepthCamera(camera, env);  % 自定义函数需实现
% 
% % 示例：设置仿真重力
% set_param('robot_simulation', 'gravity', env.gravity);

%% 扩展建议：
% 添加更多障碍物类型：
% 扩展 obstacles 结构体以支持更多形状（如圆锥、多边形）：
% obstacles.obs4.type = 'cone';
% obstacles.obs4.radius = 0.2;
% obstacles.obs4.height = 0.5;

% 动态参数更新：
% 在仿真循环中动态修改参数（如调整相机视角）：
% camera.extrinsics = trvec2tform([0.5 + 0.1*sin(t), 0, 0.3]);

% 环境约束：
% 添加工作空间限制或光照条件：
% env.workspaceLimits = [-1, 1, -1, 1, 0, 1];  % [xmin, xmax, ymin, ymax, zmin, zmax]
% env.lightIntensity = 0.8;                     % 光照强度（0~1）

%% 
% % 检查障碍物1的半径
% disp(obstacles.obs1.radius);  % 应输出 0.1
% 
% % 检查相机帧率
% disp(camera.frameRate);       % 应输出 30
