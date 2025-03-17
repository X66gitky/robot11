% 生成静态场景
function scene = generate_static_scene()
    % 加载环境参数
    env = env_params1();
    
    % 创建相机内参和外参
    intrinsics = env.d435i.intrinsics;  % 直接引用env中的参数
    extrinsics = rigid3d(...
        eul2rotm([0, -pi/6, 0], 'XYZ'), ... % 旋转矩阵（俯角30度）
        [0.5, 0, 0.3] ...                   % 平移向量 [x, y, z]
    );


    % 定义障碍物
    obstacles = struct(...
        'type', {'sphere', 'cube', 'cylinder'}, ...
        'position', {[1.0, 0.5, 0.2], [0.8, -0.3, 0.4], [0.2, 0.1, 0.3]}, ...
        'dimensions', {0.15, [0.2, 0.2, 0.2], [0.1, 0.3]}, ... % 球体半径、立方体尺寸、圆柱体[半径, 高度]
        'color', {[1,0,0], [0,1,0], [0,0,1]} ...              % 红、绿、蓝
    );

    % 构建scene结构体
    scene = struct(...
        'obstacles', obstacles, ...
        'robot_pose', [0, 0, 0, 0, 0, 0], ... % 机械臂初始位姿
        'camera', struct('intrinsics', intrinsics, 'extrinsics', extrinsics), ...
        'isDynamic', false ...
    );
end

% 生成静态场景
