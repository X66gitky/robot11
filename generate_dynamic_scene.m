% 生成动态场景
function scene = generate_dynamic_scene()
    % 基于静态场景添加动态障碍物
    scene = generate_static_scene();
    scene.isDynamic = true;

    % 添加动态障碍物（沿X轴移动的球体）
    dynamic_obstacle = struct(...
        'type', 'sphere', ...
        'position', @(t) [1.0 + 0.5*sin(2*pi*0.2*t), 0.5, 0.2], ... % 随时间t变化的位置
        'dimensions', 0.15, ...
        'color', [1,1,0] ...
    );
    scene.obstacles(end+1) = dynamic_obstacle;
end

% 生成动态场景