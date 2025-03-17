function [depthMap, colorImage] = render_scene(scene)
    % 示例：基于简单几何模型生成深度图和RGB图
    % 实际应用中需使用3D渲染引擎（如MATLAB的pcshow或第三方工具）
 % 输入验证
    if ~isfield(scene, 'camera') || ...
       ~isfield(scene.camera, 'intrinsics') || ...
       ~isfield(scene.camera.intrinsics, 'ImageSize')
        error('scene结构体缺少必要字段：camera.intrinsics.ImageSize');
    end

    % 提取相机参数
    intrinsics = scene.camera.intrinsics;
    extrinsics = scene.camera.extrinsics;
    [height, width] = intrinsics.ImageSize;  % 确保ImageSize为[height, width]

    %% 2. 初始化深度图和RGB图
    depthMap = 5.0 * ones(height, width);  % 初始化为最大探测距离（5米）
    colorImage = zeros(height, width, 3, 'single'); % RGB图像


    % 遍历每个像素
    for v = 1:height
        for u = 1:width
            % 生成射线（从相机原点出发）
            rayDirection = intrinsics.deproject([u, v, 1]);  % 射线方向（归一化）
             rayDirection = rayDirection / norm(rayDirection);
             rayDirectionWorld  = extrinsics.rotate(rayDirection);  % 转换到世界坐标系

            % 计算与障碍物的交点（简化版，仅检查球体）
            minDepth = Inf;
            hitColor = [0, 0, 0];  % 默认背景色（黑色）
             for k = 1:numel(scene.obstacles)
                obs = scene.obstacles(k);
                if strcmp(obs.type, 'sphere')
                    center = obs.position;
                    radius = obs.dimensions;
                    [hit, depth] = raySphereIntersection(...
                        extrinsics.Translation, rayDirectionWorld, center, radius);
                    if hit && depth < minDepth
                        minDepth = depth;
                        hitColor = obs.color;
                        end
                end
            end

            % 更新深度图和RGB图
            if minDepth < 5.0
                depthMap(v, u) = minDepth;
                colorImage(v, u, :) = hitColor;
            end
        end
    end
end

function [hit, depth] = raySphereIntersection(rayOrigin, rayDir, sphereCenter, sphereRadius)
    % 计算射线与球体的交点
    oc = rayOrigin - sphereCenter;
    a = dot(rayDir, rayDir);
    b = 2 * dot(oc, rayDir);
    c = dot(oc, oc) - sphereRadius^2;
    discriminant = b^2 - 4*a*c;
    if discriminant < 0
        hit = false;
        depth = Inf;
    else
        t = (-b - sqrt(discriminant)) / (2*a);
        if t > 0
            hit = true;
            depth = t;
        else
            hit = false;
            depth = Inf;
        end
    end
end

% 深度图生成（简化版）