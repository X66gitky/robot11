% robot_params.m - 机械臂参数定义（从URDF提取）

%% 基座参数（base_link）
base_link.mass = 3.1488;  % 质量（kg）
base_link.inertia = [ ... % 惯性矩阵（kg·m²）
    0.0118, -0.0000, -0.0019; 
    -0.0000, 0.0165, 0.0000; 
    -0.0019, 0.0000, 0.0136 
];
base_link.com = [-0.0191, 0.00004, 0.0624];  % 质心位置（m）

%% 关节参数（每个关节的限位、effort、velocity）
% yao_joint（关节1）
joints.yao.lower = -2.97;     % 最小角度（rad）
joints.yao.upper = 2.97;      % 最大角度（rad）
joints.yao.effort = 3000;     % 最大力矩（N·m）
joints.yao.velocity = 3.14;   % 最大速度（rad/s）

% dabi_joint（关节2）
joints.dabi.lower = -2.09;
joints.dabi.upper = 2.09;
joints.dabi.effort = 3000;
joints.dabi.velocity = 2.62;

% xiaobiqian_joint（关节3）
joints.xiaobiqian.lower = -2.0944;
joints.xiaobiqian.upper = 2.0944;
joints.xiaobiqian.effort = 3000;
joints.xiaobiqian.velocity = 3.14;

% xiaobihou_joint（关节4）
joints.xiaobihou.lower = -2.97;
joints.xiaobihou.upper = 2.97;
joints.xiaobihou.effort = 3000;
joints.xiaobihou.velocity = 3.93;

% shouwan_joint（关节5）
joints.shouwan.lower = -2.09;
joints.shouwan.upper = 2.09;
joints.shouwan.effort = 3000;
joints.shouwan.velocity = 3.93;

% shuchu_joint（关节6）
joints.shuchu.lower = -6.28;
joints.shuchu.upper = 6.28;
joints.shuchu.effort = 3000;
joints.shuchu.velocity = 3.93;

%% 连杆几何参数（从URDF中的<origin>提取）
% yao_Link（连杆1）
links.yao.origin = [0, 0, 0.175];  % 相对于父连杆的位姿（xyz）

% dabi_Link（连杆2）
links.dabi.origin = [0, 0.0135, 0.1665];

% xiaobiqian_Link（连杆3）
links.xiaobiqian.origin = [0, -0.394, 0];

% xiaobihou_Link（连杆4）
links.xiaobihou.origin = [0, -0.21, -0.0135];

% shouwan_Link（连杆5）
links.shouwan.origin = [0, 0.004, 0.156];

% shuchu_Link（连杆6）
links.shuchu.origin = [0, -0.1855, -0.00412];

%% 末端执行器参数
ee.offset = [0, 0.1855, 0];  % ee_joint的origin（相对于shuchu_Link）










%% 使用示例：控制或仿真脚本中调用参数：

% % 加载参数
% robot_params;
% 
% % 示例：获取关节1的力矩限制
% max_torque = joints.yao.effort;  % 输出 3000
% 
% % 示例：计算机械臂总质量
% total_mass = base_link.mass + ...  % 基座质量
%             yao_Link.mass + ...   % 需要补充其他连杆参数
%             dabi_Link.mass + ...  % （根据URDF继续扩展）
%             xiaobiqian_Link.mass;
%% DH参数表
% dh_params = [
%     0,       0.175,  0,      0;       % 关节1
%     -pi/2,   0.0135, 0.1665, -pi/2;   % 关节2
%     0,       -0.394, 0,       0;       % 关节3
%     pi/2,    -0.21,  -0.0135, pi/2;    % 关节4
%     -pi/2,   0.004,  0.156,  -pi/2;    % 关节5
%     pi/2,    -0.1855, -0.00412, pi/2   % 关节6
% ];
%% 验证参数一致性，确保参数与URDF一致：
% % 检查基座质量是否匹配URDF中的<mass value="3.1488...">
% disp(base_link.mass);  % 应输出 3.1488
% 
% % 检查关节1的限位
% disp(joints.yao.lower);  % 应输出 -2.97