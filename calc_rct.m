q = [0 pi/2 0 0 0 -pi/2 0]; %由于toolbox抽羊癫疯认为当关节为旋转关节时，theta所见
% 即所得，因此初始偏移值只好这样指定。

scale = 1000;%为了画图的缩放

% [a      alpha      d       theta]
mdhparams = [
    0         0     120      0;
    0      pi/2    -100      0;
    0      -pi/2   -100      0;
    400     0      -100      0;
   -400     0      -100      0;
    0       pi/2    100      0;
    0       pi/2    120      0;
];

robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies', 8);

prevBody = robot.BaseName;

for i = 1:size(mdhparams, 1)
    body = robotics.RigidBody(['link' num2str(i)]);
    joint = robotics.Joint(['joint' num2str(i)], 'revolute');
    
    % 直接传入 [theta, alpha, a, d]，标准 D-H 顺序
    a = mdhparams(i, 1) / scale;
    alpha = mdhparams(i, 2);
    d     = mdhparams(i, 3) / scale;
    theta = mdhparams(i, 4);

    % setFixedTransform 使用改进 D-H 约定
    setFixedTransform(joint, [a, alpha, d, theta], 'mdh');
    
    body.Joint = joint;
    addBody(robot, body, prevBody);
    prevBody = body.Name;
end

q = q + [4.92659305, 0, 0.80382838, -0.69382108, -1.68080363, -1.35659225, 0];

% 显示机器人模型
showdetails(robot)

show(robot, q);