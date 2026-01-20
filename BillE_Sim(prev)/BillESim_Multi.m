% BillESim_Multi is the coolest thing since sliced bread
%
% TODO: - Planner returns configuration for grabbing and placing voxels
%       - Take voxels' starting and desired positions as input
%
% Javier Garcia, jgarciagonzalez@uh.edu 
% June 1, 2022

%% Initialization
clear; clc;

% Make a simple map to test
map = zeros(15,15,5);
map(1:6,1:6,1) = 1; % Initial voxel map
ploc = [6,1;1,6;5,1;1,5;4,1;1,4;3,1;1,3;1,1];
dloc = [7,5;5,7;7,4;4,7;7,3;3,7;7,6;6,7;7,7];
curr_v = [1,1;1,2;6,6;6,5;4,3;4,4];
% % Dogbone map to test waiting vs transfering
% map = zeros(5,15,5);
% map(1:5,1:5,1) = 1; % Initial voxel map
% map(3,6:10,1) = 1;
% map(1:5,11:15,1) = 1;
% ploc = [5,1;5,15;1,1;1,15];
% dloc = [2,8;4,8;5,6;1,10];
% map(3:5,8,1) = 1;
% dloc = [5,7;5,9;5,6;1,10];
% map(1:5,8,1) = 1;
% ploc = [5,1;5,15;1,1;1,15];
% dloc = [1,7;5,9;5,6;1,10];
% curr_v = [1,1;1,2;1,15;1,14;4,3;4,4];
% % ICRA map
% map = zeros(5,15,5);
% charmap = [ '111011001110111';
%             '010010101010100';
%             '010011101010111';
%             '010010101010001';
%             '111110111111111'];
% map(:,:,1) = charmap=='1'; % Initial voxel map
% curr_v = [1,5;1,6;5,11;5,10;1,15;1,14;1,2;2,2]; % Starting positions

P = 4; % Voxel pitch

% Graphics!
addpath(genpath("stlTools"));
figure(1); clf; hold on; axis equal;
xlim([-2*P,(size(map,1)+1)*P]); ylim([-2*P,(size(map,2)+1)*P]); zlim([-P,size(map,3)*P]);
voxel_p = drawVoxels(P,map,[0,0]);
view([45,45]);

bille = struct('a1',[],'a2',[],'state',[],'next_state',[],'curr_v',[],'next_v',[],...
    'path',[],'pth_ind',[],'new_pth',[],'change',[],'cost',[],'coord',[],'pose',[],...
    'pose_d',[],'th_h',[],'ths',[],'th_ind',[],'prev_ths',[],'stage',[],'anchor',[],...
    'payload',[],'flag',[],'r_p',[],'r_v',[],'l_p',[],'l_v',[],'p_p',[]);

num_robots = size(curr_v,1)/2;
a1 = [1.2*P;1.2*P;1.2*P;1.2*P]; a2 = [0.5*P;0.5*P;0.5*P;0.5*P];
robots = [];
for i = 1:num_robots
    robot = bille;
    robot.a1 = a1(i); robot.a2 = a2(i);
    robot.state = 0;
    robot.curr_v = [curr_v(2*i-1,:);curr_v(2*i,:)];
    robot.next_v = robot.curr_v;
    robot.pth_ind = 1;
    robot = getPose(P,robot,map);
    robot.pose_d = robot.pose;
    robot.th_h = 90; 
    robot.ths = zeros(7,1); robot.th_ind = 1;
    robot.prev_ths = zeros(7,1);
    robot.stage = 0; robot.anchor = 0;
    robot.payload = 0; robot.flag = 0;
    robot = drawRobot(P,robot);
    robot.l_p = []; robot.l_v = [];
    if(mod(i,2)) robot = drawLoad(P,robot);
    end
    robot.cost = 0;
    robots = [robots;robot];
end

% Record video!
record = 0;
if(record)
    vid = VideoWriter('meh.avi');
    vid.FrameRate = 30;
    open(vid);
end

%% Main Loop
t = 0;
pickups = 0;
deliveries = 0;
while(deliveries<size(dloc,1))
    for i = 1:num_robots
        if(robots(i).state==0)
            robots = multiPathPlanner(map,robots,ploc,dloc,deliveries,pickups);
            break;
        end
    end
    
    for i = 1:num_robots
        switch robots(i).state
            case 0
                robots(i) = idle(robots(i));
            case 1 
                [robots(i),map,deliveries,pickups,voxel_p] = travel(P,robots,i,map,ploc,dloc,deliveries,pickups,voxel_p);
            case 2 
                robots(i) = fixFrontFoot(P,robots(i),map);
            case 3
                [robots(i),map,deliveries,pickups,voxel_p] = handDown(P,robots(i),map,ploc,dloc,deliveries,pickups,voxel_p);
            case 4 
                robots(i) = handUp(P,robots(i));
            case 5
                robots(i) = updatePose(P,robots(i));
            otherwise
                robots(i) = idle(robots(i));
        end
    end
    
    if(record) writeVideo(vid,getframe(gcf)); end
    pause(0.01);
    t = t+1;
end

done = zeros(num_robots,1);
while(sum(done)<num_robots)
    for i = 1:num_robots
        switch robots(i).state
            case 2 
                robots(i) = fixFrontFoot(P,robots(i),map);
            case 4 
                robots(i) = handUp(P,robots(i));
            case 5
                robots(i) = updatePose(P,robots(i));
            otherwise
                robots(i) = idle(robots(i));
                done(i) = 1;
        end
    end
    
    if(record) writeVideo(vid,getframe(gcf)); end
    pause(0.01);
    t = t+1;
end
disp('Done!!');

if(record)
    for i = 1:50
        view([45+.9*i 45+.9*i]);
        writeVideo(vid,getframe(gcf));
        pause(.05);
    end
    close(vid);
end

%% State Machine
% Chill
function robot = idle(robot)
    robot.state = robot.next_state;
end

% Follow path to pick/drop voxel
function [robot,map,deliveries,pickups,voxel_p] = travel(P,robots,i,map,ploc,dloc,deliveries,pickups,voxel_p)  
    if(robots(i).stage==0)
        % Check if the planner changed the paths
        if(robots(i).change==1)
            pth = robots(i).path; ind = robots(i).pth_ind-1;
            n_pth = robots(i).new_pth;
            if(~isempty(pth) && ind>0)
                if(pth(ind,1)==n_pth(1,1) && pth(ind,2)==n_pth(1,2)) 
                    pth = n_pth(2:end,:);
                else
                    pth = n_pth;
                end
            else
                    pth = n_pth;
            end
            robots(i).path = pth;
            robots(i).pth_ind = 1;
            robots(i).change = 0;
        end
        
        % Check if robot will run into other robots
        if(pathCollision(robots,i)==1)
            robot = robots(i);
            robot.state = 0;
            return;
        end
    end
    
    robot = robots(i);
    pth = robot.path; ind = robot.pth_ind;
    stage = robot.stage; anchor = robot.anchor;
    flag = robot.flag; cost = robot.cost;
    next_v = robot.next_v; curr_v = robot.curr_v;
    
    % Move the robot along path
    n = size(pth,1);
    flat = sum(map,3);
    draw = 0;
    if(n>1)
        % Destination reached! Lower hand
        if((ind+1)>n)
            robot.pth_ind = 1;
            [robot,map,deliveries,pickups,voxel_p] = handDown(P,robot,map,ploc,dloc,deliveries,pickups,voxel_p);
            return;
        end
        if(stage)
            % If the robot rotated +-90 or +-180, back foot can't travel
            if(flag>1) robot.next_v(2-anchor,:) = curr_v(2-anchor,:);
            end
            [ths,flag] = motionRule(P,robot,flat); % Get angles
            robot.pth_ind = ind+1;
        else                        
            robot.cost = cost+1;
%             delete(robot.p_p); robot.p_p = drawPath(P,flat,pth(ind:end,:)); % Update path

            next_v(anchor+1,:) = curr_v(2-anchor,:);
            next_v(2-anchor,:) = pth(ind,:);
            robot.next_v = next_v;
            [ths,flag] = motionRule(P,robot,flat); % Get angles
            
            % Check if pick-up/drop-off will be at an angle. If the
            % robot rotated +-90 or +-180, special rules apply
            if(ind==(n-1))
                v1 = atan2d(pth(end,2)-next_v(2,2),pth(end,1)-next_v(2,1));
                if(flag>1) v2 = atan2d(next_v(2,2)-curr_v(1,2),next_v(2,1)-curr_v(1,1));
                else v2 = atan2d(next_v(2,2)-next_v(1,2),next_v(2,1)-next_v(1,1));
                end
                th_v = boundedAngle(v1-v2);
                if(th_v~=0)
                    steps = size(ths,2)-2;
                    turn_v = [0:th_v/steps:th_v,th_v];
                    if(flag<2) ths(5-4*anchor,:) = ths(5-4*anchor,:)-turn_v;
                    else ths(5-4*anchor,:) = -turn_v;
                    end
                end
            end
        end

        draw = 1;
        robot.next_state = 1;
    else
        % Check if pick-up/drop-off will be at an angle. Raise front foot
        % in place if that is the case
        v1 = atan2d(pth(end,2)-curr_v(2,2),pth(end,1)-curr_v(2,1));
        v2 = robot.pose(7-6*anchor)+180;
        th_v = boundedAngle(v1-v2);
        if(th_v~=0)
            [ths,flag] = motionRule(P,robot,flat); % Get angles
            steps = size(ths,2)-2;
            turn_v = [0:th_v/steps:th_v,th_v];
            ths(5-4*anchor,:) = -turn_v;

            draw = 1;
            robot.next_state = 3;
        else
            [robot,map,deliveries,pickups,voxel_p] = handDown(P,robot,map,ploc,dloc,deliveries,pickups,voxel_p);
        end
    end
    
    % Update drawing
    if(draw==1)
        robot.ths = ths;
        robot.flag = flag;
        robot.th_ind = 1;
        robot.state = 5;
        robot = updatePose(P,robot);
    end
end

% Fix feet orientation after pick-ups/drop-offs at an angle
function robot = fixFrontFoot(P,robot,map)
    pose = robot.pose; anchor = robot.anchor;
    flat = sum(map,3);

    [ths,~] = motionRule(P,robot,flat);	% Get angles
    steps = size(ths,2)-2;
    th_c = boundedAngle(pose(6-4*anchor)-pose(7-6*anchor));
    turn_c = [0:th_c/steps:th_c,th_c];
    ths(5-4*anchor,:) = -turn_c;

    robot.ths = ths;
    robot.th_ind = 1;
    robot.state = 5;
    robot.next_state = 0;
    robot = updatePose(P,robot);
end

% Grab/Place voxel
function [robot,map,deliveries,pickups,voxel_p] = handDown(P,robot,map,ploc,dloc,deliveries,pickups,voxel_p)
    i = robot.th_ind;   
    
    robot.th_h = 90-i*15;
    robot = drawRobot(P,robot);
    if(robot.payload) robot = drawLoad(P,robot);
    end

    d = deliveries+1;
    if((i+1)>6)
        tx = robot.path(end,1); ty = robot.path(end,2);
        if(robot.payload)
            deliveries = deliveries+1;
            map(tx,ty,1) = 1; 
            delete(voxel_p); 
            if(d>size(ploc,1)) voxel_p = drawVoxels(P,map,[0,0]);
            else voxel_p = drawVoxels(P,map,[0,0]);
            end
            robot.payload = 0;
        else
            pickups = pickups+1;
            map(tx,ty,1) = 0; 
            delete(voxel_p); voxel_p = drawVoxels(P,map,[0,0]);
            robot = drawLoad(P,robot);
            robot.payload = 1;
        end
        robot.th_ind = 1;
        robot.state = 4;
    else
        robot.th_ind = i+1;
        robot.state = 3;
    end
end

% Raise hand after grabbing/placing voxel
function robot = handUp(P,robot)
    pose = robot.pose; anchor = robot.anchor;
    i = robot.th_ind;
    
    robot.th_h = i*15;
    robot = drawRobot(P,robot); 
    if(robot.payload) robot = drawLoad(P,robot);
    end

    if((i+1)>6)
        robot.th_ind = 1;
        if(pose(6-4*anchor) ~= pose(7-6*anchor)) robot.state = 2;
        else robot.state = 0;
        end
    else
        robot.th_ind = i+1;
    end
end

% Apply motion(ths) to robot
function robot = updatePose(P,robot)
    ths = robot.ths;
    i = robot.th_ind;
    n = size(ths,2);
    
    robot = updatePoseD(robot);
    robot = moveRobot(robot);
    robot = drawRobot(P,robot);
    if(robot.payload) 
        robot = drawLoad(P,robot);
    end
    
    if((i+1)>n)
        robot.pose = robot.pose_d;
        robot.ths(1,:) = robot.ths(1,:)+robot.prev_ths(1,:);
        robot.ths(5,:) = robot.ths(5,:)+robot.prev_ths(5,:);
        robot.prev_ths = ths(:,end);
        robot.th_ind = 1;
        robot.state = robot.next_state;
        if(robot.next_state==1)
            anchor = robot.anchor; stage = robot.stage;
            robot.curr_v(2-anchor,:) = robot.next_v(2-anchor,:);
            robot.anchor = ~anchor;   robot.stage = ~stage;
        end
    else
        robot.th_ind = i+1;
    end
end

%% Supporting functions
% Return the coords and pose(global) of the robot at starting position
function robot = getPose(P,robot,map)
    flat = sum(map,3);
    a1 = robot.a1; a2 = robot.a2;
    start = robot.curr_v;
    
    robot_s = (cat(2,start,[flat(start(1,1),start(1,2));flat(start(2,1),start(2,2))])-1)*P;
    robot_a = atan2d(robot_s(2,2)-robot_s(1,2),robot_s(2,1)-robot_s(1,1));
    robot_b = acosd((2*a1^2-norm(robot_s(2,1:2)-robot_s(1,1:2))^2)/(2*a1^2));
    robot_g = atand((robot_s(2,3)-robot_s(1,3))/norm(robot_s(2,1:2)-robot_s(1,1:2)));
    
    pose = [robot_a;(180-robot_b)/2+robot_g;robot_b;(180-robot_b)/2-robot_g;robot_a+180];
    robot.coord = [robot_s(1,:);robot_s(1,1:2),robot_s(1,3)+a2;
        a1*cosd(pose(2))*cosd(pose(1))+robot_s(1,1),a1*cosd(pose(2))*sind(pose(1))+robot_s(1,2),a1*sind(pose(2))+a2+robot_s(1,3);...
        robot_s(2,1:2),robot_s(2,3)+a2;robot_s(2,:)]';
    robot.pose = [pose(1);pose;pose(end)];
end

% Apply set of angles to drawing pose
function robot = updatePoseD(robot)
    ths = robot.ths(:,robot.th_ind);
    pose = robot.pose;
    anchor = robot.anchor;
    
    pose_d = pose;
    pose_d(3:5) = ths(2:4);
    pose_d(2+4*anchor) = boundedAngle(pose(2+4*anchor)+ths(1+4*anchor));
    pose_d(6-4*anchor) = boundedAngle(pose(6-4*anchor)+ths(1+4*anchor));
    pose_d(7-6*anchor) = boundedAngle(pose(7-6*anchor)+ths(1+4*anchor)-ths(5-4*anchor));
    robot.pose_d = pose_d;
end

% Keep an angle between -179 and 180 deg
function deg = boundedAngle(deg)
    if(deg>180) deg = deg-360;
    elseif(deg<-179) deg = deg+360;
    end
end

% This function applies rotations to the joints based on the motion rule
% and the state of the robot. The joint order is with respect to the
% anchor, not the global joint number.
function robot = moveRobot(robot)
    a1 = robot.a1; a2 = robot.a2;
    r = robot.coord; p = robot.pose_d;
    anchor = robot.anchor;
        
    coord = r;
    % Second joint
    T1 = makehgtform('zrotate',deg2rad(p(2+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-p(3+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T = T1*T2*T3;
    coord(:,3) = T(1:3,4)+r(:,2+2*anchor);
    % Third joint
    T1 = makehgtform('zrotate',deg2rad(p(2+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-p(3+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T5 = makehgtform('yrotate',deg2rad(180-p(4)));
    T6 = makehgtform('translate',[0 0 a1]);
    T = T1*T2*T3*T5*T6;
    coord(:,4-2*anchor) = T(1:3,4)+r(:,2+2*anchor);
    % Fourth joint
    coord(:,5-4*anchor) = coord(:,4-2*anchor);
    coord(3,5-4*anchor) = coord(3,5-4*anchor)-a2;
    robot.coord = coord;
end

% Generate required motion based on the state of the robot and the voxels.
% dir indicates the direction the robot is moving in (0 for right and 1 for
% left). stage is a bit more difficult to explain, but it basically
% indicates if the specific foot is moving first or second.
function [ths,flag] = motionRule(P,robot,map)
    a1 = robot.a1;
    coord = robot.coord; pose = robot.pose;
    curr_v = robot.curr_v; next_v = robot.next_v;
    stage = robot.stage; anchor = robot.anchor;
    
    steps = 5;  % Intermediate points in motion parabola (evenly spaced in x)
    ths = zeros(7,steps+2);
    x = xor(anchor,stage);
    tau = 1;
    flag = 0;
    
    % Check if the robot is just fixing the orientation of the feet
    if(norm(curr_v(2-anchor,:)-next_v(2-anchor,:))==0)
        flag = 3;
        par_x = zeros(1,6);
        dth = boundedAngle(pose(7-6*anchor)-pose(6-4*anchor));
        if(dth ~= 0) 
            ths(5-4*anchor,1:steps+1) = 0:dth/steps:dth;
            par_y = [0,1.28,1.92,1.92,1.28,0]+round(coord(3,5-4*anchor));
        else
            par_y = zeros(1,6);
        end
    else
        % If the robot is turning, compute required rotation angles for the first and fifth 
        % joint. The x's of the parabola must be "folded" for certain turn values
        th1 = atan2d(curr_v(2-anchor,2)-curr_v(1+anchor,2),curr_v(2-anchor,1)-curr_v(1+anchor,1));
        th2 = atan2d(next_v(2-anchor,2)-curr_v(1+anchor,2),next_v(2-anchor,1)-curr_v(1+anchor,1));
        dth = boundedAngle(th2-th1);

        if(abs(dth)==45 || abs(dth)==135)
            flag = 1;
            ths(1+4*anchor,1:steps+1) = 0:dth/steps:dth;
            ths(5-4*anchor,1:steps+1) = -ths(1+4*anchor,1:steps+1);
            tau = sqrt(2)-1;
            par_x = tau*((stage*P):(1-2*stage)*P/steps:(P-stage*P));
        elseif(abs(dth)==90 || abs(dth)==180)
            flag = 2;
            ths(1+4*anchor,1:steps+1) = 0:dth/steps:dth;
            par_x = (stage*P):(1-2*stage)*P/steps:(P-stage*P);
            par_x = [par_x(1:floor(length(par_x)/2)) P-par_x(floor(length(par_x)/2+1):end)];
        else
            par_x = (stage*P):(1-2*stage)*P/steps:(P-stage*P);
        end
        
        % Different y's are associated with different actions
        action = nextAction(map,stage,anchor,curr_v,next_v);
        switch action
            case "stepup_two"
                beta = tau+1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x)+P;
            case "stepup_one"
                beta = tau+1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x);
            case "stepup_over"
                beta = tau+1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x)-P;
            case "stepup_flat"
                beta = tau;
                par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2)+P;
            case "flat"
                beta = tau;
                par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
%                 par_y = (-4*par_x.^2/P+4*beta*par_x)/(4/3);
            case "stepdown_flat"
                beta = tau;
                par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2)-P;
            case "stepdown_over"
                beta = tau-1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x)+P;
            case "stepdown_one"
                beta = tau-1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x);
            case "stepdown_two"
                beta = tau-1/(4*tau);
                par_y = (-4*par_x.^2/P+4*beta*par_x)-P;
            otherwise 
                error('Robot cannot get there!');
        end  
    end
    % Slight mod to x values for helping the foot not get stuck
    par_x(2) = (par_x(1)+par_x(2))/2; par_x(end-1) = par_x(end);
    
    % Use x and y vectors to generate parabolic motions. The effect of dir
    % and stage are included in the angle computations so the same actions
    % can result in different motions
    b = ((par_x+P).^2+par_y.^2).^0.5;
    ths(3,1:steps+1) = acosd((2*a1^2-b.^2)./(2*a1^2));
    dir = (1-2*stage)*(1-2*x);
    ths(2,1:steps+1) = ((180-ths(3,1:steps+1))/2)+dir*atand(par_y./(par_x+P));
    ths(4,1:steps+1) = ((180-ths(3,1:steps+1))/2)-dir*atand(par_y./(par_x+P));
    
    % Help foot not get stuck, unlock foot that is moving
    ths(4-2*anchor,2:end-2) = ths(4-2*anchor,2:end-2)+5;
    ths(:,steps+2) = ths(:,steps+1);
    ths(7-anchor,1:steps+1) = 45; ths(7-anchor,steps+2) = 0;
end

% Big brain logic that would take too long to explain allows for mapping
% multiple states to the same action.
function action = nextAction(map,stage,anchor,curr_v,next_v)
    dz1 = map(next_v(2-anchor,1),next_v(2-anchor,2))-map(curr_v(2-anchor,1),curr_v(2-anchor,2));
    dz2 = map(next_v(1+anchor,1),next_v(1+anchor,2))-map(curr_v(1+anchor,1),curr_v(1+anchor,2));
    
    % Use the state of the robot and the next voxel location to choose the
    % required action
    if (abs(dz1)>2)
        action = "reverse";
        return;
    else
        dz1 = (1-2*stage)*dz1;
        dz2 = (1-2*stage)*dz2;
        action_table = actionTable();
        action = action_table(dz1+3,dz2+3);
    end
end

% This table contains all the possible actions for the robot. The mapping
% from nextAction corresponds to the table indeces. The "reverse" action
% takes the place of any mapping for which there currently are no motions.
function action_table = actionTable()
    action_table = ["reverse","reverse","reverse","reverse","reverse";
             "reverse","stepdown_two","stepdown_one","stepdown_over","reverse";
             "reverse","stepdown_flat","flat","stepup_flat","reverse";
             "reverse","stepup_over","stepup_one","stepup_two","reverse";
             "reverse","reverse","reverse","reverse","reverse"];
end

% Path planning for multiple robots
function robots = multiPathPlanner(map,robots,ploc,dloc,deliveries,pickups)
    flat = sum(map,3);
    n = size(robots,1); m = size(ploc,1);
    pset = zeros(n,1); dset = pset;
    
    for i = 1:n
        if(robots(i).payload) dset(i) = 1;
        elseif(robots(i).state<9) pset(i) = 1;
        end
    end
    
    p = pickups+1;
    while(sum(pset)>0)
        if(p>m)
            for i = 1:n
                if(pset(i)==1)
                    robots(i).state = 9;
                    robots(i).next_state = 9;
                end
            end
            break;
        end
        dists = zeros(n,1);
        for i = 1:n
            if(pset(i)==1)
                newmap = potFields1(flat,robots,i);
                start = robots(i).curr_v;
                goal = ploc(p,:);
                [pth,dists(i)] = pathPlanner(newmap,start,goal);
                robots(i).new_pth = pth;
            else
                dists(i) = 1e9;
            end
        end
        [~,ind] = min(dists);
        pset(ind) = 0;
        robots(ind).change = 1;
        robots(ind).next_state = 1;
        p = p+1;
    end
    
    d = deliveries+1; 
    while(sum(dset)>0)
        dists = zeros(n,1);
        for i = 1:n
            if(dset(i)==1)
                newmap = potFields1(flat,robots,i);
                start = robots(i).curr_v;
                goal = dloc(d,:);
                [pth,dists(i)] = pathPlanner(newmap,start,goal);
                robots(i).new_pth = pth;
            else
                dists(i) = 1e9;
            end
        end
        [~,ind] = min(dists);
        dset(ind) = 0;
        robots(ind).change = 1;
        robots(ind).next_state = 1;
        d = d+1;
    end
    
    for r = 1:n robots = potFields2(map,robots,i); end
end

% Implementation of Dijkstra's algorithm. The voxels are the vertices, and
% the costs of traveling from one voxel to another are the edges. The path
% produced by this function ignores collision and other related issues
function [pth,dist,no_pth] = pathPlanner(map,start,goal)
    map(goal(1),goal(2)) = 1;
    [neighbors, costs] = getGraph(map);
    
    source = (start(2,1)-1)*size(map,2)+start(2,2);
    target = (goal(1)-1)*size(map,2)+goal(2);
    unvisited = 1:1:numel(map);        
    dist = 1000*ones(1,numel(map)); dist(source) = 0;
    prev = 0*dist;
    while(length(unvisited)>1)
        [~,ind] = min(dist(unvisited));
        u = unvisited(ind);
        if (ind==1)
            unvisited = unvisited(2:end);
        elseif (ind==length(unvisited))
            unvisited = unvisited(1:end-1);
        else
            unvisited = unvisited([1:ind-1,ind+1:end]);
        end
        
        for a = 1:length(unvisited)
            v = unvisited(a);
            for b = 1:length(costs)
                if ((neighbors(1,b)==u && neighbors(2,b)==v) || (neighbors(1,b)==v && neighbors(2,b)==u))
                    temp = dist(u) + costs(b);
                    if (temp<dist(v))
                        dist(v) = temp;
                        prev(v) = u;
                    end
                end
            end
        end
    end
    
    pth = target; u = target; 
    while(prev(u)~=source)
        if(dist(u)==1000)
            no_pth = 1;
            return;
            error('No path available!');
        end
        pth = [prev(u),pth];
        u = prev(u);
    end
    pth = [ceil(pth/size(map,2));pth-(ceil(pth/size(map,2))-1)*size(map,2)]';
    dist = dist(target);
    
    if(size(pth,1)>1)
        if(norm(pth(end-1,:)-start(1,:))==0)
            if((start(2,1)-1)>0 && map(start(2,1)-1,start(2,2))>0 && (start(2,1)-1)~=start(1,1))
                temp_goal = [start(2,1)-1,start(2,2)];
            elseif(start(2,2)<size(map,2) && map(start(2,1),start(2,2)+1)>0 && (start(2,2)+1)~=start(1,2))
                temp_goal = [start(2,1),start(2,2)+1];         
            elseif(start(2,1)<size(map,1) && map(start(2,1)+1,start(2,2))>0 && (start(2,1)+1)~=start(1,1))
                temp_goal = [start(2,1)+1,start(2,2)];          
            elseif((start(2,2)-1)>0 && map(start(2,1),start(2,2)+1)>0 && (start(2,2)-1)~=start(1,2))
                temp_goal = [start(2,1),start(2,2)+1];          
            else
                no_pth = 1;
                return;
                error('No path available!');
            end
            [pth1,dist1] = pathPlanner(map,start,temp_goal);
            [pth2,dist2] = pathPlanner(map,[start(2,:);temp_goal],goal);
            pth = [pth1;pth2];
            dist = dist1+dist2;
        end
        if(pth(1,1)==start(1,1) && pth(1,2)==start(1,2))
            pth = pth(2:end,:);
        end
    end
    
    no_pth = 0;
end

% Transform voxel map to a graph Dijkstra's algorithm can use
function [verts,edges] = getGraph(n)
    verts = []; edges = [];
    k = 1;
    
    for i = 1:size(n,1)
        for j = 1:size(n,2)
            if(j>1)
                verts(1,k) = (i-1)*size(n,2)+j;
                verts(2,k) = (i-1)*size(n,2)+(j-1);
                if(n(i,j)>0) edges(k) = factorial(abs(n(i,j)-n(i,j-1)));
                else edges(k) = 1000;
                end
                k = k+1;
            end
            if(i<size(n,1))
                verts(1,k) = (i-1)*size(n,2)+j;
                verts(2,k) = i*size(n,2)+j;
                if(n(i,j)>0) edges(k) = factorial(abs(n(i,j)-n(i+1,j)));
                else edges(k) = 1000;
                end
                k = k+1;
            end
        end
    end  
end

% Potential fields around other robots paths (the whole path as of now)
function newmap = potFields1(map,robots,i)
    n = size(robots,1);
    l = size(map,1)+1; w = size(map,2)+1;
    newmap = map;
    
    for j = 1:n
        pth = robots(j).path;
        if(i~=j && ~isempty(pth))
            for k = 1:size(pth,1)
                r = pth(k,1); c = pth(k,2);
                newmap(r,c) = 5;
                if((r-1)>0 && map(r-1,c)==1) newmap(r-1,c) = 4; end
                if((c-1)>0 && map(r,c-1)==1) newmap(r,c-1) = 4; end
                if((r+1)<l && map(r+1,c)==1) newmap(r+1,c) = 4; end
                if((c+1)<w && map(r,c+1)==1) newmap(r,c+1) = 4; end
            end
        end
    end
    x1 = robots(i).curr_v(1,1); y1 = robots(i).curr_v(1,2); 
    x2 = robots(i).curr_v(2,1); y2 = robots(i).curr_v(2,2);
    newmap(x1,y1) = map(x1,y1);
    newmap(x2,y2) = map(x2,y2);
end

% Potential fields around other robots paths (the whole path as of now)
function robots = potFields2(map,robots,i)
    path1 = robots(i).new_pth;
    ind1 = robots(i).pth_ind;
    newmap = map;

    for r = 1:i-1
        path2 = robots(r).new_pth;
        for p1 = ind1:size(path1,1)
            if(p1>size(path2,1)) break; end
            if(path1(p1,1)==path2(p1,1) && path1(p1,2)==path2(p1,2))
                newmap(path1(p1,1),path1(p1,2)) = 0;
                start = robots(i).curr_v;
                goal = path1(end,:);
                [newpth,~,no_alt] = pathPlanner(newmap,start,goal);
                if(no_alt)
                    robots(i).new_pth = [path1(1:p1-2,:);path1(p1-2,:);path1(p1-2,:);path1(p1-2:end,:)];
                    robots = potFields2(newmap,robots,i);
                else
                    robots(i).new_pth = newpth;
                end
            end
        end
    end
end

% Check if the next two steps of robot i collide with other robots
function bang = pathCollision(robots,i)
    robot = robots(i);
    pth = robot.path; ind = robot.pth_ind;
    n = size(pth,1);
    
    bang = 0;
    r1 = pth(ind,1); c1 = pth(ind,2);
    for j = 1:length(robots)
        if(i~=j)
            x1 = robots(j).curr_v(1,1); y1 = robots(j).curr_v(1,2);
            x2 = robots(j).curr_v(2,1); y2 = robots(j).curr_v(2,2);
            if((abs(r1-x1)<2 && abs(c1-y1)<2) || (abs(r1-x2)<2 && abs(c1-y2)<2))
                bang = 1;
                return;
            end
            if(ind<n)
                r2 = pth(ind+1,1); c2 = pth(ind+1,2);
                if((abs(r2-x1)<1 && abs(c2-y1)<1) || (abs(r2-x2)<1 && abs(c2-y2)<1))
                    bang = 1;
                    return;
                end
            end
        end
    end
end

% Draw voxels
function voxel_p = drawVoxels(P,n,trgt)
    voxel_p = [];
    [v_vox,f_vox,~,~] = stlRead('Sim_Voxel.stl'); v_vox = v_vox*P;  % Import voxel model
    for i = 1:size(n,1)
        for j = 1:size(n,2)
            for k = 1:size(n,3)
                if(n(i,j,k))
                    vtemp = [v_vox(:,1)+P*(i-1),v_vox(:,2)+P*(j-1),v_vox(:,3)+P*(k-1)];
                    if(i==trgt(1) && j==trgt(2))
                        ptemp = patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','b');
                    else
                        ptemp = patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','r');
                    end
                    voxel_p = [voxel_p;ptemp];
                end
            end
        end
    end
end

% Draw robot (don't expand this..)
function robot = drawRobot(P,robot)
    % TOLD YOU NOT TO SEE IT   
    c = robot.coord;
    p = robot.pose_d;
    th_h = robot.th_h;
    
    if(isempty(robot.r_p))
        v_foot1 = [-1/2,0,.1;0,1/2,.1;1/2,0,.1;0,-1/2,.1;-1/2,0,0;0,1/2,0;1/2,0,0;0,-1/2,0;...
            1/6,0,.1;1/3,1/6,.1;1/3,-1/6,.1;1/6,0,0;1/3,1/6,0;1/3,-1/6,0]*P;
        f_foot1 = [1,2,3,4,1;5,6,7,8,5;1,2,6,5,1;1,4,8,5,1;3,2,6,7,3;3,4,8,7,3;...
            9,10,3,11,9;12,13,7,14,12;3,10,13,7,3;3,11,14,7,3];
        c_foot1 = [0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;1,1,1;1,1,1;1,1,1;1,1,1];    
        [v_leg1,f_leg1,~,~] = stlRead('Sim_RobotLeg.stl'); 
        [v_joint1,f_joint,~,~] = stlRead('Sim_RobotJoint.stl'); 
        [v_leg2,f_leg2,~,~] = stlRead('Sim_RobotLeg.stl'); 
        [v_joint2,f_joint2,~,~] = stlRead('Sim_RobotJoint.stl'); 
        [v_leg3,f_leg3,~,~] = stlRead('Sim_RobotLeg.stl'); 
        [v_joint3,f_joint3,~,~] = stlRead('Sim_RobotJoint.stl');
        [v_leg4,f_leg4,~,~] = stlRead('Sim_RobotLeg.stl');     
        v_foot2 = [-1/2,0,.1;0,1/2,.1;1/2,0,.1;0,-1/2,.1;-1/2,0,0;0,1/2,0;1/2,0,0;0,-1/2,0;...
            1/6,0,.1;1/3,1/6,.1;1/3,-1/6,.1;1/6,0,0;1/3,1/6,0;1/3,-1/6,0]*P;
        f_foot2 = [1,2,3,4,1;5,6,7,8,5;1,2,6,5,1;1,4,8,5,1;3,2,6,7,3;3,4,8,7,3;...
            9,10,3,11,9;12,13,7,14,12;3,10,13,7,3;3,11,14,7,3];
        c_foot2 = [0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;1,1,1;1,1,1;1,1,1;1,1,1];    
        [v_hand,f_hand,~,~] = stlRead('Sim_RobotHand.stl');
        robot.r_v = struct('a',v_foot1,'b',v_leg1,'c',v_joint1,'d',v_leg2,'e',v_joint2,...
            'f',v_leg3,'g',v_joint3,'h',v_leg4,'i',v_foot2,'j',v_hand);
        
        r1 = patch('Faces',f_foot1,'Vertices',robot.r_v.a,'FaceColor','flat','FaceVertexCData',c_foot1);
        r2 = patch('Faces',f_leg1,'Vertices',robot.r_v.b,'FaceColor','w');
        r3 = patch('Faces',f_joint,'Vertices',robot.r_v.c,'FaceColor','k');
        r4 = patch('Faces',f_leg2,'Vertices',robot.r_v.d,'FaceColor','w');
        r5 = patch('Faces',f_joint2,'Vertices',robot.r_v.e,'FaceColor','k');
        r6 = patch('Faces',f_leg3,'Vertices',robot.r_v.f,'FaceColor','w');
        r7 = patch('Faces',f_joint3,'Vertices',robot.r_v.g,'FaceColor','k');
        r8 = patch('Faces',f_leg4,'Vertices',robot.r_v.h,'FaceColor','w');
        r9 = patch('Faces',f_foot2,'Vertices',robot.r_v.i,'FaceColor','flat','FaceVertexCData',c_foot2);
        r10 = patch('Faces',f_hand,'Vertices',robot.r_v.j,'FaceColor','k');
        robot.r_p = struct('a',r1,'b',r2,'c',r3,'d',r4,'e',r5,'f',r6,'g',r7,'h',r8,'i',r9,'j',r10);
    end

    a1 = norm(c(:,3)-c(:,2));
    a2 = norm(c(:,2)-c(:,1));
    th1 = boundedAngle(90-p(3));
    th2 = boundedAngle(p(5)-90);
    th3 = p(2);
    th4 = p(1);
    th5 = p(7);
    th6 = boundedAngle(-p(7)+180);
    RotY1 = [cosd(th1),0,sind(th1);0,1,0;-sind(th1),0,cosd(th1)];
    RotY2 = [cosd(th2),0,sind(th2);0,1,0;-sind(th2),0,cosd(th2)];
    RotZ1r = [cosd(th3),-sind(th3),0;sind(th3),cosd(th3),0;0,0,1];
    RotZ1l = [cosd(th4),-sind(th4),0;sind(th4),cosd(th4),0;0,0,1];
    RotZ2l = [cosd(th5),-sind(th5),0;sind(th5),cosd(th5),0;0,0,1];
    RotZ2h = [cosd(th6),-sind(th6),0;sind(th6),cosd(th6),0;0,0,1];
    RotH = [cosd(th_h),0,sind(th_h);0,1,0;-sind(th_h),0,cosd(th_h)];
    
    r_v = robot.r_v;
    va = r_v.a*RotZ1l';
    va(:,1) = va(:,1)+c(1,1); va(:,2) = va(:,2)+c(2,1); va(:,3) = va(:,3)+c(3,1);
    set(robot.r_p.a,'Vertices',va); 
         
    vb = (a2/sqrt(2))*[r_v.b(:,[1,2])/2,r_v.b(:,3)]; 
    vb = vb*RotZ1l';
    vb(:,1) = vb(:,1)+c(1,1); vb(:,2) = vb(:,2)+c(2,1); vb(:,3) = vb(:,3)+c(3,1);
    set(robot.r_p.b,'Vertices',vb); 
        
    vc = P*[r_v.c(:,1)/3,r_v.c(:,2)/2,r_v.c(:,3)/3]; vc = vc*RotZ1r';
    vc(:,1) = vc(:,1)+c(1,2); vc(:,2) = vc(:,2)+c(2,2); vc(:,3) = vc(:,3)+c(3,2);
    set(robot.r_p.c,'Vertices',vc);
    
    vd = a1*[r_v.d(:,[1,2])/8,r_v.d(:,3)]; vd = vd*(RotZ1r*RotY1)';   
    vd(:,1) = vd(:,1)+c(1,2); vd(:,2) = vd(:,2)+c(2,2); vd(:,3) = vd(:,3)+c(3,2);
    set(robot.r_p.d,'Vertices',vd);
    
    ve = P*[r_v.e(:,1)/3,r_v.e(:,2)/2,r_v.e(:,3)/3]; ve = ve*RotZ1r';
    ve(:,1) = ve(:,1)+c(1,3); ve(:,2) = ve(:,2)+c(2,3); ve(:,3) = ve(:,3)+c(3,3);
    set(robot.r_p.e,'Vertices',ve);
    
    vf = a1*[r_v.f(:,[1,2])/8,r_v.f(:,3)]; vf = vf*(RotZ1r*RotY2)';
    vf(:,1) = vf(:,1)+c(1,4); vf(:,2) = vf(:,2)+c(2,4); vf(:,3) = vf(:,3)+c(3,4);
    set(robot.r_p.f,'Vertices',vf);
    
    vg = P*[r_v.g(:,1)/3,r_v.g(:,2)/2,r_v.g(:,3)/3]; vg = vg*RotZ1r';
    vg(:,1) = vg(:,1)+c(1,4); vg(:,2) = vg(:,2)+c(2,4); vg(:,3) = vg(:,3)+c(3,4);
    set(robot.r_p.g,'Vertices',vg);
    
    vh = (a2/sqrt(2))*[r_v.h(:,[1,2])/2,r_v.h(:,3)]; vh = vh*RotZ2l';
    vh(:,1) = vh(:,1)+c(1,4); vh(:,2) = vh(:,2)+c(2,4); vh(:,3) = vh(:,3)+c(3,5);
    set(robot.r_p.h,'Vertices',vh);
    
    vi = r_v.i*RotZ2l';
    vi(:,1) = vi(:,1)+c(1,5); vi(:,2) = vi(:,2)+c(2,5); vi(:,3) = vi(:,3)+c(3,5);
    set(robot.r_p.i,'Vertices',vi);
    
    vj = P*[r_v.j(:,1)-.35,r_v.j(:,2)-.12,r_v.j(:,3)];
    vj = vj*[1,0,0;0,0,-1;0,1,0]'; vj = vj*RotH; vj = vj*RotZ2h;
    ofs = [c(1,5),c(2,5),c(3,5)]+[.35*P,0,.12*P]*RotZ2h;
    vj(:,1) = vj(:,1)+ofs(1); vj(:,2) = vj(:,2)+ofs(2); vj(:,3) = vj(:,3)+ofs(3);
    set(robot.r_p.j,'Vertices',vj);
end

% Draw carried voxels
function robot = drawLoad(P,robot)
    c = robot.coord; p = robot.pose_d;
    th_h = robot.th_h; 
%     color = [0 .75 0];
    color = [0 0 1];

    if(isempty(robot.l_p))
        [v_load,f_load,~,~] =stlRead('Sim_Voxel.stl'); 
        robot.l_v = v_load;
        robot.l_p = patch('Faces',f_load,'Vertices',v_load,'FaceAlpha',0,'EdgeColor',color);       
    end
    l_v = robot.l_v;
    
    th6 = boundedAngle(-p(7)+180);
    RotZ2h = [cosd(th6),-sind(th6),0;sind(th6),cosd(th6),0;0,0,1];
    RotH = [cosd(th_h),0,sind(th_h);0,1,0;-sind(th_h),0,cosd(th_h)];
    
    vl = P*[l_v(:,1)+.65,l_v(:,2),l_v(:,3)-.12];
    vl = vl*RotH; vl = vl*RotZ2h;   
    ofs = [c(1,5),c(2,5),c(3,5)]+[.35*P,0,.12*P]*RotZ2h;
    vl(:,1) = vl(:,1)+ofs(1); vl(:,2) = vl(:,2)+ofs(2); vl(:,3) = vl(:,3)+ofs(3);
    set(robot.l_p,'Vertices',vl);
end

% Draw path
function path_p = drawPath(P,map,pth)
    x(1) = pth(1,1); y(1) = pth(1,2); z(1) = map(pth(1,1),pth(1,2));
    
    ind1 = 2;   ind2 = 2;
    while(ind1<size(pth,1))
        change = map(pth(ind1,1),pth(ind1,2))-map(pth(ind1-1,1),pth(ind1-1,2));
        if (change<0)
            x(ind2) = pth(ind1,1); y(ind2) = pth(ind1,2); z(ind2) = map(pth(ind1-1,1),pth(ind1-1,2));
            map(pth(ind1-1,1),pth(ind1-1,2)) = map(pth(ind1-1,1),pth(ind1-1,2))-1;
            ind2 = ind2+1;
        elseif (change>0)
            x(ind2) = pth(ind1-1,1); y(ind2) = pth(ind1-1,2); z(ind2) = map(pth(ind1,1),pth(ind1,2));
            map(pth(ind1-1,1),pth(ind1-1,2)) = map(pth(ind1-1,1),pth(ind1-1,2))+1;
            ind2 = ind2+1;
        else
            x(ind2) = pth(ind1,1); y(ind2) = pth(ind1,2); z(ind2) = map(pth(ind1-1,1),pth(ind1-1,2));
            ind2 = ind2+1;
            ind1 = ind1+1;
        end
    end
    
    path_p = plot3((x-1)*P,(y-1)*P,P*(z-1/2),'b','LineWidth',P/2);
end
