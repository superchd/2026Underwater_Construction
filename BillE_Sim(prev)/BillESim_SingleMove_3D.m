% BillESim_SingleMove is a simplified version of the simulator for
% developing new moves!
%
% Javier Garcia, jgarciagonzalez@uh.edu 
% Jan 26, 2026

%% Initialization
clear; clc;

P = 4; % Voxel pitch
start_shift = [0,-1]; % [dx,dy] voxel shift for initial footprint
model = struct('a1_scale',1.35,'a2_scale',0.5); % consistent geometry model
hand_angle_param = []; % []=auto-safe, numeric(deg)=manual fixed hand angle

addpath(genpath("stlTools"));
bille = struct('a1',[],'a2',[],'state',[],'next_state',[],'curr_v',[],'next_v',[],...
    'path',[],'pth_ind',[],'new_pth',[],'change',[],'cost',[],'coord',[],'pose',[],...
    'pose_d',[],'th_h',[],'ths',[],'th_ind',[],'prev_ths',[],'stage',[],'anchor',[],...
    'payload',[],'flag',[],'r_p',[],'r_v',[],'l_p',[],'l_v',[],'p_p',[],...
    'foot_n',[],'hand_pref',[],'hand_lock',[]);

while(1)
    close all;

    % Make a map to test
    map = zeros(10,10,5);
    map(4:7,4:7,1) = 1; % Initial voxel map. Modify for different cases
    map(4:7,7,2:4) = 1; % Stack one more wall layer

    % Create figure and draw voxels
    figure(1); clf; hold on; axis equal;
    xlim([-2*P,(size(map,2)+1)*P]); ylim([-2*P,(size(map,1)+1)*P]); zlim([-P,size(map,3)*P]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Robot Motion (Figure 1)');
    grid on;
    voxel_p = drawVoxels(P,map,[0,0]);
    view([45,45]);

    robot = bille;
    robot.path = [];
    robot.foot_n = repmat([0,0,1],[2,1]);
    
    % Size of BillE. a1 is the longer segment, a2 is the shorter segment
    robot.a1 = model.a1_scale*P;
    robot.a2 = model.a2_scale*P;

    % Starting location of the robot (back foot row 1, front foot row 2)
    start_base = [5,5;5,6];
    robot.curr_v = start_base + repmat(start_shift,[2,1]);
    robot = getPose(P,robot,map);
    robot.pose_d = robot.pose;
    robot = syncStateFromKinematics(robot);
    robot.next_state = robot.state;

    % Angles for robot motion
    robot.ths = zeros(6,1,1); 
    robot.ths(6) = 70; % initial guess, refined below by collision check
    robot.th_ind = 1;

    % Bunch of variables for movement logic
    robot.payload = 1;
    if(isempty(hand_angle_param))
        hand_in = input("Payload hand angle override in deg (Enter=auto): ","s");
    else
        hand_in = num2str(hand_angle_param);
        fprintf("Payload hand angle override from parameter: %s deg\n",hand_in);
    end
    [has_hand_override,hand_override] = parseOptionalHandAngle(hand_in);
    if(has_hand_override)
        robot.hand_lock = true;
        robot.hand_pref = hand_override;
        robot.ths(6) = hand_override;
        init_msg = sprintf("Initial payload angle set by user to %.1f deg.",hand_override);
    else
        robot.hand_lock = false;
        [robot,init_msg] = setInitialPayloadHandSafe(P,map,robot,...
            [20,0,-20,-40,-60,-80,40,55,70,90,110,130,150]);
        robot.hand_pref = robot.ths(6);
    end
    if(strlength(init_msg)>0)
        disp(init_msg);
    end
    robot = drawRobot(P,robot);
    if(robot.payload) 
        robot = drawLoad(P,robot);
    end
    state_plot = initStatePlot(robot.state);
    figure(1);

    % Choose next move to test. Example: frontwall, forward, turn90cw. end exits.
    nextMove = input("Desired move: ","s");
    switch nextMove
        case "end"
            break;
        otherwise
            [robot,ths] = motionRule(P,robot,nextMove,map);
    end

    if(isempty(ths))
        continue;
    end

    if(~isempty(robot.path))
        figure(1);
        drawPlannedFootpaths(robot.path);
    end

    for k = 1:size(ths,3)
        robot.ths = ths(:,:,k);
        pause(1);
        
        for j = 1:size(ths,2)
            if(~isempty(robot.path) && isfield(robot.path,'foot_n_traj'))
                nframes = size(robot.path.foot_n_traj,3);
                jn = min(j,nframes);
                robot.foot_n = robot.path.foot_n_traj(:,:,jn,k);
            end
            robot.th_ind = j;
            robot = updatePoseD(robot);
            robot = moveRobot(robot);
            robot = syncStateFromKinematics(robot);
            state_plot = appendStatePlot(state_plot,robot.state);
            robot = drawRobot(P,robot);
            if(robot.payload) 
                robot = drawLoad(P,robot);
            end

            pause(.2);
        end
        robot.anchor = ~robot.anchor;
        robot.pose = robot.pose_d;
        robot = syncStateFromKinematics(robot);
        state_plot = appendStatePlot(state_plot,robot.state);
    end

    pause(1);
    % break;
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

    n_foot1 = [0;0;1];
    n_foot2 = [0;0;1];
    if(isfield(robot,'foot_n') && ~isempty(robot.foot_n) && size(robot.foot_n,1)>=2 && size(robot.foot_n,2)==3)
        n_foot1 = robot.foot_n(1,:)';
        n_foot2 = robot.foot_n(2,:)';
    end
    n_foot1 = n_foot1/max(norm(n_foot1),1e-9);
    n_foot2 = n_foot2/max(norm(n_foot2),1e-9);
        
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

    if(anchor==0)
        n_fixed = n_foot1; n_moving = n_foot2;
    else
        n_fixed = n_foot2; n_moving = n_foot1;
    end

    fixed_joint_idx = 2+2*anchor;
    fixed_contact_idx = 1+4*anchor;
    moving_joint_idx = 4-2*anchor;
    moving_contact_idx = 5-4*anchor;

    coord(:,fixed_contact_idx) = coord(:,fixed_joint_idx)-a2*n_fixed;
    coord(:,moving_contact_idx) = coord(:,moving_joint_idx)-a2*n_moving;
    robot.coord = coord;
end

% Generate angles for desired motion
function [robot,ths] = motionRule(P,robot,action,map)
    a1 = robot.a1;
    robot.path = [];
    robot.foot_n = repmat([0,0,1],[2,1]);
    robot = syncStateFromKinematics(robot);
    robot.next_state = robot.state;
    
    steps = 10;  % Intermediate points in motion parabola (evenly spaced in x)
    
    % x = xor(anchor,stage);
    % dir = (1-2*stage)*(1-2*x);
        
    % Different x's and y's are associated with different actions
    switch action
        case {"frontwall","climbfrontwall","wall"}
            robot.anchor = 0;
            [plan,msg] = planFrontWallClimb(P,robot,map);
            if(isempty(plan))
                ths = [];
                warning(msg);
                return;
            end
            robot.path = plan;
            ths = plan.ths;
            if(isfield(plan,'state_target'))
                robot.next_state = plan.state_target;
            end
        case "forward"
            robot.anchor = 0;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P);
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,steps+1,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
        case "backward"
            robot.anchor = 1;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P);
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,steps+1,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
        case "turn45cw"
            robot.anchor = 0;
            tau = sqrt(2)-1;
            beta = tau;

            par_x = tau*(0:P/steps:P);
            par_x = [par_x(1:steps/2),repmat(par_x(steps/2),[1,steps]),par_x(steps/2:end)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),45/steps:45/steps:45,45*ones(1,steps/2+1)];
            ths(5,:,1) = [zeros(1,steps/2+1),-45/steps:-45/steps:-45,-45*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),-45/steps:-45/steps:-45,-45*ones(1,steps/2+1)];
            ths(5,:,2) = [zeros(1,steps/2+1),45/steps:45/steps:45,45*ones(1,steps/2+1)];
        case "turn90cw"
            robot.anchor = 0;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P/2);
            par_x = [par_x,repmat(par_x(end),[1,steps]),flip(par_x)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),90/steps:90/steps:90,90*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),-90/steps:-90/steps:-90,-90*ones(1,steps/2+1)];
        case "turn180cw"
            robot.anchor = 0;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P/2);
            par_x = [par_x,repmat(par_x(end),[1,steps]),flip(par_x)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),180/steps:180/steps:180,180*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),-180/steps:-180/steps:-180,-180*ones(1,steps/2+1)];
        case "turn45ccw"
            robot.anchor = 0;
            tau = sqrt(2)-1;
            beta = tau;

            par_x = tau*(0:P/steps:P);
            par_x = [par_x(1:steps/2),repmat(par_x(steps/2),[1,steps]),par_x(steps/2:end)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),-45/steps:-45/steps:-45,-45*ones(1,steps/2+1)];
            ths(5,:,1) = [zeros(1,steps/2+1),45/steps:45/steps:45,45*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),45/steps:45/steps:45,45*ones(1,steps/2+1)];
            ths(5,:,2) = [zeros(1,steps/2+1),-45/steps:-45/steps:-45,-45*ones(1,steps/2+1)];
        case "turn90ccw"
            robot.anchor = 0;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P/2);
            par_x = [par_x,repmat(par_x(end),[1,steps]),flip(par_x)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),-90/steps:-90/steps:-90,-90*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),90/steps:90/steps:90,90*ones(1,steps/2+1)];
        case "turn180ccw"
            robot.anchor = 0;
            tau = 1;
            beta = tau;

            par_x = tau*(0:P/steps:P/2);
            par_x = [par_x,repmat(par_x(end),[1,steps]),flip(par_x)];
            par_y = (-4*par_x.^2/P+4*beta*par_x)/(2*tau^2);
            b = ((par_x+P).^2+par_y.^2).^0.5;

            ths = zeros(6,2*steps+2,2);
            ths(6,:,:) = 110;
            ths(3,:,1) = acosd((2*a1^2-b.^2)./(2*a1^2));
            ths(2,:,1) = ((180-ths(3,:,1))/2)+atand(par_y./(par_x+P));
            ths(4,:,1) = ((180-ths(3,:,1))/2)-atand(par_y./(par_x+P));
            ths(1,:,1) = [zeros(1,steps/2+1),-180/steps:-180/steps:-180,-180*ones(1,steps/2+1)];
            
            ths(2:4,:,2) = flip(ths(2:4,:,1),2);
            ths(2:4,:,2) = flip(ths(2:4,:,2),1);
            ths(1,:,2) = [zeros(1,steps/2+1),180/steps:180/steps:180,180*ones(1,steps/2+1)];
        otherwise 
            ths = [];
            warning("Invalid move");
    end
end

% Build a simple 3D climb that places both feet on the wall in front.
% The moving foot follows a canonical path:
%   disengage -> up in z -> straight in xy -> down in z -> engage
function [plan,msg] = planFrontWallClimb(P,robot,map)
    plan = [];
    msg = "";
    robot = syncStateFromKinematics(robot);

    if(isempty(robot.curr_v) || size(robot.curr_v,1)~=2 || size(robot.curr_v,2)~=2)
        msg = "Robot current footprint is not initialized.";
        return;
    end

    back_v = robot.curr_v(1,:);
    front_v = robot.curr_v(2,:);
    heading = front_v-back_v;
    heading = sign(heading);
    if(sum(abs(heading))~=1)
        msg = "frontwall currently supports only axis-aligned footprints.";
        return;
    end

    front_top = getTopLevel(map,front_v);
    wall_vs = findFrontWallVoxels(front_v,heading,map,front_top);
    if(isempty(wall_vs))
        msg = "No climbable wall footprint was found in front of the robot.";
        return;
    end

    foot_half_span = 0.5*P;
    wall_clearance = foot_half_span + 1e-3;
    [front_start,back_start,hand_ori] = unpackState7(robot.state);

    seg_samples = 6;
    reach_max = maxReachDistance(robot.a1);
    n_up = [0,0,1];
    anchor_joint_1 = back_start + robot.a2*n_up;

    % Optional deterministic target requested by user:
    %   front foot -> voxel (5,7,2), face normal (0,-1,0)
    %   back foot  -> voxel (5,7,1), face normal (0,-1,0)
    force_target.enabled = true;
    force_target.only = false;
    force_target.wall_v = [5,7];
    force_target.levels = [2,1]; % [front_level, back_level]

    found = false;
    last_msg = "";
    for wi = 1:size(wall_vs,1)
        wall_v_c = wall_vs(wi,:);
        wall_levels = getColumnLevels(map,wall_v_c);
        force_wall = force_target.enabled && all(wall_v_c==force_target.wall_v) && all(heading==[0,1]);
        if(force_wall)
            % Center seed on voxel i-index so forced target clearly belongs to voxel (5,7,*)
            force_seed = [(wall_v_c(1)-0.5)*P,(wall_v_c(2)-0.5)*P,front_start(3)];
            [wall_xy,wall_n] = wallFaceAnchor(P,wall_v_c,heading,wall_clearance,force_seed);
        else
            [wall_xy,wall_n] = wallFaceAnchor(P,wall_v_c,heading,wall_clearance,front_start);
        end
        n_wall = wall_n(:)'/max(norm(wall_n),1e-9);

        pairs_forced = zeros(0,2);
        if(force_wall)
            lvl = force_target.levels;
            if(ismember(lvl(1),wall_levels) && ismember(lvl(2),wall_levels))
                pairs_forced = lvl;
            end
        end
        pairs_hi = buildWallFootholdCandidates(front_top,wall_levels,false);
        pairs_lo = buildWallFootholdCandidates(front_top,wall_levels,true);
        if(force_wall && force_target.only)
            pair_sets = {pairs_forced};
        else
            pair_sets = {pairs_forced,pairs_hi,pairs_lo};
        end

        for si = 1:numel(pair_sets)
            pairs = pair_sets{si};
            if(isempty(pairs))
                last_msg = sprintf("wall (%d,%d): no foothold pairs in mode %d.",...
                    wall_v_c(1),wall_v_c(2),si);
                continue;
            end

            for pi = 1:size(pairs,1)
                front_level = pairs(pi,1);
                back_level = pairs(pi,2);
                front_target_c = [wall_xy,(front_level-0.5)*P];
                back_target_c = [wall_xy,(back_level-0.5)*P];

                front_joint = front_target_c + robot.a2*n_wall;
                back_joint = back_target_c + robot.a2*n_wall;
                if(norm(front_joint-anchor_joint_1)>reach_max || norm(back_joint-front_joint)>reach_max)
                    last_msg = sprintf("wall (%d,%d) pair (%d,%d): reach pre-check failed.",...
                        wall_v_c(1),wall_v_c(2),front_level,back_level);
                    continue;
                end

                front_path_c = buildWallApproachPath(back_start,[0,0,1],front_start,front_target_c,...
                    heading,wall_n,P,robot.a1,robot.a2,seg_samples);
                back_path_c = buildBackWallApproachPath(front_target_c,wall_n,back_start,back_target_c,...
                    heading,wall_n,P,robot.a1,robot.a2,seg_samples);
                if(isempty(front_path_c) || isempty(back_path_c))
                    last_msg = sprintf("wall (%d,%d) pair (%d,%d): no path candidate.",...
                        wall_v_c(1),wall_v_c(2),front_level,back_level);
                    continue;
                end

                front_n_path_c = buildFootNormalPath(front_path_c,wall_n);
                back_n_path_c = buildFootNormalPath(back_path_c,wall_n);

                [ths_1_c,ok,msg_1] = footPathToAnglesWithNormals(robot.a1,robot.a2,robot.pose(2),...
                    back_start,[0,0,1],front_path_c,front_n_path_c,0);
                if(~ok)
                    last_msg = "front IK failed: " + msg_1;
                    continue;
                end

                base_yaw_2 = boundedAngle(robot.pose(6)+ths_1_c(1,end));
                [ths_2_c,ok,msg_2] = footPathToAnglesWithNormals(robot.a1,robot.a2,base_yaw_2,...
                    front_target_c,wall_n,back_path_c,back_n_path_c,1);
                if(~ok)
                    last_msg = "back IK failed: " + msg_2;
                    continue;
                end

                n_motion_c = max(size(ths_1_c,2),size(ths_2_c,2));
                ths_c = zeros(6,n_motion_c,2);
                ths_c(:,:,1) = resampleAngleSequence(ths_1_c,n_motion_c);
                ths_c(:,:,2) = resampleAngleSequence(ths_2_c,n_motion_c);
                [payload_safe_hand,hand_candidates] = plannerHandAnglePolicy(robot);
                ths_c(6,:,:) = payload_safe_hand;
                foot_n_traj_c = buildFootNormalTrajectory(front_n_path_c,back_n_path_c,wall_n,n_motion_c);

                % Payload-safe pre-lift/pre-rotate profile candidates.
                % Try safer hand angles first before rejecting this foothold pair.
                ok_col = false;
                msg_col = "";
                ths_best = ths_c;
                for hc = 1:numel(hand_candidates)
                    ths_try = applyPayloadSafeHandProfile(ths_c,hand_candidates(hc));
                    [ok_col,msg_col] = isTrajectoryCollisionFree(P,map,robot,ths_try,foot_n_traj_c);
                    if(ok_col)
                        ths_best = ths_try;
                        break;
                    end
                end
                if(~ok_col)
                    last_msg = "Collision rejected: " + msg_col;
                    continue;
                end

                found = true;
                wall_v = wall_v_c;
                front_target = front_target_c;
                back_target = back_target_c;
                front_path = front_path_c;
                back_path = back_path_c;
                ths = ths_best;
                foot_n_traj = foot_n_traj_c;
                break;
            end

            if(found)
                break;
            end
        end

        if(found)
            break;
        end
    end

    if(~found)
        msg = "No feasible collision-free wall-climb trajectory found for walls ahead.";
        if(strlength(last_msg)>0)
            msg = msg + " " + last_msg;
        end
        return;
    end

    state_target = packState7(front_target,back_target,hand_ori);
    plan = struct('ths',ths,'front_path',front_path,'back_path',back_path,...
        'front_target',front_target,'back_target',back_target,...
        'wall_v',wall_v,'heading',heading,'wall_n',wall_n,...
        'reachable',[front_target;back_target],'foot_n_traj',foot_n_traj,...
        'state_target',state_target);
end

function wall_vs = findFrontWallVoxels(front_v,heading,map,front_top)
    wall_vs = [];
    probe = front_v+heading;
    while(probe(1)>=1 && probe(1)<=size(map,1) && probe(2)>=1 && probe(2)<=size(map,2))
        if(getTopLevel(map,probe)>front_top)
            wall_vs = [wall_vs;probe]; %#ok<AGROW>
        end
        probe = probe+heading;
    end
end

function levels = getColumnLevels(map,v)
    col = squeeze(map(v(1),v(2),:));
    levels = find(col>0)';
end

function top = getTopLevel(map,v)
    levels = getColumnLevels(map,v);
    if(isempty(levels))
        top = 0;
    else
        top = levels(end);
    end
end

function pairs = buildWallFootholdCandidates(front_top,wall_levels,allow_lower_front)
    if(nargin<3)
        allow_lower_front = false;
    end

    wall_levels = sort(unique(wall_levels(:)'),'ascend');
    if(allow_lower_front)
        front_levels = wall_levels(wall_levels>=front_top+1);
        back_pool = wall_levels(wall_levels>=front_top);
        preferred_list = [front_top+1,front_top;...
            front_top+1,front_top+1;...
            front_top+2,front_top+1];
    else
        move_levels = wall_levels(wall_levels>=front_top+1);
        front_levels = move_levels(move_levels>=front_top+2);
        back_pool = move_levels;
        preferred_list = [front_top+2,front_top+1];
    end

    pairs = zeros(0,2);
    for i = 1:size(preferred_list,1)
        preferred = preferred_list(i,:);
        if(ismember(preferred(1),front_levels) && ismember(preferred(2),back_pool))
            if(isempty(pairs) || ~ismember(preferred,pairs,'rows'))
                pairs = [pairs;preferred]; %#ok<AGROW>
            end
        end
    end

    for fi = numel(front_levels):-1:1
        front_level = front_levels(fi);
        if(allow_lower_front)
            back_options = back_pool(back_pool<=front_level);
        else
            back_options = back_pool(back_pool<front_level);
        end
        for bi = numel(back_options):-1:1
            cand = [front_level,back_options(bi)];
            if(isempty(pairs) || ~ismember(cand,pairs,'rows'))
                pairs = [pairs;cand]; %#ok<AGROW>
            end
        end
    end
end

function [xy,normal] = wallFaceAnchor(P,wall_v,heading,clearance,lateral_seed)
    if(nargin<5 || isempty(lateral_seed))
        lateral_seed = [(wall_v(1)-0.5)*P,(wall_v(2)-0.5)*P,0];
    end
    x_seed = lateral_seed(1);
    y_seed = lateral_seed(2);
    x_min = (wall_v(1)-1)*P;
    x_max = wall_v(1)*P;
    y_min = (wall_v(2)-1)*P;
    y_max = wall_v(2)*P;

    if(all(heading==[1,0]))
        xy = [(wall_v(1)-1)*P-clearance,clampScalar(y_seed,y_min,y_max)];
        normal = [-1,0,0];
    elseif(all(heading==[-1,0]))
        xy = [wall_v(1)*P+clearance,clampScalar(y_seed,y_min,y_max)];
        normal = [1,0,0];
    elseif(all(heading==[0,1]))
        xy = [clampScalar(x_seed,x_min,x_max),(wall_v(2)-1)*P-clearance];
        normal = [0,-1,0];
    else
        xy = [clampScalar(x_seed,x_min,x_max),wall_v(2)*P+clearance];
        normal = [0,1,0];
    end
end

function [front_target,back_target,msg] = chooseWallFootholds(P,a1,a2,back_start,wall_xy,wall_n,front_top,wall_levels)
    front_target = [];
    back_target = [];
    msg = "";

    wall_levels = wall_levels(:)';
    move_levels = wall_levels(wall_levels>=front_top+1);
    if(numel(move_levels)<2)
        msg = "Front wall does not have two usable levels above current support.";
        return;
    end

    front_candidates = move_levels(move_levels>=front_top+2);
    if(isempty(front_candidates))
        msg = "No wall level is high enough for front foot to move one level above the back foot.";
        return;
    end

    reach_max = maxReachDistance(a1);
    n_up = [0,0,1];
    n_wall = wall_n(:)'/max(norm(wall_n),1e-9);
    anchor_joint_1 = back_start + a2*n_up;
    for fi = numel(front_candidates):-1:1
        front_level = front_candidates(fi);
        back_options = move_levels(move_levels<front_level);
        if(isempty(back_options))
            continue;
        end
        back_level = back_options(end); % highest feasible level below front
        z_back = (back_level-0.5)*P;
        z_front = (front_level-0.5)*P;
        cand_front = [wall_xy,z_front];
        cand_back = [wall_xy,z_back];
        front_joint = cand_front + a2*n_wall;
        back_joint = cand_back + a2*n_wall;
        if(norm(front_joint-anchor_joint_1)<=reach_max && norm(back_joint-front_joint)<=reach_max)
            front_target = cand_front;
            back_target = cand_back;
            return;
        end
    end

    msg = "No reachable (front-high, back-lower) wall foothold pair was found.";
end

function path = buildSafeStraightPath(anchor_pt,start_pt,target_pt,P,a1,samples_per_segment)
    safe_z = computeSafeZ(anchor_pt,start_pt,target_pt,P,a1);

    waypoints = [start_pt;
        start_pt(1),start_pt(2),safe_z;
        target_pt(1),target_pt(2),safe_z;
        target_pt];
    path = interpolateWaypoints(waypoints,samples_per_segment);
end

function path = buildWallApproachPath(anchor_pt,anchor_n,start_pt,target_pt,heading,target_n,...
        P,a1,a2,samples_per_segment)
    heading = heading(:)';
    if(norm(heading)<1e-9)
        path = buildSafeStraightPathAdaptive(anchor_pt,anchor_n,start_pt,target_pt,target_n,...
            P,a1,a2,samples_per_segment);
        return;
    end
    heading = heading/norm(heading);

    safe_z = computeSafeZ(anchor_pt,start_pt,target_pt,P,a1);
    % Payload-aware extra lift before moving toward wall.
    safe_z = min(safe_z + 0.5*P,target_pt(3)+1.5*P);
    approach_scan = [2.0,1.75,1.5,1.25,1.0,0.75,0.5,0.25,0.0]*P;
    path = [];
    for i = 1:numel(approach_scan)
        pre_xy = target_pt(1:2)-approach_scan(i)*heading;
        waypointsA = [start_pt;
            start_pt(1),start_pt(2),safe_z;
            pre_xy(1),pre_xy(2),safe_z;
            pre_xy(1),pre_xy(2),target_pt(3);
            target_pt];
        candA = interpolateWaypoints(waypointsA,samples_per_segment);
        candA_n = buildFootNormalPath(candA,target_n);
        if(pathWithinReachWithNormals(anchor_pt,anchor_n,candA,candA_n,a1,a2))
            path = candA;
            return;
        end

        % Alternative for tall wall targets: move toward wall first, then rise.
        waypointsB = [start_pt;
            pre_xy(1),pre_xy(2),start_pt(3);
            pre_xy(1),pre_xy(2),target_pt(3);
            target_pt];
        candB = interpolateWaypoints(waypointsB,samples_per_segment);
        candB_n = buildFootNormalPath(candB,target_n);
        if(pathWithinReachWithNormals(anchor_pt,anchor_n,candB,candB_n,a1,a2))
            path = candB;
            return;
        end
    end

    % Fallback in case no pre-contact distance is feasible.
    path = buildSafeStraightPathAdaptive(anchor_pt,anchor_n,start_pt,target_pt,target_n,...
        P,a1,a2,samples_per_segment);
end

function path = buildBackWallApproachPath(anchor_pt,anchor_n,start_pt,target_pt,heading,target_n,...
        P,a1,a2,samples_per_segment)
    heading = heading(:)';
    if(norm(heading)<1e-9)
        path = buildSafeStraightPathAdaptive(anchor_pt,anchor_n,start_pt,target_pt,target_n,...
            P,a1,a2,samples_per_segment);
        return;
    end
    heading = heading/norm(heading);

    approach_scan = [2.0,1.75,1.5,1.25,1.0,0.75,0.5,0.25,0.0]*P;
    lift_scan = [0.75,0.6,0.45,0.3]*P;
    path = [];
    for i = 1:numel(approach_scan)
        pre_xy = target_pt(1:2)-approach_scan(i)*heading;
        for li = 1:numel(lift_scan)
            z_lift = start_pt(3)+lift_scan(li);
            z_lift = min(z_lift,target_pt(3));

            % Natural back-foot climb: small disengage + diagonal rise + short final approach
            mid_xy = start_pt(1:2) + 0.6*(pre_xy-start_pt(1:2));
            z_mid = z_lift + 0.65*(target_pt(3)-z_lift);
            waypointsN = [start_pt;
                start_pt(1),start_pt(2),z_lift;
                mid_xy(1),mid_xy(2),z_mid;
                pre_xy(1),pre_xy(2),target_pt(3);
                target_pt];
            candN = interpolateWaypoints(waypointsN,samples_per_segment);
            candN_n = buildFootNormalPath(candN,target_n);
            if(pathWithinReachWithNormals(anchor_pt,anchor_n,candN,candN_n,a1,a2))
                path = candN;
                return;
            end

            waypointsA = [start_pt;
                start_pt(1),start_pt(2),z_lift;
                pre_xy(1),pre_xy(2),z_lift;
                pre_xy(1),pre_xy(2),target_pt(3);
                target_pt];
            candA = interpolateWaypoints(waypointsA,samples_per_segment);
            candA_n = buildFootNormalPath(candA,target_n);
            if(pathWithinReachWithNormals(anchor_pt,anchor_n,candA,candA_n,a1,a2))
                path = candA;
                return;
            end

            waypointsB = [start_pt;
                pre_xy(1),pre_xy(2),start_pt(3);
                pre_xy(1),pre_xy(2),target_pt(3);
                target_pt];
            candB = interpolateWaypoints(waypointsB,samples_per_segment);
            candB_n = buildFootNormalPath(candB,target_n);
            if(pathWithinReachWithNormals(anchor_pt,anchor_n,candB,candB_n,a1,a2))
                path = candB;
                return;
            end
        end
    end

    path = buildSafeStraightPathAdaptive(anchor_pt,anchor_n,start_pt,target_pt,target_n,...
        P,a1,a2,samples_per_segment);
end

function path = buildSafeStraightPathAdaptive(anchor_pt,anchor_n,start_pt,target_pt,target_n,...
        P,a1,a2,samples_per_segment)
    z_max = computeSafeZ(anchor_pt,start_pt,target_pt,P,a1);
    z_min = max(start_pt(3),target_pt(3));
    z_trials = linspace(z_max,z_min,7);

    for z = z_trials
        waypoints = [start_pt;
            start_pt(1),start_pt(2),z;
            target_pt(1),target_pt(2),z;
            target_pt];
        cand = interpolateWaypoints(waypoints,samples_per_segment);
        cand_n = buildFootNormalPath(cand,target_n);
        if(pathWithinReachWithNormals(anchor_pt,anchor_n,cand,cand_n,a1,a2))
            path = cand;
            return;
        end
    end

    path = [];
end

function safe_z = computeSafeZ(anchor_pt,start_pt,target_pt,P,a1)
    xy_1 = norm(start_pt(1:2)-anchor_pt(1:2));
    xy_2 = norm(target_pt(1:2)-anchor_pt(1:2));
    xy_max = max([xy_1,xy_2]);
    reach_max = maxReachDistance(a1);
    max_dz = sqrt(max(reach_max^2-xy_max^2,0));

    safe_z_nom = max(start_pt(3),target_pt(3))+P;
    safe_z_max = anchor_pt(3)+0.95*max_dz;
    safe_z = min(safe_z_nom,safe_z_max);
    safe_z = max(safe_z,max(start_pt(3),target_pt(3)));
end

function ok = pathWithinReach(anchor_pt,path_pts,a1)
    reach_max = maxReachDistance(a1);
    d = path_pts-anchor_pt;
    dist = sqrt(sum(d.^2,2));
    ok = all((dist<reach_max) & (dist>1e-6));
end

function ok = pathWithinReachWithNormals(anchor_contact,anchor_n,path_pts,path_n,a1,a2)
    reach_max = maxReachDistance(a1);
    anchor_n = anchor_n(:)'/max(norm(anchor_n),1e-9);
    anchor_joint = anchor_contact + a2*anchor_n;
    ok = true;
    for i = 1:size(path_pts,1)
        n = path_n(i,:)/max(norm(path_n(i,:)),1e-9);
        joint = path_pts(i,:) + a2*n;
        b = norm(joint-anchor_joint);
        if(b>=reach_max || b<1e-6)
            ok = false;
            return;
        end
    end
end

function n_path = buildFootNormalPath(path_pts,wall_n)
    n0 = [0,0,1];
    n1 = wall_n(:)';
    n1 = n1/max(norm(n1),1e-9);
    n = size(path_pts,1);
    s = linspace(0,1,n)';
    % Rotate toward wall earlier so joint-chain distance stays within reach
    % during high-z approach points.
    alpha = 1-(1-s).^3; % ease-out

    n_path = zeros(n,3);
    for i = 1:n
        v = (1-alpha(i))*n0 + alpha(i)*n1;
        n_path(i,:) = v/max(norm(v),1e-9);
    end
end

function out = resampleAngleSequence(in,n_out)
    n_in = size(in,2);
    if(n_in==n_out)
        out = in;
        return;
    end
    if(n_out>n_in)
        out = [in,repmat(in(:,end),[1,n_out-n_in])];
        return;
    end
    x = 1:n_in;
    xi = linspace(1,n_in,n_out);
    out = interp1(x,in',xi,'linear')';
end

function out = resampleVectorSequence(in,n_out)
    n_in = size(in,1);
    if(n_in==n_out)
        out = in;
        return;
    end
    if(n_out>n_in)
        out = [in;repmat(in(end,:),[n_out-n_in,1])];
        out = normalizeRows(out);
        return;
    end
    x = 1:n_in;
    xi = linspace(1,n_in,n_out);
    out = interp1(x,in,xi,'linear');
    out = normalizeRows(out);
end

function foot_n_traj = buildFootNormalTrajectory(front_n_path,back_n_path,wall_n,n_motion)
    front_n_seq = resampleVectorSequence(front_n_path,n_motion);
    back_n_seq = resampleVectorSequence(back_n_path,n_motion);
    foot_n_traj = zeros(2,3,n_motion,2);
    for i = 1:n_motion
        foot_n_traj(:,:,i,1) = [0,0,1;front_n_seq(i,:)];
        foot_n_traj(:,:,i,2) = [back_n_seq(i,:);wall_n];
    end
end

function ths_out = applyPayloadSafeHandProfile(ths_in,safe_h)
    ths_out = ths_in;
    if(isempty(ths_in) || size(ths_in,2)<2)
        return;
    end

    base_h = ths_in(6,1,1);
    n = size(ths_in,2);
    prep = min(6,max(2,round(0.2*n)));

    % Dedicated pre-lift/pre-rotate: hold body/foot trajectory briefly,
    % rotate payload hand to a safer attitude before wall engagement.
    base_col = ths_in(:,1,1);
    for j = 1:prep
        a = (j-1)/max(prep-1,1);
        ths_out(1:5,j,1) = base_col(1:5);
        ths_out(6,j,1) = (1-a)*base_h + a*safe_h;
    end
    if(prep<n)
        ths_out(6,prep+1:end,1) = safe_h;
    end

    for j = 1:n
        a = (j-1)/max(n-1,1);
        ths_out(6,j,2) = (1-a)*safe_h + a*base_h;
    end
end

function [ok,msg] = isTrajectoryCollisionFree(P,map,robot0,ths,foot_n_traj)
    ok = true;
    msg = "";
    occ = occupiedVoxelBoxes(map,P);
    if(isempty(occ))
        return;
    end

    sim = robot0;

    for k = 1:size(ths,3)
        sim.ths = ths(:,:,k);
        for j = 1:size(ths,2)
            if(~isempty(foot_n_traj))
                nframes = size(foot_n_traj,3);
                jn = min(j,nframes);
                sim.foot_n = foot_n_traj(:,:,jn,k);
            end
            sim.th_ind = j;
            sim = updatePoseD(sim);
            sim = moveRobot(sim);

            % Allow planner to start from current configuration and enforce
            % collision-free motion after the very first sample.
            if(k==1 && j==1)
                continue;
            end
            [hit,detail] = robotPayloadCollision(sim,occ,P);
            if(hit)
                ok = false;
                msg = sprintf("phase %d step %d: %s",k,j,detail);
                return;
            end
        end
        sim.anchor = ~sim.anchor;
        sim.pose = sim.pose_d;
    end
end

function [robot,msg] = setInitialPayloadHandSafe(P,map,robot,angle_candidates)
    msg = "";
    if(~isfield(robot,'payload') || ~robot.payload)
        return;
    end
    if(nargin<4 || isempty(angle_candidates))
        angle_candidates = [0,20,40,55,70,90,110];
    end

    occ = occupiedVoxelBoxes(map,P);
    if(isempty(occ))
        return;
    end

    best_angle = robot.ths(6);
    best_clear = -inf;
    for i = 1:numel(angle_candidates)
        a = angle_candidates(i);
        robot_try = robot;
        robot_try.ths(6) = a;
        [hit,~] = robotPayloadCollision(robot_try,occ,P);
        clear_now = payloadClearance(robot_try,occ,P);

        if(~hit)
            robot.ths(6) = a;
            msg = sprintf("Initial payload angle auto-set to %.1f deg (collision-free).",a);
            return;
        end

        if(clear_now>best_clear)
            best_clear = clear_now;
            best_angle = a;
        end
    end

    robot.ths(6) = best_angle;
    msg = sprintf("No collision-free initial payload angle found; using best-clearance angle %.1f deg.",...
        best_angle);
end

function occ = occupiedVoxelBoxes(map,P)
    idx = find(map>0);
    if(isempty(idx))
        occ = zeros(0,6);
        return;
    end
    [ii,jj,kk] = ind2sub(size(map),idx);
    occ = [(ii-1)*P,ii*P,(jj-1)*P,jj*P,(kk-1)*P,kk*P];
end

function [hit,detail] = robotPayloadCollision(robot,occ,P)
    hit = false;
    detail = "";
    if(isempty(robot.coord) || size(robot.coord,2)<5)
        return;
    end

    % v1 policy: prioritize payload-vs-environment collision.
    % Body-link collision in this simulator frame can create many false positives.
    check_body = false;
    if(check_body)
        link_r = 0.08*P;
        joint_r = 0.16*P;
        c = robot.coord';
        joints = c(3,:);
        for i = 1:size(joints,1)
            if(sphereHitsOccupied(joints(i,:),joint_r,occ))
                hit = true;
                detail = "joint intersects occupied voxel";
                return;
            end
        end

        segs = [2,3;3,4];
        for i = 1:size(segs,1)
            p0 = c(segs(i,1),:);
            p1 = c(segs(i,2),:);
            if(segmentHitsOccupied(p0,p1,link_r,occ))
                hit = true;
                detail = "link sweep intersects occupied voxel";
                return;
            end
        end
    end

    if(isfield(robot,'payload') && robot.payload)
        th_h = 110;
        if(isfield(robot,'ths') && ~isempty(robot.ths) && size(robot.ths,1)>=6)
            th_h = robot.ths(6);
        end
        c_payload = payloadCenterApprox(P,robot.coord,robot.pose_d,th_h);
        r_payload = 0.45*P;
        if(sphereHitsOccupied(c_payload,r_payload,occ))
            hit = true;
            detail = "payload intersects occupied voxel";
            return;
        end
    end
end

function dmin = payloadClearance(robot,occ,P)
    if(isempty(occ) || isempty(robot.coord) || size(robot.coord,2)<5)
        dmin = inf;
        return;
    end

    th_h = 110;
    if(isfield(robot,'ths') && ~isempty(robot.ths) && size(robot.ths,1)>=6)
        th_h = robot.ths(6);
    end

    c_payload = payloadCenterApprox(P,robot.coord,robot.pose_d,th_h);
    r_payload = 0.45*P;

    dx = max(max(occ(:,1)-c_payload(1),0),c_payload(1)-occ(:,2));
    dy = max(max(occ(:,3)-c_payload(2),0),c_payload(2)-occ(:,4));
    dz = max(max(occ(:,5)-c_payload(3),0),c_payload(3)-occ(:,6));
    d_box = sqrt(dx.^2 + dy.^2 + dz.^2);
    dmin = min(d_box)-r_payload;
end

function tf = segmentHitsOccupied(p0,p1,r,occ)
    d = norm(p1-p0);
    n = max(2,ceil(d/max(r,1e-6))+1);
    s = linspace(0,1,n)';
    tf = false;
    for i = 1:n
        c = p0 + s(i)*(p1-p0);
        if(sphereHitsOccupied(c,r,occ))
            tf = true;
            return;
        end
    end
end

function tf = sphereHitsOccupied(c,r,occ)
    dx = max(max(occ(:,1)-c(1),0),c(1)-occ(:,2));
    dy = max(max(occ(:,3)-c(2),0),c(2)-occ(:,4));
    dz = max(max(occ(:,5)-c(3),0),c(3)-occ(:,6));
    d2 = dx.^2 + dy.^2 + dz.^2;
    tf = any(d2 <= r^2);
end

function c_payload = payloadCenterApprox(P,coord,pose_d,th_h)
    if(isempty(pose_d))
        c_payload = coord(:,5)';
        return;
    end

    th6 = boundedAngle(-pose_d(7)+180);
    RotZ2h = [cosd(th6),-sind(th6),0;sind(th6),cosd(th6),0;0,0,1];
    RotH = [cosd(th_h),0,sind(th_h);0,1,0;-sind(th_h),0,cosd(th_h)];

    local_center = [1.15,0.50,0.38]*P;
    ofs = coord(:,5)' + [0.35*P,0,0.12*P]*RotZ2h;
    c_payload = local_center*RotH*RotZ2h + ofs;
end

function pts = interpolateWaypoints(waypoints,samples_per_segment)
    pts = waypoints(1,:);
    for i = 1:size(waypoints,1)-1
        t = linspace(0,1,samples_per_segment+1)';
        seg = waypoints(i,:)+(waypoints(i+1,:)-waypoints(i,:)).*t;
        pts = [pts;seg(2:end,:)]; %#ok<AGROW>
    end
end

function [ths_phase,ok,msg] = footPathToAngles(a1,base_yaw,anchor_pt,path_pts)
    ok = true;
    msg = "";
    n = size(path_pts,1);
    ths_phase = zeros(6,n);
    reach_max = maxReachDistance(a1);

    for i = 1:n
        d = path_pts(i,:)-anchor_pt;
        xy = norm(d(1:2));
        b = norm(d);

        if(b<1e-6 || b>reach_max)
            ok = false;
            msg = sprintf("Planned footprint path unreachable at point %d (dist=%.3f, max=%.3f).",...
                i,b,reach_max);
            return;
        end

        th3 = acosd((2*a1^2-b^2)/(2*a1^2));
        gamma = atan2d(d(3),xy);
        th2 = (180-th3)/2 + gamma;
        th4 = (180-th3)/2 - gamma;

        if(xy<1e-9)
            phi = base_yaw;
        else
            phi = atan2d(d(2),d(1));
        end
        yaw = boundedAngle(phi-base_yaw);

        ths_phase(1,i) = yaw;
        ths_phase(2,i) = th2;
        ths_phase(3,i) = th3;
        ths_phase(4,i) = th4;
        ths_phase(5,i) = yaw;
    end
end

function [ths_phase,ok,msg] = footPathToAnglesWithNormals(a1,a2,base_yaw,...
        anchor_contact,anchor_n,path_pts,path_n,moving_anchor)
    ok = true;
    msg = "";
    n = size(path_pts,1);
    ths_phase = zeros(6,n);
    reach_max = maxReachDistance(a1);

    anchor_n = anchor_n(:)'/max(norm(anchor_n),1e-9);
    anchor_joint = anchor_contact + a2*anchor_n;

    for i = 1:n
        move_n = path_n(i,:)/max(norm(path_n(i,:)),1e-9);
        move_joint = path_pts(i,:) + a2*move_n;
        d = move_joint-anchor_joint;
        xy = norm(d(1:2));
        b = norm(d);

        if(b<1e-6 || b>reach_max)
            ok = false;
            msg = sprintf("Planned footprint path unreachable at point %d (dist=%.3f, max=%.3f).",...
                i,b,reach_max);
            return;
        end

        th3 = acosd((2*a1^2-b^2)/(2*a1^2));
        gamma = atan2d(d(3),xy);
        shoulder = (180-th3)/2 + gamma;
        complement = (180-th3)/2 - gamma;
        if(moving_anchor==0)
            th2 = shoulder;
            th4 = complement;
        else
            th2 = complement;
            th4 = shoulder;
        end

        if(xy<1e-9)
            phi = base_yaw;
        else
            phi = atan2d(d(2),d(1));
        end
        yaw = boundedAngle(phi-base_yaw);

        ths_phase(1,i) = yaw;
        ths_phase(2,i) = th2;
        ths_phase(3,i) = th3;
        ths_phase(4,i) = th4;
        ths_phase(5,i) = yaw;
    end
end

function out = normalizeRows(in)
    out = in;
    for i = 1:size(in,1)
        n = norm(in(i,:));
        if(n<1e-9)
            out(i,:) = [0,0,1];
        else
            out(i,:) = in(i,:)/n;
        end
    end
end

function drawPlannedFootpaths(plan)
    hold on;
    plot3(plan.front_path(:,1),plan.front_path(:,2),plan.front_path(:,3),...
        '-g','LineWidth',1.5);
    plot3(plan.back_path(:,1),plan.back_path(:,2),plan.back_path(:,3),...
        '-m','LineWidth',1.5);
    plot3(plan.front_target(1),plan.front_target(2),plan.front_target(3),...
        'og','MarkerFaceColor','g','MarkerSize',6);
    plot3(plan.back_target(1),plan.back_target(2),plan.back_target(3),...
        'om','MarkerFaceColor','m','MarkerSize',6);
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
    th_h = robot.ths(6);
    
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

    n_back = [0,0,1];
    n_front = [0,0,1];
    if(isfield(robot,'foot_n') && ~isempty(robot.foot_n) && size(robot.foot_n,1)>=2 && size(robot.foot_n,2)==3)
        n_back = robot.foot_n(1,:);
        n_front = robot.foot_n(2,:);
    end
    RotFootBack = footPoseRotation(n_back,th4);
    RotFootFront = footPoseRotation(n_front,th5);
    
    r_v = robot.r_v;
    va = r_v.a*RotFootBack';
    va(:,1) = va(:,1)+c(1,1); va(:,2) = va(:,2)+c(2,1); va(:,3) = va(:,3)+c(3,1);
    set(robot.r_p.a,'Vertices',va); 
         
    vb = (a2/sqrt(2))*[r_v.b(:,[1,2])/2,r_v.b(:,3)]; 
    vb = vb*RotFootBack';
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
    
    vh = (a2/sqrt(2))*[r_v.h(:,[1,2])/2,r_v.h(:,3)]; vh = vh*RotFootFront';
    vh(:,1) = vh(:,1)+c(1,5); vh(:,2) = vh(:,2)+c(2,5); vh(:,3) = vh(:,3)+c(3,5);
    set(robot.r_p.h,'Vertices',vh);
    
    vi = r_v.i*RotFootFront';
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
    th_h = robot.ths(6); 

    if(isempty(robot.l_p))
        [v_load,f_load,~,~] =stlRead('Sim_Voxel.stl'); 
        robot.l_v = v_load;
        robot.l_p = patch('Faces',f_load,'Vertices',v_load,'FaceAlpha',0,'EdgeColor',[0 .75 0]);       
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

function R = footPoseRotation(n,yaw_deg)
    n = n(:)';
    if(norm(n)<1e-9)
        n = [0,0,1];
    else
        n = n/norm(n);
    end
    Rz = [cosd(yaw_deg),-sind(yaw_deg),0;...
          sind(yaw_deg), cosd(yaw_deg),0;...
          0,             0,            1];
    R_align = alignZToVector(n);
    R = R_align*Rz;
end

function R = alignZToVector(n)
    z = [0;0;1];
    n = n(:);
    n = n/max(norm(n),1e-9);
    v = cross(z,n);
    s = norm(v);
    c = dot(z,n);

    if(s<1e-10)
        if(c>0)
            R = eye(3);
        else
            R = [1,0,0;0,-1,0;0,0,-1];
        end
        return;
    end

    vx = [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
    R = eye(3)+vx+vx*vx*((1-c)/(s^2));
end

% Robot state (7D):
%   [front_x,front_y,front_z,back_x,back_y,back_z,hand_orientation]'
function robot = syncStateFromKinematics(robot)
    front_xyz = [0,0,0];
    back_xyz = [0,0,0];
    hand_ori = 0;

    if(isfield(robot,'coord') && ~isempty(robot.coord) && size(robot.coord,1)>=3 && size(robot.coord,2)>=5)
        back_xyz = robot.coord(:,1)';
        front_xyz = robot.coord(:,5)';
    elseif(isfield(robot,'state') && ~isempty(robot.state))
        [front_xyz,back_xyz,hand_ori] = unpackState7(robot.state);
    end

    if(isfield(robot,'pose_d') && ~isempty(robot.pose_d) && numel(robot.pose_d)>=7)
        hand_ori = boundedAngle(robot.pose_d(7));
    elseif(isfield(robot,'pose') && ~isempty(robot.pose) && numel(robot.pose)>=7)
        hand_ori = boundedAngle(robot.pose(7));
    end

    robot.state = packState7(front_xyz,back_xyz,hand_ori);
end

function state = packState7(front_xyz,back_xyz,hand_ori)
    front_xyz = sanitizeVec3(front_xyz,[0,0,0]);
    back_xyz = sanitizeVec3(back_xyz,[0,0,0]);
    hand_ori = sanitizeScalar(hand_ori,0);

    state = [front_xyz,back_xyz,hand_ori]';
end

function [front_xyz,back_xyz,hand_ori] = unpackState7(state)
    s = state(:)';
    if(numel(s)<7)
        s = [s,zeros(1,7-numel(s))];
    end

    front_xyz = s(1:3);
    back_xyz = s(4:6);
    hand_ori = s(7);
end

function v = sanitizeVec3(x,default_v)
    if(nargin<2)
        default_v = [0,0,0];
    end

    if(isempty(x))
        v = default_v;
        return;
    end

    x = x(:)';
    if(numel(x)<3)
        x = [x,default_v(numel(x)+1:3)];
    end
    v = x(1:3);

    if(any(~isfinite(v)))
        v = default_v;
    end
end

function s = sanitizeScalar(x,default_s)
    if(nargin<2)
        default_s = 0;
    end

    if(isempty(x))
        s = default_s;
        return;
    end

    x = x(1);
    if(~isfinite(x))
        s = default_s;
    else
        s = x;
    end
end

function state_plot = initStatePlot(state0)
    state0 = ensureState7(state0);

    front0 = state0(1:3);
    back0 = state0(4:6);
    hand0 = wrapAngle360Deg(state0(7));

    fig = figure(2);
    clf(fig);
    tl = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');

    c = lines(3);

    ax_front = nexttile(tl,1);
    hold(ax_front,'on'); grid(ax_front,'on');
    hf = gobjects(1,3);
    hf(1) = plot(ax_front,0,front0(1),'-','LineWidth',1.2,'Color',c(1,:));
    hf(2) = plot(ax_front,0,front0(2),'-','LineWidth',1.2,'Color',c(2,:));
    hf(3) = plot(ax_front,0,front0(3),'-','LineWidth',1.2,'Color',c(3,:));
    title(ax_front,'Front Foot Position');
    ylabel(ax_front,'Position');
    legend(ax_front,{'x','y','z'},'Location','eastoutside');
    xlim(ax_front,[0,1]);
    updateCoordAxisLimits(ax_front,front0(:)');

    ax_back = nexttile(tl,2);
    hold(ax_back,'on'); grid(ax_back,'on');
    hb = gobjects(1,3);
    hb(1) = plot(ax_back,0,back0(1),'-','LineWidth',1.2,'Color',c(1,:));
    hb(2) = plot(ax_back,0,back0(2),'-','LineWidth',1.2,'Color',c(2,:));
    hb(3) = plot(ax_back,0,back0(3),'-','LineWidth',1.2,'Color',c(3,:));
    title(ax_back,'Back Foot Position');
    ylabel(ax_back,'Position');
    legend(ax_back,{'x','y','z'},'Location','eastoutside');
    xlim(ax_back,[0,1]);
    updateCoordAxisLimits(ax_back,back0(:)');

    ax_hand = nexttile(tl,3);
    hold(ax_hand,'on'); grid(ax_hand,'on');
    hh = plot(ax_hand,0,hand0,'-','LineWidth',1.2,'Color',[0.85,0.33,0.10]);
    title(ax_hand,'Front Hand Orientation');
    ylabel(ax_hand,'Angle (deg)');
    xlabel(ax_hand,'Time step');
    legend(ax_hand,{'orientation'},'Location','eastoutside');
    xlim(ax_hand,[0,1]);
    ylim(ax_hand,[0,360]);

    state_plot = struct('fig',fig,'ax_front',ax_front,'ax_back',ax_back,'ax_hand',ax_hand,...
        'line_front',hf,'line_back',hb,'line_hand',hh,...
        't',0,'front',front0(:)','back',back0(:)','hand',hand0);
end

function state_plot = appendStatePlot(state_plot,state_k)
    state_k = ensureState7(state_k);

    t_new = state_plot.t(end)+1;
    state_plot.t(end+1,1) = t_new;

    front_k = state_k(1:3)';
    back_k = state_k(4:6)';
    hand_k = wrapAngle360Deg(state_k(7));

    state_plot.front(end+1,:) = front_k;
    state_plot.back(end+1,:) = back_k;
    state_plot.hand(end+1,1) = hand_k;

    for i = 1:3
        set(state_plot.line_front(i),'XData',state_plot.t,'YData',state_plot.front(:,i));
        set(state_plot.line_back(i),'XData',state_plot.t,'YData',state_plot.back(:,i));
    end
    set(state_plot.line_hand,'XData',state_plot.t,'YData',state_plot.hand);

    t_max = max(1,state_plot.t(end));
    set(state_plot.ax_front,'XLim',[0,t_max]);
    set(state_plot.ax_back,'XLim',[0,t_max]);
    set(state_plot.ax_hand,'XLim',[0,t_max]);

    updateCoordAxisLimits(state_plot.ax_front,state_plot.front);
    updateCoordAxisLimits(state_plot.ax_back,state_plot.back);
    set(state_plot.ax_hand,'YLim',[0,360]);

    drawnow limitrate nocallbacks;
end

function s = ensureState7(s)
    s = s(:)';
    if(numel(s)<7)
        s = [s,zeros(1,7-numel(s))];
    end
    s = s(1:7);
    if(any(~isfinite(s)))
        s(~isfinite(s)) = 0;
    end
end

function updateCoordAxisLimits(ax,data)
    y_min = min(data(:));
    y_max = max(data(:));
    if(abs(y_max-y_min)<1e-6)
        y_min = y_min-1;
        y_max = y_max+1;
    else
        pad = 0.1*(y_max-y_min);
        y_min = y_min-pad;
        y_max = y_max+pad;
    end
    set(ax,'YLim',[y_min,y_max]);
end

function a = wrapAngle360Deg(a)
    a = mod(a,360);
    if(a<0)
        a = a+360;
    end
end

function [has_override,angle] = parseOptionalHandAngle(hand_in)
    has_override = false;
    angle = 0;

    if(isempty(hand_in))
        return;
    end

    txt = strtrim(string(hand_in));
    if(strlength(txt)==0 || strcmpi(txt,"auto"))
        return;
    end

    val = str2double(txt);
    if(~isfinite(val))
        warning('Invalid hand angle input "%s". Falling back to auto mode.',txt);
        return;
    end

    has_override = true;
    angle = boundedAngle(val);
end

function [safe_h,hand_candidates] = plannerHandAnglePolicy(robot)
    default_scan = [20,0,-20,40,55,70,90,110];
    safe_h = 70;
    if(isfield(robot,'hand_pref') && ~isempty(robot.hand_pref) && isfinite(robot.hand_pref))
        safe_h = boundedAngle(robot.hand_pref);
    end

    if(isfield(robot,'hand_lock') && robot.hand_lock)
        hand_candidates = safe_h;
    else
        hand_candidates = unique([safe_h,default_scan],'stable');
    end
end

function reach_max = maxReachDistance(a1)
    % Single source of truth for kinematic reach bound.
    reach_max = 2*a1*0.999;
end

function v = clampScalar(v,vmin,vmax)
    v = min(max(v,vmin),vmax);
end
