% BillESim_Single builds upon WalkSim3D. Now that the basic robot functions
% are well defined, the next step is to actually grab and place voxels. As
% of now, the locations to grab and place a voxel are hardcoded
%
% TODO: - Planner returns configuration for grabbing and placing voxels
%       - Take voxels' starting and desired positions as input
%
% Javier Garcia, jgarciagonzalez@uh.edu 
% June 1, 2022

%% Initialization
clear; clc;

% Make a UH logo!!
n = zeros(15,15,5);
n(1:6,1:6,1) = 1; n(7,1,1) = 1; % Initial voxel map
flat_n = sum(n,3);
start_base = [1,6;1,5];   % Starting positions [back_foot; front_foot]
start_shift = [0,-1];     % [dx,dy] shift applied to both feet
start = start_base + repmat(start_shift,[2,1]);
ploc = [7,1;6,1;5,1;4,1;3,1;2,1;1,1;6,2;5,2;4,2;3,2;2,2;1,2;
    6,3;5,3;4,3;3,3;2,3;1,3;6,4;5,4;4,4;3,4;2,4;1,4;
    6,5;5,5;4,5;3,5;2,5;1,5;6,6;5,6;4,6;3,6;2,6;1,6];
dloc = [1,7;1,8;1,9;2,8;3,8;4,8;5,8;5,9;5,10;5,11;5,12;4,12;3,12;
    2,12;1,12;1,11;1,13;4,9;4,10;4,11;6,10;7,10;8,10;8,9;8,11;
    6,11;6,12;6,13;6,14;5,14;4,14;4,13;4,15;7,14;8,14;8,13;8,15];
% Spell UH without INT STRUCT
% n = zeros(20,15,5);
% n(1:6,1:6,1) = 1; n(7,[1,2],1) = 1; % Initial voxel map
% flat_n = sum(n,3);
% start = [1,6;1,5];  % Starting positions
% ploc = [7,1;6,1;5,1;4,1;3,1;2,1;1,1;7,2;6,2;5,2;4,2;3,2;2,2;1,2;
%     6,3;5,3;4,3;3,3;2,3;1,3;6,4;5,4;4,4;3,4;2,4;1,4;
%     6,5;5,5;4,5;3,5;2,5;1,5;6,6;5,6;4,6;3,6;2,6;1,6];
% dloc = [1,7;1,8;1,9;2,8;3,8;4,8;5,8;5,9;5,10;5,11;5,12;4,12;3,12;2,12;
%     1,12;1,11;1,13;1,14;1,15;1,16;2,15;3,15;4,15;5,15;5,14;5,16;3,16;
%     3,17;3,18;3,19;4,19;5,19;5,18;5,20;2,19;1,19;1,18;1,20];
% Spell UH with INT STRUCT
% n = zeros(20,20,5);
% n(1:6,1:6,1) = 1; n(7,[1,2],1) = 1; % Initial voxel map
% flat_n = sum(n,3);
% start = [1,6;1,5];  % Starting positions
% ploc = [7,1;6,1;5,1;4,1;3,1;2,1;1,1;7,2;6,2;5,2;4,2;3,2;2,2;1,2;
%     6,3;5,3;4,3;3,3;2,3;1,3;6,4;5,4;4,4;3,4;2,4;1,4;
%     6,5;5,5;4,5;3,5;2,5;1,5;6,6;5,6;4,6;3,6;2,6;1,6;1,7;1,17;1,10];
% dloc = [1,7;1,8;1,9;1,10;1,11;1,12;1,13;1,14;1,15;1,16;1,17;1,18;1,19;1,20;
%     2,19;3,19;4,19;5,19;5,18;5,20;3,18;2,15;3,15;4,15;5,15;5,16;5,14;3,16;
%     3,17;2,12;3,12;4,12;5,12;5,11;2,8;3,8;4,8;5,8;5,9;5,10;1,7];
% Testing difficult cases
% n = zeros(10,10,5);
% n(1:3,2:3,1) = 1; n(1,4,1) = 1; % Initial voxel map
% n(2,3,1) = 0;
% flat_n = sum(n,3);
% start = [2,2;1,2];  % Starting positions
% ploc = [1,4];
% dloc = [1,1];
% n = zeros(10,10,5);
% n(2,2:4,1) = 1; n(1,3,1) = 1; % Initial voxel map
% flat_n = sum(n,3);
% start = [2,2;2,3];  % Starting positions
% ploc = [1,3];
% dloc = [2,1];

P = 4; % Voxel pitch
a1 = 1.2*P; % Length of robot legs
a2 = 0.5*P; % Height of robot feet 

% Calculate starting robot pose and state
[robot,pose] = getState(P,a1,a2,start,flat_n); % Global pose
pose_d = pose; % Drawing pose

% Graphics!
addpath(genpath("stlTools"));
figure(1); clf; hold on; axis equal;
xlim([-2*P,(size(n,1)+1)*P]); ylim([-2*P,(size(n,2)+1)*P]); zlim([-P,size(n,3)*P]);
voxel_p = drawVoxels(P,n,ploc(1,:),[]);
[robot_p,robot_v] = drawRobot(P,robot,pose_d,90,[],[]);
load_p = []; load_v = [];
view([45,45]);
cost = 0;
txt = ['Cumulative cost: ' num2str(cost) ''];
txt_p = annotation('textbox',[.65,.35,.25,.5],'String',txt,...
    'FontSize',20,'EdgeColor','none');

% Record video!
record = 0;
if(record)
    vid = VideoWriter('meh.avi');
    vid.FrameRate = 30;
    open(vid);
end

%% Movement
deliveries = 1;
payload = 0;
flag = 0;
curr_v = start; next_v = start;
stage = 0;  anchor = 0;
prev_ths = zeros(7,1);
while(deliveries <= size(dloc,1))
    % Fix feet orientation after pick-ups/drop-offs at an angle
    if(pose(6-4*anchor) ~= pose(7-6*anchor))
        [ths,flag] = motionRule(P,robot,pose,flat_n,curr_v,next_v,stage,anchor);	% Get angles
        steps = size(ths,2)-2;
        th_c = boundedAngle(pose(6-4*anchor)-pose(7-6*anchor));
        turn_c = [0:th_c/steps:th_c,th_c];
        ths(5-4*anchor,:) = -turn_c;
        
        for i = 1:size(ths,2)   %Update robot drawing
            pose_d = updatePoseD(ths(:,i),pose,anchor);
            robot = moveRobot(robot,pose_d,anchor);
            [robot_p,robot_v] = drawRobot(P,robot,pose_d,90,robot_p,robot_v);
            if(payload)
                [load_p,load_v] = drawLoad(P,robot,pose_d,90,load_p,load_v);
            end
            if(record) writeVideo(vid,getframe(gcf)); end
            pause(0.01);
        end
        pose = pose_d;
        ths(1,:) = ths(1,:)+prev_ths(1,:);
        ths(5,:) = ths(5,:)+prev_ths(5,:);
        prev_ths = ths(:,end);
    end
    
    % Get the path for the robot to reach voxel
    if(payload) [pth,~] = pathPlanner(flat_n,curr_v,dloc(deliveries,:));
    else [pth,~] = pathPlanner(flat_n,curr_v,ploc(deliveries,:));
    end    

    % Move the robot to destination
    if(size(pth,1)>1)
        path_p = drawPath(P,flat_n,pth(1:end,:));
        ind = 1;
        while(1)
            if(stage)
                % If the robot rotated +-90 or +-180, back foot can't travel
                if(flag>1) next_v(2-anchor,:) = curr_v(2-anchor,:);
                end
                [ths,flag] = motionRule(P,robot,pose,flat_n,curr_v,next_v,stage,anchor); % Get angles
                ind = ind+1;
            else
                cost = cost+1; delete(txt_p); %Update cost
                txt = ['Cumulative cost: ' num2str(cost) ''];
                txt_p = annotation('textbox',[.65,.35,.25,.5],'String',txt,...
                    'FontSize',20,'EdgeColor','none');
                delete(path_p); path_p = drawPath(P,flat_n,pth(ind:end,:)); % Update path
                
                next_v(anchor+1,:) = curr_v(2-anchor,:);
                next_v(2-anchor,:) = pth(ind,:);
                [ths,flag] = motionRule(P,robot,pose,flat_n,curr_v,next_v,stage,anchor); % Get angles
                % Check if pick-up/drop-off will be at an angle. If the
                % robot rotated +-90 or +-180, special rules apply
                if(ind==(size(pth,1)-1))
                    v1 = atan2d(pth(end,2)-next_v(2,2),pth(end,1)-next_v(2,1));
                    if(flag>1) v2 = atan2d(next_v(2,2)-curr_v(1,2),next_v(2,1)-curr_v(1,1));
                    else v2 = atan2d(next_v(2,2)-next_v(1,2),next_v(2,1)-next_v(1,1));
                    end
                    th_v = boundedAngle(v1-v2);
                    if(th_v ~=0)
                        steps = size(ths,2)-2;
                        turn_v = [0:th_v/steps:th_v,th_v];
                        if(flag<2) ths(5-4*anchor,:) = ths(5-4*anchor,:)-turn_v;
                        else ths(5-4*anchor,:) = -turn_v;
                        end
                    end
                end
            end

            for i = 1:size(ths,2) %Update robot drawing
                pose_d = updatePoseD(ths(:,i),pose,anchor);
                robot = moveRobot(robot,pose_d,anchor);
                [robot_p,robot_v] = drawRobot(P,robot,pose_d,90,robot_p,robot_v);
                if(payload)
                    [load_p,load_v] = drawLoad(P,robot,pose_d,90,load_p,load_v);
                end
                if(record) writeVideo(vid,getframe(gcf)); end
                pause(0.01);
            end
            pose = pose_d;
            ths(1,:) = ths(1,:)+prev_ths(1,:);
            ths(5,:) = ths(5,:)+prev_ths(5,:);
            prev_ths = ths(:,end);
 
            curr_v(2-anchor,:) = next_v(2-anchor,:);
            anchor = ~anchor;   stage = ~stage;
            if(ind == size(pth,1)) break;
            end
        end
    else
        % Check if pick-up/drop-off will be at an angle. Raise front foot
        % in place if that is the case
        v1 = atan2d(pth(end,2)-curr_v(2,2),pth(end,1)-curr_v(2,1));
        v2 = pose(7-6*anchor)+180;
        th_v = boundedAngle(v1-v2);
        if(th_v ~=0)
            [ths,flag] = motionRule(P,robot,pose,flat_n,curr_v,next_v,stage,anchor); % Get angles
            steps = size(ths,2)-2;
            turn_v = [0:th_v/steps:th_v,th_v];
            ths(5-4*anchor,:) = -turn_v;

            for i = 1:size(ths,2) %Update robot drawing
                pose_d = updatePoseD(ths(:,i),pose,anchor);
                robot = moveRobot(robot,pose_d,anchor);
                [robot_p,robot_v] = drawRobot(P,robot,pose_d,90,robot_p,robot_v);
                if(payload)
                    [load_p,load_v] = drawLoad(P,robot,pose_d,90,load_p,load_v);
                end
                if(record) writeVideo(vid,getframe(gcf)); end
                pause(0.01);
            end
            pose = pose_d;
            ths(1,:) = ths(1,:)+prev_ths(1,:);
            ths(5,:) = ths(5,:)+prev_ths(5,:);
            prev_ths = ths(:,end);
        end
    end
    
    if(payload) disp('Destination reached! Placing...');
    else disp('Voxel reached! Grabbing...');
    end

    % Grab/Place voxel
    for i = 1:6
        [robot_p,robot_v] = drawRobot(P,robot,pose_d,90-i*15,robot_p,robot_v);
        if(payload)
            [load_p,load_v] = drawLoad(P,robot,pose_d,90-i*15,load_p,load_v);
        end
        if(record) writeVideo(vid,getframe(gcf)); end
        pause(0.01);
    end
    
    if(payload)
        deliveries = deliveries+1;
        delete(voxel_p); 
        if(deliveries>size(ploc,1)) voxel_p = drawVoxels(P,n,[0,0],[]);
        else voxel_p = drawVoxels(P,n,ploc(deliveries,:),[]);
        end
        if(record) writeVideo(vid,getframe(gcf)); end
        flat_n = sum(n,3);
        payload = 0;
    else
        n(ploc(deliveries,1),ploc(deliveries,2),1) = 0;
        delete(voxel_p); voxel_p = drawVoxels(P,n,[0,0],dloc(deliveries,:));
        n(dloc(deliveries,1),dloc(deliveries,2),1) = 1; 
        [load_p,load_v] = drawLoad(P,robot,pose_d,0,load_p,load_v);
        if(record) writeVideo(vid,getframe(gcf)); end
        flat_n = sum(n,3);
        payload = 1;
    end

    for i = 1:6
        [robot_p,robot_v] = drawRobot(P,robot,pose_d,i*15,robot_p,robot_v); 
        if(payload)
            [load_p,load_v] = drawLoad(P,robot,pose_d,i*15,load_p,load_v);
        end
        if(record) writeVideo(vid,getframe(gcf)); end
        pause(0.01);
    end
end
disp('Done!!');

if(record)
    for i = 1:50
        view([45+.9*i 45+.9*i]);
        writeVideo(vid,getframe(gcf));
        pause(.01);
    end
    close(vid);
end

%% Supporting functions
% Return the state and pose(global) of the robot at starting position
function [robot,pose] = getState(P,a1,a2,start,flat_n)
    robot_s = (cat(2,start,[flat_n(start(1,1),start(1,2));flat_n(start(2,1),start(2,2))])-1)*P;
    robot_a = atan2d(robot_s(2,2)-robot_s(1,2),robot_s(2,1)-robot_s(1,1));
    robot_b = acosd((2*a1^2-norm(robot_s(2,1:2)-robot_s(1,1:2))^2)/(2*a1^2));
    robot_g = atand((robot_s(2,3)-robot_s(1,3))/norm(robot_s(2,1:2)-robot_s(1,1:2)));
    
    pose = [robot_a;(180-robot_b)/2+robot_g;robot_b;(180-robot_b)/2-robot_g;robot_a+180];
    robot = [robot_s(1,:);robot_s(1,1:2),robot_s(1,3)+a2;
        a1*cosd(pose(2))*cosd(pose(1))+robot_s(1,1),a1*cosd(pose(2))*sind(pose(1))+robot_s(1,2),a1*sind(pose(2))+a2+robot_s(1,3);...
        robot_s(2,1:2),robot_s(2,3)+a2;robot_s(2,:)]';
    pose = [pose(1);pose;pose(end)];
end

% Apply set of angles to drawing pose
function pose_d = updatePoseD(ths,pose,anchor)
    pose_d = pose;
    pose_d(3:5) = ths(2:4);
    pose_d(2+4*anchor) = boundedAngle(pose(2+4*anchor)+ths(1+4*anchor));
    pose_d(6-4*anchor) = boundedAngle(pose(6-4*anchor)+ths(1+4*anchor));
    pose_d(7-6*anchor) = boundedAngle(pose(7-6*anchor)+ths(1+4*anchor)-ths(5-4*anchor));
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
function robot = moveRobot(r,p,anchor)
    a1 = norm(r(:,3)-r(:,2));
    a2 = norm(r(:,2)-r(:,1));
        
    robot = r;
    % Second joint
    T1 = makehgtform('zrotate',deg2rad(p(2+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-p(3+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T = T1*T2*T3;
    robot(:,3) = T(1:3,4)+r(:,2+2*anchor);
    % Third joint
    T1 = makehgtform('zrotate',deg2rad(p(2+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-p(3+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T5 = makehgtform('yrotate',deg2rad(180-p(4)));
    T6 = makehgtform('translate',[0 0 a1]);
    T = T1*T2*T3*T5*T6;
    robot(:,4-2*anchor) = T(1:3,4)+r(:,2+2*anchor);
    % Fourth joint
    robot(:,5-4*anchor) = robot(:,4-2*anchor);
    robot(3,5-4*anchor) = robot(3,5-4*anchor)-a2;
end

% Generate required motion based on the state of the robot and the voxels.
% dir indicates the direction the robot is moving in (0 for right and 1 for
% left). stage is a bit more difficult to explain, but it basically
% indicates if the specific foot is moving first or second.
function [ths,flag] = motionRule(P,robot,pose,flat_n,curr_v,next_v,stage,anchor)
    a1 = norm(robot(:,3)-robot(:,2));
    steps = 5;  % Intermediate points in motion parabola (evenly spaced in x)
    ths = zeros(7,steps+2);
    x = xor(anchor,stage);
    tau = 1;
    flag = 0;
    
    % Check if the robot is just fixing the orientation of the feet
    if(norm(curr_v(2-anchor,:)-next_v(2-anchor,:))==0)
        flag = 3;
        dth = boundedAngle(pose(7-6*anchor)-pose(6-4*anchor));
        if(dth ~= 0) ths(5-4*anchor,1:steps+1) = 0:dth/steps:dth;
        end
                    
        par_x = zeros(1,6);
        par_y = [0,1.28,1.92,1.92,1.28,0]+round(robot(3,5-4*anchor));
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
            par_x = tau*((stage*P):(1-2*stage)*P/steps:(P-stage*P));
            par_x = [par_x(1:floor(length(par_x)/2)) tau*P-par_x(floor(length(par_x)/2+1):end)];
        else
            par_x = tau*((stage*P):(1-2*stage)*P/steps:(P-stage*P));
        end
        
        % Different y's are associated with different actions
        action = nextAction(flat_n,stage,anchor,curr_v,next_v);
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
    
    % Use x and y vectors to generate parabolic motions. The effect of dir
    % and stage are included in the angle computations so the same actions
    % can result in different motions
    b = ((par_x+P).^2+par_y.^2).^0.5;
    ths(3,1:steps+1) = acosd((2*a1^2-b.^2)./(2*a1^2));
    dir = (1-2*stage)*(1-2*x);
    ths(2,1:steps+1) = ((180-ths(3,1:steps+1))/2)+dir*atand(par_y./(par_x+P));
    ths(4,1:steps+1) = ((180-ths(3,1:steps+1))/2)-dir*atand(par_y./(par_x+P));
    
    % Unlock foot that is moving
    ths(:,steps+2) = ths(:,steps+1);
    ths(7-anchor,1:steps+1) = 45; ths(7-anchor,steps+2) = 0;
end

% Big brain logic that would take too long to explain allows for mapping
% multiple states to the same action.
function action = nextAction(flat_n,stage,anchor,curr_v,next_v)
    dz1 = flat_n(next_v(2-anchor,1),next_v(2-anchor,2))-flat_n(curr_v(2-anchor,1),curr_v(2-anchor,2));
    dz2 = flat_n(next_v(1+anchor,1),next_v(1+anchor,2))-flat_n(curr_v(1+anchor,1),curr_v(1+anchor,2));
    
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

% Implementation of Dijkstra's algorithm. The voxels are the vertices, and
% the costs of traveling from one voxel to another are the edges. The path
% produced by this function ignores collision and other related issues
function [pth,dist] = pathPlanner(flat_n,start,goal)
    [neighbors, costs] = getGraph(flat_n);
    
    source = (start(2,1)-1)*size(flat_n,2)+start(2,2);
    target = (goal(1)-1)*size(flat_n,2)+goal(2);
    unvisited = 1:1:numel(flat_n);        
    dist = 1000*ones(1,numel(flat_n)); dist(source) = 0;
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
            error('No path available!');
        end
        pth = [prev(u),pth];
        u = prev(u);
    end
    pth = [ceil(pth/size(flat_n,2));pth-(ceil(pth/size(flat_n,2))-1)*size(flat_n,2)]';
    dist = dist(target);
    
    if(size(pth,1)>1)
        if(norm(pth(end-1,:)-start(1,:))==0)
            if((start(2,1)-1)>0 && flat_n(start(2,1)-1,start(2,2))>0 && (start(2,1)-1)~=start(1,1))
                temp_goal = [start(2,1)-1,start(2,2)];
            elseif(start(2,2)<size(flat_n,2) && flat_n(start(2,1),start(2,2)+1)>0 && (start(2,2)+1)~=start(1,2))
                temp_goal = [start(2,1),start(2,2)+1];         
            elseif(start(2,1)<size(flat_n,1) && flat_n(start(2,1)+1,start(2,2))>0 && (start(2,1)+1)~=start(1,1))
                temp_goal = [start(2,1)+1,start(2,2)];          
            elseif((start(2,2)-1)>0 && flat_n(start(2,1),start(2,2)+1)>0 && (start(2,2)-1)~=start(1,2))
                temp_goal = [start(2,1),start(2,2)+1];          
            else
                error('No path available!');
            end
            [pth1,dist1] = pathPlanner(flat_n,start,temp_goal);
            [pth2,dist2] = pathPlanner(flat_n,[start(2,:);temp_goal],goal);
            pth = [pth1;pth2];
            dist = dist1+dist2;
        end
        if(pth(1,1)==start(1,1) && pth(1,2)==start(1,2))
            pth = pth(2:end,:);
        end
    end
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

% Draw voxels
function voxel_p = drawVoxels(P,n,p_trgt,d_trgt)
    voxel_p = [];
    [v_vox,f_vox,~,~] = stlRead('Sim_Voxel.stl'); v_vox = v_vox*P;  % Import voxel model
    for i = 1:size(n,1)
        for j = 1:size(n,2)
            for k = 1:size(n,3)
                if(n(i,j,k))
                    vtemp = [v_vox(:,1)+P*(i-1),v_vox(:,2)+P*(j-1),v_vox(:,3)+P*(k-1)];
                    if(i == p_trgt(1) && j == p_trgt(2))
                        ptemp = patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','b');
                    else
                        ptemp = patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','r');
                    end
                    voxel_p = [voxel_p;ptemp];
                end
            end
        end
    end
    
    if(~isempty(d_trgt))
        i = d_trgt(1); j = d_trgt(2);
        vtemp = [v_vox(:,1)+P*(i-1),v_vox(:,2)+P*(j-1),v_vox(:,3)];
        ptemp = patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','b','LineStyle',':');
        voxel_p = [voxel_p;ptemp];
    end
end

% Draw robot (don't expand this..)
function [robot_p,robot_v] = drawRobot(P,r,p,th_hand,robot_p,robot_v)
    % TOLD YOU NOT TO SEE IT   
    if(isempty(robot_p))
        v_foot1 = [-1/2,0,.1;0,1/2,.1;1/2,0,.1;0,-1/2,.1;-1/2,0,0;0,1/2,0;1/2,0,0;0,-1/2,0;...
            1/6,0,.1;1/3,1/6,.1;1/3,-1/6,.1;1/6,0,0;1/3,1/6,0;1/3,-1/6,0]*P;
        f_foot1 = [1,2,3,4,1;5,6,7,8,5;1,2,6,5,1;1,4,8,5,1;3,2,6,7,3;3,4,8,7,3;...
            9,10,3,11,9;12,13,7,14,12;3,10,13,7,3;3,11,14,7,3];
        c_foot1 = [0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;1,1,1;1,1,1;1,1,1;1,1,1];    
%         [v_foot1,f_foot1,~,~] = stlRead('Sim_RobotFootS.stl');     
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
%         [v_foot2,f_foot2,~,~] = stlRead('Sim_RobotFootS.stl'); 
        [v_hand,f_hand,~,~] = stlRead('Sim_RobotHand.stl');
        robot_v = struct('a',v_foot1,'b',v_leg1,'c',v_joint1,'d',v_leg2,'e',v_joint2,...
            'f',v_leg3,'g',v_joint3,'h',v_leg4,'i',v_foot2,'j',v_hand);
        
        r1 = patch('Faces',f_foot1,'Vertices',robot_v.a,'FaceColor','flat','FaceVertexCData',c_foot1);
        r2 = patch('Faces',f_leg1,'Vertices',robot_v.b,'FaceColor','w');
        r3 = patch('Faces',f_joint,'Vertices',robot_v.c,'FaceColor','k');
        r4 = patch('Faces',f_leg2,'Vertices',robot_v.d,'FaceColor','w');
        r5 = patch('Faces',f_joint2,'Vertices',robot_v.e,'FaceColor','k');
        r6 = patch('Faces',f_leg3,'Vertices',robot_v.f,'FaceColor','w');
        r7 = patch('Faces',f_joint3,'Vertices',robot_v.g,'FaceColor','k');
        r8 = patch('Faces',f_leg4,'Vertices',robot_v.h,'FaceColor','w');
        r9 = patch('Faces',f_foot2,'Vertices',robot_v.i,'FaceColor','flat','FaceVertexCData',c_foot2);
        r10 = patch('Faces',f_hand,'Vertices',robot_v.j,'FaceColor','k');
        robot_p = struct('a',r1,'b',r2,'c',r3,'d',r4,'e',r5,'f',r6,'g',r7,'h',r8,'i',r9,'j',r10);
    end

    a1 = norm(r(:,3)-r(:,2));
    a2 = norm(r(:,2)-r(:,1));
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
    RotH = [cosd(th_hand),0,sind(th_hand);0,1,0;-sind(th_hand),0,cosd(th_hand)];
    
%      = v_foot1*P; v_foot1 = v_foot1*[1,0,0;0,0,-1;0,1,0]';
    va = robot_v.a*RotZ1l';
    va(:,1) = va(:,1)+r(1,1); va(:,2) = va(:,2)+r(2,1); va(:,3) = va(:,3)+r(3,1);
    set(robot_p.a,'Vertices',va); 
         
    vb = (a2/sqrt(2))*[robot_v.b(:,[1,2])/2,robot_v.b(:,3)]; 
    vb = vb*RotZ1l';
    vb(:,1) = vb(:,1)+r(1,1); vb(:,2) = vb(:,2)+r(2,1); vb(:,3) = vb(:,3)+r(3,1);
    set(robot_p.b,'Vertices',vb); 
        
    vc = P*[robot_v.c(:,1)/3,robot_v.c(:,2)/2,robot_v.c(:,3)/3]; vc = vc*RotZ1r';
    vc(:,1) = vc(:,1)+r(1,2); vc(:,2) = vc(:,2)+r(2,2); vc(:,3) = vc(:,3)+r(3,2);
    set(robot_p.c,'Vertices',vc);
    
    vd = a1*[robot_v.d(:,[1,2])/8,robot_v.d(:,3)]; vd = vd*(RotZ1r*RotY1)';   
    vd(:,1) = vd(:,1)+r(1,2); vd(:,2) = vd(:,2)+r(2,2); vd(:,3) = vd(:,3)+r(3,2);
    set(robot_p.d,'Vertices',vd);
    
    ve = P*[robot_v.e(:,1)/3,robot_v.e(:,2)/2,robot_v.e(:,3)/3]; ve = ve*RotZ1r';
    ve(:,1) = ve(:,1)+r(1,3); ve(:,2) = ve(:,2)+r(2,3); ve(:,3) = ve(:,3)+r(3,3);
    set(robot_p.e,'Vertices',ve);
    
    vf = a1*[robot_v.f(:,[1,2])/8,robot_v.f(:,3)]; vf = vf*(RotZ1r*RotY2)';
    vf(:,1) = vf(:,1)+r(1,4); vf(:,2) = vf(:,2)+r(2,4); vf(:,3) = vf(:,3)+r(3,4);
    set(robot_p.f,'Vertices',vf);
    
    vg = P*[robot_v.g(:,1)/3,robot_v.g(:,2)/2,robot_v.g(:,3)/3]; vg = vg*RotZ1r';
    vg(:,1) = vg(:,1)+r(1,4); vg(:,2) = vg(:,2)+r(2,4); vg(:,3) = vg(:,3)+r(3,4);
    set(robot_p.g,'Vertices',vg);
    
    vh = (a2/sqrt(2))*[robot_v.h(:,[1,2])/2,robot_v.h(:,3)]; vh = vh*RotZ2l';
    vh(:,1) = vh(:,1)+r(1,4); vh(:,2) = vh(:,2)+r(2,4); vh(:,3) = vh(:,3)+r(3,5);
    set(robot_p.h,'Vertices',vh);
    
%     v_foot2 = v_foot2*P; v_foot2 = v_foot2*[1,0,0;0,0,-1;0,1,0]'; 
    vi = robot_v.i*RotZ2l';
    vi(:,1) = vi(:,1)+r(1,5); vi(:,2) = vi(:,2)+r(2,5); vi(:,3) = vi(:,3)+r(3,5);
    set(robot_p.i,'Vertices',vi);
    
    vj = P*[robot_v.j(:,1)-.35,robot_v.j(:,2)-.12,robot_v.j(:,3)];
    vj = vj*[1,0,0;0,0,-1;0,1,0]'; vj = vj*RotH; vj = vj*RotZ2h;
    ofs = [r(1,5),r(2,5),r(3,5)]+[.35*P,0,.12*P]*RotZ2h;
    vj(:,1) = vj(:,1)+ofs(1); vj(:,2) = vj(:,2)+ofs(2); vj(:,3) = vj(:,3)+ofs(3);
    set(robot_p.j,'Vertices',vj);
end

% Draw carried voxels
function [load_p,load_v] = drawLoad(P,r,p,th_hand,load_p,load_v)
    if(isempty(load_p))
        [v_load,f_load,~,~] =stlRead('Sim_Voxel.stl'); 
        load_v = v_load;
        load_p = patch('Faces',f_load,'Vertices',v_load,'FaceAlpha',0,'EdgeColor',[0 .75 0]);
    end
    
    th6 = boundedAngle(-p(7)+180);
    RotZ2h = [cosd(th6),-sind(th6),0;sind(th6),cosd(th6),0;0,0,1];
    RotH = [cosd(th_hand),0,sind(th_hand);0,1,0;-sind(th_hand),0,cosd(th_hand)];
    
    vl = P*[load_v(:,1)+.65,load_v(:,2),load_v(:,3)-.12];
    vl = vl*RotH; vl = vl*RotZ2h;   
    ofs = [r(1,5),r(2,5),r(3,5)]+[.35*P,0,.12*P]*RotZ2h;
    vl(:,1) = vl(:,1)+ofs(1); vl(:,2) = vl(:,2)+ofs(2); vl(:,3) = vl(:,3)+ofs(3);
    set(load_p,'Vertices',vl);
end

% Draw path
function path_p = drawPath(P,flat_n,pth)
    x(1) = pth(1,1); y(1) = pth(1,2); z(1) = flat_n(pth(1,1),pth(1,2));
    
    ind1 = 2;   ind2 = 2;
    while(ind1<size(pth,1))
        change = flat_n(pth(ind1,1),pth(ind1,2))-flat_n(pth(ind1-1,1),pth(ind1-1,2));
        if (change<0)
            x(ind2) = pth(ind1,1); y(ind2) = pth(ind1,2); z(ind2) = flat_n(pth(ind1-1,1),pth(ind1-1,2));
            flat_n(pth(ind1-1,1),pth(ind1-1,2)) = flat_n(pth(ind1-1,1),pth(ind1-1,2))-1;
            ind2 = ind2+1;
        elseif (change>0)
            x(ind2) = pth(ind1-1,1); y(ind2) = pth(ind1-1,2); z(ind2) = flat_n(pth(ind1,1),pth(ind1,2));
            flat_n(pth(ind1-1,1),pth(ind1-1,2)) = flat_n(pth(ind1-1,1),pth(ind1-1,2))+1;
            ind2 = ind2+1;
        else
            x(ind2) = pth(ind1,1); y(ind2) = pth(ind1,2); z(ind2) = flat_n(pth(ind1-1,1),pth(ind1-1,2));
            ind2 = ind2+1;
            ind1 = ind1+1;
        end
    end
    
    path_p = plot3((x-1)*P,(y-1)*P,P*(z-1/2),'b','LineWidth',P/2);
end
