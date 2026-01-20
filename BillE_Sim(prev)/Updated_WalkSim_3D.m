% Big boy time now. WalkSim_3D implements a 3D simulator for the BillE bot.
% In addition to extending the motion rules from WalkSim_2D to 3D, a
% Manhattan distance based path planner moves the bot from the origin to a
% set goal. The simulator tries to use a TCP connection to send the angle
% values to the bot's computer
%
% TODO: - Allow robot to start on raised ground (fixed?)
%       - Correctly map rotations to required robot angles (hand must not
%       face diagonally)
%       - Fix planner bugs when x and y dim aren't equal
%
% Javier Garcia, jgarciagonzalez@uh.edu 
% June 08, 2021

%% Initialization
clear; clc;
P = 4;  % Voxel pitch

% Voxel map. Each n is an xy slice, with x=row-1 and y=col-1
n1 = [1,1,1,1,1; 1,1,1,1,1; 1,1,1,1,1; 1,1,1,1,1; 1,1,1,1,1];  
n2 = [0,1,0,0,0; 0,1,0,1,0; 0,1,0,1,0; 0,1,0,1,0; 0,0,0,1,0]; 
n3 = [0,1,0,0,0; 0,1,0,1,0; 0,1,0,1,0; 0,1,0,1,0; 0,0,0,1,0]; 
n4 = [0,1,0,0,0; 0,1,0,1,0; 0,1,0,1,0; 0,1,0,1,0; 0,0,0,1,0]; 
n5 = [0,0,0,0,0; 0,0,0,0,0; 0,0,0,0,0; 0,0,0,0,0; 0,0,0,0,0]; 
n = cat(3,n1,n2,n3,n4,n5);

% %Start Mike's section for loading in matrix from matlab code
% map = load('map.mat', 'adjacency2');
% n = map.adjacency2;

flat_n = sum(n,3);

start = [1,1;2,1];  goal = [5,5];   % Set the starting and goal positions of the robot
a1 = 1.2*P;     % Length of robot legs
a2 = 0.5625*P;     % Height of robot feet 

% Calculate starting robot state and pose
robot_s = (cat(2,start,[flat_n(start(1,1),start(1,2));flat_n(start(2,1),start(2,2))])-1)*P;
robot_a = atan2d(robot_s(2,2)-robot_s(1,2),robot_s(2,1)-robot_s(1,1));
robot_b = acosd((2*a1^2-norm(robot_s(2,1:2)-robot_s(1,1:2))^2)/(2*a1^2));
robot_g = atand((robot_s(2,3)-robot_s(1,3))/norm(robot_s(2,1:2)-robot_s(1,1:2)));
pose = [robot_a;(180-robot_b)/2+robot_g;robot_b;(180-robot_b)/2-robot_g;robot_a+180];
robot = [robot_s(1,:);robot_s(1,1:2),robot_s(1,3)+a2;
    a1*cosd(pose(2))*cosd(pose(1))+robot_s(1,1),a1*cosd(pose(2))*sind(pose(1))+robot_s(1,2),a1*sind(pose(2))+a2+robot_s(1,3);...
    robot_s(2,1:2),robot_s(2,3)+a2;robot_s(2,:)]';

% Get the path for the robot to follow
[pth1,dist1] = pathPlanner(flat_n,start(1,:),goal); 
[pth2,dist2] = pathPlanner(flat_n,start(2,:),goal);
if(dist1<dist2)
    pth = pth1;
    anchor = 1;
else
    pth = pth2;
    anchor = 0;
end
pth = [ceil(pth'/size(n,2)),pth'-size(n,1)*(ceil(pth'/size(n,2))-1)];

% Open socket to communicate with raspberrypi
comms = 1;
try
    pi = tcpclient('192.168.137.42',42069,"ConnectTimeout",10,"Timeout",60);
catch
    comms = 0;
end

% Graphics!
figure(1); clf; hold on; axis equal;
addpath(genpath("stlTools"));
drawVoxels(P,n);
path_p = drawPath(P,flat_n,pth);
robot_p = drawRobot(P,robot,pose);
r = plot3(robot(1,:),robot(2,:),robot(3,:),'-ob','MarkerFaceColor','b'); 
r.XDataSource = 'robot(1,:)'; r.YDataSource = 'robot(2,:)'; r.ZDataSource = 'robot(3,:)';
view([45,45]);

%% Movement loop
if comms
    mode = 1;
    write(pi,mode,"uint8");
end
stage = 0;
curr_v = start; next_v = start;
ind = 1; mat = 1;
while(norm(curr_v(1,:)-goal)>0 && norm(curr_v(2,:)-goal)>0)
    if (stage)
        [action,ths] = motionRule(P,robot,flat_n,curr_v,next_v,stage,anchor);	% Get angles
        for i = 1:size(ths,2)   % Update robot drawing
            pose(2:4) = ths(2:4,i);
            robot = moveRobot(robot,pose,ths(:,i),anchor);
            delete(robot_p); robot_p = drawRobot(P,robot,pose); 
            pause(.2); refreshdata; drawnow;
        end
        if comms
            switch billEComm(pi,action,stage,anchor)
                case 1
                    disp("Motion complete");
                case 2
                    error("Motion failed");
                otherwise
                    error("Unexpected result");
            end                
        end
        ind = ind+1;
        curr_v = next_v;
    else
        delete(path_p); path_p = drawPath(P,flat_n,pth(ind:end,:));
        next_v(anchor+1,:) = curr_v(2-anchor,:);
        next_v(2-anchor,:) = pth(ind,:);
        [action,ths] = motionRule(P,robot,flat_n,curr_v,next_v,stage,anchor);  % Get angles
        for i = 1:size(ths,2)   %Update robot drawing
            pose(2:4) = ths(2:4,i);
            robot = moveRobot(robot,pose,ths(:,i),anchor);
            delete(robot_p); robot_p = drawRobot(P,robot,pose);
            pause(.2); refreshdata; drawnow;
        end
        if comms
            switch billEComm(pi,action,stage,anchor)
                case 1
                    disp("Motion complete");
                case 2
                    error("Motion failed");
                otherwise
                    error("Unexpected result");
            end
        end
    end
    pose([1,5]) = pose([1,5])+ths(1+4*anchor,end);
    thetas(:,:,mat) = ths; mat = mat+1;
    anchor = ~anchor;   stage = ~stage;
    if comms
        cont = 1;
        write(pi,cont,"uint8");
    end
end 
disp('Destination reached!');

%% Supporting functions
% This function applies rotations to the joints based on the motion rule
% and the state of the robot. The joint order is with respect to the
% anchor, not the global joint number.
function robot = moveRobot(r,p,ths,anchor)
    a1 = norm(r(:,3)-r(:,2));
    a2 = norm(r(:,2)-r(:,1));
        
    robot = r;
    % Second joint
    T1 = makehgtform('zrotate',deg2rad(p(1+4*anchor)+ths(1+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-ths(2+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T = T1*T2*T3;
    robot(:,3) = T(1:3,4)+r(:,2+2*anchor);
    % Third joint
    T1 = makehgtform('zrotate',deg2rad(p(1+4*anchor)+ths(1+4*anchor)));
    T2 = makehgtform('yrotate',deg2rad(90-ths(2+2*anchor)));
    T3 = makehgtform('translate',[0 0 a1]);
    T5 = makehgtform('yrotate',deg2rad(180-ths(3)));
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
function [action,ths] = motionRule(P,robot,flat_n,curr_v,next_v,stage,anchor)
    a1 = norm(robot(:,3)-robot(:,2));
    steps = 5;  % Intermediate points in motion parabola (evenly spaced in x)
    ths = zeros(7,steps+2);
    tau = 1;
    
    % If the robot is turning, compute required rotation angles for the
    % first and fifth joint
    x = xor(anchor,stage);
    v1 = [next_v(~x+1,:)-curr_v(x+1,:) 0];
    v2 = [next_v(x+1,:)-curr_v(x+1,:) 0];
    turn = asind(sum(cross(v2,v1))/(norm(v1)*norm(v2)));
    if(turn)
        ths(1+4*anchor,1:steps+1) = 0:turn/steps:turn;
        ths(5-4*anchor,2:steps+2) = -turn:turn/steps:0;
        tau = sqrt(2)-1;
    end
    
    % Different y's are associated with different actions 
    par_x = tau*((stage*P):(1-2*stage)*P/steps:(P-stage*P));
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
            par_y = (-4*par_x.^2/P+4*beta*par_x);
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
    
    % Use x and y vectors to generate parabolic motions. The effect of dir
    % and stage are included in the angle computations so the same actions
    % can result in different motions
    b = ((par_x+P).^2+par_y.^2).^0.5;
    ths(3,1:steps+1) = acosd((2*a1^2-b.^2)./(2*a1^2));
    dir = (1-2*stage)*(1-2*x);
    ths(2,1:steps+1) = ((180-ths(3,1:steps+1))/2)+dir*atand(par_y./(par_x+P));
    ths(4,1:steps+1) = ((180-ths(3,1:steps+1))/2)-dir*atand(par_y./(par_x+P));
    
    alpha = acosd(((1+stage)^2+.5^2)/(2*1.2*sqrt((1+stage)^2+.5^2)))+atand(.5/(1+stage));
    beta = acosd(((1+stage)^2+.5^2)/(2*1.2*sqrt((1+stage)^2+.5^2)))-atand(.5/(1+stage));
    gamma = 180-2*acosd(((1+stage)^2+.5^2)/(2*1.2*sqrt((1+stage)^2+.5^2)));
    ths(2+2*anchor,2) = alpha; 
    ths(3,2) = gamma; 
    ths(4-2*anchor,2) = beta; 
    
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
    
    source = (start(1)-1)*size(flat_n,1)+start(2);
    target = (goal(1)-1)*size(flat_n,1)+goal(2);
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
    dist = dist(target);
end

% Transform voxel map to a graph Dijkstra's algorithm can use
function [verts,edges] = getGraph(n)
    verts = []; edges = [];
    k = 1;
    for i = 1:size(n,1)
        for j = 1:size(n,2)
            if(n(i,j)>0)
                if((j-1)>0 && n(i,j-1)>0)
                    verts(1,k) = (i-1)*size(n,1)+j;
                    verts(2,k) = (i-1)*size(n,1)+(j-1);
                    edges(k) = factorial(abs(n(i,j)-n(i,j-1)));
                    k = k+1;
                end
                if((i+1)<=size(n,2) && n(i+1,j)>0)
                    verts(1,k) = (i-1)*size(n,1)+j;
                    verts(2,k) = i*size(n,1)+j;
                    edges(k) = factorial(abs(n(i,j)-n(i+1,j)));
                    k = k+1;
                end
            end
        end
    end  
end

function flag = billEComm(pi,action,stage,anchor)
    switch action
        case "stepup_two"
            ID = 9;
        case "stepup_one"
            ID = 6;
        case "stepup_over"
            ID = 4;
        case "stepup_flat"
            ID = 2;
        case "flat"
            ID = 0;
        case "stepdown_flat"
            ID = 1;
        case "stepdown_over"
            ID = 3;
        case "stepdown_one"
            ID = 5;
        case "stepdown_two"
            ID = 7;
        otherwise 
            ID = 0;
    end
    
    disp([ID stage anchor]);
    write(pi,[ID stage anchor],"uint8");
    try
        flag = read(pi,1,"uint8");
    catch
        error("Lost communication with BillE");
    end
end

% Draw robot (don't expand this..)
function robot_p = drawRobot(P,robot,pose)
    % TOLD YOU NOT TO SEE IT
    a1 = norm(robot(:,3)-robot(:,2));
    a2 = norm(robot(:,2)-robot(:,1));
    th1 = 90-pose(2);
    th2 = pose(4)-90;
    th3 = atan2d(robot(2,4)-robot(2,2),robot(1,4)-robot(1,2));
    RotY1 = [cosd(th1),0,sind(th1);0,1,0;-sind(th1),0,cosd(th1)];
    RotY2 = [cosd(th2),0,sind(th2);0,1,0;-sind(th2),0,cosd(th2)];
    RotZ = [cosd(th3),-sind(th3),0;sind(th3),cosd(th3),0;0,0,1];
    RotH = [0,0,1;0,1,0;-1,0,0];
    
    [v_foot1,f_foot1,~,~] = stlRead('Sim_RobotFootS.stl'); 
    v_foot1 = v_foot1*P; v_foot1 = v_foot1*[1,0,0;0,0,-1;0,1,0]'; 
    v_foot1(:,1) = v_foot1(:,1)+robot(1,1); v_foot1(:,2) = v_foot1(:,2)+robot(2,1); v_foot1(:,3) = v_foot1(:,3)+robot(3,1);
    
    [v_leg1,f_leg1,~,~] = stlRead('Sim_RobotLeg.stl'); 
    v_leg1 = (a2/sqrt(2))*[v_leg1(:,[1,2])/2,v_leg1(:,3)]; v_leg1 = v_leg1*RotZ';
    v_leg1(:,1) = v_leg1(:,1)+robot(1,1); v_leg1(:,2) = v_leg1(:,2)+robot(2,1); v_leg1(:,3) = v_leg1(:,3)+robot(3,1);
    
    [v_joint1,f_joint,~,~] = stlRead('Sim_RobotJoint.stl'); 
    v_joint1 = P*[v_joint1(:,1)/3,v_joint1(:,2)/2,v_joint1(:,3)/3]; v_joint1 = v_joint1*RotZ';
    v_joint1(:,1) = v_joint1(:,1)+robot(1,2); v_joint1(:,2) = v_joint1(:,2)+robot(2,2); v_joint1(:,3) = v_joint1(:,3)+robot(3,2);
    
    [v_leg2,f_leg2,~,~] = stlRead('Sim_RobotLeg.stl'); 
    v_leg2 = a1*[v_leg2(:,[1,2])/8,v_leg2(:,3)]; v_leg2 = v_leg2*(RotZ*RotY1)';   
    v_leg2(:,1) = v_leg2(:,1)+robot(1,2); v_leg2(:,2) = v_leg2(:,2)+robot(2,2); v_leg2(:,3) = v_leg2(:,3)+robot(3,2);
    
    [v_joint2,f_joint2,~,~] = stlRead('Sim_RobotJoint.stl'); 
    v_joint2 = P*[v_joint2(:,1)/3,v_joint2(:,2)/2,v_joint2(:,3)/3]; v_joint2 = v_joint2*RotZ';
    v_joint2(:,1) = v_joint2(:,1)+robot(1,3); v_joint2(:,2) = v_joint2(:,2)+robot(2,3); v_joint2(:,3) = v_joint2(:,3)+robot(3,3);
    
    [v_leg3,f_leg3,~,~] = stlRead('Sim_RobotLeg.stl'); 
    v_leg3 = a1*[v_leg3(:,[1,2])/8,v_leg3(:,3)]; v_leg3 = v_leg3*(RotZ*RotY2)';
    v_leg3(:,1) = v_leg3(:,1)+robot(1,4); v_leg3(:,2) = v_leg3(:,2)+robot(2,4); v_leg3(:,3) = v_leg3(:,3)+robot(3,4);
    
    [v_joint3,f_joint3,~,~] = stlRead('Sim_RobotJoint.stl');
    v_joint3 = P*[v_joint3(:,1)/3,v_joint3(:,2)/2,v_joint3(:,3)/3]; v_joint3 = v_joint3*RotZ';
    v_joint3(:,1) = v_joint3(:,1)+robot(1,4); v_joint3(:,2) = v_joint3(:,2)+robot(2,4); v_joint3(:,3) = v_joint3(:,3)+robot(3,4);
    
    [v_leg4,f_leg4,~,~] = stlRead('Sim_RobotLeg.stl'); 
    v_leg4 = (a2/sqrt(2))*[v_leg4(:,[1,2])/2,v_leg4(:,3)]; v_leg4 = v_leg4*RotZ'; 
    v_leg4(:,1) = v_leg4(:,1)+robot(1,4); v_leg4(:,2) = v_leg4(:,2)+robot(2,4); v_leg4(:,3) = v_leg4(:,3)+robot(3,5);
    
    [v_foot2,f_foot2,~,~] = stlRead('Sim_RobotFootS.stl'); 
    v_foot2 = v_foot2*P; v_foot2 = v_foot2*[1,0,0;0,0,-1;0,1,0]'; 
    v_foot2(:,1) = v_foot2(:,1)+robot(1,5); v_foot2(:,2) = v_foot2(:,2)+robot(2,5); v_foot2(:,3) = v_foot2(:,3)+robot(3,5);
    
    [v_hand,f_hand,~,~] = stlRead('Sim_RobotHand.stl'); 
    v_hand(:,1) = v_hand(:,1)-.35; v_hand(:,2) = v_hand(:,2)-.12; v_hand = v_hand*P; 
    v_hand = v_hand*[1,0,0;0,0,-1;0,1,0]'; v_hand = v_hand*RotH; v_hand = v_hand*RotZ';
    ofs = [robot(1,5),robot(2,5),robot(3,5)]+[.35*P,0,.12*P]*RotZ';
    v_hand(:,1) = v_hand(:,1)+ofs(1); v_hand(:,2) = v_hand(:,2)+ofs(2); v_hand(:,3) = v_hand(:,3)+ofs(3);
    
    r1 = patch('Faces',f_foot1,'Vertices',v_foot1,'FaceColor','k');
    r2 = patch('Faces',f_leg1,'Vertices',v_leg1,'FaceColor','w');
    r3 = patch('Faces',f_joint,'Vertices',v_joint1,'FaceColor','k');
    r4 = patch('Faces',f_leg2,'Vertices',v_leg2,'FaceColor','w');
    r5 = patch('Faces',f_joint2,'Vertices',v_joint2,'FaceColor','k');
    r6 = patch('Faces',f_leg3,'Vertices',v_leg3,'FaceColor','w');
    r7 = patch('Faces',f_joint3,'Vertices',v_joint3,'FaceColor','k');
    r8 = patch('Faces',f_leg4,'Vertices',v_leg4,'FaceColor','w');
    r9 = patch('Faces',f_foot2,'Vertices',v_foot2,'FaceColor','k');
    r10 = patch('Faces',f_hand,'Vertices',v_hand,'FaceColor','k');
    robot_p = [r1;r2;r3;r4;r5;r6;r7;r8;r9;r10];
end

% Draw voxels
function drawVoxels(P,n)
    [v_vox,f_vox,~,~] = stlRead('Sim_Voxel.stl'); v_vox = v_vox*P;  % Import voxel model
    for i = 1:size(n,1)
        for j = 1:size(n,2)
            for k = 1:size(n,3)
                if(n(i,j,k))
                    vtemp = [v_vox(:,1)+P*(i-1),v_vox(:,2)+P*(j-1),v_vox(:,3)+P*(k-1)];
                    fprintf('%i',k)
                    switch k
                        case 1
                            patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor',[1 0 0]);
                        case 2
                            patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor',[1 .33 0]);
                        case 3
                            patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor',[1 .66 0]);
                        case 4
                            patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor',[1 1 0]);
                        %patch('Faces',f_vox,'Vertices',vtemp,'FaceAlpha',0,'EdgeColor','r');
                    end
                end
            end
        end
    end
end

% Draw path
function path_p = drawPath(P,flat_n,pth)
    x(1) = pth(1,1); y(1) = pth(1,2); z(1) = flat_n(pth(1,1),pth(1,2));
    
    ind1 = 2;   ind2 = 2;
    while(ind1<size(pth,1)+1)
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
