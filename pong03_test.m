%+-------------------------------------------------------+%
%|                DAVE'S MATLAB PONG v0.3                |%
%|                  by David Buckingham                  |%
%|                                                       |%
%| a fast-paced two-player game inspired by Atari's Pong |%
%+-------------------------------------------------------+%


%------------------Added functions-----------------------------
% k2 = Kin2('depth') : initialize kinect depth camera
% floorDetection(k2) : detect the floor to correct the camera tilt on the images
% pointTracker(k2) : this function allows to use body movements to move the
%                     paddles using the kinect depth camera
% computeMeanShift : compute mean shift for 2D points
% get3DPoints : compute the 3D point coordinates in millimeters
% getCameraParameters : compute the focal lengths and principal point of a camera
% setGameArea: set the game area using the image of the area illuminated by the projector
% colorDepthMapping : determine the point correspondences in the depth image
%                     from points selected in the color image
% minFunMapping : objective function to minimize the error of mapping points
%                 between the color and depth images
% floorPoints : get the coordinates of points on the floor from pixel locations
% scalePlayers : get a scale factor and an offset for the detected positions
%                of the players to map them to the plot units used on the game
% captureDepthImage : capture an image from the kinect depth camera
% runme : run the game
%
%------------------------------------------------------------

% this is a test that uses captured images from the kinect
% to run type:
% p = pong03_test;
% p.runme

classdef pong03_test < handle
    properties(Constant)
        %field of view of the depth camera in degrees
        FOV_DEPTH_CAMERA_HORZ = 70.6;
        FOV_DEPTH_CAMERA_VERT = 60.0;
        %field of view of the color camera in degrees
        FOV_COLOR_CAMERA_HORZ = 84.1;
        FOV_COLOR_CAMERA_VERT = 53.8;
        %rotation and translation from the depth to the color camera systems
        ROTATION_DEPTH_TO_COLOR = [1 0 0; 0 1 0; 0 0 1];
        TRANSLATION_DEPTH_TO_COLOR = [-50; 0; 0];  %millimeters
        
        %game settings
        MAX_POINTS = 3;
        START_DELAY = 1;
        
        %movement
        FRAME_DELAY = .01; %animation frame duration in seconds, .01 is good.
        MIN_BALL_SPEED = 2; %each round ball starts at this speed
        MAX_BALL_SPEED = 15; %wont accelerate bast this, dont set too high or bugs.
        BALL_ACCELERATION = 0.2; %how much ball accelerates each bounce.
        PADDLE_SPEED = 2;
        PADDLE_MOTION_THRESHOLD = 5;  %minimum motion of a player to move the paddle
        B_FACTOR = 1;
        P_FACTOR = 2;
        Y_FACTOR = 0.01;
        GOAL_BUFFER = 5;
        
        %layout/structure
        BALL_RADIUS = 1.5; %radius to calculate bouncing
        WALL_WIDTH = 3;
        FIGURE_WIDTH = 1920; %pixels
        FIGURE_HEIGHT = 1080;
        PLOT_W = 150; %width in plot units. this will be main units for program
        PLOT_H = 100; %height
        GOAL_SIZE = 50;
        GOAL_TOP = (pong03.PLOT_H + pong03.GOAL_SIZE)/2;
        GOAL_BOT = (pong03.PLOT_H - pong03.GOAL_SIZE)/2;
        PADDLE_H = 18; %height
        PADDLE_W = 3; %width
        PADDLE = [0 pong03.PADDLE_W pong03.PADDLE_W 0 0; ...
            pong03.PADDLE_H pong03.PADDLE_H 0 0 pong03.PADDLE_H];
        PADDLE_SPACE = 10; %space between paddle and goal
        
        %appearance
        FIGURE_COLOR = [0, 0, 0]; %program background
        AXIS_COLOR = [.15, .15, .15]; %the court
        CENTER_RADIUS = 15; %radius of circle in center of court.
        BALL_MARKER_SIZE = 10; %aesthetic, does not affect physics, see BALL_RADIUS
        BALL_COLOR = [.1, .7, .1];
        BALL_OUTLINE = [.7, 1, .7];
        BALL_SHAPE = 'o';
        PADDLE_LINE_WIDTH = 2;
        WALL_COLOR = [.3, .3, .8]; %format string for drawing walls
        PADDLE_COLOR = [1, .5, 0];
        CENTERLINE_COLOR = pong03.PADDLE_COLOR .* .8; %format string for centerline
        PAUSE_BACKGROUND_COLOR = pong03.FIGURE_COLOR;
        PAUSE_TEXT_COLOR = [.9, .9, .9];
        PAUSE_EDGE_COLOR = pong03.BALL_COLOR;
        TITLE_COLOR = 'w';
        
        %messages
        PAUSE_WIDTH = 36; %min pause message width, DO NOT MODIFY, KEEP AT 36
        MESSAGE_X = 38; %location of message displays. 38, 55 is pretty centered
        MESSAGE_Y = 55;
        MESSAGE_PAUSED = ['             GAME PAUSED' 10 10];
        MESSAGE_INTRO = [...
            '             welcome to ' 10 10 ...
            '         DAVE' 39 'S MATLAB PONG' 10 10 ...
            '     first to get ' num2str(pong03.MAX_POINTS) ' points wins!' 10 10 ...
            '    player 1:           player 2:' 10 ...
            ' use (a) and (z)     use arrow keys' 10 10 ...
            ];
        MESSAGE_CONTROLS = '  pause:(p)   reset:(r)   quit:(q)';
    end
    properties
        fig = []; %main program obj.figure
        quitGame = false; %guard for main loop. when true, program ends
        paused = []; %true if game is paused
        score = []; %1x2 vector holding player scores
        winner = []; %normally 0. 1 if player1 wins, 2 if player2 wins
        ballPlot = []; %main plot, includes ball and walls
        paddle1Plot = []; %plot for paddle
        paddle2Plot = [];
        ballVector=[]; %normalized vector for ball movement
        ballSpeed=[];
        ballX = []; %ball location
        ballY = [];
        paddle1V = []; %holds either 0, -1, or 1 for paddle movement
        paddle2V = [];
        paddle1 = []; %2x5 matrix describing paddle, based on PADDLE
        paddle2 = [];
        rotation_matrix = [];  %rotation matrix to correct the floor slope
        floor_vector = [];  %unitary vector normal to the floor
        floor_depth = [];  %closest distance between the depth camera and the floor
        depth_max = [];   %maximal depth to detect only objects above the floor
        %scale factor and offset to convert the detected positions of the
        %players in millimeters to the plot unit used in the game
        scale_players = [];
    end
    methods
        %===============================================================
        %constructor
        function obj = pong03_test
            
        end
        
        %==============================================================
        %sets up main program obj.figure
        %plots ball, walls, paddles
        %called once at start of program
        function createFigure(obj)
            %ScreenSize is a four-element vector: [left, bottom, width,%height]
            scrsz = get(0,'ScreenSize');
            obj.fig = figure('Position',[(scrsz(3)-obj.FIGURE_WIDTH)/2 ...
                (scrsz(4)-obj.FIGURE_HEIGHT)/2 obj.FIGURE_WIDTH, obj.FIGURE_HEIGHT]);
            %register keydown and keyup listeners
            set(obj.fig,'KeyPressFcn',@obj.keyDown, 'KeyReleaseFcn', @obj.keyUp);
             %obj.figure can't be resized
             set(obj.fig, 'Resize', 'off');
             axis([0 obj.PLOT_W 0 obj.PLOT_H]);
             axis manual;
             %set color for the court, hide axis ticks.
             set(gca, 'color', obj.AXIS_COLOR, 'YTick', [], 'XTick', []);
             %set background color for obj.figure
             set(obj.fig, 'color', obj.FIGURE_COLOR);
             hold on;
             %plot walls
             topWallXs = [0,0,obj.PLOT_W,obj.PLOT_W];
             topWallYs = [obj.GOAL_TOP,obj.PLOT_H,obj.PLOT_H,obj.GOAL_TOP];
             bottomWallXs = [0,0,obj.PLOT_W,obj.PLOT_W];
             bottomWallYs = [obj.GOAL_BOT,0,0,obj.GOAL_BOT];
             plot(topWallXs, topWallYs, '-', 'LineWidth', obj.WALL_WIDTH, 'Color', obj.WALL_COLOR);
             plot(bottomWallXs, bottomWallYs, '-', 'LineWidth', obj.WALL_WIDTH, 'Color', obj.WALL_COLOR);
             %calculate circle to draw on court
             thetas = linspace(0, (2*pi), 100);
             circleXs = (obj.CENTER_RADIUS .* cos(thetas)) + (obj.PLOT_W / 2);
             circleYs = (obj.CENTER_RADIUS .* sin(thetas))+ (obj.PLOT_H / 2);
             %draw lines on court
             centerline = plot([obj.PLOT_W/2, obj.PLOT_W/2],[obj.PLOT_H, 0],'--');
             set(centerline, 'Color', obj.CENTERLINE_COLOR);
             centerCircle = plot(circleXs, circleYs,'--');
             set(centerCircle, 'Color', obj.CENTERLINE_COLOR);
             %plot ball, we'll set ball location in refreshPlot
             obj.ballPlot = plot(0,0);
             set(obj.ballPlot, 'Marker', obj.BALL_SHAPE);
             set(obj.ballPlot, 'MarkerEdgeColor', obj.BALL_OUTLINE);
             set(obj.ballPlot, 'MarkerFaceColor', obj.BALL_COLOR);
             set(obj.ballPlot, 'MarkerSize', obj.BALL_MARKER_SIZE);
             %plot paddles, we'll set paddle locations in refreshPlot
             obj.paddle1Plot = plot(0,0, '-', 'LineWidth', obj.PADDLE_LINE_WIDTH);
             obj.paddle2Plot = plot(0,0, '-', 'LineWidth', obj.PADDLE_LINE_WIDTH);
             set(obj.paddle1Plot, 'Color', obj.PADDLE_COLOR);
             set(obj.paddle2Plot, 'Color', obj.PADDLE_COLOR);
        end
        
        %=============================================================
        %resets game to starting conditions called from main loop at program start
        %called from keydown when user hits 'r', called from checkGoal after someone wins
        %sets some variables, calls reset game, and calls pauseGame with intro message
        function newGame(obj)
            obj.winner = 0;
            obj.score = [0, 0];
            obj.paddle1V = 0; %velocity
            obj.paddle2V = 0; %velocity
            obj.paddle1 = [obj.PADDLE(1,:)+obj.PADDLE_SPACE; obj.PADDLE(2,:)+((obj.PLOT_H - obj.PADDLE_H)/2)];
            obj.paddle2 = [obj.PADDLE(1,:)+ obj.PLOT_W - obj.PADDLE_SPACE - obj.PADDLE_W; obj.PADDLE(2,:)+((obj.PLOT_H - obj.PADDLE_H)/2)];
            obj.resetGame;
            if ~obj.quitGame %incase we try to quit from winner message
                obj.pauseGame([obj.MESSAGE_INTRO, obj.MESSAGE_CONTROLS]);
            end
        end
        
        %=================================================================
        %resets ball location speed and direction, resets title string to display scores
        %called from newGame, called from checkGoal after each goal
        function resetGame(obj)
            obj.bounce([1-(2*rand), 1-(2*rand)]);
            obj.ballSpeed = obj.MIN_BALL_SPEED;
            obj.ballX = obj.PLOT_W/2;
            obj.ballY = obj.PLOT_H/2;
            %here 19 is the space between the scores
            titleStr = sprintf('%d / %d%19d / %d', obj.score(1), obj.MAX_POINTS, obj.score(2), obj.MAX_POINTS);
            t = title(titleStr, 'Color', obj.TITLE_COLOR);
            set(t, 'FontName', 'Courier','FontSize', 15, 'FontWeight', 'Bold');
            obj.refreshPlot;
            if ~obj.quitGame; %make sure we don't wait to quit if use hit 'q'
                pause(obj.START_DELAY);
            end
        end
        
        %================================================================
        %calculates new ball location
        %checks if it will hit any walls or paddles, f it does, call bounce to change ball vector
        %move ball to new location, called from main loop on every frame
        function moveBall(obj)
            %paddle boundaries, useful for hit testing ball
            p1T = obj.paddle1(2,1);
            p1B = obj.paddle1(2,3);
            p1L = obj.paddle1(1,1);
            p1R = obj.paddle1(1,2);
            p1Center = ([p1L p1B] + [p1R p1T])./2;
            p2T = obj.paddle2(2,1);
            p2B = obj.paddle2(2,3);
            p2L = obj.paddle2(1,1);
            p2R = obj.paddle2(1,2);
            p2Center = ([p2L p2B] + [p2R p2T])./2;
            %while hit calculate new vectors until we know it wont hit something
            %temporary new ball location, only apply if ball doesn't hit anything
            newX = obj.ballX + 0.5*(obj.ballSpeed * obj.ballVector(1));
            newY = obj.ballY + 0.5*(obj.ballSpeed * obj.ballVector(2));
            %hit test right wall
            if (newX > (obj.PLOT_W - obj.BALL_RADIUS) && (obj.ballY<obj.GOAL_BOT+obj.BALL_RADIUS || newY>obj.GOAL_TOP-obj.BALL_RADIUS))
                %hit right wall
                if (newY > obj.GOAL_BOT && newY < obj.GOAL_TOP - obj.BALL_RADIUS)
                    %hit bottom goal edge
                    obj.bounce([newX - obj.PLOT_W, newY - obj.GOAL_BOT]);
                    elseif (newY < obj.GOAL_TOP && newY > obj.GOAL_BOT + obj.BALL_RADIUS)
                        %hit top goal edge
                        obj.bounce([newX - obj.PLOT_W, newY - obj.GOAL_TOP]);
                else
                    %hit flat part of right wall
                    obj.bounce([-1 * abs(obj.ballVector(1)), obj.ballVector(2)]);
                end
                %hit test left wall
            elseif(newX < obj.BALL_RADIUS && (newY<obj.GOAL_BOT+obj.BALL_RADIUS || newY>obj.GOAL_TOP-obj.BALL_RADIUS))
                %hit left wall
                if (newY > obj.GOAL_BOT && newY < obj.GOAL_TOP - obj.BALL_RADIUS)
                    %hit bottom goal edge
                    obj.bounce([newX, newY - obj.GOAL_BOT]);
                elseif(newY < obj.GOAL_TOP && newY > obj.GOAL_BOT + obj.BALL_RADIUS)
                    %hit top goal edge
                    obj.bounce([newX, newY - obj.GOAL_TOP]);
                else
                    obj.bounce([abs(obj.ballVector(1)), obj.ballVector(2)]);
                end
                %hit test top wall
            elseif(newY > (obj.PLOT_H - obj.BALL_RADIUS))
                %hit top wall
                obj.bounce([obj.ballVector(1), -1 * (obj.Y_FACTOR + abs(obj.ballVector(2)))]);
                %hit test bottom wall
            elseif(newY < obj.BALL_RADIUS)
                %hit bottom wall,
                obj.bounce([obj.ballVector(1), (obj.Y_FACTOR + abs(obj.ballVector(2)))]);
                %hit test paddle 1
            elseif(newX < p1R + obj.BALL_RADIUS && newX > p1L - obj.BALL_RADIUS ...
                    && newY < p1T + obj.BALL_RADIUS && newY > p1B - obj.BALL_RADIUS)
                obj.bounce([(obj.ballX-p1Center(1)) * obj.P_FACTOR, newY-p1Center(2)]);
                %hit test paddle 2
            elseif(newX < p2R + obj.BALL_RADIUS && newX > p2L - obj.BALL_RADIUS ...
                    && newY < p2T + obj.BALL_RADIUS && newY > p2B - obj.BALL_RADIUS)
                obj.bounce([(obj.ballX-p2Center(1)) * obj.P_FACTOR, newY-p2Center(2)]);
            else
                %no hits
            end
            %move ball to new location
            obj.ballX = newX;
            obj.ballY = newY;
        end
        
        %==============================================================
        %uses paddle velocity set paddles called from main loop on every frame
        function movePaddles(obj)
            %set new paddle y locations
            obj.paddle1(2,:) = obj.paddle1(2,:) + (obj.PADDLE_SPEED * obj.paddle1V);
            obj.paddle2(2,:) = obj.paddle2(2,:) + (obj.PADDLE_SPEED * obj.paddle2V);
            %if paddle out of bounds, move it in bounds
            if obj.paddle1(2,1) > obj.PLOT_H
                obj.paddle1(2,:) = obj.PADDLE(2,:) + obj.PLOT_H - obj.PADDLE_H;
            elseif obj.paddle1(2,3) < 0
                obj.paddle1(2,:) = obj.PADDLE(2,:);
            end
            if obj.paddle2(2,1) > obj.PLOT_H
                obj.paddle2(2,:) = obj.PADDLE(2,:) + obj.PLOT_H - obj.PADDLE_H;
            elseif obj.paddle2(2,3) < 0
                obj.paddle2(2,:) = obj.PADDLE(2,:);
            end
        end
        
        %===============================================================
        %sets data in plots
        %calls matlab's drawnow to refresh plots, uses matlab pause to create
        %animation frame called from main loop on every frame
        function refreshPlot(obj)
            set(obj.ballPlot, 'XData', obj.ballX, 'YData', obj.ballY);
            set(obj.paddle1Plot, 'Xdata', obj.paddle1(1,:), 'YData', obj.paddle1(2,:));
            set(obj.paddle2Plot, 'Xdata', obj.paddle2(1,:), 'YData', obj.paddle2(2,:));
            drawnow;
            pause(obj.FRAME_DELAY);
        end
        
        %================================================================
        %check ballX to see if ball passed through goal
        %update score and see if anybody won
        %call resetGame to reset ball location etc.
        %if somebody won, then call pauseGame to display message, call newGame
        %called from main loop on every frame
        function checkGoal(obj)
            goal = false;
            if obj.ballX > obj.PLOT_W + obj.BALL_RADIUS + obj.GOAL_BUFFER
                obj.score(1) = obj.score(1) + 1;
                if obj.score(1) == obj.MAX_POINTS;
                    obj.winner = 1;
                end
                goal = true;
            elseif obj.ballX < 0 - obj.BALL_RADIUS - obj.GOAL_BUFFER
                obj.score(2) = obj.score(2) + 1;
                if obj.score(2) == obj.MAX_POINTS;
                    obj.winner = 2;
                end
                goal = true;
            end
            if goal %a goal was made
                pause(obj.START_DELAY);
                obj.resetGame;
                if obj.winner > 0 %somebody won
                    obj.pauseGame(['      PLAYER ' num2str(obj.winner) ' IS THE WINNER!!!' 10])
                    obj.newGame;
                else %nobody won
                end
            end
        end
        
        %================================================================
        %sets paused variable to true
        %starts a while loop guarded by pause variable displays provided string message
        %called from newGame at game start, called from keyDown when user hits 'p'
        %called from checkGoal when someone scores
        function pauseGame(obj,input)
            obj.paused = true;
            str = '      hit any key to continue...';
            spacer = 1:obj.PAUSE_WIDTH;
            spacer(:) = uint8(' ');
            while obj.paused
                printText = [spacer 10 input 10 str 10];
                h = text(obj.MESSAGE_X,obj.MESSAGE_Y,printText);
                set(h, 'BackgroundColor', obj.PAUSE_BACKGROUND_COLOR)
                set(h, 'Color', obj.PAUSE_TEXT_COLOR)
                set(h,'EdgeColor',obj.PAUSE_EDGE_COLOR);
                set(h, 'FontSize',5,'FontName','Courier','LineStyle','-','LineWidth',1);
                pause(obj.FRAME_DELAY)
                delete(h);
            end
        end
        
        %==============================================================
        %sets paused to false
        %called from keyDown when user hits any key
        function unpauseGame(obj)
            obj.paused = false;
        end
        
        %===============================================================
        %takes input vector as argument
        %increases dx/dy for more horizontal movement
        %normalizes vector sets as new ball vector, accelerates ball
        %called by moveBall whenever ball hits something
        function bounce (obj,tempV)
            %increase dx by a random amount
            %helps keep the ball moving more horizontally than vertically
            tempV(1) = tempV(1) * ((rand/obj.B_FACTOR) + 1);
            %normalize vector
            tempV = tempV ./ (sqrt(tempV(1)^2 + tempV(2)^2));
            obj.ballVector = tempV;
            %ajust to make things interesting, bouncing accelerates ball
            if (obj.ballSpeed + obj.BALL_ACCELERATION < obj.MAX_BALL_SPEED)
                obj.ballSpeed = obj.ballSpeed + obj.BALL_ACCELERATION;
            end
        end
        
        %============================================================
        %listener registered in createFigure
        %listens for input, sets appropriate variables and calls functions
        function keyDown(obj,src,event)
            switch event.Key
                case 'a'
                    obj.paddle1V = 1;
                case 'z'
                    obj.paddle1V = -1;
                case 'uparrow'
                    obj.paddle2V = 1;
                case 'downarrow'
                    obj.paddle2V = -1;
                case 'p'
                    if ~obj.paused
                        obj.pauseGame([obj.MESSAGE_PAUSED obj.MESSAGE_CONTROLS]);
                    end
                case 'r'
                    obj.newGame;
                case 'q'
                    obj.unpauseGame;
                    obj.quitGame = true;
            end
            obj.unpauseGame;
        end
        
        %==========================================================
        %listener registered in createFigure
        %used to stop paddles on keyup
        function keyUp(obj,src,event)
            switch event.Key
                case 'a'
                    if obj.paddle1V == 1
                        obj.paddle1V = 0;
                    end
                case 'z'
                    if obj.paddle1V == -1
                        obj.paddle1V = 0;
                    end
                case 'uparrow'
                    if obj.paddle2V == 1
                        obj.paddle2V = 0;
                    end
                case 'downarrow'
                    if obj.paddle2V == -1
                        obj.paddle2V = 0;
                    end
            end
        end
        
        %================================================================
        %detect the floor in the first image to find the rotation matrix
        %that is used to correct the floor slope
        function floorDetection(obj,k2,dirDepth)
            img_new = [];
            while(isempty(img_new))
                img_new = obj.captureDepthImage(k2,dirDepth); %capture depth image
                pause(1);
            end
            %3D point coordinates in millimeters
            [X,Y,Z] = obj.get3DPoints(img_new);
            %find the normal to the floor plane using RANSAC
            num_points = length(Z);
            tol = 10;  %tolerance for fitting points to the plane in mm
            n = [0; 0; 1];  %inicial value of the vector normal to the floor
            np_floor = 0; %initial number of points at the floor
            fdepth = 0;  %initial value of the floor depth
            num_iters = 1000;
            rng(2);
            for iter=1:num_iters
                %find a plane from 3 random points
                ind = randperm(num_points,3);
                p1 = [X(ind(1)); Y(ind(1)); Z(ind(1))];
                p2 = [X(ind(2)); Y(ind(2)); Z(ind(2))];
                p3 = [X(ind(3)); Y(ind(3)); Z(ind(3))];
                %check if the points are not collinear
                if( norm(cross((p2-p1),(p3-p2)))/(norm(p2-p1)*norm(p3-p2)) > 0.1 )
                    pc = mean([p1 p2 p3],2);
                    q1 = p1 - pc;
                    q2 = p2 - pc;
                    q3 = p3 - pc;
                    A = [q1 q2 q3]';
                    [U,S,V] = svd(A);
                    np = V(:,3) * sign(V(3,3));
                    %number of points lying at the plane
                    d = pc'*np;
                    nfit = sum(abs([X Y Z]*np - d)<=tol);
                    if( nfit > np_floor )
                        n = np;
                        np_floor = nfit;
                        fdepth = d;
                    end
                end
            end
            %find the rotation matrix to make the floor horizontal
            A = [n, [0,0,1]'];
            [U,S,V] = svd(A);
            B = U'*A;
            R1 = [B(1,1) -B(2,1); B(2,1) B(1,1)];
            R2 = [B(1,2) -B(2,2); B(2,2) B(1,2)];
            R = U * [R2*R1' [0; 0]; 0 0 1] * U';
            obj.rotation_matrix = R;
            obj.floor_vector = n;
            obj.floor_depth = fdepth;
            %tolerance to suppress the floor on the images
            ftol = 100;
            obj.depth_max = fdepth - ftol;
        end
        
        %============================================================
        %move the paddles using body movements of two persons detected
        %by the inect depth camera
        function pointTracker(obj,k2,dirDepth)
            img_new = obj.captureDepthImage(k2,dirDepth);
            R = obj.rotation_matrix;
            if(not(isempty(img_new)))
                %3D point coordinates in millimeters
                [X,Y,Z] = obj.get3DPoints(img_new);
                %segment image regions above the floor
                Zr = zeros(size(img_new));
                Yr = zeros(size(img_new));
                ind = (img_new > 0);                
                Zr(ind) = R(3,1)*X + R(3,2)*Y + R(3,3)*Z;
                Yr(ind) = R(2,1)*X + R(2,2)*Y + R(2,3)*Z;
                %binary image for segmenting objects above the floor       
                imb = (Zr<=obj.depth_max) & (Zr>0);
                imb = imerode(imb,ones(3,3));
                
                %detect players using Mean shift algorithm
                [rows,cols] = size(imb);
                hwidth = round(0.5*cols);
                [yL,xL] = find([imb(:,1:hwidth), zeros(rows,cols-hwidth)]);
                [yR,xR] = find([zeros(rows,hwidth), imb(:,hwidth+1:cols)]);
                if( not(isempty(xL)) && not(isempty(xR)) )
                    th = 1;  %threshold for settling convergence in pixels
                    sigma = 30;  %bandwith in pixels
                    [xLeft,yLeft] = obj.computeMeanShift(xL,yL,sigma,th);
                    [xRight,yRight] = obj.computeMeanShift(xR,yR,sigma,th);
                    %location of the players in millimeters
                    mYLeft = Yr(round(yLeft),round(xLeft));
                    mYRight = Yr(round(yRight),round(xRight));
                    %location of the players in the game area
                    gLeft = obj.scale_players(1)*mYLeft + obj.scale_players(2);
                    gRight = obj.scale_players(3)*mYRight + obj.scale_players(4);
                    %current paddle location
                    p1Center = (obj.paddle1(2,1) + obj.paddle1(2,3))/2;
                    p2Center = (obj.paddle2(2,1) + obj.paddle2(2,3))/2;
                    %control the motion of the paddles
                    %paddle1V and paddle2V holds either 0, -1 or 1
                    %depending on where they must move
                    obj.paddle1V = double((gLeft-p1Center) > obj.PADDLE_MOTION_THRESHOLD) ...
                        - double((p1Center-gLeft) > obj.PADDLE_MOTION_THRESHOLD);
                    obj.paddle2V = double((gRight-p2Center) > obj.PADDLE_MOTION_THRESHOLD) ...
                        - double((p2Center-gRight) > obj.PADDLE_MOTION_THRESHOLD);
                end
            end
        end
        
        %===============================================================
        %compute mean shift for 2D points
        function [xm,ym] = computeMeanShift(obj,xp,yp,sigma,th)
            xm = mean(xp);
            ym = mean(yp);
            dif = 1000;
            while( dif > th )
                g = exp(-0.5*((xp-xm).^2 + (yp-ym).^2)/sigma^2);
                sum_g = sum(g);
                dx = sum(xp.*g) / sum_g - xm;
                dy = sum(yp.*g) / sum_g - ym;
                dif = sqrt(dx^2 + dy^2);
                xm = xm + dx;
                ym = ym + dy;
            end
        end       
        
        %===============================================================
        %compute the 3D point coordinates in millimeters
        function [X,Y,Z] = get3DPoints(obj,img_new)
            [rows,cols] = size(img_new);
            %get horizontal and vertical focal lengths and principal point
            [fx,fy,cc] = obj.getCameraParameters(obj.FOV_DEPTH_CAMERA_HORZ, ...
                obj.FOV_DEPTH_CAMERA_VERT,cols,rows);
            %find 3D coordinates in millimeters
            [um,vm] = meshgrid(1:cols,1:rows);
            Z = double(img_new(:));
            ind = (Z>0);
            Z = Z(ind);
            X = Z.*(um(ind)-cc(1))/fx;
            Y = Z.*(vm(ind)-cc(2))/fy;
        end
        
        %=============================================================
        %compute the focal lengths and principal point of a camera
        function [fx,fy,cc] = getCameraParameters(obj,fov_horz,fov_vert,cols,rows)
            %focal length in pixel width dimension
            fx = (cols/2)/tand(fov_horz/2);
            %focal length in pixel height dimension
            fy = (rows/2)/tand(fov_vert/2);            
            %principal point assumed to be at the image center
            cc = 0.5*[cols, rows];
        end
        
        %=========================================================
        %set the game area using the image of the area illuminated by the projector
        function [ud,vd] = setGameArea(obj,img_color,img_depth)
            figure(1); imshow(img_color); hold on;
            title('Select the four vertices of the area illuminated by the projector');
            uc = zeros(4,1);
            vc = zeros(4,1);
            cx = rand(4,3);
            for i=1:4
                [uc(i),vc(i)] = ginput(1);
                scatter(uc(i),vc(i),50,cx(i,:),'filled');
            end
            [ud,vd] = colorDepthMapping(obj,uc,vc,img_color,img_depth);
            figure(2); imshow(img_depth,[]); hold on;
            scatter(ud,vd,30,cx,'filled');
            pause();
            close('all');
        end 
        
        %======================================================
        %determine the point correspondences in the depth image from points
        %selected in the color image
        function [ud,vd] = colorDepthMapping(obj,uc,vc,img_color,img_depth)
            %color camera parameters
            [rows,cols] = size(img_color(:,:,1));
            [fx_c,fy_c,cc_c] = obj.getCameraParameters(obj.FOV_COLOR_CAMERA_HORZ, ...
                obj.FOV_COLOR_CAMERA_VERT,cols,rows);
            Xc_n = (uc - cc_c(1))/fx_c;  % Xc/Zc
            Yc_n = (vc - cc_c(2))/fy_c;  % Xc/Zc
            %depth camera parameters
            [rows,cols] = size(img_depth);
            [fx_d,fy_d,cc_d] = obj.getCameraParameters(obj.FOV_DEPTH_CAMERA_HORZ, ...
                obj.FOV_DEPTH_CAMERA_VERT,cols,rows);
            %vector to optimize includes ud, vd and Zc
            n = length(uc);
            p0 = [cc_d(1)*ones(n,1), cc_d(2)*ones(n,1), 2000*ones(n,1)];
            options = optimset('display','off');
            p = fminsearch(@obj.minFunMapping,p0,options,Xc_n,Yc_n,fx_d,fy_d,cc_d);
            ud = p(:,1);
            vd = p(:,2);
        end
        
        %================================================
        %objective function to minimize the error of mapping point between
        %the color and depth images
        function res = minFunMapping(obj,p,Xc_n,Yc_n,fx,fy,cc)
            %points on the floor in the color camera system
            Zc = p(:,3);
            Xc = Xc_n.*Zc;
            Yc = Yc_n.*Zc;
            C = [Xc, Yc, Zc];
            %points on the floor in the depth camera system
            ud = p(:,1);
            vd = p(:,2);
            [Xd,Yd,Zd] = obj.floorPoints(ud,vd,fx,fy,cc);
            D = [Xd, Yd, Zd];
            %error of mapping
            np = size(Xd,1);
            P = obj.ROTATION_DEPTH_TO_COLOR*D' + repmat(obj.TRANSLATION_DEPTH_TO_COLOR,1,np);
            res = sqrt(mean((P(1,:)'-C(:,1)).^2 + (P(2,:)'-C(:,2)).^2 + (P(3,:)'-C(:,3)).^2));
        end
        
        %=============================================================
        %get the coordinates of points on the floor from pixel locations
        function [Xd,Yd,Zd] = floorPoints(obj,ud,vd,fx,fy,cc)
            nx = obj.floor_vector(1);
            ny = obj.floor_vector(2);
            nz = obj.floor_vector(3);
            d = obj.floor_depth;
%             c = d./(nz*(fx*ny*(vd-cc(2)) + fx*fy*nz + fy*nx*(ud-cc(1))));
%             Xd = (fy*nz + vd - cc(2))./c;
%             Yd = (fx*nz + ud - cc(1))./c;
%             Zd = (d - nx*Xd - ny*Yd)/nz;
            a11 = fx*nz + (ud-cc(1))*nx;
            a12 = (ud-cc(1))*ny;
            a21 = (vd-cc(2))*nx;
            a22 = fy*nz + (vd-cc(2))*ny;
            b1 = (ud-cc(1))*d;
            b2 = (vd-cc(2))*d;
            Xd = (a12.*b2 - a22.*b1)./(a12.*a21 - a11.*a22);
            Yd = (a21.*b1 - a11.*b2)./(a12.*a21 - a11.*a22);
            Zd = (d - nx*Xd - ny*Yd)/nz;
        end
        
        %===============================================================
        %get a scale factor and an offset for the detected positions of the
        %players to map them to the plot units used for the game
        function scalePlayers(obj,ud,vd,cols,rows)
            %ud and vd are the four vertices corresponding to the area
            %illuminated by the projector mapped to the depth image
            %left and right vertices
            [us,inds] = sort(ud);
            pleft = [ud(inds(1:2)), vd(inds(1:2))];
            pright = [ud(inds(3:4)), vd(inds(3:4))];
            p = [pleft; pright];
            %get the focal length and principal point of the depth camera
            [fx,fy,cc] = obj.getCameraParameters(obj.FOV_DEPTH_CAMERA_HORZ, ...
                obj.FOV_DEPTH_CAMERA_VERT,cols,rows);
            %location of the vertices on the floor in millimeters
            [X,Y,Z] = obj.floorPoints(p(:,1),p(:,2),fx,fy,cc);
            %compute the scale and offset for the rotated coordinates
            R = obj.rotation_matrix;
            Yleft = R(2,1)*X(1:2) + R(2,2)*Y(1:2) + R(2,3)*Z(1:2);
            Yright = R(2,1)*X(3:4) + R(2,2)*Y(3:4) + R(2,3)*Z(3:4);
            %scale and offset for the player 1
            k1 = obj.PLOT_H / (min(Yleft) - max(Yleft));
            b1 = -k1 * max(Yleft);
            %scale and offset for the player 2
            k2 = obj.PLOT_H / (min(Yright) - max(Yright));
            b2 = -k2 * max(Yright);
            obj.scale_players = [k1,b1,k2,b2];
        end

        %===============================================================
        %capture an image from the kinect depth camera
        function img_new = captureDepthImage(obj,k2,dirDepth)
%             validData = k2.updateData;
%             if( validData )
%                 depth = k2.getDepth; % Capture a depth image
%                 img_new = imresize(depth,0.5,'nearest'); %resize image a half of total size
%             else
%                 img_new = [];
%             end
            nimg = mod(k2-1,length(dirDepth)) + 1;
            fn = strcat('images/',dirDepth(nimg).name);
            im_depth = imread(fn);
%             img_new = double(imresize(im_depth,0.5,'nearest'));
            img_new = double(im_depth);
        end

        %==============================================================
        %run the game
        function runme(obj)
%             k2 = Kin2('depth');  %initialize kinect depth camera
            k2 = 0;
            dirDepth = dir(strcat('images/*.png'));
            dirColor = dir(strcat('images/*.jpg'));
            obj.floorDetection(k2,dirDepth);
            
            %take a color and depth images to set the game area
            img_depth = imread(strcat('images/',dirDepth(1).name));
            img_color = imread(strcat('images/',dirColor(1).name));
            
            %set the game area
            [ud,vd] = obj.setGameArea(img_color,img_depth);
            
            %compute the scale and offset of the players to map their
            %positions to the game plot units
            [rows,cols] = size(img_depth);
            obj.scalePlayers(ud,vd,cols,rows);
            obj.createFigure;
            obj.newGame;
            tr = clock;
            while ~obj.quitGame
                obj.moveBall;
                %the pointTracker function allows to move the paddles
                %using body movement seen by the kinect camera
                obj.pointTracker(k2,dirDepth);
                obj.movePaddles;
                obj.refreshPlot;
                obj.checkGoal;
                k2 = k2 + 1;
            end
            close(obj.fig);
            tr = clock - tr;
            etime = 3600*tr(4) + 60*tr(5) + tr(6);
            fps = k2/etime;
            fprintf('Elapsed time = %0.3f s\n', etime);
            fprintf('fps = %0.3f\n',fps);
        end
    end
end
