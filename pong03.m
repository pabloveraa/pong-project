%+-------------------------------------------------------+%
%|                DAVE'S MATLAB PONG v0.3                |%
%|                  by David Buckingham                  |%
%|                                                       |%
%| a fast-paced two-player game inspired by Atari's Pong |%
%+-------------------------------------------------------+%


%------------------Added functions-----------------------------
% k2 = Kin2('depth') : initialize kinect depth camera
% floorDetection(k2) : detect the floor to correct the camera tilt on the
% images
% pointTracker(k2) : this function allows to use body movements to move the
% paddles using the kinect depth camera
%
% these functions requiere:
% Terven Juan, Cordova-Esparza Diana,  Kin2. A Kinect 2 Toolbox for MATLAB, 
% Science of Computer Programming, 2016, http://dx.doi.org/10.1016/j.scico.2016.05.009
%------------------------------------------------------------

% to run type:
% p = pong03;
% p.runme

classdef pong03 < handle
    properties(Constant)
        %game settings
        MAX_POINTS = 5;
        START_DELAY = 1;
        
        %movement
        FRAME_DELAY = .01; %animation frame duration in seconds, .01 is good.
        MIN_BALL_SPEED = 2; %each round ball starts at this speed
        MAX_BALL_SPEED = 15; %wont accelerate bast this, dont set too high or bugs.
        BALL_ACCELERATION = 0.2; %how much ball accelerates each bounce.
        PADDLE_SPEED = 2;
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
        loc_prev = []; %previous locations of detected persons
        loc_new = [];  %new locations of detected persons
        rotation_matrix = [];  %rotation matrix to correct the floor slope
        depth_max = [];   %maximal depth to detect only objects above the floor
    end
    methods
        %===============================================================
        %constructor
        function obj = pong03
            
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
        function floorDetection(obj,k2)
            img_new = [];
            while(isempty(img_new))
                img_new = obj.captureImage(k2); %capture depth image
                pause(1);
            end
            [rows,cols] = size(img_new);
            fov = 70.6;  %depth camera field of view
            fc = (cols/2)/tand(fov/2);  %focal length
            cc = 0.5*[cols, rows];  %principal point assumed to be at the image center
            %find 3D coordinates in millimeters
            [um,vm] = meshgrid(1:cols,1:rows);
            Z = double(img_new(:));
            X = Z.*(um(:)-cc(1))/fc;
            Y = Z.*(vm(:)-cc(2))/fc;
            %find the floor plane using RANSAC
            n = length(Z);
            tol = 10;  %tolerance for fitting points to the plane
            s = [];  %variable for determining the plane parameters
            np = 0; %initial number of points at the floor
            num_iters = 1000;
            rng(1);
            for iter=1:num_iters
                %find a plane from 3 random points
                ind = randperm(n,3);
                M = [X(ind), Y(ind), Z(ind)];
                if( rcond(M)>0.001 )
                    sp = M\ones(3,1);
                    d = 1/norm(sp);
                    a = sp(1)*d;
                    b = sp(2)*d;
                    c = sp(3)*d;
                    %number of points lying at the plane
                    nfit = sum(abs(a*X + b*Y + c*Z - d)<=tol);
                    if( nfit>np )
                        np = nfit;
                        s = sp;
                    end
                end
            end
            %floor plane parameters
            d = 1/norm(s);
            a = s(1)*d;
            b = s(2)*d;
            c = s(3)*d;
            %find the rotation matrix to make the floor horizontal
            thx = asind(-b);
            thy = asind(-a/cosd(thx));
            obj.rotation_matrix = [cosd(thy) 0 sind(thy); -sind(thx)*sind(thy) cosd(thx) ...
                sind(thx)*cosd(thy); -cosd(thx)*sind(thy) -sind(thx) cosd(thx)*cosd(thy)];
            %depth of the floor from the kinect camera
            R = obj.rotation_matrix;
            Zr = R(3,1)*X + R(3,2)*Y + R(3,3)*Z;
            ind = abs(a*X + b*Y + c*Z - d)<=tol;
            floor_depth = mean(Zr(ind));
            ftol = 100; %tolerance to suppress the floor on the images
            obj.depth_max = floor_depth - ftol;
        end
        
        %============================================================
        %move the paddles using body movements of two persons detected
        %by the inect depth camera
        function pointTracker(obj,k2)
            img_new = obj.captureImage(k2);
            thr_pad = 5;  %minimum displacement in mm to move the paddles
            R = obj.rotation_matrix;
            if(not(isempty(img_new)))
                [rows,cols] = size(img_new);
                fov = 70.6;
                cc = 0.5*[cols, rows];  %principal point assumed to be at the image center
                f = (cols/2)/tand(fov/2);  %focal length
                [um,vm] = meshgrid(1:cols,1:rows);
                Z = double(img_new(:));
                X = Z.*(um(:)-cc(1))/f;
                Y = Z.*(vm(:)-cc(2))/f;
                %segment image regions above the floor
                Zr = R(3,1)*X + R(3,2)*Y + R(3,3)*Z;
                Yr = R(2,1)*X + R(2,2)*Y + R(2,3)*Z;
                ind = (Zr<=obj.depth_max) & (Zr>0);
                imb = zeros(rows,cols);
                imb(ind) = 1;
                imb = imerode(imb,ones(3,3));
                %median value of Ys for objects above the ground
                iLeft = find(imb(:,1:round(0.5*cols)));
                iRight = find(imb(:,round(0.5*cols):cols));
                if (not(isempty(iLeft)) && not(isempty(iRight)))
                    mYLeft = median(Yr(iLeft));
                    mYRight = median(Yr(iRight + round(0.5*cols*rows)));
                    obj.loc_new = [mYLeft, mYRight];
                    if(not(isempty(obj.loc_prev)))
                        dym = obj.loc_new(1)-obj.loc_prev(1);
                        obj.paddle1V = double(dym>thr_pad) - double(dym<(-thr_pad));
                        dym = obj.loc_new(2)-obj.loc_prev(2);
                        obj.paddle2V = double(dym>thr_pad) - double(dym<(-thr_pad));
                    else
                        obj.paddle1V = 0;
                        obj.paddle2V = 0;
                    end
                else
                    obj.loc_new = [];
                    obj.paddle1V = 0;
                    obj.paddle2V = 0;
                end
            else
                pause(0.05);
            end
            obj.loc_prev = obj.loc_new;
        end
        
        %===============================================================
        function img_new = captureImage(obj,k2)
            validData = k2.updateData;
            if( validData )
                depth = k2.getDepth; % Capture a depth image
                img_new = imresize(depth,0.5,'nearest'); %resize image a half of total size
            else
                img_new = [];
            end
        end
        
        %==============================================================
        function runme(obj)
            obj.createFigure;
            obj.newGame;
            k2 = Kin2('depth');  %initialize kinect depth camera
            obj.floorDetection(k2);
            while ~obj.quitGame
                obj.moveBall;
                %the pointTracker function allows to move the paddles
                %using body movement seen by the kinect camera
                obj.pointTracker(k2);
                obj.movePaddles;
                obj.refreshPlot;
                obj.checkGoal;
            end
            close(obj.fig);
        end
    end
end
