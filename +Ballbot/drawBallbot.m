function drawBallbot(q, f, pObs, rObs)
phi = q(1);
theta = q(2); 

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Rotation matrix
rotmat = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

clf(f)
hold on

ballRadius = rk;
ballX = phi*ballRadius;
rectangle('Position',[ballX-ballRadius, 0, 2*ballRadius, 2*ballRadius],'Curvature',[1 1])
ballCenter = [ballX; ballRadius];
ballPatchColor = [50, 168, 149]/255;

% Create a patch on the ball
colorize_circle(phi, ballCenter, ballPatchColor, ballRadius); 

% Draw wheel
wheelRadius = rw; 
wheelPatchColor = [168, 50, 50]/255;
wheelCenter = ballCenter + (ballRadius + wheelRadius) * [sin(theta); cos(theta)];
rectangle('Position',[wheelCenter(1)-wheelRadius, wheelCenter(2)-wheelRadius, 2*wheelRadius, 2*wheelRadius],'Curvature',[1 1])
wheelRotationAngle = (phi-theta)*ballRadius/wheelRadius - theta; 
colorize_circle(-wheelRotationAngle, wheelCenter, wheelPatchColor, wheelRadius); 



% Draw pendulum starting on Y axis straight down
barThickness = .01;
barLength = l-rw-rk;
barColor = [.5,.5,.5];
bodyBar = [barThickness/2 * [-1, 1, 1, -1];
            barLength * [0, 0, 1, 1]];
pendulumTransformed = rotmat(-theta) * bodyBar + wheelCenter;
fill(pendulumTransformed(1,:), pendulumTransformed(2,:),barColor)
COMcenter = wheelCenter + barLength * [sin(theta); cos(theta)];
rCoM = wheelRadius/3;
drawCOM(theta, COMcenter, [.5, .5, .5], rCoM); 

% Draw an obstacle
if ~isempty(pObs)
    colorize_circle(0, pObs, 'k', rObs-rCoM);
    colorize_circle(pi/4, pObs, 'k', rObs-rCoM);

    % Draw outer safety zone on obstacle
    draw_circle(pObs(1), pObs(2),rObs, [.5,.5,.5], 2,':')
end


axis equal
axis([-1 1.5 -.025 1])
% axis off
xlabel('$x$ Position (m)','interpreter','latex')
ylabel('$y$ Position (m)','interpreter','latex')
set(gca,'TickLabelInterpreter','latex')


% Draw the ground
fill([xlim, flip(xlim)], [0, 0, -.025, -.025], [0,0,0])

% Finally, drawnow
drawnow
end

function draw_circle(x,y,r, circle_color, circle_thickness,linestyle)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit, 'Color',circle_color,'LineWidth',circle_thickness,'LineStyle',linestyle);
end


function colorize_circle(phi, ballCenter, ballPatchColor, ballRadius)
eighth_circle_angles = linspace(0, pi/4, 100); 
for ix = 0:3
    anglesToFill = phi + eighth_circle_angles + ix*pi/2;
    patch1 = [ballCenter, ballCenter + ballRadius * [sin(anglesToFill); cos(anglesToFill)]];
    fill(patch1(1,:), patch1(2,:),ballPatchColor)
end

end

function drawCOM(phi, ballCenter, ballPatchColor, ballRadius)
rectangle('Position',[ballCenter(1)-ballRadius, ballCenter(2)-ballRadius, 2*ballRadius, 2*ballRadius],'Curvature',[1 1])

eighth_circle_angles = linspace(0, pi/2, 100); 
for ix = 0:1
    anglesToFill = phi + eighth_circle_angles + ix*pi;
    patch1 = [ballCenter, ballCenter + ballRadius * [sin(anglesToFill); cos(anglesToFill)]];
    fill(patch1(1,:), patch1(2,:),ballPatchColor)
end

eighth_circle_angles = linspace(0, pi/2, 100); 
for ix = 0:1
    anglesToFill = phi + eighth_circle_angles + ix*pi + pi/2;
    patch1 = [ballCenter, ballCenter + ballRadius * [sin(anglesToFill); cos(anglesToFill)]];
    fill(patch1(1,:), patch1(2,:),[1,1,1])
end

end