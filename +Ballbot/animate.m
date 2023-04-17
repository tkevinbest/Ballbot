function animate(t, q, videoName, pObs, rObs)
% Adapted from class code
if ~exist('videoName','var')
    saveVideo = false;
else
    saveVideo = true; 
end

if ~exist('pObs', 'var')
    pObs = [];
    rObs = [];
end


% Animation
FPS = 60; % Frame rate in frames per second

t_anim = 0:1/FPS:max(t);
% Interpolate our angles
q_anim = interp1(t, q, t_anim); 


if saveVideo
    warning('off','MATLAB:audiovideo:VideoWriter:mp4FramePadded');
    if ~exist('Animations', 'dir')
       mkdir('Animations')
    end
    v = VideoWriter(fullfile('Animations',videoName),'MPEG-4');
    v.Quality = 100;
    v.FrameRate = FPS;
    open(v);
end

f = figure(); % Create new figure for animation
set(f,'outerposition',[89 177 1440 1080]) % Set its size and pos.
set(f,'color',[0.8 0.9 1])
for iter = 1:numel(t_anim)
    Ballbot.drawBallbot(q_anim(iter,:), f, pObs, rObs);  
    if saveVideo
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end

if saveVideo
    close(v);
    warning('on','MATLAB:audiovideo:VideoWriter:mp4FramePadded');
end
end
