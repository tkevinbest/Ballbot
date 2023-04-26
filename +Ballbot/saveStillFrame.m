function saveStillFrame(q, videoName, pObs, rObs)
% Adapted from class code

if ~exist('pObs', 'var')
    pObs = [];
    rObs = [];
end

if ~isfolder('Figures')
       mkdir('Figures')
end

f = figure(); % Create new figure for animation
set(f,'outerposition',[89 177 1440 1080]) % Set its size and pos.
set(f,'color',[0.8 0.9 1])

numToSave = size(q, 2); 

for iter = 1:numToSave
    Ballbot.drawBallbot(q(:,iter), f, pObs, rObs);  
    exportgraphics(f, ['./Figures/',videoName,'_',num2str(iter,4),'.pdf'],...
        'ContentType','vector','BackgroundColor','none');
end
end


