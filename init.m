disp("Executing init...");
%% CLEAR FUNS FOLDER
switch n
    case 2
        pth = 'funs2R/';
    case 3
        pth = 'funs3R/';
    otherwise
        disp("Not implemented");
end
% if clear_funs
%     if 7 == exist(['./',pth], 'dir')
%         disp("Rimossa cartella funs.");
%         rmdir(['./',pth],'s');
%     end
%     disp("Creata cartella funs.");
%     mkdir(['./',pth]);
% end
%% ADD THE FUNS FOLDER TO THE MATLAB PATH FOR FINDING THE FUNCTIONS TO RUN IN SIMULATION
warning('off','MATLAB:rmpath:DirNotFound');
added_path = [pwd,'/',pth];
rmpath(pwd);
rmpath([pwd,'/funs2R']);
rmpath([pwd,'/funs3R']);
addpath(pwd);
addpath(added_path);
disp("Init done.");
