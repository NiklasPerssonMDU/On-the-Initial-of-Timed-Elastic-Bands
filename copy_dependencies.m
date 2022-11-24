function copy_dependencies( start_mfile, destination )
% copy_dependencies 
% Copies all external code files necessary to run a script into a subfolder. 
%
% Use cases are for example
%  - execute on a remote server
%  - give code away
%  - for backup / archive
% 
% Careful: All necessary code must be on Matlab's path when invoking 
% copy_dependencies; it doesn't warn if anything is currently missing.   
% Therefore to be on the safe side, run the "start_mfile" firstly in order
% to see if everything works. Afterwards you can run copy_dependencies. 
%
% Input arguments
%  - start_mfile  :  Existing Matlab file that is supposed to run (required) 
%  - destination  :  Folder where to save the dependencies (default: new subfolder in current directory)  
%

% Own dependencies 
% [flist,plist] = matlab.codetools.requiredFilesAndProducts('copy_dependencies.m'); [flist'; {plist.Name}']

% Process arguments
arguments
  % https://de.mathworks.com/help/matlab/matlab_oop/property-validator-functions.html#bvkk9xo
  start_mfile    (1,:)         {mustBeFile, mustBeNonempty} 
  destination    (1,:)         {mustBeText} = ''
end

% Destination not specified?
if nargin < 2
  % Make default destination
  [~,name,~] = fileparts(start_mfile);
  destination = sprintf('%s-dependencies-%s', name, datestr(now,'yyyymmdd-HHMMSS')); 
end

% Find dependent files
fprintf('Looking for dependent files...   ')
flist = matlab.codetools.requiredFilesAndProducts(start_mfile); 
nFiles = length(flist);
fprintf('Found %u files.   Copying to %s ...   ', nFiles, java.io.File(destination).getCanonicalPath)

% Create target dir if not exists
if exist(destination,'dir')~=7
    mkdir(destination)
end

% Copy the individual code files
for iFile = 1:nFiles
    % source file
    file = flist{iFile}; 
    % destination filename
    [~,name,ext] = fileparts(file);
    filename = [name,ext];
    destfile = fullfile(destination,filename);
    % copy
    copyfile(file,destfile);
end

fprintf('Done. \n')

end

