

function ret = all2str(param)

% Function to concert any data to a string in
% matlab: 
% Original See http://stackoverflow.com/questions/12799161/is-there-a-matlab-function-to-convert-any-data-structure-to-a-string
% Author: angainor
% Modified Philip Long to include symbilic
% variables
%

if isempty(param)
    if iscell(param)
        ret = '(empty cell)';
    elseif isstruct(param);
        ret = '(empty struct)';
    else
        ret = '(empty)';
    end
    return;
end

if ischar(param)
    ret = param;
    return;
end

if isnumeric(param)
    ret = num2str(param);
    return;
end

if iscell(param)
    ret = all2str(param{1});
    for i=2:numel(param)
        ret = [ret ', ' all2str(param{i})];
    end
    return;
end

if isstruct(param)
    ret = '(structure)';
    return;
end

if    strcmp(class(param),'sym')   
    ret = char(param);
    return;
end