function output = interleave2(varargin) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function interleaves the rows or columns of the specified vectors or
% matrices A, B, C, .... The last argument indicates whether to interleave
% the rows or columns by passing 'row' or 'col', respectively. If
% interleaving the rows, the number of columns of A and B must be the same;
% if interleaving the columns, the number of rows must be the same. 
%
% The order of input vectors/matrices indicate the order of interleaving
% (e.g. first row of A will be the first row, first row of B will be the
% second row of the interleaved matrix).
% 
% Extra rows or columns will be appended to the end of the final vector or
% matrix.
%
% If all inputs are vectors, their elements will be interleaved without
% the vectors needing to be all rows or all columns if the last argument
% ('row' or 'col') is not specified. If the orientation of the interleaving
% is specified, then the program will follow that orientation.
% 
% Examples:
% 1) Interleaving rows of matrices
% A = [1 2           B = [5 6
%      3 4]               7 8]     
% C = interleave2(A, B, 'row')
% C = [1 2
%      5 6
%      3 4
%      7 8]
%
% 2) Interleaving columns of matrices
% C = interleave2(A, B, 'col')
% C = [1 5 2 6
%      3 7 4 8]
%
% 3) Interleaving vectors
% A = [1 2 3 4]      B = [5 6 7 8 9]'
% C = interleave2(A, B)
% C = [1 5 2 6 3 7 4 8 9]
%
% 4) Interleaving >2 matrices
% A = [1 2           B = [5 6         C = [9 10      D = [13 14
%      3 4]               7 8]            11 12]          15 16]
% E = interleave2(A, B, C, D, 'col')
% E = [1 5  9 13 2 6 10 14
%      3 7 11 15 4 8 12 16]
% 
% 5) Interleaving columns of 2 matrices with unequal columns
% A = [1 2           B = [5 6  7  8
%      3 4]               9 10 11 12] 
% C = interleave2(A, B, 'col')
% C = [1 5 2 6  7  8
%      3 9 4 10 11 12]
%     
% 6) Interleaving >2 vectors of unequal lengths
% A = [1 2 3 4]    B = [5 6 7]    C = [8 9 10 11 12 13]
% D = interleave2(A, B, C, D)
% D = [1 5 8 2 6 9 3 7 10 4 11 12 13]
% 
% Written by Steven Shimizu
% March 4, 2014
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Check if interleaving orientation is specified
if ischar(varargin{end}) %check if last argument is a char array
    orientation = varargin{end};
    
    %Then check if interleaving orientation is anything but 'row' or 'col'
    if ~(isequal(orientation,'row') || isequal(orientation,'col'))
        %if not equal to 'row' or 'col', throw an error
        error('Last argument should specify ''row'' or ''col'' unless the inputs only consist of vectors.');
    else
        %The orientation is specified, then there are n-1 input numeric
        %arrays (single numbers, vectors or 2d matrices).
        numInputs = nargin - 1;
        
        for i = 1:1:numInputs
            if ~isnumeric(varargin{i})
                error('Inputs must be numeric arrays.');
            end
        end
    end
    
%check if last input argument is a numeric array, then check that all
%of the input arguments are vectors only, otherwise throw an error.
elseif isnumeric(varargin{end})
    orientation = 'none';
    numInputs = nargin;
    for i = 1:1:numInputs
        if ~isvector(varargin{i})
            error('The interleaving orientation ''row'' or ''col'' was not specified, which is only valid if all inputs are vectors.');
        end
    end
    
end
%Now that all inputs are known to be numeric arrays and interleaving
%orientation is specified, check that they are the right size
sizeArray = zeros(numInputs,2);
lengthList = zeros(numInputs,1);
for i = 1:1:numInputs
    sizeArray(i,:) = size(varargin{i});
    if isequal(orientation,'none')
        lengthList(i) = length(varargin{i});
    end   
end
if isequal(orientation,'row') && ~(sum(sizeArray(:,2)==sizeArray(1,2)) == numInputs)
    %check if number of columns are the same
    error('The number of columns of input matrices must be the same for interleaving the rows');
elseif isequal(orientation,'col') && ~(sum(sizeArray(:,1)==sizeArray(1,1)) == numInputs)
    error('The number of rows of input matrices must be the same for interleaving the columns');
end
output = [];        
%If only a single numeric array is passed, then return the same array
if numInputs == 1
    output = varargin{1};
elseif isequal(orientation,'none')
    %interleave vectors
    output = zeros(sum(lengthList),1);
    maxLength = max(lengthList);
    i_output = 1;
    for i = 1:1:maxLength
        for n = 1:1:numInputs
            if i <= lengthList(n)
                output(i_output) = varargin{n}(i);
                i_output = i_output + 1;
            end
        end
    end
elseif isequal(orientation,'row')
    %interleave by rows
    output = zeros(sum(sizeArray(:,1)),sizeArray(1,2));
    maxRows = max(sizeArray(:,1));
    i_output = 1;
    for i = 1:1:maxRows
        for n = 1:1:numInputs
            if i <= sizeArray(n,1)
                output(i_output,:) = varargin{n}(i,:);
                i_output = i_output + 1;
            end
        end
    end
elseif isequal(orientation,'col')
    %interleave by columns
    output = zeros(sizeArray(1,1),sum(sizeArray(:,2)));
    maxCols = max(sizeArray(:,2));
    i_output = 1;
    for i = 1:1:maxCols
        for n = 1:1:numInputs
            if i <= sizeArray(n,2)
                output(:,i_output) = varargin{n}(:,i);
                i_output = i_output + 1;
            end
        end
    end
end
end