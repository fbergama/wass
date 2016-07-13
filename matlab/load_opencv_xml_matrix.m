function M = load_opencv_xml_matrix( filename )
%LOAD_OPENCV_MATRIX 


text = fileread(filename);
%disp(text);

% Extract rows
matchStr = regexp(text,'<rows>(\d*)</rows>','tokens');
assert( numel(matchStr)==1, 'No <rows> section found' );
rows = floor( str2double( cell2mat(matchStr{1}) ) );

% Extract columns
matchStr = regexp(text,'<cols>(\d*)</cols>','tokens');
assert( numel(matchStr)==1, 'No <cols> section found' );
cols = floor( str2double( cell2mat(matchStr{1}) ) );

% Extract data
%matchStr = regexp(text,'<data>\s*([a-zA-Z_0-9\.\+\-]*)*','tokens')
matchStr = regexp(text,'<data>([a-zA-Z_0-9\.\+\-\s]*)</data>','tokens');
datanum = matchStr{1}{1};
matchStr = regexp(datanum,'([a-zA-Z_0-9\.\+\-]*)*','tokens');
assert( numel(matchStr)==rows*cols, 'Data size is not valid');

N = rows*cols;
M = zeros(N,1);
for II=1:numel(matchStr)
    M(II) = str2double( cell2mat( matchStr{II}) );
end

M = reshape( M, cols, rows )';

end

