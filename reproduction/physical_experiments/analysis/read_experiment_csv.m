function record = read_experiment_csv(filePath)
%READ_EXPERIMENT_CSV Parse one section-based physical-experiment CSV file.
%   RECORD = READ_EXPERIMENT_CSV(FILEPATH) supports the 57-column format
%   (with T_se) and the 41-column format (without T_se). Raw values are
%   preserved exactly; the experiment length unit is nanometers.

arguments
    filePath (1, 1) string
end

fid = fopen(filePath, 'rt');
if fid < 0
    error('ExperimentCSV:FileOpen', 'Cannot open file: %s', filePath);
end
cleanup = onCleanup(@() fclose(fid));

record = struct('FilePath', filePath, 'Sections', struct());
currentSection = '';

while ~feof(fid)
    line = strtrim(string(fgetl(fid)));
    if strlength(line) == 0
        continue;
    end

    sectionToken = regexp(line, '^# === SECTION: ([A-Za-z0-9_]+)', ...
        'tokens', 'once');
    if ~isempty(sectionToken)
        currentSection = sectionToken{1};
        if ~isfield(record.Sections, currentSection)
            record.Sections.(currentSection) = emptySection();
        end
        continue;
    end

    if isempty(currentSection)
        continue;
    end

    if startsWith(line, 'Index,')
        record.Sections.(currentSection).Header = split(line, ',')';
    elseif startsWith(line, 'FINAL_CALIB_MATRIX_TCS,')
        values = parseNumericLine(extractAfter(line, 'FINAL_CALIB_MATRIX_TCS,'));
        if numel(values) == 16
            record.Sections.(currentSection).FinalTcs = reshape(values, 4, 4)';
        end
    elseif ~isempty(regexp(line, '^[+-]?(\d|\.)', 'once'))
        values = parseNumericLine(line);
        if ~isempty(values)
            raw = record.Sections.(currentSection).RawData;
            if isempty(raw)
                record.Sections.(currentSection).RawData = values(:)';
            elseif size(raw, 2) == numel(values)
                record.Sections.(currentSection).RawData(end + 1, :) = values(:)';
            else
                error('ExperimentCSV:ColumnMismatch', ...
                    'Inconsistent column count in %s, section %s.', ...
                    filePath, currentSection);
            end
        end
    end
end

sectionNames = fieldnames(record.Sections);
for i = 1:numel(sectionNames)
    name = sectionNames{i};
    record.Sections.(name) = decodeSection(record.Sections.(name), ...
        filePath, name);
end
end

function section = emptySection()
section = struct('Header', strings(0, 1), 'RawData', [], ...
    'RawColumnCount', 0, 'Index', [], 'T_cs_list', [], 'T_se_list', [], ...
    'Twist_c', [], 'Twist_s', [], 'Pc_prev', [], 'Pc_curr', [], ...
    'Ps_prev', [], 'Ps_curr', [], 'FinalTcs', []);
end

function section = decodeSection(section, filePath, sectionName)
raw = section.RawData;
if isempty(raw)
    return;
end

section.RawColumnCount = size(raw, 2);
numRows = size(raw, 1);
section.Index = raw(:, 1);
section.T_cs_list = reshapeTransforms(raw(:, 2:17));

switch size(raw, 2)
    case 57
        section.T_se_list = reshapeTransforms(raw(:, 18:33));
        section.Twist_c = raw(:, 34:39);
        section.Twist_s = raw(:, 40:45);
        section.Pc_prev = raw(:, 46:48);
        section.Pc_curr = raw(:, 49:51);
        section.Ps_prev = raw(:, 52:54);
        section.Ps_curr = raw(:, 55:57);
    case 41
        section.T_se_list = zeros(4, 4, 0);
        section.Twist_c = raw(:, 18:23);
        section.Twist_s = raw(:, 24:29);
        section.Pc_prev = raw(:, 30:32);
        section.Pc_curr = raw(:, 33:35);
        section.Ps_prev = raw(:, 36:38);
        section.Ps_curr = raw(:, 39:41);
    otherwise
        error('ExperimentCSV:UnsupportedColumns', ...
            'Unsupported %d-column data in %s, section %s.', ...
            size(raw, 2), filePath, sectionName);
end

if size(section.T_cs_list, 3) ~= numRows
    error('ExperimentCSV:TransformCount', ...
        'Transform count mismatch in %s.', filePath);
end

section.RawData = []; % Avoid retaining a duplicate copy in memory.
end

function transforms = reshapeTransforms(flatRows)
numRows = size(flatRows, 1);
transforms = zeros(4, 4, numRows);
for i = 1:numRows
    transforms(:, :, i) = reshape(flatRows(i, :), 4, 4)';
end
end

function values = parseNumericLine(line)
line = replace(string(line), ',', ' ');
values = sscanf(char(line), '%f')';
end
