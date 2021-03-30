function out = ceigenvec(str, enum, state)
    pattern = '\[([0-9]+)\]\[([0-9]+)\]';
    out = regexprep(str, pattern, '(${enum}::${char(state(str2num($1) + 1))})');
end

