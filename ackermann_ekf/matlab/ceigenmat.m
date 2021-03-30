function out = ceigenvec(str, enum1, state1, enum2, state2)
    pattern = '\[([0-9]+)\]\[([0-9]+)\]';
    out = regexprep(str, pattern, '(${enum1}::${char(state1(str2num($1) + 1))}, ${enum2}::${char(state2(str2num($2) + 1))})');
end

