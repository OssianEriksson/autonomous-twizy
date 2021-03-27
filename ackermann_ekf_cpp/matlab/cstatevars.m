function out = cstatevars(str, state)
    out = str;
    [sorted, ~] = sortsyms(state);
    for var=sorted'
        c = char(var);
        out = regexprep(out, sprintf('(?<!:)%s', c), 'x_(State::${c})');
    end
end

