function out = ceigenvecn(str, cname, vector)
    out = str;
    [~, idx] = sortsyms(vector);
    for i=idx'
        pattern = sprintf('(?<!:)%s', char(vector(i)));
        out = regexprep(out, pattern, sprintf('%s(%d)', cname, i - 1));
    end
end