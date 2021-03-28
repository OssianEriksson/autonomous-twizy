function symdump(string, filename)
    fid = fopen(filename, 'wt');
    fprintf(fid, string);
    fclose(fid);
end

