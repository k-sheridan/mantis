width = 10

height = 10

spacing = .32

density = 0.25

fid = fopen('out.yaml', 'w+');

% Nested for loops 1st for the height second for the width
 %Measures the white 
 
%white
fprintf(fid, "whiteMap: '");

%print x lines

for y = (-1 * ((height - 1) / 2) + 1:1:((height - 1) / 2) - 1)
for x = ((-1*((width - 1) / 2)):density:((width - 1) / 2))
fprintf(fid, "%f, %f, 0;\n", spacing * x, spacing * y);
end
end

for x = ((-1*((width - 1) / 2)):1:((width - 1) / 2))
for y = ((-1 * ((height - 1) / 2) + density):density:(((height - 1) / 2) -density))
fprintf(fid, "%f, %f, 0;\n", spacing * x, spacing * y);
end
end
fprintf(fid, "'");


fprintf(fid, "\n\ngreenMap: '");
 
 yG = (height - 1) / 2;
 centeryG = yG * spacing
 for xG = ((-1 * (width - 1) / 2):density: (width -1) / 2)
      fprintf(fid, "%f, %f, 0; \n", spacing * xG, centeryG);
 end
 
fprintf(fid, "'");


fprintf(fid, "\n\nredMap: '");
 yR = -1 * (height - 1) / 2;
 centeryR = yR * spacing
 for xR = ((-1 * (width - 1) / 2):density: (width -1) / 2)
      fprintf(fid, "%f, %f, 0; \n", spacing * xR, centeryR);
 end

fprintf(fid, "'");

fclose(fid)
