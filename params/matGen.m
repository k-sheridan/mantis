width = 9

height = 9

spacing = .32



% Nested for loops 1st for the height second for the width
 %Measures the white 
 
 fprintf("WHITE\n");
 for xW = (-1 * ((width - 1) / 2):1:((width - 1) / 2)) 
     for yW = (-1 * ((height - 1) / 2) + 1: 1:((height - 1) / 2) - 1)
      fprintf("%f, %f, 0;\n", spacing * xW, spacing * yW);
      end
 end
 
 fprintf("\n\nGREEN\n");
 
 yG = (height - 1) / 2;
 centeryG = yG * spacing
 for xG = ((-1 * (width - 1) / 2): 1: (width -1) / 2)
      fprintf("%f, %f, 0; \n", spacing * xG, centeryG);
 end
 
 
 fprintf("\n\nRED\n");
 
 yR = -1 * (height - 1) / 2;
 centeryR = yR * spacing
 for xR = ((-1 * (width - 1) / 2): 1: (width -1) / 2)
      fprintf("%f, %f, 0; \n", spacing * xR, centeryR);
 end
 