function matrix = translation_matrix(x,y,z)
%% Deepak Khosla, May 31, 2019
%% Jacob Moorman, June 21, 2019

translation=[x; y; z];

matrix = [eye(3)   translation;
          0 0 0    1];

end
