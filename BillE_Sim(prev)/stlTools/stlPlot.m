function obj = stlPlot(v, f, name, color)
%STLPLOT is an easy way to plot an STL object
%V is the Nx3 array of vertices
%F is the Mx3 array of faces
%NAME is the name of the object, that will be displayed as a title
if nargin < 4
    color = [0.8 0.8 1.0];
end

%figure;
object.vertices = v;
object.faces = f;
obj = patch(object,'FaceColor',       color, ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
%camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([45 45]);
grid on;
%title(name);
