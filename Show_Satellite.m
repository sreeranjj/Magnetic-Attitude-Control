function Show_Satellite(X_all)
% CUBE_PLOT plots a cube with dimension of X, Y, Z.
%
% INPUTS:
% origin = set origin point for the cube in the form of [x,y,z].
% X      = cube length along x direction.
% Y      = cube length along y direction.
% Z      = cube length along z direction.
% color  = STRING, the color patched for the cube.
%         List of colors
%         b blue
%         g green
%         r red
%         c cyan
%         m magenta
%         y yellow
%         k black
%         w white
% OUPUTS:
% Plot a figure in the form of cubics. And Translates and Rotates it..
%
%
% ------------------------------Code Starts Here------------------------------ %

r=X_all(:,1:3)*1000;
q=X_all(:,7:10);
siz_r=length(X_all);
r0=[0,0,0];
X=1;
Y=1;
Z=1;

q=[q(:,4),q(:,1:3)];
%r1 is z-axis rotation, r2 is y-axis rotation, and r3 is x-axis rotation.
[r1,r2,r3]=quat2angle(q);

% Define the vertexes of the unit cubic
ver = [ .5  .5 -.5;
       -.5  .5 -.5;
       -.5  .5  .5;
        .5  .5  .5;
       -.5 -.5  .5;
        .5 -.5  .5;
        .5 -.5 -.5;
       -.5 -.5 -.5];
a=.52;
       
%  Define the faces of the unit cubic
fac = [ 1 2 3 4;
        4 3 5 6;
        6 7 8 5;
        1 2 8 7;
        6 7 1 4;
        2 3 5 8];
 %% Th_ver_x
[y,z,x] = cylinder([0,.1]);  
 x=.3*x;
 thx(1)=surface((x-.02),(y+a),(z+a),'FaceColor','red');
 thx(2)=surface((x-.02),(y-a),(z-a),'FaceColor','red');
 thx(3)=surface((-x+.02),(y+a),(z-a),'FaceColor','red');
 thx(4)=surface((-x+.02),(y-a),(z+a),'FaceColor','red');
 
 %% Th_ver_y
 %
  [z,x,y] = cylinder([0,.1]);  
   y=.3*y;
   thy(1)=surface((x+a),(-y+.02),(z+a),'FaceColor','blue');
   thy(2)=surface((x+a),(y-.02),(z-a),'FaceColor','blue');
   thy(3)=surface((x-a),(-y+.02),(z-a),'FaceColor','blue');
   thy(4)=surface((x-a),(y-.02),(z+a),'FaceColor','blue');
 
 %}
 %% Th_ver_z
 %
  [x,y,z] = cylinder([0,.1]);  
   z=.3*z;
   thz(1)=surface((x+a),(y+a),(z-.02),'FaceColor','green');
   thz(2)=surface((x-a),(y-a),(z-.02),'FaceColor','green');
   thz(3)=surface((x-a),(y+a),(-z+.02),'FaceColor','green');
   thz(4)=surface((x+a),(y-a),(-z+.02),'FaceColor','green');
 %}
%% 
cube = [ver(:,1)*X+r0(1),ver(:,2)*Y+r0(2),ver(:,3)*Z+r0(3)];


%

view(3);
pbaspect([1,1,1]);

h(1:4)= thx;
h(5:8)= thy;
h(9:12)=thz;
h(13) = patch('Faces',fac,'Vertices',cube,'FaceColor','yellow');
h(1:4)= thx;

%}



%%
%{
pause(1)
for i = 1:1:siz_r
    view(3)
    ax = axes('XLim',[-5+r(i,1) 5+r(i,1)],'YLim',[-5+r(i,2) 5+r(i,2)],'ZLim',[-3+r(i,3) 3+r(i,3)]);
    t = hgtransform('Parent',ax);
    set(h,'Parent',t);
    tr = makehgtform('translate',[r(i,1), r(i,2) ,r(i,3)]);
    Rot = makehgtform('xrotate',r3(i),'yrotate',r2(i),'zrotate',r1(i));
    % Concatenate the transforms and
    % set the transform Matrix property
    set(t,'Matrix',tr*Rot)
    hold on;
    plot3(r(i,1), r(i,2) ,r(i,3),'Marker','*','MarkerSize',.001);
    view(3)
    drawnow
    pause(.05)
    axis off

end

%}
end