function Rotate_satellite(q_all)
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
% Plot a figure in the form of cubics.
%
% EXAMPLES
% cube_plot(2,3,4,'red')
%
% ------------------------------Code Starts Here------------------------------ %
r0=[0,0,0];
X=1;
Y=1;
Z=1;
q_all=[q_all(:,4),q_all(:,1:3)];
%r1 is z-axis rotation, r2 is y-axis rotation, and r3 is x-axis rotation.
[r1,r2,r3]=quat2angle(q_all);
s=size(r1,1);
% Define the vertexes of the unit cubic
ver = [ .5  .5 -.5;
       -.5  .5 -.5;
       -.5  .5  .5;
        .5  .5  .5;
       -.5 -.5  .5;
        .5 -.5  .5;
        .5 -.5 -.5;
       -.5 -.5 -.5];
%  Define the faces of the unit cubic
fac = [ 1 2 3 4;
        4 3 5 6;
        6 7 8 5;
        1 2 8 7;
        6 7 1 4;
        2 3 5 8];
 %%
[x,y,z] = cylinder([0,.1]);  
 x1=x+.6*ones(2,21);
 y1=y; 
 z1=.2*z;
 th(1)=surface(x1,y1, z1,'FaceColor','blue');
 th(2)=surface(x1,y1,-z1,'FaceColor','blue');
 
 x1=x-.6*ones(2,21);
 y1=y; 
 z1=.2*z;
 th(3)=surface(x1,y1, z1,'FaceColor','blue');
 th(4)=surface(x1,y1,-z1,'FaceColor','blue');
 
 x1=x;
 y1=y+.6*ones(2,21); 
 z1=.2*z;
 th(5)=surface(x1,y1, z1,'FaceColor','blue');
 th(6)=surface(x1,y1,-z1,'FaceColor','blue');
 
 x1=x;
 y1=y-.6*ones(2,21); 
 z1=.2*z;
 th(7)=surface(x1,y1, z1,'FaceColor','blue');
 th(8)=surface(x1,y1,-z1,'FaceColor','blue');
 %}

 %%
 yt=y;
 zt=z;
 z =-yt;
 y=zt;
 
 x1=x;
 y1=.2*y; 
 z1=z+.6*ones(2,21);
 th1(1)=surface(x1,-y1,z1,'FaceColor','blue');
 th1(2)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=x;
 y1=.2*y; 
 z1=z-.6*ones(2,21);
 th1(3)=surface(x1,-y1,z1,'FaceColor','blue');
 th1(4)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=x+.6*ones(2,21);
 y1=.2*y; 
 z1=z;
 th1(5)=surface(x1,-y1, z1,'FaceColor','blue');
 th1(6)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=x-.6*ones(2,21);
 y1=.2*y; 
 z1=z;
 th1(7)=surface(x1,-y1, z1,'FaceColor','blue');
 th1(8)=surface(x1,y1,z1,'FaceColor','blue');
 %}
 %%
[x,y,z] = cylinder([0,.1]); 
 
 xt = x;
 zt = z;
 x  = zt;
 z  =-xt; 
 
 x1=.2*x;
 y1=y; 
 z1=z+.6*ones(2,21);
 th2(1)=surface(-x1,y1,z1,'FaceColor','blue');
 th2(2)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=.2*x;
 y1=y; 
 z1=z-.6*ones(2,21);
 th2(3)=surface(-x1,y1,z1,'FaceColor','blue');
 th2(4)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=.2*x;
 y1=y+.6*ones(2,21); 
 z1=z;
 th2(5)=surface(-x1,y1, z1,'FaceColor','blue');
 th2(6)=surface(x1,y1,z1,'FaceColor','blue');
 
 x1=.2*x;
 y1=y-.6*ones(2,21); 
 z1=z;
 th2(7)=surface(-x1,y1, z1,'FaceColor','red');
 th2(8)=surface( x1,y1, z1,'FaceColor','red');
 
 
 
 


%% 
cube = [ver(:,1)*X+r0(1),ver(:,2)*Y+r0(2),ver(:,3)*Z+r0(3)];
ax = axes('XLim',[-1 1],'YLim',[-1 1],'ZLim',[-1 1]);
pbaspect([1 1 1]);
view(3);
grid on;

h= th;
h(9:16)=th1;
h(17:24)=th2;
h(25) = patch('Faces',fac,'Vertices',cube,'FaceColor','yellow');

%{
h(2) = surface( x1, y1, z1,'FaceColor','cyan');  
h(3) = surface( x1, y1,-z1,'FaceColor','cyan');  
h(4) = surface( z1, x1, y1,'FaceColor','cyan');  
h(5) = surface(-z1, x1, y1,'FaceColor','cyan');  
h(6) = surface( x1, z1, y1,'FaceColor','cyan');  
h(7) = surface( x1,-z1, y1,'FaceColor','cyan');  
%}
t = hgtransform('Parent',ax);

set(h,'Parent',t);
%%
%Video 
%
video_writer = VideoWriter('Rotation_mpeg','MPEG-4');
video_writer.Quality = 40;
video_writer.FrameRate = 30;
open(video_writer);
%}
pause(1)
for r = 1:5:s
    Rx = makehgtform('xrotate',r3(r),'yrotate',r2(r),'zrotate',r1(r));
    %Ry = makehgtform('yrotate',r);
    
    % Concatenate the transforms and
    % set the transform Matrix property
    set(t,'Matrix',Rx);
    drawnow
    writeVideo(video_writer, getframe(gcf));
%     if r==1
%         pause(1)
%     end
    pause(.05)

end
close(video_writer); 
end