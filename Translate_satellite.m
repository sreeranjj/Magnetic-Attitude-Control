function Translate_satellite(r)
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
siz_r=length(r);
r0=[0,0,0];
X=1;
Y=1;
Z=1;
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
ax = axes('XLim',[-500 500],'YLim',[-500 500],'ZLim',[-500 500]);
pbaspect([1,1,1]);
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
pause(1)
for i = 1:1:siz_r
    
    Rz = makehgtform('translate',[r(i,1), r(i,2) ,r(i,3)]);
    % Concatenate the transforms and
    % set the transform Matrix property
    set(t,'Matrix',Rz)
    hold on;
   plot3(r(i,1), r(i,2) ,r(i,3),'Marker','*','MarkerSize',.001);
    drawnow
    pause(.05)

end


end