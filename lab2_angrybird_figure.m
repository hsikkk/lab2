% angry bird figure
figure	% create a new figure window (if you don't want the previous figure to be erased)
n=3;
[x y z] = meshgrid(-n:n,-n:n,-n:n); % creates 3D mesh grid
data = sqrt(x.^2 + y.^2 + z.^2);    % radius
pp = patch(isosurface(x,y,z,data,n)); % sphere with radius=3
h=isonormals(x,y,z,data,pp);
axis([-4 4 -4 4 -4 4]) 
% rgb=(1,0,0), i.e., red color for the sphere
% ones(7,7,7) means 3-dim matrix of size 7x7x7.
% 7 since there are 2n+1=7 data points in each axis. 
isocolors(x,y,z,ones(7,7,7),zeros(7,7,7),zeros(7,7,7),pp);
set(pp,'FaceColor','interp','EdgeColor','none')
view(90,0);     % this will create a view such that x,y,z axes are facing front,
                % right, up on the screen, respectively.
daspect([1 1 1]);   % aspect ratio = 1:1:1
hold on         % to draw more plots without erasing previous ones
t=(0:16)*2*pi/16;
eye1=[3*ones(1,17);0.5*cos(t)-0.5;0.5*sin(t)+1];    % 16-gon (right eye)
eye2=[3*ones(1,17);0.5*cos(t)+0.5;0.5*sin(t)+1];    % left eye
e1=patch(eye1(1,:),eye1(2,:),eye1(3,:),'w');        % white color
e2=patch(eye2(1,:),eye2(2,:),eye2(3,:),'w');
eye3=[3.1*ones(1,17);0.15*cos(t)-0.25;0.15*sin(t)+1];   % pupil of right eye
eye4=[3.1*ones(1,17);0.15*cos(t)+0.25;0.15*sin(t)+1];   % pupil of left eye
e3=patch(eye3(1,:),eye3(2,:),eye3(3,:),'k');        % black color
e4=patch(eye4(1,:),eye4(2,:),eye4(3,:),'k');
eb=[3.1 3.1 3.1 3.1 3.1 3.1 3.1;0 1.6 1.3 0 -1.3 -1.6 0;1.1 1.5 1.9 1.4 1.9 1.5 1.1];   % eyebrow
e5=patch(eb(1,:),eb(2,:),eb(3,:),'k');  % black color
camlight    % creates a light right and up from camera.
lighting phong  % make the sphere shiny
