figure  % create a new figure window (if you don't want the previous figure to be erased)
h=plot(0,0);   % empty plot (with a point at (0,0))
axis off square % remove axis lines and make it a square
axis([-1.2 1.2 -1.2 1.2])
hold on         % to plot more without erasing old ones
t=(0:100)*2*pi/100;
plot(cos(t),sin(t))     % big circle
for k=1:16,
    plot([0 cos(k*2*pi/16)],[0 sin(k*2*pi/16)], 'b')     % lines
    ht=text(1.1*sin(k*2*pi/16),1.1*cos(k*2*pi/16),sprintf('%d', round(k*360/16))); % numbers
    set(ht,'HorizontalAlignment','center')      % text is now center aligned
end
nx=[-0.1 0.1 0 -0.1];   % x axis values of a triangular hand
ny=[0 0 1 0];           % y axis values of a triangular hand
northh=patch(nx,ny,'r');   % north hand is red
southh=patch(-nx,-ny,'w'); % south hand is white
