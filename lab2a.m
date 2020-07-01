figure	% create a new figure window
h=plot(0,0,'o');   % initial plot (with a point at (0,0))
axis([-1.5 1.5 -1.5 1.5]) % set the ranges of x and y axes
axis square			% make the figure square
while 1             % infinite loop. You can stop it by typing Ctrl-C
    d=ee405;	% get sensor data from ee405 board
    acc=d.acc;	% get accelerometer reading as a 1x3 vector
		% (-1.5g ~ 1.5g for each axis)
    set(h,'XData',-acc(1),'YData',-acc(2));	% change the position of 'o' 
				% to be (-acc(1),-acc(2))
	% minus is used to invert the acceleration values such that
	% when the right hand side of the board is
	% down, the point on the screen moves to the right not to the left.
    drawnow	% draw immediately without waiting until the end of program
    pause(0.03);	% delay by about 30msec
end
