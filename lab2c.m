% plots and prints out x,y,z magnetic field sensor readings

mconst=[0.2 1.3 1.2];   % change this based on your compensation values
range = 1;  % range is +/- 1 gauss for plotting
figure	% create a new figure window (if you don't want the previous figure to be erased)
subplot(2,2,1);
h1=plot(0,0);
title('x axis'); grid on		% turn on grid
axis([0 100 -range range]) % set the ranges of x and y axes
subplot(2,2,2); % second subplot of 2x2 figures
h2=plot(0,0);   % empty plot (with a point at (0,0))
title('y axis'); grid on
axis([0 100 -range range]) % set the ranges of x and y axes
subplot(2,2,3); % third subplot of 2x2 figures
h3=plot(0,0);   % empty plot (with a point at (0,0))
title('z axis');grid on
axis([0 100 -range range]) % set the ranges of x and y axes
drawnow			% draw immediately without waiting until the end of program
while 1           % infinite loop. You can stop it by typing Ctrl-C
    for k=1:100,
        d=ee405;		% get sensor data from ee405 board
        mag=d.mag-mconst;     % compensation for magnetic field created by parts on the board
        mag                 % prints out mag vector
        if k>1
            subplot(2,2,1); hold on; plot([k-1,k],[mag0(1) mag(1)],'b');
            subplot(2,2,2); hold on; plot([k-1,k],[mag0(2) mag(2)],'g');
            subplot(2,2,3); hold on; plot([k-1,k],[mag0(3) mag(3)],'r');
        end
        mag0=mag;		% preserve the previous value
        drawnow			% draw immediately without waiting until the end of program
        pause(0.03);    % pause for about 30msec
    end
    subplot(2,2,1); hold off; plot(0,0); axis([0 100 -range range]); title('x axis'); grid on
    subplot(2,2,2); hold off; plot(0,0); axis([0 100 -range range]); title('y axis'); grid on
    subplot(2,2,3); hold off; plot(0,0); axis([0 100 -range range]); title('z axis'); grid on
    drawnow
end
