% 3 separate plots of (x,y,z) acceleration

% Try to change the orientation of the board to see how x,y,z acceleration values change

figure	% create a new figure window (if you don't want the previous figure to be erased)
subplot(2,2,1); % first subplot of 2x2 figures
h1=plot(0,0);   % empty plot (with a point at (0,0))
title('x axis'); grid on
axis([0 100 -1.5 1.5]) % set the ranges of x and y axes
subplot(2,2,2); % second subplot of 2x2 figures
h2=plot(0,0);   % empty plot (with a point at (0,0))
title('y axis'); grid on
axis([0 100 -1.5 1.5]) % set the ranges of x and y axes
subplot(2,2,3); % third subplot of 2x2 figures
h3=plot(0,0);   % empty plot (with a point at (0,0))
title('z axis'); grid on
axis([0 100 -1.5 1.5]) % set the ranges of x and y axes
alpf=zeros(1,3);	% low pass filtered acceleration value
while 1           % infinite loop. You can stop it by typing Ctrl-C
    for k=1:100,
        d=ee405;
        acc=d.acc;		% accelerometer reading
        Tlpf = 10;		% time constant (in # of samples) of LPF
        alpf = (1-1/Tlpf) * alpf + (1/Tlpf) * acc;	% low-pass filtered
        if k>1
            % uncomment one of the following 2 sets of 3 lines

            % plot instantaneous acceleration values
            subplot(2,2,1); hold on; plot([k-1,k],[acc0(1),acc(1)],'b');
            subplot(2,2,2); hold on; plot([k-1,k],[acc0(2),acc(2)],'g');
            subplot(2,2,3); hold on; plot([k-1,k],[acc0(3),acc(3)],'r');

            % or, you can plot instantaneous - smoothed (low-pass filtered)
            %subplot(2,2,1); hold on; plot([k-1,k],[alpf0(1) alpf(1)],'b');
            %subplot(2,2,2); hold on; plot([k-1,k],[alpf0(2) alpf(2)],'g');
            %subplot(2,2,3); hold on; plot([k-1,k],[alpf0(3) alpf(3)],'r');
            drawnow
        end
        acc0=acc;		% preserve the previous value
        alpf0=alpf;		% preserve the previous value
    end
    subplot(2,2,1); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('x axis'); grid on
    subplot(2,2,2); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('y axis'); grid on
    subplot(2,2,3); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('z axis'); grid on
    drawnow	% draw immediately without waiting until the end of program
    pause(0.03);    % delay by about 30msec
end
