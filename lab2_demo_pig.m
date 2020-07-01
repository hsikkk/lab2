f=double(imread('pig640.jpg'))/255;

figure	% create a new figure window (if you don't want the previous figure to be erased)
subplot(1,5,1); % first subplot of 2x2 figures
image(f)
axis off equal

while 1           % infinite loop. You can stop it by typing Ctrl-C
    subplot(1,5,2); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('x axis'); grid on
    subplot(1,5,3); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('y axis'); grid on
    subplot(1,5,4); hold off; plot(0,0); axis([0 100 -1.5 1.5]); title('z axis'); grid on
    subplot(1,5,5); hold off; plot(0,0); axis([0 100 0 2.5]); title('norm'); grid on
 
    d=ee405;
    acc0=d.acc;		% accelerometer reading
    accnorm0=norm(acc0);  % norm
    for k=1:100,
        d=ee405;		% get sensor data from ee405 board
        acc=d.acc;		% accelerometer reading
        accnorm=norm(acc);  % norm
        if (accnorm > 1.5) && ~d.is_playing
            ee405('say','pig.wav');
        end
        if k>1
            subplot(1,5,2); hold on; plot([k-1,k],[acc0(1),acc(1)],'b');
            subplot(1,5,3); hold on; plot([k-1,k],[acc0(2),acc(2)],'g');
            subplot(1,5,4); hold on; plot([k-1,k],[acc0(3),acc(3)],'r');
            subplot(1,5,5); hold on; plot([k-1,k],[accnorm0,accnorm],'k');
            drawnow
        end
        acc0=acc;		% preserve the previous value
        accnorm0=accnorm;
        pause(0.015)
    end
end
