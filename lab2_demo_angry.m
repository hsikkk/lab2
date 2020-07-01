f=double(imread('angrybird640.jpg'))/255;

figure	% create a new figure window (if you don't want the previous figure to be erased)
subplot(1,5,1); % first subplot of 1x5 figures
image(f)
axis off equal

state = 0;
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

        if (state == 0) & (accnorm > 1.4) & ~d.is_playing % high g condition (probably thrown)
            state = 32; % timeout of 1 second (if no zero gravity condition occurs)
        elseif state > 0
            state = state - 1; % decrease the variable, if 0 is reached wait for another high g condition
            if accnorm < 0.3 % zero gravity detected
                ee405('say','angry.wav'); % angrybird sound effect 
                state = 0;
            end
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
