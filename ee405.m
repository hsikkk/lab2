function varargout=ee405(varargin)
% Communication with one or more EE405 boards
% Written by Sae-Young Chung 
% Last update: 2016/3/26

% ee405('com3')   % open serial port in windows
% ee405('/dev/ttyUSB0') % open serial port in linux
    % 'sudo usermod -a -G dialout username' needs to be done for serial
    % port access (needs to be done only once)
    % log out and log back in after that
% ee405('EE405_001')  % open bluetooth device
% ee405, ee405(id) % returns the most recent sensor data
%   id, ver, vcpu, seq, acc, mag, hum, temp, light, is_playing, joy_stick,
%   date_time_valid, photo_tr, rx_buffer_free
%   specify board id if there are multiple boards
%   humidity & temperature are measured periodically (default = 1min)

% ee405('reset')  % reset all connections
% ee405('close')  % close a connection
% ee405('open')   % open a connection
% ee405('closeall') % close all connections
% ee405('openall') % open all connections
% ee405('recent')   % returns the last received packet (for debugging)
% ee405('status')   % returns status (for debugging)
% ee405('firmware', file_name) % udpate firmware

id_requested=[];

k=1;
if nargin >= 1
    if isnumeric(varargin{1})
        id_requested=varargin{1};
        k=2;
    end
end
cmd=[];
param=[];
param2=[];
param3=[];
param4=[];
param5=[];
if nargin >= k
    cmd=varargin{k};
    if nargin >= k+1
        param=varargin{k+1};
        if nargin >= k+2
            param2=varargin{k+2};
            if nargin >= k+3
                param3=varargin{k+3};
                if nargin >= k+4
                    param4=varargin{k+4};
                    if nargin >= k+5
                        param5=varargin{k+5};
                    end
                end
            end
        end
    end
end

persistent timer_obj last_timer_event
persistent nb   % number of boards
persistent s    % array of serial port or bluetooth object
persistent port_name
persistent on_cleanup_obj

persistent ee405_ver
persistent max_payload_size min_length
persistent max_nboards max_tx_queue_packets max_zigbee_packets max_zigbee_packet_size max_midi_packets max_midi_packet_size
persistent input_buffer_size output_buffer_size
persistent print_debug_info print_debug_info2
persistent print_debug_info_once

persistent state 
persistent in_callback block_callback
persistent d    % packet from the board
persistent n    % number of bytes in d
persistent t    % time stamp for the packet
persistent id   % id of the board
persistent d_decoded  % decoded packet
persistent received % 1 if a new packet is received
persistent last_seq_rx % last seq # received at the board
persistent seq_skipped % whether a seq # is skipped at the board
persistent tx_queue tx_queue_size tx_queue_tot_size tx_queue_seq % tx queue 
persistent tx_queue_h tx_queue_t tx_queue_c tx_queue_time_stamp tx_retx_count
persistent seq       % seq. number for tx to the board
persistent gps_data  % gps data (only one gps is supported)
persistent zigbee_packets zigbee_h zigbee_t  % zigbee packets, head, tail
persistent zigbee_sensor_data zigbee_sensor_data_source_id
persistent midi_packets midi_h midi_t    % midi packets (only one midi device is supported)
persistent last_reported_seq
persistent servo_pos servo_speed servo_load
persistent servo_advanced_mode
persistent ss_seq ss_last max_tx_seq

% output variables
persistent board_time     % time returned from a board
persistent ram start_addr_ram len_ram           % ram data returned from a board
persistent servo_mode     % servo data returned from a board
persistent servo_status   % servo status returned from a board
persistent servo_id_monitor
persistent zigbee_status

if isempty(nb)
    nb=0;
    init
    
    ee405_ver = 17;
    print_debug_info=0;
    print_debug_info2=0;
    
    timer_obj=timer('TimerFcn',@timer_callback,'BusyMode','drop',...
        'ExecutionMode','fixedRate', 'StartDelay',0.01,...
        'Period',0.03);
    start(timer_obj);
    last_timer_event=clock;
    on_cleanup_obj = onCleanup(@() cleanupFun(timer_obj));
end

if nb==0
    if strcmpi(cmd,'help')
        print_help('ee405 commands.txt')
        return
    elseif ~strncmpi(cmd,'com',3) && ~strncmpi(cmd,'EE405',5) && ~strncmpi(cmd,'/dev/tty',8)
        error('Not initialized. Specify com port or bluetooth device name, e.g., ee405(''com3'') or ee405(''EE405_005'').')
    end
end

if etime(clock,last_timer_event)>1       % if the timer is not working (can happen if ctrl-c is pressed)
    stop(timer_obj)
    init
    start(timer_obj);
    last_timer_event=clock;
    for kw=1:nb                          % added on 2016/03/13
        if ~isempty(s{k})
            transmit(s{k}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
        end
    end
    wait_for_id
end

if strcmpi(cmd,'help')
    print_help('ee405 commands.txt')
    return
elseif strncmpi(cmd,'com',3) || strncmpi(cmd,'EE405',5) || strncmpi(cmd,'/dev/tty',8)
    for k=1:nb
        if strcmpi(cmd,port_name{k}) && ~isempty(s{k})
            block_callback(k)=1;
            if strcmp(s{k}.Status,'closed')
                try
                    fopen(s{k});
                catch
                    s{k}=[];
                    block_callback(k)=0;
                    error('Port open failure')
                end
                disp('Port opened')
            else
                disp('Port already open')
            end
            block_callback(k)=0;
            return
        end
    end
    new_id=nb+1;
    for k=1:nb
        if isempty(s{k})
            new_id=k;
            break;
        end
    end
    if new_id>max_nboards
        error(sprintf('This program does not support more than %d ee405 boards.', max_nboards));
    end
    
    block_callback(new_id)=1;
    init(new_id);
    id(new_id)=0;
    if strncmpi(cmd,'com',3) || strncmpi(cmd,'/dev/tty',8)
        s{new_id}=serial(cmd,'baud',921600);
    else
        try
            s{new_id}=Bluetooth(cmd,1);
        catch
            s{new_id}=[];
            block_callback(new_id)=0;
            error(sprintf('Cannot find Bluetooth device %s.', cmd));
        end
    end
    
    s{new_id}.Timeout=10;   % safe to use 10 sec due to slow bluetooth
    s{new_id}.InputBufferSize=input_buffer_size;    % if this is not big enough, data can be lost
    s{new_id}.OutputBufferSize=output_buffer_size;

    port_name{new_id}=cmd;
    try
        fopen(s{new_id});
        flushinput(s{new_id});
    catch
        fclose(s{new_id})
        s{new_id}=[];            % disable it
        port_name{new_id}='';
        block_callback(new_id)=0;
        error(sprintf('Cannot open %s', cmd));
    end
    transmit(s{new_id}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
                                                                    % calling this is ok since block_callback(new_id)==0
    if new_id>nb           
        nb=new_id;      % update nb if needed so that the callback can recognize the new board
    end

    block_callback(new_id)=0;    % enable callback
    wait_for_id(new_id);
    for k=1:nb
        if k~=new_id
            if id(k)==id(new_id)    % this can happen, e.g., when a board is connected using bluetooth, disconnected, and then connected using serial
                block_callback(k)=1;
                if strcmp(s{k}.Status,'open')
                    fclose(s{k})
                end
                s{k}=[];            % disable it
                port_name{k}='';
                block_callback(k)=0;
            end
        end
    end
    disp(sprintf('Board %d connected via %s', id(new_id), cmd));
    if d_decoded{new_id}.ver < ee405_ver
        fn_firmware=sprintf('ee405_v%d.hex',ee405_ver);
        block_callback(new_id)=1;
        fupd_result=firmware(s{new_id}, fn_firmware, (d_decoded{new_id}.ver <= 15));
        init(new_id);
        id(new_id)=0;
        block_callback(new_id)=0;
        wait_for_id(new_id);
        if ~strcmp(fupd_result,'success')
            error('Firmware update failed');
        end
    end
    if print_debug_info
        s{new_id}
    end
    return
end

if isempty(id_requested)
    for k=1:nb
        if ~isempty(s{k})  % the first valid one
            id_requested=id(k);
            break;
        end
    end
end
idx=[];
for k=1:nb
    if ~isempty(s{k}) && (id(k)==id_requested)
        idx=k;
        break;
    end
end

if isempty(idx)
    error(sprintf('Board %d not recognized. Connect the board and initialize the port, e.g., ee405(''com3'') or ee405(''EE405_005'').', id_requested));
end

if strcmpi(despace(cmd),'close')
    block_callback(idx)=1;
    fclose(s{idx});
    block_callback(idx)=0;
    disp('Port closed')
    return
elseif strcmpi(despace(cmd),'closeall')
    for k=1:nb
        if isempty(s{k})
            continue;
        end
        block_callback(k)=1;
        fclose(s{k});
        block_callback(k)=0;
    end
    disp('Ports closed')
    return
elseif strcmpi(despace(cmd),'open')
    block_callback(idx)=1;
    if strcmp(s{idx}.Status,'closed')
        try
            init(idx);
            fopen(s{idx});
            flushinput(s{nb});
        catch
            block_callback(idx)=0;
            error('Port open failure')
        end
        disp('Port opened')
    else
        disp('Port is already open.')
    end
    transmit(s{idx}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
                                                                 % calling this is ok since block_callback(idx)==0
    block_callback(idx)=0;
    wait_for_id(idx);
    return
elseif strcmpi(despace(cmd),'openall')
    for k=1:nb
        if isempty(s{k})
            continue;
        end
        block_callback(k)=1;
        if strcmp(s{k}.Status,'closed')
            try
                init(k);
                fopen(s{k});
                flushinput(s{k});
            catch
                block_callback(k)=0;
                error(sprintf('Port open failed for %s', port_name(k)))
            end
        end
        transmit(s{k}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
                                                                   % calling this is ok since block_callback(k)==0
        block_callback(k)=0;
    end
    wait_for_id
    return
elseif strcmpi(despace(cmd),'reset')
    for k=1:nb
        if isempty(s{k})
            continue;
        end
        block_callback(k)=1;
        if strcmp(s{k}.Status,'open')
            fclose(s{k})
        end
        if strcmp(s{k}.Status,'closed')
            try
                init(k);
                fopen(s{k});
                flushinput(s{k});
            catch
                block_callback(k)=0;
                error(sprintf('Port open failed for %s', port_name(k)))
            end
        end
        transmit(s{k}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
                                                                   % calling this is ok since block_callback(k)==0
        block_callback(k)=0;
    end
    wait_for_id
    return
end

if strcmp(s{idx}.Status,'open') && (etime(clock,t(idx,:))>2)    % something wrong
    disp(sprintf('No response from board %d. Resetting the communication port.', id(idx)))
    block_callback(idx)=1;
    fclose(s{idx});
    pause(0.01)
    fopen(s{idx});
    flushinput(s{idx});
    init(idx);
    transmit(s{idx}, packetize(0, [], []), output_buffer_size);  % transmit an empty packet to reset the rx seq counter in the board
                                                                 % calling this is ok since block_callback(idx)==0
    block_callback(idx)=0;
    t0=clock;
    while 1
        if etime(clock,t0)>5
            error(sprintf('Board %d is not responding.', id(idx)))
        end
        pause(0.01)
        if etime(clock,t(idx,:))<0.1
            disp('OK')
            break;      % board is responding
        end
    end
end

if strcmpi(despace(cmd),'status')
    block_callback(idx)=1;
    s{idx}
    s{idx}.BytesAvailableFcn
    [idx tx_queue_seq(idx, tx_queue_t(idx)) tx_queue_h(idx) tx_queue_t(idx) tx_queue_c(idx)]
    block_callback(idx)=0;
    return
elseif strcmpi(despace(cmd), 'firmware')
    if isempty(param)
        error('File name not specified when calling ee405(''firmware'',file_name)')
    end
    if ~ischar(param)
        error('File name invalid when calling ee405(''firmware'',file_name)')
    end
    block_callback(idx)=1;
    fupd_result=firmware(s{idx}, param, (d_decoded{idx}.ver <= 15));
    init(idx);
    id(idx)=0;
    block_callback(idx)=0;
    wait_for_id(idx);
    if ~strcmp(fupd_result,'success')
        error('Firmware update failed');
    end
    return    
elseif strcmpi(despace(cmd),'midirx')
    if midi_h~=midi_t
        midi_t=midi_t+1;
        if midi_t > max_midi_packets
            midi_t=1;
        end
        mp=midi_packets(midi_t,:);
        varargout{1}.key=mp(1);
        varargout{1}.vol=mp(2);
        varargout{1}.time=mp(3);
    else
        varargout{1}=[];
    end
    return
elseif strcmpi(despace(cmd),'zigbeerx')
    if zigbee_h(idx)~=zigbee_t(idx)
        zigbee_t(idx)=zigbee_t(idx)+1;
        if zigbee_t(idx) > max_zigbee_packets
            zigbee_t(idx)=1;
        end
        zp=squeeze(zigbee_packets(idx,zigbee_t(idx),:));
        varargout{1}.source=zp(1);
        varargout{1}.dest=zp(2);
        varargout{1}.seq=zp(3);
        varargout{1}.lqi=zp(4);
        varargout{1}.rssi=zp(5);
        varargout{1}.time=zp(6);
        varargout{1}.data=zp(8:zp(7)+7)';        % row vector
    else
        varargout{1}=[];
    end
    return
elseif strcmpi(despace(cmd),'gps')
    % returns the most recent gps data
    varargout{1}=gps_data;
    return
elseif strcmpi(despace(cmd),'printdebuginfoonce')
    print_debug_info_once=1;
    return
elseif strcmpi(despace(cmd),'getservo')
    % call ee405('servomonitor',id) before using this
    % pos=ee405('getservo',id); returns the position of the servo
    % [pos,speed]=ee405('getservo',id); returns the position & speed
    % [pos,speed,load]=ee405('getservo',id); returns position, speed, load
    % pos=-1 will be returned if there's no response from the servo
    if isempty(param)
        error('Servo id not specified when calling ee405(''getservo'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''getservo'', servo_id). Servo id should be between 0 and 253.')
    end
    varargout{1}=servo_pos(param+1);
    if nargout>=2
        varargout{2}=servo_speed(param+1);
    end
    if nargout>=3
        varargout{3}=servo_load(param+1);
    end
    return
elseif strcmpi(despace(cmd),'recent')
    block_callback(idx)=1;
    varargout{1}=d_decoded{idx};
    block_callback(idx)=0;
    last_reported_seq(idx)=varargout{1}.seq;
    return
elseif isempty(cmd)
    while 1
        block_callback(idx)=1;
        varargout{1}=d_decoded{idx};
        block_callback(idx)=0;           
        if varargout{1}.seq ~= last_reported_seq(idx)
            last_reported_seq(idx)=varargout{1}.seq;
            return
        end
        pause(0.01)
    end
end

% now, cmd contains a command to send a packet to the board
if strcmpi(despace(cmd),'analoginputenable')
    % Enable A/D converter.
    % A/D converter reading is reported by the ee405.m function 32 times a
    % second. A/D converter input is the ADC+ port.
    % 0~3.3V is converted to 0~1023 (i.e., 10-bit quantization)
    block_callback(idx)=1;
    b=packetize(seq(idx), 81,[33 0]);    % channel 33 is ADC+, ref==0 means AVCC
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'analogoutput')
    % Set the analog output at the DAC port. d is a 12-bit integer (0 ~ 4095).
    % 0 is 0V and 4095 is 3.3V.
    if isempty(param)
        error('Argument not specified when calling ee405(''analogoutput'', value). ''value'' should be between 0 and 4095.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 4096)
        error('Argument invalid when calling ee405(''analogoutput'', value). ''value'' should be between 0 and 4095.')
    end
    iparam=floor(param);
    block_callback(idx)=1;
    b=packetize(seq(idx), 82,[bitand(iparam,255) bitand(bitshift(iparam,-8),255)]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'digitalinputenable')
    block_callback(idx)=1;
    b=packetize(seq(idx), 80, 1); 
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'digitaloutput')
    % PWM pin
    if isempty(param)
        error('Argument not specified when calling ee405(''digitaloutput'', value). ''value'' should be 0 or 1.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param==0 || param == 1)
        error('Argument invalid when calling ee405(''digitaloutput'', value). ''value'' should be 0 or 1.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 83, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'disabletorque')
    if isempty(param)
        error('Servo id not specified when calling ee405(''disabletorque'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''disabletorque'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 22, [param 24 0]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'gettime')
    % Returns date & time of ee405 board.
    block_callback(idx)=1;
    board_time{idx}=[];
    b=packetize(seq(idx), 40, []);
    block_callback(idx)=0;
    % more processing to be done below
elseif strcmpi(despace(cmd),'settime')
    % Set date & time in ee405 board
    % Usage)
    %   param=[]: set date & time using PC's system time
    %   param=[year,month,date,hour,min,sec]: set specific date & time  
    %     year: 2000 ~ 2099, month: 1 ~ 12, date: 1 ~ 31, hour: 0 ~ 23, min: 0 ~ 59, sec: 0 ~ 59
    if ~isempty(param)
        if ~isnumeric(param) || length(param)~=6
            error(sprintf(['Date and time invalid when calling ee405(''settime'', [year month date hour min sec]).\n'...
                           'year: 2000 ~ 2099, month: 1 ~ 12, date: 1 ~ 31, hour: 0 ~ 23, min: 0 ~ 59, sec: 0 ~ 59']))
        end
        iparam=floor(param);
        iyear=iparam(1);
        imonth=iparam(2);
        idate=iparam(3);   % do not use 'date' since it is a matlab function
        ihour=iparam(4);
        imin=iparam(5);    % do not use 'min' since it is a matlab function
        isec=iparam(6);
    else
        clk=clock;
        iyear=clk(1);
        imonth=clk(2);
        idate=clk(3);
        ihour=clk(4);
        imin=clk(5);
        isec=floor(clk(6));
    end
    if (iyear >= 2000) && (iyear <= 2099) && (imonth >= 1) && (imonth <= 12) && ...
        (idate >= 1) && (idate <= 31) && (ihour >= 0) && (ihour <= 23) && ...
        (imin >= 0) && (imin <= 59) && (isec >= 0) && (isec <= 59)
        block_callback(idx)=1;
        b=packetize(seq(idx),41,[iyear - 2000 imonth idate ihour imin isec]);
        block_callback(idx)=0;
    else
        error('Date and time invalid')
    end
elseif strcmpi(despace(cmd),'enabletorque')
    if isempty(param)
        error('Servo id not specified when calling ee405(''enabletorque'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''enabletorque'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),22,[param 24 1]); 
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'fullcolorled')
    % data is a NLEDs * 3 matrix containing (R,G,B) values for each pixel.
    % R,G,B values are from 0 to 255 (integers)
    % NLEDs is the number of LED's, which should be between 1 and 64.
    if isempty(param)
        error('Color data not specified when calling ee405(''fullcolorled'', color_data)')
    end
    [NLEDs,ncolors]=size(param);
    if ncolors ~= 3
        error('Color data invalid when calling ee405(''fullcolorled'', color_data). ''color_data'' should be a matrix of size N*3.')
    end
    if (NLEDs > 64) || (NLEDs < 1)
        error('Number of LED''s should be between 1 and 64 when calling ee405(''fullcolorled'', color_data).')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),87,reshape(param',1,NLEDs*ncolors)); 
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'gpsenable')
    if isempty(param)
        error('Missing argument when calling ee405(''gpsenable'', value). ''value'' should be 0 or 1.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param==0 || param == 1)
        error('Parameter invalid when calling ee405(''gpsenable'', value). ''value'' should be 0 or 1.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 70,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'irled')
    if isempty(param)
        error('Missing argument when calling ee405(''lrled'', value). ''value'' should be 0 or 1.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param==0 || param == 1)
        error('Parameter invalid when calling ee405(''irled'', value). ''value'' should be 0 or 1.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),85,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'led')
    % change the status of the LED
    % param=[red green yellow]
    % red, green, yellow are 0 (off) or 1 (on)
    if isempty(param)
        error('Missing argument when calling ee405(''led'', [red green yellow]). Color values should be 0 (off) or 1 (on).')
    end
    if ~isnumeric(param) || length(param)~=3
        error('Parameter invalid when calling ee405(''led'', [red green yellow]). Color values should be 0 (off) or 1 (on).')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),84,param); 
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'midi')
    % send a midi command
    if isempty(param)
        error('Missing argument when calling ee405(''midi'', [byte1 byte2 ...])')
    end
    if ~isnumeric(param)
        error('Parameter invalid when calling ee405(''midi'', [byte1 byte2 ...])')
    end
    if max(param)>255 || min(param)<0
        error('Parameter invalid when calling ee405(''midi'', [byte1 byte2 ...]). Byte values should be between 0 and 255.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),60,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'midirxenable')
    if isempty(param)
        error('Missing argument when calling ee405(''midirxenable'', value). ''value'' should be 0 or 1.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param==0 || param == 1)
        error('Parameter invalid when calling ee405(''midirxenable'', value). ''value'' should be 0 or 1.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),61,param); 
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'phototrenable')
    if isempty(param)
        error('Missing argument when calling ee405(''phototrenable'', value). ''value'' should be 0 or 1.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param==0 || param == 1)
        error('Parameter invalid when calling ee405(''phototrenable'', value). ''value'' should be 0 or 1.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),86,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'readram')
    % ee405('readram',start_address, length)
    if isempty(param) || isempty(param2)
        error('Missing argument when calling ee405(''readram'', start_address, length)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param <=65535)
        error('Parameter invalid when calling ee405(''readram'', start_address, length). Start address should between 0 and 65535.')
    end
    if ~isnumeric(param2) || length(param2)~=1 || ~(param2>=1 && param2 <=256 && param+param2 <= 65536)
        error('Length invalid when calling ee405(''readram'', start_address, length).')
    end
    block_callback(idx)=1;
    ram{idx}=[];
    start_addr_ram(idx)=param;
    len_ram(idx)=param2;
    b=packetize(seq(idx),50,[bitand(start_addr_ram(idx),255) bitand(bitshift(start_addr_ram(idx),-8),255) len_ram(idx)]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'say')
    % Plays audio file 'filename.wav'.
    % If there's something being played already, this is ignored.
    if isempty(param)
        error('File name not specified when calling ee405(''say'',file_name)')
    end
    if ~ischar(param)
        error('File name invalid when calling ee405(''say'',file_name)')
    end
    if length(param)>12
        error('File name too long when calling ee405(''say'',file_name)')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 10, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'sayqueue')
    % Plays audio file 'filename.wav'.
    % If there's something being played already, put it in the play list queue.
    if isempty(param)
        error('File name not specified when calling ee405(''sayqueue'',file_name)')
    end
    if ~ischar(param)
        error('File name invalid when calling ee405(''sayqueue'',file_name)')
    end
    if length(param)>12
        error('File name too long when calling ee405(''sayqueue'',file_name)')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 11, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'saynum')
    % usage) ee405('saynum',x): say an integer x, where -2^31 <= x <= 2^31-1
    %          If there's something being played already, put it in the play list queue.
    if isempty(param)
        error('Number not specified when calling ee405(''saynum'', number)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=-2^31 && param <= 2^31-1)
        error('Number invalid or out of range when calling ee405(''saynum'', number). ''number'' should be between -2^31 and 2^31-1.')
    end
    k=int32(param);
    block_callback(idx)=1;
    b=packetize(seq(idx),15,[bitand(k,255) bitand(bitshift(k,-8),255)  bitand(bitshift(k,-16),255)  bitand(bitshift(k,-24),255)]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'saytime')
    block_callback(idx)=1;
    b=packetize(seq(idx),13,[]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'setvolume')
    if isempty(param)
        error('Volume not specified when calling ee405(''setvolume'', volume)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=-999 || param <=999)
        error('Volume invalid or out of range when calling ee405(''setvolume'', volume). ''volume'' should be between 0(loudest) and 3(quietest).')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),16,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'sensorperiod')
    if isempty(param)
        error('Period not specified when calling ee405(''sensorperiod'', period)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=1 || param <=255)
        error('Period invalid or out of range when calling ee405(''sensorperiod'', period). ''period'' should be between 1 and 255.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),88,param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'servo')
    % Usage 1) ee405('servo',id,pos) or ee405('servo',id,pos,speed) for controlling the angle of a single servo motor in normal mode
    % Usage 2) ee405('servo',[id1 id2 ... id_n],[pos1 pos2 ... pos_n]) or
    %          ee405('servo',[id1 id2 ... id_n],[pos1 pos2 ... pos_n],[speed1 speed2 ... speed_n]) for
    %            controlling the angles of multiple servos in normal mode
    % pos is from 0 to 1023
    % speed is from -1023 to 1023.
    servo_help=sprintf(['Usage 1: ee405(''servo'',id,pos) or ee405(''servo'',id,pos,speed)\n'...
                        '           for controlling the angle of a single servo motor in normal mode\n'...
                        'Usage 2: ee405(''servo'',[id1 id2 ... id_n],[pos1 pos2 ... pos_n]) or\n'...
                        '         ee405(''servo'',[id1 id2 ... id_n],[pos1 pos2 ... pos_n],[speed1 speed2 ... speed_n])\n'...
                        '           for controlling the angles of multiple servos in normal mode']);
    if norm(size(param)-size(param2))>0 || (~isempty(param3) && norm(size(param)-size(param3))>0)
        error([sprintf('Invalid parameters. Correct usage is\n') servo_help])
    end
    if isempty(param) || isempty(param2)
        error(servo_help)
    end
    servo_id=param;
    lparam=length(param);
    speed=zeros(1,lparam);
    if ~isempty(param3)
        speed=param3;
        speed=1023*(speed>1023)+speed.*(speed<=1023).*(speed>=0);
    end
    pos=1023*(param2>1023)+param2.*(param2<=1023).*(param2>=0);
    pos=int16(pos);
    speed=int16(speed);
    if lparam==1
        block_callback(idx)=1;
        b=packetize(seq(idx),22,[servo_id 30 bitand(pos, 255) bitand(bitshift(pos,-8),255) bitand(speed, 255) bitand(bitshift(speed,-8),255)]);
        block_callback(idx)=0;
    elseif lparam>1
        servo_id=reshape(servo_id,1,lparam);
        pos=reshape(pos,1,lparam);
        speed=reshape(speed,1,lparam);
        dservo=zeros(1,5*lparam);
        for k=1:lparam
            dservo(k*5-4)=servo_id(k);
            dservo(k*5-3)=bitand(pos(k), 255);
            dservo(k*5-2)=bitand(bitshift(pos(k),-8),255);
            dservo(k*5-1)=bitand(speed(k), 255);
            dservo(k*5)=bitand(bitshift(speed(k),-8),255);
        end
        block_callback(idx)=1;
        b=packetize(seq(idx),23,[lparam 30 dservo]);
        block_callback(idx)=0;
    end
elseif strcmpi(despace(cmd),'servomode')
    % returns the mode (wheel or normal) of the servo with id
    % return valus is a string
    %   'normal' if normal mode
    %   'wheel' if wheel mode
    %   'no response' if there is no response from the servo
    if isempty(param)
        error('Servo id not specified when calling ee405(''servomode'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''servomode'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    servo_mode=[];
    servo_id_monitor=param;
    b=packetize(seq(idx),20,[servo_id_monitor 8 2]);
    block_callback(idx)=0;
    insert_tx_queue(b);
    
    % added on 2016/03/28
    % retransmission may be needed because ee405 board may not return 
    % the result when servo monitor is running.
    for n_retries=1:10
        for k=1:10
            if ~isempty(servo_mode)
                varargout{1}=servo_mode;
                return;
            end
            pause(0.016)
        end
        block_callback(idx)=1;
        b=packetize(seq(idx),20,[servo_id_monitor 8 2]);
        block_callback(idx)=0;
        insert_tx_queue(b);
    end
    error('Time out. Try again.')   
elseif strcmpi(despace(cmd),'servomonitor')
    % start monitoring servo status (position, speed, load)
    % ids is an 1xn vector containing n id's of servo
    if isempty(param)
        error('Servo id(s) not specified when calling ee405(''servomonitor'', [servo_id1 servo_id2 ...])')
    end
    if ~isnumeric(param) ||  ~(min(param)>=0 && max(param) < 254)
        error('Servo id(s) invalid when calling ee405(''servomonitor'', [servo_id1 servo_id2 ...]). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),21,[36 6 reshape(param,1,length(param))]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'servonormalmode')
    % change to wheel mode for servo id
    if isempty(param)
        error('Servo id not specified when calling ee405(''servonormalmode'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''servonormalmode'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),22,[param 8 255 3]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'servospeed')
    % Usage) ee405('servospeed',id,speed) for controlling the speed of a single servo motor in wheel mode
    %     or ee405('servospeed',[id1 id2 ... id_n],[speed1 speed2 ... speed_n]) for
    %          controlling the speeds of multiple servos in wheel mode
    % speed is from 0 to 1023.
    servo_help=sprintf(['Usage 1: ee405(''servospeed'',id,speed)\n'...
                        '           for controlling the speed of a single servo motor in normal mode\n'...
                        'Usage 2: ee405(''servospeed'',[id1 id2 ... id_n],[speed1 speed2 ... speed_n])\n'...
                        '           for controlling the speed of multiple servos in normal mode']);
    if norm(size(param)-size(param2))>0
        error([sprintf('Invalid parameters. Correct usage is\n') servo_help])
    end
    if isempty(param) || isempty(param2)
        error(servo_help)
    end
    speed = 1023 * (param2 > 1023) - 1023 * (param2 < -1023) + param2.*(param2 >= -1023).*(param2 <= 1023);
    pol=int16(speed >= 0);
    speed=int16(speed);
    speed=bitor(abs(speed),bitshift(pol, 10));
    servo_id=param;
    lparam=length(param);
    if lparam==1
        block_callback(idx)=1;
        b=packetize(seq(idx),22,[servo_id 32 bitand(speed, 255) bitand(bitshift(speed,-8),255)]);
        block_callback(idx)=0;
    elseif lparam>1
        servo_id=reshape(servo_id,1,lparam);
        speed=reshape(speed,1,lparam);
        dservo=zeros(1,3*lparam);
        for k=1:lparam
            dservo(k*3-2)=servo_id(k);
            dservo(k*3-1)=bitand(speed(k), 255);
            dservo(k*3)=bitand(bitshift(speed(k),-8),255);
        end
        block_callback(idx)=1;
        b=packetize(seq(idx),23,[lparam 32 dservo]);
        block_callback(idx)=0;
    end
elseif strcmpi(despace(cmd),'servostatus')
    % returns servo status
    if isempty(param)
        error('Servo id not specified when calling ee405(''servostatus'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''servostatus'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    servo_status=[];
    servo_id_monitor=param;
    b=packetize(seq(idx),20,[servo_id_monitor 0 50]);
    block_callback(idx)=0;
    insert_tx_queue(b);

    % added on 2016/03/28
    % retransmission may be needed because ee405 board may not return 
    % the result when servo monitor is running.
    for n_retries=1:10
        for k=1:10
            if ~isempty(servo_status)
                varargout{1}=servo_status;
                return;
            end
            pause(0.016)
        end
        block_callback(idx)=1;
        b=packetize(seq(idx),20,[servo_id_monitor 0 50]);
        block_callback(idx)=0;
        insert_tx_queue(b);
    end
    error('Time out. Try again.')
elseif strcmpi(despace(cmd),'servowheelmode')
    % change to wheel mode for servo id
    if isempty(param)
        error('Servo id not specified when calling ee405(''servowheelmode'', servo_id). Servo id should be between 0 and 253.')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
        error('Servo id invalid when calling ee405(''servowheelmode'', servo_id). Servo id should be between 0 and 253.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx),22,[param 8 0 0]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'setservoadvancedmode')
    servo_advance_mode=param;
    return
elseif strcmpi(despace(cmd),'setservoid')
    if servo_advanced_mode
        % Change the id of the servo to 'id'.
        if isempty(param)
            error('Servo id not specified when calling ee405(''setservoid'', servo_id). Servo id should be between 0 and 253.')
        end
        if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
            error('Servo id invalid when calling ee405(''setservoid'', servo_id). Servo id should be between 0 and 253.')
        end
        block_callback(idx)=1;
        b=packetize(seq(idx),28,[254 3 param]);
        block_callback(idx)=0;
    else
        error('Command not allowed')
    end
elseif strcmpi(despace(cmd),'setservorate')
    if servo_advanced_mode
        % Change the baud rate of the servo from 1Mbps to 115200bps.
        % There should be only one servo motor connected.
        block_callback(idx)=1;
        b=packetize(seq(idx),27,[]);
        block_callback(idx)=0;
    else
        error('Command not allowed')
    end
elseif strcmpi(despace(cmd),'initservo')
    if servo_advanced_mode
        % Change the baud rate to 115200bps and change the id of the servo to 'id'.
        if isempty(param)
            error('Servo id not specified when calling ee405(''initservo'', servo_id). Servo id should be between 0 and 253.')
        end
        if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 254)
            error('Servo id invalid when calling ee405(''initservo'', servo_id). Servo id should be between 0 and 253.')
        end
        id_to_init=param;
        ee405(id_requested,'setservorate',id_to_init);
        pause(1)
        ee405(id_requested,'setservoid',id_to_init);
        pause(0.2)
        ee405(id_requested,'servostatus',id_to_init)
        ee405(id_requested,'servo',id_to_init,0);
        pause(1)
        ee405(id_requested,'servo',id_to_init,512);
        return
    else
        error('Command not allowed')
    end
elseif strcmpi(despace(cmd),'stopaudio')
    block_callback(idx)=1;
    b=packetize(seq(idx), 12, []);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'testtx')
    % force the board to transmit dummy zigbee rx packets to the host
    % param=total # of packets
    % param2 = # of packets in each transmission at 32Hz
    % param3 = length of each packet in bytes
    if isempty(param) || isempty(param2) || isempty(param3)
        error('Parameters not specified when calling ee405(''testtx'', total_packets, npackets_per_slot, length)')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 1, [bitand(uint32(param),255), bitand(bitshift(uint32(param),-8),255), param2, param3]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'testzigbeetx')
    % force the board to transmit dummy zigbee packets to dest
    % param = dest. addr.
    % param2 = total # of packets
    % param3 = # of packets in each transmission at 32Hz
    % param4 = length of each packet in bytes
    if isempty(param) || isempty(param2) || isempty(param3) || isempty(param4)
        error('Parameters not specified when calling ee405(''testzigbeetx'', dest_addr, total_packets, npackets_per_slot, length)')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 2, [bitand(uint32(param),255), bitand(bitshift(uint32(param),-8),255), bitand(uint32(param2),255), bitand(bitshift(uint32(param2),-8),255), param3, param4]);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbee')
    % transmits a zigbee data to dest
    zn=length(param2);
    if zn > 114
        error('Data too big when calling ee405(''zigbee'', dest_addr, [byte1 byte2 ...]). Max. payload size is 114 bytes.')
        % due to additional crc32, max. payload size is 114 bytes (since ver. 7)
    end
    if isempty(param)
        error('Destination address not specified when calling ee405(''zigbee'', dest_addr, [byte1 byte2 ...])')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 65536)
        error('Destination address invalid when calling ee405(''zigbee'', dest_addr, [byte1 byte2 ...]). Destination address should be between 0 and 65535.')
    end
    zd=zeros(1,2+zn);
    zd(1)=bitand(param, 255);
    zd(2)=bitand(bitshift(param,-8),255);
    if zn>0
        zd(3:end)=reshape(param2,1,zn);
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 30, zd);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeerate')
    % change the zigbee phy rate
    if isempty(param)
        error('Rate not specified when calling ee405(''zigbeerate'', rate)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param < 4)
        error('Rate invalid when calling ee405(''zigbeerate'', rate). ''rate'' should be 0(250 kbps), 1(500 kbps), 2(1000kbps), or 3(2000kbps).')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 250, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeechannel')
    % change the zigbee channel
    if isempty(param)
        error('Channel not specified when calling ee405(''zigbeechannel'', channel)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=11 && param <= 26)
        error('Channel invalid when calling ee405(''zigbeechannel'', channel). ''channel'' should be between 11 and 26.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 34, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeemode')
    % change the zigbee mode (promiscuous or normal)
    if isempty(param)
        error('Mode not specified when calling ee405(''zigbeemode'', mode)')
    end
    if ~(strcmp(param,'promiscuous') || strcmp(param,'normal'))
        error('Mode invalid when calling ee405(''zigbeemode'', mode). ''mode'' should be ''promiscuous'' or ''normal''.')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 36, strcmp(param,'promiscuous'));
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeestatus')
    % returns zigbee status
    block_callback(idx)=1;
    b=packetize(seq(idx), 38,[]);
    zigbee_status=[];
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeetxpower')
    % change the transmit power
    % mapping between pwr (0 ~ 15) and tx power in dBm
    % pwr   dBm
    %===========
    %   0    3
    %   1    2.8
    %   2    2.3
    %   3    1.8
    %   4    1.3
    %   5    0.7
    %   6    0
    %   7    -1
    %   8    -2
    %   9    -3
    %  10    -4
    %  11    -5
    %  12    -7
    %  13    -9
    %  14    -12
    %  15    -17
    if isempty(param)
        error('''power'' not specified when calling ee405(''zigbeetxpower'', power). ''power'' should be between 0(3dBm) and 15(-17dBm).')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param <= 15)
        error('''power'' invalid when calling ee405(''zigbeetxpower'', power). ''power'' should be between 0(3dBm) and 15(-17dBm).')
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 35, param);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeetxtest')    
    % Force the board to transmit sensor data repeatedly (every 20msec)
    % after the board is powered with battery & joy stick == left while powering up.
    % This is disabled when the board is power by USB.
    % Usage) ee405('zigbeetxtest', dest_addr, data, channel, power)
    % Channel will be 11 if not specified.
    % Power will be 0 (3dBm) if not specified.
    % If data is not specified or data==[], then sensor reading will be transmitted
    %   (accelerometer, magnetic sensor, humidity, temperature, light, joy_stick).
    % Use ee405('zigbeesensordata',source_id) to get sensor data from the received zigbee packet.
    if isempty(param)
        error(sprintf(['Parameter(s) not specified when calling ee405(''zigbeetxtest'', dest_addr, data, channel, power).\n'...
                       '  ''data'', ''channel'', ''power'' are optional. If ''data'' is not specified, sensor data will be transmitted.']))
    end
    zigbeetxtest_data=param2;
    zigbeetxtest_data_len=length(zigbeetxtest_data);
    zigbeetxtest_buff=zeros(1,4+zigbeetxtest_data_len);
    zigbeetxtest_buff(1)=bitand(param, 255);
    zigbeetxtest_buff(2)=bitand(bitshift(param,-8),255);
    if ~isempty(param3)
        zigbeetxtest_buff(3)=param3;
    else
        zigbeetxtest_buff(3)=11;
    end
    if ~isempty(param4)
        zigbeetxtest_buff(4)=param4;
    else
        zigbeetxtest_buff(4)=0;
    end
    if zigbeetxtest_data_len>0
        zigbeetxtest_buff(5:end)=reshape(zigbeetxtest_data,1,zigbeetxtest_data_len);
    end
    block_callback(idx)=1;
    b=packetize(seq(idx), 39, zigbeetxtest_buff);
    block_callback(idx)=0;
elseif strcmpi(despace(cmd),'zigbeesensordata')
    % get sensor data transmitted by another board that is configured to
    % transmit sensor data periodically using ee405('zigbeetxtest').
    % need to specify the source id
    if isempty(param)
        error('Source id not specified when calling ee405(''zigbeesensordata'', source_id)')
    end
    if ~isnumeric(param) || length(param)~=1 || ~(param>=0 && param <= 15)
        error('Source id invalid when calling ee405(''zigbeesensordata'', source_id)')
    end
    zigbee_sensor_data_source_id=param;
    t0=clock;
    while 1
        block_callback(idx)=1;
        zigbee_sensor_data_saved=double(zigbee_sensor_data);
        zigbee_sensor_data=[];
        block_callback(idx)=0;
        if length(zigbee_sensor_data_saved)>=15
            varargout{1}.acc=(zigbee_sensor_data_saved(1:3)-32) / 32 * 1.5;      % 32 means 1.5g
            varargout{1}.mag=[zigbee_sensor_data_saved(4)+zigbee_sensor_data_saved(5)*256 ...
                zigbee_sensor_data_saved(6)+zigbee_sensor_data_saved(7)*256 ...
                zigbee_sensor_data_saved(8)+zigbee_sensor_data_saved(9)*256];
            varargout{1}.mag=varargout{1}.mag-65536*(varargout{1}.mag>=32768);   % conversion for 2's complement
            varargout{1}.mag=varargout{1}.mag/600;        % convert to gauss
            varargout{1}.hum=zigbee_sensor_data_saved(10)+zigbee_sensor_data_saved(11)*256;
            if varargout{1}.hum >= 32768   % conversion for 2's complement
                varargout{1}.hum = varargout{1}.hum - 65536;
            end
            varargout{1}.hum = varargout{1}.hum / 10;
            varargout{1}.temp=zigbee_sensor_data_saved(12)+zigbee_sensor_data_saved(13)*256;
            if varargout{1}.temp >= 32768   % conversion for 2's complement
                varargout{1}.temp = varargout{1}.temp - 65536;
            end
            varargout{1}.temp = varargout{1}.temp / 10;
            varargout{1}.light = zigbee_sensor_data_saved(14);
            varargout{1}.joy_stick=bitand(zigbee_sensor_data_saved(15),7);
            return
        end
        if etime(clock,t0)>1
            error('No response from the source board')
        end
        pause(0.01)
    end
else
    error('Unrecognized command')
end

insert_tx_queue(b);

% process commands that need to return something
% servo-related processing is done above (servostatus, servomode)
if strcmpi(despace(cmd),'gettime')
    name_of_days=['sun';'mon';'tue';'wed';'thu';'fri';'sat'];
    for k=1:30
        if ~isempty(board_time{idx})
            varargout{1}.year=board_time{idx}(1)+2000;
            varargout{1}.month=board_time{idx}(2);
            varargout{1}.date=board_time{idx}(3);
            varargout{1}.day=name_of_days(board_time{idx}(4),:);
            varargout{1}.hour=board_time{idx}(5);
            varargout{1}.min=board_time{idx}(6);
            varargout{1}.sec=board_time{idx}(7);
            return
        end
        pause(0.016)
    end
    error('Time out. Try again.')
elseif strcmpi(despace(cmd),'readram')
    for k=1:30
        if ~isempty(ram{idx})
            varargout{1}=ram{idx};
            return
        end
        pause(0.016)
    end
    error('Time out. Try again.')
elseif strcmpi(despace(cmd),'zigbeestatus')
    for k=1:30
        if ~isempty(zigbee_status)
            varargout{1}=zigbee_status;
            return;
        end
        pause(0.016)
    end
    error('Time out. Try again.')
end


return

    function init(varargin)            % do not initialize nb, s, port_name, board_time, ram here
        if nargin==0
            max_payload_size = 1500;
            min_length=32;      % minimum possible packet length from the board

            max_nboards = 3;    % max. # of ee405 boards that can be supported
            max_tx_queue_packets=100;
            max_zigbee_packets=10000;
            max_zigbee_packet_size=121;
            max_midi_packets=10000;
            max_midi_packet_size=3;

            input_buffer_size=4096;
            output_buffer_size=4096;

            print_debug_info_once=0;
            
            state=zeros(max_nboards,1);
            in_callback=[];
            d=zeros(max_nboards,max_payload_size);
            t=zeros(max_nboards,6);
            n=zeros(max_nboards,1);
            id=zeros(max_nboards,1);
            received=zeros(max_nboards,1);
            last_seq_rx=-ones(max_nboards,1);
            seq_skipped=zeros(max_nboards,1);
            tx_queue=zeros(max_nboards,max_tx_queue_packets,max_payload_size+8,'uint8');
            tx_queue_size=zeros(max_nboards,max_tx_queue_packets);
            tx_queue_tot_size=zeros(max_nboards,1);
            tx_queue_seq=zeros(max_nboards,max_tx_queue_packets);
            tx_queue_h=ones(max_nboards,1);
            tx_queue_t=ones(max_nboards,1);
            tx_queue_c=ones(max_nboards,1);
            tx_queue_time_stamp=zeros(max_nboards,max_tx_queue_packets);
            tx_retx_count=zeros(max_nboards,2);
            seq=ones(max_nboards,1);    % seq. # starts from 1
            gps_data=[];
            zigbee_packets=zeros(max_nboards,max_zigbee_packets,max_zigbee_packet_size);
            zigbee_h=ones(max_nboards,1);
            zigbee_t=ones(max_nboards,1);
            midi_packets=zeros(max_midi_packets,max_midi_packet_size);
            midi_h=1;
            midi_t=1;
            last_reported_seq=-ones(max_nboards,1);
            servo_pos=-ones(256,1);
            servo_speed=zeros(256,1);
            servo_load=zeros(256,1);
            servo_mode=[];
            servo_status=[];
            start_addr_ram=zeros(max_nboards,1);
            len_ram=zeros(max_nboards,1);
            ss_seq=zeros(max_nboards,1);
            ss_last=zeros(max_nboards,1);
            max_tx_seq=zeros(max_nboards,1);
            block_callback=zeros(max_nboards,1);    % do this last, just in case           
        else  % in_callback, block_callback, id, midi are not changed
            idx_to_init=varargin{1};
            state(idx_to_init)=0;
            d(idx_to_init,:)=zeros(1,max_payload_size);
            t(idx_to_init,:)=zeros(1,6);
            n(idx_to_init)=0;
            received(idx_to_init)=0;
            last_seq_rx(idx_to_init)=-1;
            seq_skipped(idx_to_init)=0;
            tx_queue(idx_to_init,:,:)=zeros(1,max_tx_queue_packets,max_payload_size+8,'uint8');
            tx_queue_size(idx_to_init,:)=zeros(1,max_tx_queue_packets);
            tx_queue_tot_size(idx_to_init)=0;
            tx_queue_seq(idx_to_init,:)=zeros(1,max_tx_queue_packets);
            tx_queue_h(idx_to_init)=1;
            tx_queue_t(idx_to_init)=1;
            tx_queue_c(idx_to_init)=1;
            tx_queue_time_stamp(idx_to_init,:)=zeros(1,max_tx_queue_packets);
            tx_retx_count(idx_to_init,:)=zeros(1,2);
            seq(idx_to_init)=1;
            zigbee_packets(idx_to_init,:,:)=zeros(1,max_zigbee_packets,max_zigbee_packet_size);
            zigbee_h(idx_to_init)=1;
            zigbee_t(idx_to_init)=1;
            last_reported_seq(idx_to_init)=-1;
            ss_seq(idx_to_init)=0;
            ss_last(idx_to_init)=0;
            max_tx_seq(idx_to_init)=0;
        end
    end         % end of function init

    function wait_for_id(varargin)
        if nargin>=1
            kws=varargin{1};
        else
            kws=1:nb;
        end
        t0=clock;
        pclosed=0;
        for kw=kws
            if ~isempty(s{kw})
                while id(kw)==0
                    pause(0.01)
                    if etime(clock,t0)>2
                        port_name_old=port_name{kw};
                        block_callback(kw)=1;
                        try
                            fclose(s{kw})
                        catch
                        end
                        s{kw}=[];            % disable it
                        port_name{kw}='';
                        block_callback(kw)=0;
                        pclosed=1;
                        disp(sprintf('No response from port %s.', port_name_old));
                    end
                end
            end
        end
        if pclosed
            error('Port(s) closed')
        end
    end

    function insert_tx_queue(pkt_to_send)

        if strcmp(s{idx}.Status,'closed')
            error('Cannot send a command. Port is currently closed.')
        end

        packet_length=length(pkt_to_send);
        if packet_length-12>max_payload_size
            error('Data size too big. Cannot be transmitted to the board.');
        end

        %'check'
        %[tx_queue_h(idx) tx_queue_t(idx) tx_queue_c(idx) tx_queue_time_stamp(idx)]
        %[tx_queue_seq(idx, previous(tx_queue_h(idx))) tx_queue_seq(idx, previous(tx_queue_t(idx))) tx_queue_seq(idx, previous(tx_queue_c(idx))) ]
        %[tx_queue_tot_size(idx) last_seq_rx(idx) seq_skipped(idx)]
        %d_decoded{idx}.rx_buffer_free
        % if tx_queue_tot_size(idx)>=1000
        %     'tx queue total size exceeded 1000'
        %     [tx_queue_h(idx) tx_queue_t(idx) tx_queue_c(idx) tx_queue_time_stamp(idx)]
        %     [tx_queue_seq(idx, previous(tx_queue_h(idx))) tx_queue_seq(idx, previous(tx_queue_t(idx))) tx_queue_seq(idx, previous(tx_queue_c(idx))) ]
        %     [tx_queue_tot_size(idx) last_seq_rx(idx) seq_skipped(idx)]
        %     [d_decoded{idx}.rx_buffer_free d_decoded{idx}.seq d_decoded{idx}.last_seq]
        %     %print_debug_info=1;
        %     %print_debug_info2=1;
        % end

        tx_queue_h2=tx_queue_h(idx)+1;
        if tx_queue_h2 > max_tx_queue_packets
            tx_queue_h2=1;
        end
        t0=clock;
        while (tx_queue_h2==tx_queue_c(idx)) || (tx_queue_tot_size(idx)>=4096)    % wait if tx_queue is full (there may be non-transmitted packets or non-confirmed packets) or too much data
        %    block_callback(idx)=1;
        %    disp('time out error')
        %    while 1
        %        seq(idx)=seq(idx)+1;
        %        b=packetize(seq(idx), 10, double('1.wav'));
        %        transmit(s{idx}, b, output_buffer_size);
        %        beep
        %        pause(2)
        %    end
            pause(0.01);
            if etime(clock,t0)>2
                'timeout'
                [tx_queue_h(idx) tx_queue_t(idx) tx_queue_c(idx) tx_queue_time_stamp(idx)]
                [tx_queue_seq(idx, previous(tx_queue_h(idx))) tx_queue_seq(idx, previous(tx_queue_t(idx))) tx_queue_seq(idx, previous(tx_queue_c(idx))) ]
                tx_queue_tot_size(idx)
                last_seq_rx(idx)
                seq_skipped(idx)
                t0=clock;
                return     % added on 2016/3/28
            end
        end

        tx_queue(idx,tx_queue_h(idx),1:packet_length)=pkt_to_send;
        tx_queue_size(idx,tx_queue_h(idx))=packet_length;
        tx_queue_tot_size(idx)=tx_queue_tot_size(idx)+packet_length;
        tx_queue_seq(idx,tx_queue_h(idx))=seq(idx);
        tx_queue_h(idx)=tx_queue_h2;

        seq(idx)=seq(idx)+1;
    end

    function cleanupFun(tobj)
%         p=0;
%         try
%             for k=1:nb
%                 if ~isempty(s{k})
%                     fclose(s{k})
%                     p=1;
%                 end
%             end
%             if p
%                 disp('Port(s) closed')
%             end
%         catch
%         end
        stop(tobj)
        delete(tobj)
        disp('Timer deleted')
    end
    
    function timer_callback(object, event)
        last_timer_event=clock;
        
        if ~isempty(in_callback)
            %disp(sprintf('Re-entrance'))
            %pause
            return
        end
        in_callback=1;
        
        persistent cnt crc dr nr 
        
        % the following are defined as persistent not to overwrite global variables - because this is a nested function
        persistent bb cc buff klocal cr rr zr mr gr
        persistent klocal2 midi_h2 zigbee_h2 ls ss rf tt tc
        persistent slocal klocalmain e servo_id_returned
        persistent servo_pos_returned servo_speed_returned servo_load_returned

        if isempty(cnt)
            cnt=zeros(max_nboards,1);
            crc=zeros(max_nboards,1);
            dr=zeros(max_nboards,max_payload_size);
            nr=zeros(max_nboards,1);
        end

        for klocalmain=1:nb
            if block_callback(klocalmain)
                continue
            end
            slocal=s{klocalmain};
            if isempty(slocal)
                continue
            end
            if ~strcmp(slocal.Status,'open')
                continue
            end
            cc=slocal.BytesAvailable;
            if cc==0
                continue;
            end
            [bb,cc]=fread(slocal, cc, 'uint8');
            buff=double(bb);
            
            for klocal=1:cc
                cr=buff(klocal);
                if state(klocalmain)==0
                    if cr==154       % 0x9a
                        state(klocalmain)=state(klocalmain)+1;   % this may not happen due to padding with 0xff at the end of each packet
                    end
                elseif state(klocalmain)==1
                    if cr==43        % 0x2b
                        state(klocalmain)=state(klocalmain)+1;
                    else
                        if print_debug_info
                            disp('bad header')
                            %pause
                        end
                        state(klocalmain)=0;
                    end
                elseif state(klocalmain)==2
                    nr(klocalmain)=cr;
                    state(klocalmain)=state(klocalmain)+1;
                elseif state(klocalmain)==3
                    nr(klocalmain)=nr(klocalmain)+cr*256;
                    if (nr(klocalmain) > max_payload_size) || (nr(klocalmain) < min_length - 8)
                        if print_debug_info
                            disp('wrong length')
                            %pause
                        end
                        state(klocalmain)=0;
                    else
                        cnt(klocalmain)=0;
                        state(klocalmain)=state(klocalmain)+1;
                    end
                elseif state(klocalmain)==4
                    cnt(klocalmain)=cnt(klocalmain)+1;
                    dr(klocalmain,cnt(klocalmain))=cr;
                    if cnt(klocalmain)==nr(klocalmain)
                        state(klocalmain)=state(klocalmain)+1;
                    end
                elseif state(klocalmain)==5
                    crc(klocalmain)=cr;
                    state(klocalmain)=state(klocalmain)+1;
                elseif state(klocalmain)==6
                    crc(klocalmain)=crc(klocalmain)+cr * 256;
                    state(klocalmain)=state(klocalmain)+1;
                elseif state(klocalmain)==7
                    crc(klocalmain)=crc(klocalmain)+cr * 256^2;
                    state(klocalmain)=state(klocalmain)+1;
                elseif state(klocalmain)==8
                    crc(klocalmain)=crc(klocalmain)+cr * 256^3;
                    if crc(klocalmain)==crc32(dr(klocalmain,1:nr(klocalmain)))
                        d(klocalmain,:)=dr(klocalmain,:);
                        t(klocalmain,:)=clock;
                        n(klocalmain)=nr(klocalmain);
                        id(klocalmain)=dr(klocalmain,5);
                        received(klocalmain)=1;
                        %disp(sprintf('Length = %d', n(klocalmain)))
                        %dr(klocalmain,1)
                        [rr, zr, mr, gr, ls, ss]=decode_message(dr(klocalmain,1:nr(klocalmain)));
                        d_decoded{klocalmain}=rr;
                        if ~isfield(rr,'ack')
                            rr.ack=[];  % for printing ack info during debugging
                        end
                        if isfield(rr,'time')
                            board_time{klocalmain}=rr.time;
                        end
                        if isfield(rr,'ram')
                            if rr.ram(1) + 256 * rr.ram(2) == start_addr_ram(klocalmain)
                                if rr.ram(3) == len_ram(klocalmain)
                                    ram{klocalmain}=rr.ram(4:end);
                                end
                            end
                        end
                        if isfield(rr,'servo')
                            for klocal2=1:length(rr.servo)
                                % process periodic data from servos (pos, speed, load)
                                servo_id_returned=rr.servo{klocal2}(1);
                                if length(rr.servo{klocal2})-3==6           % periodic data
                                    if rr.servo{klocal2}(3)==36             % starting address==36
                                        e=double(rr.servo{klocal2});
                                        servo_pos_returned=e(4)+256*e(5);
                                        servo_speed_returned=e(6)+256*e(7);
                                        if bitget(servo_speed_returned,11)
                                            servo_speed_returned=bitand(servo_speed_returned,1023);
                                        else
                                            servo_speed_returned=-bitand(servo_speed_returned,1023);
                                        end
                                        servo_load_returned=e(8)+256*e(9);
                                        if bitget(servo_load_returned,11)
                                            servo_load_returned=bitand(servo_load_returned,1023);
                                        else
                                            servo_load_returned=-bitand(servo_load_returned,1023);
                                        end
                                        servo_pos(servo_id_returned+1)=servo_pos_returned;
                                        servo_speed(servo_id_returned+1)=servo_speed_returned;
                                        servo_load(servo_id_returned+1)=servo_load_returned;
                                    end
                                end
                                % now, process servo_mode & servo_status
                                if servo_id_returned ~= servo_id_monitor
                                    continue;
                                end
                                if length(rr.servo{klocal2})==1
                                    if isempty(servo_mode)
                                        servo_mode='no response';
                                    end
                                    if isempty(servo_status)
                                        servo_status.id=servo_id_returned;
                                        servo_status.errmsg='No response from the specified servo';
                                    end
                                    continue;
                                elseif (length(rr.servo{klocal2})-3==2)     % response to servo mode inquiry
                                    if (rr.servo{klocal2}(3)~=8)            % starting address should be 8
                                        continue;
                                    end
                                    if (rr.servo{klocal2}(4)==0) && (rr.servo{klocal2}(5)==0)
                                        servo_mode='wheel';
                                    else
                                        servo_mode='normal';
                                    end
                                    continue;
                                elseif (length(rr.servo{klocal2})-3==50)    % response to servo status inquiry
                                    if (rr.servo{klocal2}(3)~=0)            % starting address should be 0
                                        continue;
                                    end
                                    servo_status.id=rr.servo{klocal2}(1);
                                    servo_status.err=rr.servo{klocal2}(2);
                                    e=double(rr.servo{klocal2}(5:end));
                                    servo_status.model=rr.servo{klocal2}(4) + 256*rr.servo{klocal2}(5);
                                    servo_status.version=e(2);
                                    servo_status.stored_id=e(3);
                                    servo_status.baudrate=e(4);
                                    servo_status.return_delay_time=e(5);
                                    servo_status.cw_angle_limit=e(6)+e(7)*256;
                                    servo_status.ccw_angle_limit=e(8)+e(9)*256;
                                    servo_status.temperature_limit_high=e(11);
                                    servo_status.voltage_limit_low=e(12);
                                    servo_status.voltage_limit_high=e(13);
                                    servo_status.max_torque=e(14)+256*e(15);
                                    servo_status.status_return_level=e(16);
                                    servo_status.alarm_led=e(17);
                                    servo_status.alarm_shutdown=e(18);
                                    servo_status.down_calibration=e(20)+256*e(21);
                                    servo_status.up_calibration=e(22)+256*e(23);
                                    servo_status.torque_enable=e(24);
                                    servo_status.led=e(25);
                                    servo_status.cw_compliance_margin=e(26);
                                    servo_status.ccw_compliance_margin=e(27);
                                    servo_status.cw_compliance_slope=e(28);
                                    servo_status.ccw_compliance_slope=e(29);
                                    servo_status.goal_position=e(30)+256*e(31);
                                    servo_status.moving_speed=e(32)+256*e(33);
                                    servo_status.torque_limit=e(34)+256*e(35);
                                    servo_status.present_position=e(36)+256*e(37);
                                    servo_status.present_speed=e(38)+256*e(39);
                                    servo_status.present_load=e(40)+256*e(41);
                                    servo_status.present_voltage=e(42);
                                    servo_status.present_temperature=e(43);
                                    servo_status.registered_instruction=e(44);
                                    servo_status.moving=e(46);
                                    servo_status.lock=e(47);
                                    servo_status.punch=e(48)+256*e(49);
                                    continue;
                                end
                            end
                        end
                        if isfield(rr,'zigbee_status')
                            zigbee_status=rr.zigbee_status;
                        end

                        if isempty(ls)
                            ls=0;
                            ss=0;
                        end
                        last_seq_rx(klocalmain)=ls;
                        seq_skipped(klocalmain)=ss;

                        if ~isempty(gr)
                            gps_data=gr;
                        end
                        if ~isempty(mr)
                            for klocal2=1:size(mr,1)
                                midi_h2=midi_h+1;
                                if midi_h2 > max_midi_packets
                                    midi_h2=1;
                                end
                                if midi_h2~=midi_t    % buffer is not full
                                    midi_h=midi_h2;
                                    midi_packets(midi_h,:)=mr(klocal2,:);
                                end
                            end
                        end
                        if ~isempty(zr)
                            for klocal2=1:size(zr,1)
                                if zr(klocal2,1)==zigbee_sensor_data_source_id
                                    zigbee_sensor_data=zr(klocal2,8:end);
                                end
                                zigbee_h2=zigbee_h(klocalmain)+1;
                                if zigbee_h2 > max_zigbee_packets
                                    zigbee_h2=1;
                                end
                                if zigbee_h2~=zigbee_t(klocalmain)    % buffer is not full
                                    zigbee_h(klocalmain)=zigbee_h2;
                                    zigbee_packets(klocalmain,zigbee_h2,:)=zr(klocal2,:);
                                end
                            end
                        end

                        if print_debug_info_once
                            disp('debug info once')
                            klocalmain 
                            [tx_queue_h(klocalmain) tx_queue_t(klocalmain) tx_queue_c(klocalmain)]
                            [tx_queue_seq(klocalmain, tx_queue_h(klocalmain)) tx_queue_seq(klocalmain, tx_queue_t(klocalmain)) tx_queue_seq(klocalmain, tx_queue_c(klocalmain))]
                            [tx_queue_seq(klocalmain, previous(tx_queue_h(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_t(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_c(klocalmain)))]
                            [rr.seq ls ss max_tx_seq(klocalmain)]
                            tx_retx_count(klocalmain,:)
                            rr.ack
                            print_debug_info_once=0;
                        end

                        if ls<=max_tx_seq(klocalmain)
                            tc=tx_queue_c(klocalmain);
                            if ls
                                while (tx_queue_seq(klocalmain, tc)<=ls) && (tc~=tx_queue_h(klocalmain))   % note that ls<=max_tx_seq(klocalmain) holds
                                    tx_queue_tot_size(klocalmain)=tx_queue_tot_size(klocalmain)-tx_queue_size(klocalmain,tc);
                                    tx_queue_time_stamp(klocalmain,tc)=0;
                                    if tc==tx_queue_t(klocalmain)
                                        tx_queue_t(klocalmain)=next(tc);
                                    end
                                    tc=next(tc);
                                    tx_queue_c(klocalmain)=tc;
                                end
                            end

                            if ss && ~ss_last(klocalmain)
                                ss_seq(klocalmain)=ls;      % last successful seq when ss is changed from 0 to 1
                            elseif ~ss
                                ss_seq(klocalmain)=0;
                            end
                            ss_last(klocalmain)=ss;
                            
                            tc=tx_queue_c(klocalmain);
                            if ss_seq(klocalmain)>0
                                if tx_queue_c(klocalmain) ~= tx_queue_h(klocalmain)   % this should be satisfied. just in case
                                    tx_queue_t(klocalmain)=tx_queue_c(klocalmain);
                                    tt=tx_queue_t(klocalmain);
                                    transmit(slocal, zeros(1,16,'uint8')+255, output_buffer_size);  % send 0xff's
                                    transmit(slocal, squeeze(tx_queue(klocalmain,tt,1:tx_queue_size(klocalmain,tt))), output_buffer_size);
                                    tx_queue_time_stamp(klocalmain,tt)=rr.seq;
                                    tt=next(tt);
                                    tx_queue_t(klocalmain)=tt;
                                    %disp('retransmit due to seq. skipped')
                                    tx_retx_count(klocalmain,1)=tx_retx_count(klocalmain,1)+1;
                                end
                            elseif (tc~=tx_queue_h(klocalmain)) && (tx_queue_time_stamp(klocalmain,tc)>0) && (rr.seq>tx_queue_time_stamp(klocalmain,tc)+10+20*(next(tc)~=tx_queue_h(klocalmain))) % time out (shorter time out if no next packet, longer time out if next packet exists because ee405 board will likely to set seq. skipped flag)
                                tx_queue_t(klocalmain)=tx_queue_c(klocalmain);
                                tt=tx_queue_t(klocalmain);
                                transmit(slocal, zeros(1,16,'uint8')+255, output_buffer_size);  % send 0xff's
                                transmit(slocal, squeeze(tx_queue(klocalmain,tt,1:tx_queue_size(klocalmain,tt))), output_buffer_size);
                                tx_queue_time_stamp(klocalmain,tt)=rr.seq;
                                tt=next(tt);
                                tx_queue_t(klocalmain)=tt;
                                %disp('retransmit due to timeout')
                                tx_retx_count(klocalmain,2)=tx_retx_count(klocalmain,2)+1;
                            else
                                rf=rr.rx_buffer_free;
                                tt=tx_queue_t(klocalmain);
                                while 1
                                    if tx_queue_h(klocalmain)~=tt
                                        if rf-tx_queue_size(klocalmain,tt) < 2048  % not enough rx buffer free at the board
                                            if print_debug_info
                                                disp('Not enough rx buffer free')
                                                [klocalmain tx_queue_seq(klocalmain, previous(tx_queue_h(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_t(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_c(klocalmain))) tx_queue_h(klocalmain) tx_queue_t(klocalmain) tx_queue_c(klocalmain) rr.seq ls ss]
                                                rr.ack
                                            end
                                            break;
                                        end
                                        rf=rf-tx_queue_size(klocalmain,tt);
                                        if print_debug_info
                                            disp('sending a packet')
                                            [klocalmain tx_queue_seq(klocalmain, previous(tx_queue_h(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_t(klocalmain))) tx_queue_seq(klocalmain, previous(tx_queue_c(klocalmain))) tx_queue_h(klocalmain) tx_queue_t(klocalmain) tx_queue_c(klocalmain) rr.seq ls ss]
                                            rr.ack
                                        end
                                        transmit(slocal, squeeze(tx_queue(klocalmain,tt,1:tx_queue_size(klocalmain,tt))), output_buffer_size);
                                        if tx_queue_seq(klocalmain, tt) > max_tx_seq(klocalmain)
                                            max_tx_seq(klocalmain)=tx_queue_seq(klocalmain, tt);
                                        end
                                        tx_queue_time_stamp(klocalmain,tt)=rr.seq;
                                        tt=next(tt);
                                        tx_queue_t(klocalmain)=tt;
                                    else
                                        break;
                                    end
                                end
                            end
                        end
                    else
                        if print_debug_info
                            klocalmain
                            object
                            object.BytesAvailable
                            crc(klocalmain)
                            crc32(dr(klocalmain,1:nr(klocalmain)))
                            dr(klocalmain,1:nr(klocalmain))
                            disp('crc error')
                            pause
                        end
                    end
                    state(klocalmain)=0;
                end
            end
        end
        
        in_callback=[];
        
    end         % end of function timer_callback

    function r=previous(x)
        x=x-1;
        if x <= 0
            x=max_tx_queue_packets;
        end
        r=x;
    end         % end of function previous()
    function r=next(x)
        x=x+1;
        if x > max_tx_queue_packets
            x=1;
        end
        r=x;
    end         % end of function next()

end         % end of function ee405

function [r, zigbee, midi, gps, last_seq, seq_skipped]=decode_message(e)
zigbee=[];
midi=[];
gps=[];
last_seq=[];
seq_skipped=0;
tx_power=[3 2.8 2.3 1.8 1.3 0.7 0 -1 -2 -3 -4 -5 -7 -9 -12 -17];    % in dBm
r.id=e(5);
r.ver=e(21) + e(22) * 256;

%         if r.ver < ee405_ver
%             fn=sprintf('ee405_v%d.hex',ee405_ver);
%             if strcmp(firmware(s,fn),'success')
%                 pause(1);   % wait for the board to reboot
%                 r=ee405(s,varargin);
%                 return;
%             else
%                 error('Firmware update failed');
%             end
%         end

r.vcpu=e(23) * 0.02;
r.seq=e(1) + 256 * (e(2) + 256 * (e(3) + 256 * e(4)));
r.acc=(e(6:8)-32) / 32 * 1.5;      % 32 means 1.5g
r.mag=[e(9)+e(10)*256 e(11)+e(12)*256 e(13)+e(14)*256];
r.mag=r.mag-65536*(r.mag>=32768);   % conversion for 2's complement
r.mag=r.mag/600;                    % convert to gauss
r.hum=e(15)+e(16)*256;
if r.hum >= 32768   % conversion for 2's complement
    r.hum = r.hum - 65536;
end
r.hum = r.hum / 10;
r.temp=e(17)+e(18)*256;
if r.temp >= 32768   % conversion for 2's complement
    r.temp = r.temp - 65536;
end
r.temp = r.temp / 10;
r.light = e(19);
r.is_playing=bitget(e(20),1);
r.joy_stick=bitand(bitshift(e(20),-1),7);
r.date_time_valid=bitand(bitshift(e(20),-4),1);
%r.seq_inc = e(24);

k=25;
nservo=0;
nzigbee=0;
nmidi=0;
nack=0;
while k <= length(e)
    if e(k)==0
        len=e(k+1);
        nack=nack+1;
        r.ack(nack)=e(k+2) + 256 * (e(k+3) + 256 * (e(k+4) + 256 * e(k+5)));
        k=k+len+2;
    elseif e(k)==1
        len=e(k+1);
        r.rx_buffer_free=e(k+2) + 256 * e(k+3);
        k=k+len+2;
    elseif e(k)==20
        len=e(k+1);
        nservo=nservo+1;
        r.servo{nservo}=e(k+2:k+2+len-1);
        k=k+len+2;
    elseif e(k)==30
        len=e(k+1);
        nzigbee=nzigbee+1;
%         r.zigbee{nzigbee}.source=e(k+4)+256*e(k+5);
%         r.zigbee{nzigbee}.dest=e(k+2)+256*e(k+3);
%         r.zigbee{nzigbee}.seq=e(k+6);
%         r.zigbee{nzigbee}.lqi=e(k+7);
%         r.zigbee{nzigbee}.rssi=e(k+8)-91;
%         r.zigbee{nzigbee}.time=r.seq;   % seq. number
%         r.zigbee{nzigbee}.data=e(k+9:k+2+len-1);
        zigbee(nzigbee,:)=zeros(1,121);
        zigbee(nzigbee,1)=e(k+4)+256*e(k+5);    % source addr
        zigbee(nzigbee,2)=e(k+2)+256*e(k+3);    % dest addr
        zigbee(nzigbee,3)=e(k+6);               % zigbee seq. number
        zigbee(nzigbee,4)=e(k+7);               % lqi
        zigbee(nzigbee,5)=e(k+8)-91;            % rssi
        zigbee(nzigbee,6)=r.seq;                % seq. number of ee405 packet
        zigbee(nzigbee,7)=len-7;                % length of the packet
        zigbee(nzigbee,8:len)=e(k+9:k+2+len-1);      % data
        k=k+len+2;
    elseif e(k)==31
        len=e(k+1);
        r.zigbee_status.trx_status=e(k+2);
        r.zigbee_status.tx_power=tx_power(e(k+3)+1);
        %r.zigbee_status.rssi=e(k+4);   % for debugging only
        %r.zigbee_status.ed=e(k+5);   % for debugging only
        r.zigbee_status.channel=e(k+6);
        %r.zigbee_status.tx_in_progress=e(k+7);   % for debugging only
        %r.zigbee_status.rx_in_progress=e(k+8);   % for debugging only
        %r.zigbee_status.n_packets_in_queue=e(k+9);   % for debugging only
        k=k+len+2;
    elseif e(k)==40
        len=e(k+1);
        r.time=e(k+2:k+2+len-1);
        k=k+len+2;
    elseif e(k)==50
        len=e(k+1);
        r.ram=e(k+2:k+2+len-1);
        k=k+len+2;
    elseif e(k)==61
        len=e(k+1);
        nmidi=nmidi+1;
%         r.midi{nmidi}.key=e(k+2);
%         r.midi{nmidi}.vol=e(k+3);
%         r.midi{nmidi}.time=(e(k+4) + 256 * (e(k+5) + 256 * (e(k+6) + 256 * e(k+7))))/115200;  % in seconds
        midi(nmidi,:)=zeros(1,3);
        midi(nmidi,1)=e(k+2);   % key
        midi(nmidi,2)=e(k+3);   % vol
        midi(nmidi,3)=(e(k+4) + 256 * (e(k+5) + 256 * (e(k+6) + 256 * e(k+7))))/115200;   % time stamp in seconds
        k=k+len+2;
    elseif e(k)==70
        len=e(k+1);
        gps.h=e(k+2);
        gps.m=e(k+3);
        gps.s=e(k+4)+(e(k+5)+e(k+6)*256)/1000;
        gps.lat_d=e(k+8);
        if (e(k+7)=='S'), gps.lat_d=-gps.lat_d;end
        gps.lat_m=e(k+9);
        gps.lat_s=e(k+10)+(e(k+11)+e(k+12)*256)/1000;
        gps.lon_d=e(k+14);
        if (e(k+13)=='W'), gps.lon_d=-gps.lon_d;end
        gps.lon_m=e(k+15);
        gps.lon_s=e(k+16)+(e(k+17)+e(k+18)*256)/1000;
        gps.alt=e(k+19) + 256 * (e(k+20) + 256 * (e(k+21) + 256 * e(k+22)));
        if e(k+22) >= 128
            gps.alt = gps.alt - 2^32 - (e(k+23)+e(k+24)*256)/1000;
        else
            gps.alt = gps.alt + (e(k+23)+e(k+24)*256)/1000;
        end
        gps.fix=e(k+25);
        gps.nsatellites=e(k+26);
        k=k+len+2;
    elseif e(k)==80    % digital input (ADC- port): 0 or 1 is returned
        len=e(k+1);
        r.digital_input=e(k+2);
        k=k+len+2;
    elseif e(k)==81    % analog input (ADC+ port): 0~1023 is returned
        len=e(k+1);
        r.analog_input=e(k+2);
        k=k+len+2;
    elseif e(k)==86    % photo transistor: 0~255 is returned
        len=e(k+1);
        r.photo_tr=e(k+2);
        k=k+len+2;
    elseif e(k)==240
        len=e(k+1);
        r.debug_info=e(k+2:k+2+len-1);
        k=k+len+2;
    elseif e(k)==2
        len=e(k+1);
        last_seq=e(k+2) + 256 * (e(k+3) + 256 * (e(k+4) + 256 * e(k+5)));
        seq_skipped=e(k+6);
        r.last_seq=last_seq;          % for debugging
        r.seq_skipped=seq_skipped;    % for debugging
        k=k+len+2;
    %elseif e(k)==241
    %    len=e(k+1);
    %    r.bluetoothrate=e(k+2) + 256 * (e(k+3) + 256 * (e(k+4) + 256 * e(k+5)));
    %    r.bluetoothname_ok=e(k+6);
    %    k=k+len+2;
    else
        len=e(k+1);
        k=k+len+2;
    end
end
end         % end of function decode_message

function b=packetize(seq, cmd, data)
% Packetize one or more command(s) to ee405 board.
% cmd: vector containing command(s)
% data: data 
% For example, packetize(seq,10,'1.wav') packetizes a command 10 with data '1.wav' (audio playback)
% and packetize(seq,22,[5 30 0 1 240 0]) packetizes a servo command to a servo with id=5.
% To send more than one command at once, which is faster and more efficient,
% do packetize(seq,[10 22],{'1.wav', [5 30 0 1 240 0]}),

m=length(cmd);
if m==0
    n=0;
elseif m==1 % data is a vector
    n=length(data);
else    % data is a cell of vectors
    n=0;
    for k=1:m
        n=n+length(data{k});
    end
end
tn=4+n+2*m; % total length including seq. & data (excluding preamble, length, & crc)

%if tn > max_payload_size
%    error('Data size is too big')
%end

seq=uint32(seq);

pn=4;      % length of preamble (this helps re-sync in case the previous packet loses up to this many bytes)
n_to_transmit=pn+12+tn;  % total # of bytes to transmit
b=zeros(1,n_to_transmit,'uint8')+255;   % pn 0xff's
b(pn+1)=hex2dec('3c');
b(pn+2)=hex2dec('7d');
b(pn+3)=bitand(tn,255);
b(pn+4)=bitand(bitshift(tn,-8),255);
b(pn+5)=bitxor(bitand(tn,255),255);
b(pn+6)=bitxor(bitand(bitshift(tn,-8),255),255);
b(pn+7)=bitand(tn,255);
b(pn+8)=bitand(bitshift(tn,-8),255);
b(pn+9)=bitand(seq,255);
b(pn+10)=bitand(bitshift(seq,-8),255);
b(pn+11)=bitand(bitshift(seq,-16),255);
b(pn+12)=bitand(bitshift(seq,-24),255);
if m==1
    b(pn+13)=cmd;
    b(pn+14)=n;
    b(pn+15:pn+14+n)=data;
elseif m>1
    j=pn+13;
    for k=1:m
        b(j)=cmd(k);
        b(j+1)=length(data{k});
        b(j+2:j+1+length(data{k}))=data{k};
        j=j+2+length(data{k});
    end
end
c=crc32(b(pn+7:pn+8+tn));
b(pn+9+tn)=bitand(c,255);
b(pn+10+tn)=bitand(bitshift(c,-8),255);
b(pn+11+tn)=bitand(bitshift(c,-16),255);
b(pn+12+tn)=bitand(bitshift(c,-24),255);

end         % end of function packetize

function b=packetize15(seq, cmd, data)  % for boards with version <= 15
m=length(cmd);
if m==0
    n=0;
elseif m==1 % data is a vector
    n=length(data);
else    % data is a cell of vectors
    n=0;
    for k=1:m
        n=n+length(data{k});
    end
end
tn=4+n+2*m; % total length including seq. & data (excluding preamble, length, & crc)

%if tn > max_payload_size
%    error('Data size is too big')
%end

seq=uint32(seq);

pn=0;      % length of preamble
n_to_transmit=pn+8+tn;  % total # of bytes to transmit
b=zeros(1,n_to_transmit,'uint8')+255;   % pn 0xff's
b(pn+1)=hex2dec('3c');
b(pn+2)=hex2dec('7d');
b(pn+3)=bitand(tn,255);
b(pn+4)=bitand(bitshift(tn,-8),255);
b(pn+5)=bitand(seq,255);
b(pn+6)=bitand(bitshift(seq,-8),255);
b(pn+7)=bitand(bitshift(seq,-16),255);
b(pn+8)=bitand(bitshift(seq,-24),255);
if m==1
    b(pn+9)=cmd;
    b(pn+10)=n;
    b(pn+11:pn+10+n)=data;
elseif m>1
    j=pn+9;
    for k=1:m
        b(j)=cmd(k);
        b(j+1)=length(data{k});
        b(j+2:j+1+length(data{k}))=data{k};
        j=j+2+length(data{k});
    end
end
c=crc32(b(pn+5:pn+4+tn));
b(pn+5+tn)=bitand(c,255);
b(pn+6+tn)=bitand(bitshift(c,-8),255);
b(pn+7+tn)=bitand(bitshift(c,-16),255);
b(pn+8+tn)=bitand(bitshift(c,-24),255);

end         % end of function packetize_15

function transmit(s, b, varargin)

if nargin>=3
    output_buffer_size = varargin{1};
else
    output_buffer_size = s.OutputBufferSize;
end

% Only up to (OutputBufferSize - BytesToOutput) bytes can be transmitted at a time
%   for both serial & bluetooth.
% The output buffer size can be changed by doing "set(s,'OutputBufferSize',xxx)"
%   but it can only be done when the port is closed.

while output_buffer_size - s.BytesToOutput < length(b)
end

if strcmp(s.Type,'serial')
    % for serial
    fwrite(s,b,'uint8');
else
    % for bluetooth
    % If 'sync' mode is used for bluetooth (default for fwrite), there's excessive delay between characters
    %   and it takes 500 ~ 800 msec to transmit just 500 bytes.
    % However, in 'async' mode, no such behavrior and it only takes 10~20 msec.
    %   (but, a long packet in 'async' mode is still transmitted in multiple subpackets
    %   with a few msec interval between subpackets.)
    while ~strcmp(s.TransferStatus,'idle') end  % needed since 'async' even when transmit buffer is not full
    fwrite(s,b,'uint8','async');   % this returns immediately
end
end         % end of function transmit

function c=crc32(x)
% calculate crc32
% x: vector of octets (type doesn't matter - can be either uint8 or double)
% Matlab version written by Sae-Young Chung in 2014
% Original c code is from http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
% Last update: 2015/3/12

persistent crc32_table
persistent m1
if isempty(crc32_table)
crc32_table = uint32([
  hex2dec('00000000'), hex2dec('77073096'), hex2dec('ee0e612c'), hex2dec('990951ba'), hex2dec('076dc419'),...
  hex2dec('706af48f'), hex2dec('e963a535'), hex2dec('9e6495a3'), hex2dec('0edb8832'), hex2dec('79dcb8a4'),...
  hex2dec('e0d5e91e'), hex2dec('97d2d988'), hex2dec('09b64c2b'), hex2dec('7eb17cbd'), hex2dec('e7b82d07'),...
  hex2dec('90bf1d91'), hex2dec('1db71064'), hex2dec('6ab020f2'), hex2dec('f3b97148'), hex2dec('84be41de'),...
  hex2dec('1adad47d'), hex2dec('6ddde4eb'), hex2dec('f4d4b551'), hex2dec('83d385c7'), hex2dec('136c9856'),...
  hex2dec('646ba8c0'), hex2dec('fd62f97a'), hex2dec('8a65c9ec'), hex2dec('14015c4f'), hex2dec('63066cd9'),...
  hex2dec('fa0f3d63'), hex2dec('8d080df5'), hex2dec('3b6e20c8'), hex2dec('4c69105e'), hex2dec('d56041e4'),...
  hex2dec('a2677172'), hex2dec('3c03e4d1'), hex2dec('4b04d447'), hex2dec('d20d85fd'), hex2dec('a50ab56b'),...
  hex2dec('35b5a8fa'), hex2dec('42b2986c'), hex2dec('dbbbc9d6'), hex2dec('acbcf940'), hex2dec('32d86ce3'),...
  hex2dec('45df5c75'), hex2dec('dcd60dcf'), hex2dec('abd13d59'), hex2dec('26d930ac'), hex2dec('51de003a'),...
  hex2dec('c8d75180'), hex2dec('bfd06116'), hex2dec('21b4f4b5'), hex2dec('56b3c423'), hex2dec('cfba9599'),...
  hex2dec('b8bda50f'), hex2dec('2802b89e'), hex2dec('5f058808'), hex2dec('c60cd9b2'), hex2dec('b10be924'),...
  hex2dec('2f6f7c87'), hex2dec('58684c11'), hex2dec('c1611dab'), hex2dec('b6662d3d'), hex2dec('76dc4190'),...
  hex2dec('01db7106'), hex2dec('98d220bc'), hex2dec('efd5102a'), hex2dec('71b18589'), hex2dec('06b6b51f'),...
  hex2dec('9fbfe4a5'), hex2dec('e8b8d433'), hex2dec('7807c9a2'), hex2dec('0f00f934'), hex2dec('9609a88e'),...
  hex2dec('e10e9818'), hex2dec('7f6a0dbb'), hex2dec('086d3d2d'), hex2dec('91646c97'), hex2dec('e6635c01'),...
  hex2dec('6b6b51f4'), hex2dec('1c6c6162'), hex2dec('856530d8'), hex2dec('f262004e'), hex2dec('6c0695ed'),...
  hex2dec('1b01a57b'), hex2dec('8208f4c1'), hex2dec('f50fc457'), hex2dec('65b0d9c6'), hex2dec('12b7e950'),...
  hex2dec('8bbeb8ea'), hex2dec('fcb9887c'), hex2dec('62dd1ddf'), hex2dec('15da2d49'), hex2dec('8cd37cf3'),...
  hex2dec('fbd44c65'), hex2dec('4db26158'), hex2dec('3ab551ce'), hex2dec('a3bc0074'), hex2dec('d4bb30e2'),...
  hex2dec('4adfa541'), hex2dec('3dd895d7'), hex2dec('a4d1c46d'), hex2dec('d3d6f4fb'), hex2dec('4369e96a'),...
  hex2dec('346ed9fc'), hex2dec('ad678846'), hex2dec('da60b8d0'), hex2dec('44042d73'), hex2dec('33031de5'),...
  hex2dec('aa0a4c5f'), hex2dec('dd0d7cc9'), hex2dec('5005713c'), hex2dec('270241aa'), hex2dec('be0b1010'),...
  hex2dec('c90c2086'), hex2dec('5768b525'), hex2dec('206f85b3'), hex2dec('b966d409'), hex2dec('ce61e49f'),...
  hex2dec('5edef90e'), hex2dec('29d9c998'), hex2dec('b0d09822'), hex2dec('c7d7a8b4'), hex2dec('59b33d17'),...
  hex2dec('2eb40d81'), hex2dec('b7bd5c3b'), hex2dec('c0ba6cad'), hex2dec('edb88320'), hex2dec('9abfb3b6'),...
  hex2dec('03b6e20c'), hex2dec('74b1d29a'), hex2dec('ead54739'), hex2dec('9dd277af'), hex2dec('04db2615'),...
  hex2dec('73dc1683'), hex2dec('e3630b12'), hex2dec('94643b84'), hex2dec('0d6d6a3e'), hex2dec('7a6a5aa8'),...
  hex2dec('e40ecf0b'), hex2dec('9309ff9d'), hex2dec('0a00ae27'), hex2dec('7d079eb1'), hex2dec('f00f9344'),...
  hex2dec('8708a3d2'), hex2dec('1e01f268'), hex2dec('6906c2fe'), hex2dec('f762575d'), hex2dec('806567cb'),...
  hex2dec('196c3671'), hex2dec('6e6b06e7'), hex2dec('fed41b76'), hex2dec('89d32be0'), hex2dec('10da7a5a'),...
  hex2dec('67dd4acc'), hex2dec('f9b9df6f'), hex2dec('8ebeeff9'), hex2dec('17b7be43'), hex2dec('60b08ed5'),...
  hex2dec('d6d6a3e8'), hex2dec('a1d1937e'), hex2dec('38d8c2c4'), hex2dec('4fdff252'), hex2dec('d1bb67f1'),...
  hex2dec('a6bc5767'), hex2dec('3fb506dd'), hex2dec('48b2364b'), hex2dec('d80d2bda'), hex2dec('af0a1b4c'),...
  hex2dec('36034af6'), hex2dec('41047a60'), hex2dec('df60efc3'), hex2dec('a867df55'), hex2dec('316e8eef'),...
  hex2dec('4669be79'), hex2dec('cb61b38c'), hex2dec('bc66831a'), hex2dec('256fd2a0'), hex2dec('5268e236'),...
  hex2dec('cc0c7795'), hex2dec('bb0b4703'), hex2dec('220216b9'), hex2dec('5505262f'), hex2dec('c5ba3bbe'),...
  hex2dec('b2bd0b28'), hex2dec('2bb45a92'), hex2dec('5cb36a04'), hex2dec('c2d7ffa7'), hex2dec('b5d0cf31'),...
  hex2dec('2cd99e8b'), hex2dec('5bdeae1d'), hex2dec('9b64c2b0'), hex2dec('ec63f226'), hex2dec('756aa39c'),...
  hex2dec('026d930a'), hex2dec('9c0906a9'), hex2dec('eb0e363f'), hex2dec('72076785'), hex2dec('05005713'),...
  hex2dec('95bf4a82'), hex2dec('e2b87a14'), hex2dec('7bb12bae'), hex2dec('0cb61b38'), hex2dec('92d28e9b'),...
  hex2dec('e5d5be0d'), hex2dec('7cdcefb7'), hex2dec('0bdbdf21'), hex2dec('86d3d2d4'), hex2dec('f1d4e242'),...
  hex2dec('68ddb3f8'), hex2dec('1fda836e'), hex2dec('81be16cd'), hex2dec('f6b9265b'), hex2dec('6fb077e1'),...
  hex2dec('18b74777'), hex2dec('88085ae6'), hex2dec('ff0f6a70'), hex2dec('66063bca'), hex2dec('11010b5c'),...
  hex2dec('8f659eff'), hex2dec('f862ae69'), hex2dec('616bffd3'), hex2dec('166ccf45'), hex2dec('a00ae278'),...
  hex2dec('d70dd2ee'), hex2dec('4e048354'), hex2dec('3903b3c2'), hex2dec('a7672661'), hex2dec('d06016f7'),...
  hex2dec('4969474d'), hex2dec('3e6e77db'), hex2dec('aed16a4a'), hex2dec('d9d65adc'), hex2dec('40df0b66'),...
  hex2dec('37d83bf0'), hex2dec('a9bcae53'), hex2dec('debb9ec5'), hex2dec('47b2cf7f'), hex2dec('30b5ffe9'),...
  hex2dec('bdbdf21c'), hex2dec('cabac28a'), hex2dec('53b39330'), hex2dec('24b4a3a6'), hex2dec('bad03605'),...
  hex2dec('cdd70693'), hex2dec('54de5729'), hex2dec('23d967bf'), hex2dec('b3667a2e'), hex2dec('c4614ab8'),...
  hex2dec('5d681b02'), hex2dec('2a6f2b94'), hex2dec('b40bbe37'), hex2dec('c30c8ea1'), hex2dec('5a05df1b'),...
  hex2dec('2d02ef8d')]);
m1=uint32(hex2dec('ffffffff'));
end

m=uint8(x);
n=length(m);
c = uint32(0);
for k=1:n
	c = bitxor(bitshift(c, -8), crc32_table(int32(bitxor(uint8(bitand(c, 255)), m(k))) + 1));
end
c = bitxor(c, m1);

end       % end of function crc32

% to be used by firmware(s,fn)
function ee405send(s, cmd, data)
transmit(s, packetize(0, cmd, data));   % seq. number does not matter since the firmware is being updated
end

function ret=firmware(s, fn, old_ver)
% update firmware of ee405 board
% s: serial port variable (bluetooth will not work)
% fn: file name for *.hex

persistent h
if isempty(h)      % means this is called the first time
    h=waitbar(0,{'','Updating EE405 firmware. Please wait.' ''},'Name','EE405 firmware update');
else  % this means firmware.m was prevously called
    if isvalid(h)  % if the previous dialog box is still active, delete it such that we can show a new progress bar in foreground
        delete(h)
    end
    h=waitbar(0,{'','Updating EE405 firmware. Please wait.' ''},'Name','EE405 firmware update');
end
ret='';

if strncmp(s.Name,'Blue',4)
    waitbar(0,h,{'' 'Can not update firmware over bluetooth.' ''});
    return
end

[d,start_addr,size,crc]=read_hex(fn);

if size==0
    ret = 'file open failure';
    waitbar(0,h,{'' 'File open failure.' ''});
    return
end

buf=zeros(1,256+16,'uint8');

if old_ver
    transmit(s, packetize15(0, 243, []));
else
    ee405send(s,243,[]);    % send command to reset cpu
end

start_time=clock;       % do not use tic, toc since other program may use them
i = 0;
rst_msg_displayed = 0;
while 1
    if (etime(clock, start_time) > 3) && ~rst_msg_displayed   % if still no response, instruct to press reset
        waitbar(0,h,{'' 'Press the reset button on EE405 board.' ''});
        rst_msg_displayed = 1;
    end
    if s.BytesAvailable > 11
        flushinput(s)
    end
    if s.BytesAvailable
        c=fread(s,1,'uint8');
        if i == 0
            i = (i + 1) * (c == '*');
        elseif i == 1
            i = (i + 1) * (c == 'E');
        elseif i == 2
            i = (i + 1) * (c == 'u');
        elseif i == 3
            i = (i + 1) * (c == 'z');
        elseif i == 4
            i = (i + 1) * (c == '@');
        elseif i == 5
            i = (i + 1) * (c == 'r');
        elseif i == 6
            i = (i + 1) * (c == 'P');
        elseif i == 7
            i = (i + 1) * (c == '9');
        end
        if i == 8
            break;
        end
    end
end

fwrite(s,'S','uint8');

if rst_msg_displayed
    waitbar(0,h,{'' 'Updating EE405 firmware. Please wait.' ''});
end

while 1
    while ~s.BytesAvailable
    end
    c=fread(s,1,'uint8');
    if c == 'O'	% OK from the device
        break;
    end
end

buf(1:8)=uint8([hex2dec('73') hex2dec('91') hex2dec('08') hex2dec('12') 0 0 0 0]);

for i = 0:256:size-1
    waitbar(i/size,h,{'' 'Updating EE405 firmware. Please wait.' ''});
    if size >= i + 256
        len = 256;
    else
        len = size - i;
    end
    buf(9) = bitand(len, 255);
    buf(10) = bitshift(len, -8);
    si = start_addr + i;
    buf(11) = bitand(si, 255);
    buf(12) = bitand(bitshift(si, -8), 255);
    buf(13) = bitand(bitshift(si, -16), 255);
    buf(14) = bitand(bitshift(si, -24), 255);
    buf(15) = 0;
    buf(16) = 0;
    
    for j=1:len
        buf(j + 16) = d(i + j);
    end
    fwrite(s, buf(1:16+len), 'uint8');
    while 1
        while ~s.BytesAvailable
        end
        c=fread(s,1,'uint8');
        if c == 'O'	% OK from the device
            break;
        end
    end
end

buf(9) = 0;
buf(10) = 0;
buf(11) = bitand(crc, 255);
buf(12) = bitand(bitshift(crc, -8), 255);
buf(13) = bitand(bitshift(crc, -16), 255);
buf(14) = bitand(bitshift(crc, -24), 255);

fwrite(s, buf(1:16), 'uint8');

while 1
    while ~s.BytesAvailable
    end
    c=fread(s,1,'uint8');
    if c == 'C'	% CRC OK
        ret = 'success';
        waitbar(1,h,{'' 'Firmware update successful!' ''});
        while s.BytesAvailable < 10     % wait for a packet after reboot
        end
        ee405send(s,14,[]);
        break;
    elseif c == 'B' % CRC bad
        ret = 'crc bad';
        waitbar(1,h,{'' 'Firmware update failed!' ''});
        break;
    else
        ret = 'unknown failure';
        waitbar(1,h,{'' 'Firmware update failed!' ''});
        break;
    end
end
drawnow

end


function [d,start_addr,size,crc]=read_hex(fn)
e = 99;
start_addr = -1;
size = 0;
crc = 0;
d=[];
f = fopen(fn, 'r');
if f==-1
    return
end
d=zeros(1,1024 * 1024,'uint8');
seg = 0;

while 1
    try
        s = fgets(f, 255);
    catch
        e = 1;
        break;
    end
    if length(s) < 11
        e = 1;
        braek;
    end
    if s(1) ~= ':'
        e = 1;
        break;
    end
    len = hex2byte(s(2:3));
    if len < 0
        e = 2;
        break;
    end
    if length(s) < len * 2 + 11
        e = 2;
        break;
    end
    if hex2byte(s(4:5)) < 0
        e = 3;
        break;
    end
    if hex2byte(s(6:7)) < 0
        e = 4;
        break;
    end
    addr = hex2byte(s(4:5)) * 256 + hex2byte(s(6:7));
    rec_type = hex2byte(s(8:9));
    if rec_type < 0
        e = 5;
        break;
    end
    if rec_type == 1
        e = 0;
        break;
    elseif rec_type == 2
        seg = hex2byte(s(10:11));
        if seg < 0
            e = 6;
            break;
        end
    elseif rec_type == 0
        if start_addr < 0
            start_addr = bitshift(seg, 12) + addr;
        end
        if ((start_addr + size) ~= (bitshift(seg, 12) + addr))
            e = 7;
            break;
        end
        for i=1:len
            d(size+i) = hex2byte(s((10 + (i - 1) * 2):(11 + (i - 1) * 2)));
        end
        size = size + len;
    elseif rec_type == 3
        % ignore this
    else
        e = 8;
        break;
    end
    checksum = 0;
    for k=1:len+5
        checksum = checksum + hex2byte(s(k*2:k*2+1));
    end
    if mod(checksum, 256)
        e = 9;
        break;
    end
end
if e == 0
    d=d(1:size);
    crc = crc32(d);
else
    start_addr = 0;
    crc = 0;
    size = 0;
end
fclose(f);
end


function r=nibble2int(s)
if ((s >= '0') && (s <= '9'))
    r = s - '0';
elseif ((s >= 'A') && (s <= 'F'))
    r = s - 'A' + 10;
elseif ((s >= 'a') && (s <= 'f'))
    r = s - 'a' + 10;
else
    r = -1;
end
end


function r=hex2byte(s)
i = nibble2int(s(1));
if (i >= 0)
    j = nibble2int(s(2));
    if (j >= 0)
        r = bitor(bitshift(i, 4), j);
    else
        r = -1;
    end
else
    r = -1;
end
end

function r=despace(s)
% remove white spaces in the string s
c=1;
for k=1:length(s)
    if s(k)~=' '
        r(c)=s(k);
        c=c+1;
    end
end
if c==1
    r=[];
end
end

function print_help(fn)
f=fopen(fn,'r');
while 1
    s=fgetl(f);
    if ~ischar(s)
        break;
    end
    if isempty(s)
        disp(' ');
    else
        disp(s)
    end
end
fclose(f);
end
