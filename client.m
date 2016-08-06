function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%
%   Example:
%       client (Linux/Mac)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
port = '/dev/ttyUSB0';

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('a: Read current sensor (ADC counts)     b: Read current sensor (mA)\n');
    fprintf('c: Read encoder (counts)                d: Read encoder (deg)\n');
    fprintf('e: Reset encoder                        f: Set PWM (-100 to 100)\n');
    fprintf('g: Set current gains                    h: Get current gains\n');
    fprintf('i: Set position gains                   j: Get position gains\n');
    fprintf('k: Test current control                 l: Go to angle (deg)\n');
    fprintf('m: Load step trajectory                 n: Load cubic trajectory\n');
    fprintf('o: Execute trajectory                   p: Unpower the motor\n');
    fprintf('q: Quit client                          r: Get mode\n');
    
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'
            current_counts = fscanf(mySerial, '%d');
            fprintf('\nCurrent: %d ADC counts\n\n', current_counts);
            
        case 'b'
            current_amps = fscanf(mySerial, '%d');
            current_amps = current_amps/1000;
            fprintf('\nCurrent: %5.2f mA\n\n', current_amps);
        
        case 'c'
            counts = fscanf(mySerial,'%d');            
            fprintf('\nThe motor angle is %d counts.\n\n', counts);
        
         case 'd'
            deg = fscanf(mySerial,'%d');
            deg = deg/10;
            fprintf('\nThe motor angle is %5.2f degrees.\n\n', deg);
            
        case 'e'
            reset = fscanf(mySerial, '%d');
            fprintf('\nEncoder Reset: %d\n', reset);
            if reset == 32768
                fprintf('Encoder has reset successfully!\n\n');
            end
        
        case 'f'
            duty_cycle = input('\nWhat PWM value would you like [-100 to 100]? ');
            fprintf(mySerial, '%d\n', duty_cycle);
            if duty_cycle > 0
                fprintf('PWM has been set to %d in the counterclokcwise direction\n\n', duty_cycle);
            else
                fprintf('PWM has been set to %d in the clokcwise direction\n\n', duty_cycle);
            end
            
        case 'g'            
            kpcur = input('Enter your desired Kp current gain [recommended: 4.76]: ');
            kicur = input('Enter your desired Ki current gain [recommended: 0.32]: ');
            fprintf(mySerial, '%d %d\n',[kpcur*1000, kicur*1000]);
            fprintf('Sending Kp = %5.4f, and Ki = %5.4f to the current controller\n\n', kpcur, kicur)
            
        case 'h'
            current_gains = fscanf(mySerial,'%d %d');
            kpcur = current_gains(1)/1000;
            kicur = current_gains(2)/1000;
            fprintf('The current controller is using Kp = %5.4f and Ki = %5.4f\n\n', kpcur, kicur)
            
        case 'i'
            kppos = input('Enter your desired Kp position gain [recommended: 4.76]: ');
            kipos = input('Enter your desired Ki position gain [recommended: 0.32]: ');
            kdpos = input('Enter your desired Kd position gain [recommended: 10.63]: ');
            fprintf(mySerial, '%d %d %d\n',[kppos*1000, kipos*1000, kdpos*1000]);
            fprintf('Sending Kp = %5.4f, Ki = %5.4f, and Kd = %5.4f\n\n', kppos, kipos, kdpos)
            
        case 'j' 
            position_gains = fscanf(mySerial,'%d %d %d');
            kppos = position_gains(1)/1000;
            kipos = position_gains(2)/1000;
            kdpos = position_gains(3)/1000;
            fprintf('The position controller is using Kp = %5.4f, Ki = %5.4f, and Kd = %5.4f\n\n', kppos, kipos, kdpos)    
            
        case 'k'
            [score,data] = read_plot_matrix(mySerial); % plot the data
            fprintf('Plot generated!\n\n')
            
        case 'l'
            angle = input('Enter the desired motor angle in degrees: ');
            fprintf(mySerial, '%d\n', angle);
            fprintf('Motor moving to %d degrees\n\n', angle)
            
        case 'm'
            param = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]\n');
            stepref = genRef(param, 'step');
            numsam = size(stepref,2);
            fprintf(mySerial, '%d\n', numsam); 
            for i=1:numsam
                fprintf(mySerial, '%d\n', stepref(i));
            end
            fprintf('Plotting the desired trajectory and sending to the PIC32 ... completed.\n\n')
        
        case 'n'
            paramcubic = input('Enter cubic trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]\n');
            cubicref = genRef(paramcubic, 'cubic');
            % cubicref = cubic(200, paramcubic, 'cubic');
            numsam = size(cubicref,2);
            fprintf(mySerial, '%d\n', numsam); 
            for i=1:numsam
                fprintf(mySerial, '%d\n', cubicref(i));
            end
            fprintf('Plotting the desired trajectory and sending to the PIC32 ... completed.\n\n')
        
        case 'o'
            fprintf('Executing trajectory ...\n\n')
            % [score,data] = read_plot_matrix(mySerial);
            data = plot_traj(mySerial);
            
        case 'p'
            % fprintf(mySerial, '%d\n', 0);
            fprintf('\nIDLE mode initiated. Motor has been unpowered successfully!\n\n');
            
        case 'r'
            mode = fscanf(mySerial, '%d');
            if mode == 0
                fprintf('\nMode is IDLE\n\n');
            elseif mode == 1
                fprintf('\nMode is PWM\n\n');
            elseif mode == 2
                fprintf('\nMode is ITEST\n\n');
            elseif mode == 3
                fprintf('\nMode is HOLD\n\n');
            elseif mode == 4
                fprintf('\nMode is TRACK\n\n');
            end
            
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('\nInvalid Selection %c\n\n', selection);
        
%         case 'x'                         % exercise operation
%             n1 = input('Enter first number: '); % get the first number to send
%             n2 = input('Enter second number: '); % get the second number to send
%             fprintf(mySerial, '%d %d\n',[n1,n2]); % send the numbers for addition
%             sum = fscanf(mySerial,'%d');   % get the sum
%             fprintf('Read - Sum of n1 + n2: %d\n',sum);     % print it to the screen
        
        
    end
end

end
