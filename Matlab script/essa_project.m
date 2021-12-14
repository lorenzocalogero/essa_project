clc, clear variables, close all

% Import keypress emulator class
import java.awt.Robot;
import java.awt.event.*;
robot = Robot;

disp('Move the cursor into the new window  within 5 sec');
pause(5)

s = serialport('COM3',115200); % Connect to serial port COM3
Ts = 5e-03; % Sensor sampling period (s)
T = 30; % Duration of the emulation (s)
N = T/Ts;

fprintf('Move the board for %d sec\n',T);

for i=1:1:N
	%tic
	line = readline(s); % Get serial data line (up to terminal newline character)
	rpy = sscanf(line,'%f')'; % Get RPY angles value
	
	if(length(rpy)==3) % The data line has been correctly acquired
		if rpy(1)>20
			robot.keyPress(java.awt.event.KeyEvent.VK_RIGHT); % Right arrow key press
		else
			robot.keyRelease(java.awt.event.KeyEvent.VK_RIGHT); % Right arrow key release
		end
		
		if rpy(1)<-20
			robot.keyPress(java.awt.event.KeyEvent.VK_LEFT); % Left arrow key press
		else
			robot.keyRelease(java.awt.event.KeyEvent.VK_LEFT); % Left arrow key release
		end

		if rpy(2)>20
			robot.keyPress(java.awt.event.KeyEvent.VK_SPACE); % Spacebar key press
		else
			robot.keyRelease(java.awt.event.KeyEvent.VK_SPACE); % Spacebar key release
		end
	end
	%t = toc;
end

robot.keyRelease(java.awt.event.KeyEvent.VK_LEFT);
robot.keyRelease(java.awt.event.KeyEvent.VK_RIGHT);
robot.keyRelease(java.awt.event.KeyEvent.VK_SPACE);
clear s % Close the serial connection
clc
















