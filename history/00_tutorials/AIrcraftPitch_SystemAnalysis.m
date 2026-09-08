% Define the 's' variable for transfer functions
s = tf('s');

% Create the Aircraft Pitch model (the "Plant")
P_pitch = (1.151*s + 0.1774) / (s^3 + 0.739*s^2 + 0.921*s);

% 1. Create a time vector
% This creates a list of numbers starting at 0, counting up by 0.01, 
% and stopping at 10. It tells MATLAB: "Run the simulation for 10 
% seconds and take a data point every 0.01 seconds."
t = [0:0.01:10];

% 2. Run the Step Response simulation
% step(...) is a command that applies a "jump" to your system.
% P_pitch is your airplane model.
% 0.2 multiplies the input, meaning we are moving the elevator flap 
% by 0.2 radians (about 11 degrees) instead of just 1 radian.
% 't' tells the simulation to use the specific time limit we defined above.
step(0.2 * P_pitch, t);
axis([0 10 0 0.8]);
ylabel('pitch angle (rad)');
title('Open-loop Step Response');

pole(P_pitch) 
% One is at 0 (Dangerous)


%Moving to Closed LOOP systems --------------------------------

% 1. Create the Closed-Loop System
% the 'feedback' command takes our plant (P_pitch) and 
% connects the output back to the input. 
% The '1' represents a simple sensor that perfectly measures the output.
sys_cl = feedback(P_pitch, 1);

% 2. Run the Step Response for the Closed-Loop system
% We still use a 0.2 radian (11 degree) input.
% Notice we don't necessarily need 't' here; MATLAB will find a 
% good range to show the system settling.
figure;
step(0.2 * sys_cl);

% 3. Add labels for clarity
ylabel('Pitch Angle (rad)');
title('Closed-loop Step Response');
grid on; % Adds a grid to make reading the values easier


% 1. Find the 'Poles' of the closed-loop system
% Poles tell us about stability and speed.
poles = pole(sys_cl) % All are on the Left/Negative side (Safe!)

% 2. Find the 'Zeros' of the closed-loop system
% Zeros tell us about the 'shape' of the response (like overshoot).
zeros = zero(sys_cl)

% 3. Plot them on a Map (Pole-Zero Map)
% 'X' marks the Poles, 'O' marks the Zeros
figure; 
pzmap(P_pitch); 
grid on;
title('Where the "DNA" of our Plane lives');