%% Create the battery charging MPC controller used by the Simulink model
% The prediction model uses charging-current magnitude as a positive input.
% Measured outputs are SOC fraction (0 to 1) and terminal voltage (V).

Ts_mpc = 1.0;                 % controller sample time, s
capacity_Ah = 20;             % from the source Battery block
R0_ohm = 0.012;               % from the source Battery block
Rp_ohm = 0.008;               % documented polarization-model assumption
tau_p_s = 60;                 % documented polarization time-constant assumption
soc_nominal = 0.40;           % source-model initial SOC
voltage_nominal_V = 24.0;     % source-model nominal battery voltage
soc_target = 0.75;            % target charge level and charge cutoff
voltage_limit_V = 26.15;      % existing source-model CV reference
current_limit_A = 15;         % physical battery-current safety limit
current_command_limit_A = 13.5; % derated to cover switching/current-loop overshoot

ap = exp(-Ts_mpc/tau_p_s);
dOCV_dSOC = (voltage_limit_V - voltage_nominal_V) / ...
    (soc_target - soc_nominal);

% State vector: [SOC deviation; polarization-voltage deviation;
%                achieved-current deviation].
% Input: positive charging-current command magnitude (A). The achieved
% current state introduces the retained inner-loop/actuator delay and avoids
% nonphysical direct feedthrough in the MPC prediction model.
% Outputs: [SOC fraction; terminal voltage (V)].
k_soc = Ts_mpc/(3600*capacity_Ah);
A = [1, 0, k_soc; 0, ap, Rp_ohm*(1-ap); 0, 0, 0];
B = [0; 0; 1];
C = [1, 0, 0; dOCV_dSOC, 1, R0_ohm];
D = [0; 0];

predictionPlant = ss(A, B, C, D, Ts_mpc);
predictionPlant.InputName = {'ChargeCurrent_A'};
predictionPlant.OutputName = {'SOC_fraction'; 'TerminalVoltage_V'};
predictionPlant = setmpcsignals(predictionPlant, 'MV', 1, 'MO', [1 2]);

predictionHorizon = 30;
controlHorizon = 5;
batteryMPC = mpc(predictionPlant, Ts_mpc, predictionHorizon, controlHorizon);

% Nominal offsets make the linear model operate around the source model's
% initial condition without forcing terminal voltage to track its limit.
batteryMPC.Model.Nominal.U = 0;
batteryMPC.Model.Nominal.Y = [soc_nominal; voltage_nominal_V];
batteryMPC.Model.Nominal.X = [0; 0; 0];

% Objective: prioritize SOC progress and smooth current. Voltage is handled
% as a hard safety constraint, not a maximum-voltage tracking objective.
batteryMPC.Weights.OutputVariables = [8, 0];
batteryMPC.Weights.ManipulatedVariables = 0.03;
batteryMPC.Weights.ManipulatedVariablesRate = 0.25;

batteryMPC.MV.Min = 0;
batteryMPC.MV.Max = current_command_limit_A;
batteryMPC.MV.RateMin = -2;
batteryMPC.MV.RateMax = 2;
batteryMPC.MV.ScaleFactor = current_command_limit_A;

batteryMPC.OV(1).Min = 0;
batteryMPC.OV(1).Max = soc_target;
batteryMPC.OV(1).ScaleFactor = 1;
batteryMPC.OV(2).Min = 18;
batteryMPC.OV(2).Max = voltage_limit_V;
batteryMPC.OV(2).ScaleFactor = voltage_limit_V - 18;
batteryMPC.OV(2).MaxECR = 0;  % hard maximum-voltage constraint

% A finite ECR weight keeps other soft constraints subordinate to voltage.
batteryMPC.Weights.ECR = 1e5;

controllerMetadata = struct( ...
    'sample_time_s', Ts_mpc, ...
    'prediction_horizon', predictionHorizon, ...
    'control_horizon', controlHorizon, ...
    'soc_target', soc_target, ...
    'voltage_limit_V', voltage_limit_V, ...
    'current_limit_A', current_limit_A, ...
    'current_command_limit_A', current_command_limit_A, ...
    'current_sign_at_mpc_output', 'positive charging magnitude', ...
    'current_sign_at_inner_loop', 'negative for charging', ...
    'thermal_constraint', 'not modeled; source Battery block exposes no temperature output', ...
    'matlab_function_blocks_added', 0);

setupDir = fileparts(mfilename('fullpath'));
save(fullfile(setupDir, 'battery_mpc_controller.mat'), ...
    'batteryMPC', 'predictionPlant', 'controllerMetadata');

assignin('base', 'batteryMPC', batteryMPC);
assignin('base', 'controllerMetadata', controllerMetadata);

