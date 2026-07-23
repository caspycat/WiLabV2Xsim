% Smoke test for the default exit-ramp highway Scenario.

close all
clear
clc

simulatorRoot = fileparts(mfilename("fullpath"));
projectRoot = fileparts(simulatorRoot);
addpath(fullfile(projectRoot,"src"));
addpath(simulatorRoot);

configFile = fullfile( ...
    simulatorRoot,"ConfigFiles","ExitRampHighwaySmoke.cfg");
outputFolder = fullfile( ...
    simulatorRoot,"Output","ExitRampHighwaySmoke");

WiLabV2Xsim(configFile,"outputFolder",outputFolder);
