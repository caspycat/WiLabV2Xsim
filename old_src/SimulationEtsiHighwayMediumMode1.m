% Sample ETSI highway simulation using controlled mode-1 scheduling.

close all
clear
clc

simulatorRoot = fileparts(mfilename("fullpath"));
projectRoot = fileparts(simulatorRoot);
addpath(fullfile(projectRoot,"src"));
addpath(simulatorRoot);

configFile = fullfile( ...
    simulatorRoot,"ConfigFiles","EtsiHighwayMediumMode1.cfg");
outputFolder = fullfile( ...
    simulatorRoot,"Output","EtsiHighwayMediumMode1");

WiLabV2Xsim(configFile,"outputFolder",outputFolder);
