clc
clear
close all
load TestTrack.mat
load ROB599_ControlsProject_part1_Team11.mat
Nobs=1;
Xobs = generateRandomObstacles(Nobs,TestTrack);
[Y,~]=forwardIntegrateControlInput(ROB599_ControlsProject_part1_input);
Cood=[Y(:,1) Y(:,3)];
getTrajectoryInfo(Cood,ROB599_ControlsProject_part1_input,Xobs,TestTrack)