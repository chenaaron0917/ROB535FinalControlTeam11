clc;clear;close all
load 'TestTrack.mat'
Nobs=15;
Xobs = generateRandomObstacles(Nobs,TestTrack);
%% View Original Map
figure(1)
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),...
'.k',TestTrack.bl(1,:),TestTrack.bl(2,:),'r',...
TestTrack.br(1,:),TestTrack.br(2,:),'r')
hold on
for i=1:size(Xobs,2)
    for k=1:size(Xobs{i},1)
        plot(Xobs{i}(:,1),Xobs{i}(:,2),'b',[Xobs{i}(1,1);Xobs{i}(4,1)],[Xobs{i}(1,2);Xobs{i}(4,2)],'b')
    end
    XobsPos=mean(Xobs{i});
    scatter(XobsPos(1),XobsPos(2))
end
%% Generate New Cline and View
%newTestTrack=generateNewTestTrack(TestTrack,Xobs);
%plot(newTestTrack.cline(1,:),newTestTrack.cline(2,:),'-k')

%% Here to turn in
controlIn=ROB599_ControlsProject_part2_Team11(TestTrack,Xobs);
%% Here to check
[Y,~]=forwardIntegrateControlInput(controlIn);
Cood=[Y(:,1) Y(:,3)];
getTrajectoryInfo(Cood,controlIn,Xobs,TestTrack)
% Plot final Trajectory with Obstacles
figure(2)
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),...
'.k',TestTrack.bl(1,:),TestTrack.bl(2,:),'r',...
TestTrack.br(1,:),TestTrack.br(2,:),'r')
hold on
for i=1:size(Xobs,2)
    for k=1:size(Xobs{i},1)
        plot(Xobs{i}(:,1),Xobs{i}(:,2),'b',[Xobs{i}(1,1);Xobs{i}(4,1)],[Xobs{i}(1,2);Xobs{i}(4,2)],'b')
    end
end
plot(Cood(:,1),Cood(:,2))