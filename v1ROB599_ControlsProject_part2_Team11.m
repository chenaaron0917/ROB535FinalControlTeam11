function U = v1ROB599_ControlsProject_part2_Team11(TestTrack,Xobs)
TestTrack=generateNewTestTrack(TestTrack,Xobs); % generate new TestTrack with obstacles
n=50000;U=[];
x=[287,5,-176,0,2,0]; % X,latV,Y,lonU,heading,rotspeed
desiredU=5;d_Kp=5.3;dist_Kp=1.2;F_Kp=2600; % P control parameters
for t=1:n
    Cood=[x(1) x(3)]; % find closest two points
    [idx,~,~]=getNearPtsInfo(TestTrack,Cood,1);
    if idx==size(TestTrack.cline,2)
        t
        break
    end
    [~,desiredPsi,dist_error]=getNearPtsInfo(TestTrack,Cood,2);
    delta=dist_Kp*dist_error+d_Kp*(desiredPsi-x(5)); % P control for steering
    delta=max(min(delta,1.2),-1.2); % limit steering angle
    F=F_Kp*(desiredU-x(2)); % P control for F,gain * longVerror
    U=cat(1,U,[delta F]); % store control input
    x=ode1([delta F],x); % update states using ode
end
end
function newTestTrack=generateNewTestTrack(TestTrack,Xobs)
newTestTrack=TestTrack;
for i=1:size(Xobs,2)
    d=[1.7 1.1 0.5]; % translate cline
    XobsPos=mean(Xobs{i});% get obstacle center position
    [~,~,distError]=getNearPtsInfo(newTestTrack,XobsPos,2); % find distance to two closest points
    [nearPtsIdx,~,~]=getNearPtsInfo(newTestTrack,XobsPos,6); % find six closest points index
    ptDeg=TestTrack.theta(nearPtsIdx); % get cline point heading angle
    if distError>0 % obstacle on the right side
        d=-d;% move six points to left, else d=d; move six points to right
    end
    % change local cline
    newTestTrack.cline(:,nearPtsIdx)=newTestTrack.cline(:,nearPtsIdx)+d(3)*[cos(ptDeg-pi/2);sin(ptDeg-pi/2)];
    newTestTrack.cline(:,nearPtsIdx(2:5))=newTestTrack.cline(:,nearPtsIdx(2:5))+d(2)*[cos(ptDeg(2:5)-pi/2);sin(ptDeg(2:5)-pi/2)];
    newTestTrack.cline(:,nearPtsIdx(3:4))=newTestTrack.cline(:,nearPtsIdx(3:4))+d(1)*[cos(ptDeg(3:4)-pi/2);sin(ptDeg(3:4)-pi/2)];
end
end
function [nearPtsIdx,angle,distError]=getNearPtsInfo(TestTrack,Pos,k)
    nearPtsIdx=knnsearch(TestTrack.cline',Pos,'K',k);
    nearPtsIdx=sort(nearPtsIdx); % get near points index
    dir=TestTrack.cline(:,nearPtsIdx(end))'-TestTrack.cline(:,nearPtsIdx(1))';
    dir=dir/norm(dir);
    angle=acos(dir(1)); % direction of two points in rad
    relativeCood=[Pos-TestTrack.cline(:,nearPtsIdx(1))' 0];
    dist_error=cross(relativeCood,[dir 0]);
    distError=dist_error(3);% get lat distance error
end