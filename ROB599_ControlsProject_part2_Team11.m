function U = ROB599_ControlsProject_part2_Team11(TestTrack,Xobs)
TestTrack=generateNewTestTrack(TestTrack,Xobs); % generate new TestTrack with obstacles
n=50000;U=[];deltaold=0;
x=[287,5,-176,0,2,0]; % X,latV,Y,lonU,heading,rotspeed
desiredU=5;d_Kp=6.2;dist_Kp=1;F_Kp=2600; % P control parameters
time0=tic;
for t=1:n
    if mod(t,1000)==0 && toc(time0)>600
      break
    end
    Cood=[x(1) x(3)]; % find closest two points
    [idx,~,~]=getNearPtsInfo(TestTrack,Cood,1);
    if idx==size(TestTrack.cline,2)
        t
        break
    end
    [~,desiredPsi,dist_error]=getNearPtsInfo(TestTrack,Cood,2);
    delta=dist_Kp*dist_error+d_Kp*(desiredPsi-x(5)); % P control for steering
    delta=deltaold+max(min(delta-deltaold,0.05),-0.05);
    delta=max(min(delta,1.2),-1.2); % limit steering angle
    deltaold=delta;
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
function x1=ode1(U,x)
dt=0.01;

%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;
%generate input functions
delta_f=U(1);
F_x=U(2);
%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));
%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end
%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
      
x1=x+dzdt'*dt;
end
