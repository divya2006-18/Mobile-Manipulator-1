%% QBot Platform (datasheet)
qb.D = 0.570; qb.H = 0.227; qb.mass = 17; qb.vNom = 0.7; qb.aNom = 0.5;
%% QBot ASSUMED, replace with real values
qb.rw = 0.05; qb.track = 0.40; qb.wheelW = 0.03; qb.gc = 0.027;
qb.mWheel = 0.5; qb.mCastor = 0.05;
qb.mBody = qb.mass - 2*qb.mWheel - 4*qb.mCastor;      % 15.8 kg

%% QArm Mini (datasheet)
qa.reach = 0.382; qa.wNom = deg2rad(60); qa.wMax = deg2rad(180);
qa.grasp = 0.070; qa.payload = 0.150;
%% QArm ASSUMED, replace with real values
qa.h1 = 0.10;                                   % shoulder height above mount
qa.L2 = 0.160; qa.L3 = 0.142; qa.L4 = 0.080;    % sum = 0.382 m
qa.mount = [0.10 0 qb.H];                       % arm base position in chassis frame

%% Task
A    = [0.5; 0.5; 0];            % start x, y, theta
goal = [2.5; 2.0];               % B, inside the 3.6 x 3.6 m arena
lem.cx = 0.22; lem.cy = 0; lem.a = 0.05; lem.z = 0.05; lem.w = 0.6;
armP  = [qa.h1 qa.L2 qa.L3 qa.L4];
lemP  = [lem.cx lem.cy lem.a lem.z lem.w];
qHome = deg2rad([0; 90; -120; 30]);             % stowed pose
Ta    = 4;                                      % s, stowed pose -> start of the infinity

%% Reachability check (rerun whenever you change geometry)
tt = linspace(0,2*pi,400); d = 1+sin(tt).^2;
px = lem.cx + lem.a*cos(tt)./d;  py = lem.cy + lem.a*sin(tt).*cos(tt)./d;
dz = lem.z + qa.L4 - qa.h1;
c3 = (px.^2+py.^2 + dz^2 - qa.L2^2 - qa.L3^2)/(2*qa.L2*qa.L3);
assert(all(abs(c3)<0.95), 'Infinity not safely reachable: change cx, z or a');
fprintf('Elbow q3: %.0f to %.0f deg\n', min(rad2deg(-acos(c3))), max(rad2deg(-acos(c3)))); %[output:09a50c90]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":4.5}
%---
%[output:09a50c90]
%   data: {"dataType":"text","outputData":{"text":"Elbow q3: -111 to -52 deg\n","truncated":false}}
%---
