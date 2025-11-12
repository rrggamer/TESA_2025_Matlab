function [f,Phee_value]  = fcn(m,g,desire,x,xdot,x2dot,K,Kp,Kv,Kr,Kw,I)
%#codegen
% Controller แบบง่าย: คุมแค่ความสูง (z) ก่อน
% ป้องกัน NaN/Inf ด้วยการ clamp ค่า

%% 1) แกะ desired จากเวกเตอร์ desire (18 ตัว)
xt      = desire(1);
yt      = desire(2);
zt      = desire(3);

dxt     = desire(4);
dyt     = desire(5);
dzt     = desire(6);

ddxt    = desire(7);
ddyt    = desire(8);
ddzt    = desire(9);

% ที่เหลือยังไม่ใช้ตอนนี้
% dddxt   = desire(10);
% dddyt   = desire(11);
% dddzt   = desire(12);
% ddddxt  = desire(13);
% ddddyt  = desire(14);
% ddddzt  = desire(15);
psit    = desire(16); %#ok<NASGU>
dpsit   = desire(17); %#ok<NASGU>
ddpsit  = desire(18); %#ok<NASGU>

%% 2) แกะ state จริง (สมมติ x(1:3) = [x;y;z], xdot(1:3) = [vx;vy;vz])
r   = x(1:3);        % [x;y;z]
v   = xdot(1:3);     % [vx;vy;vz]

z   = r(3);
vz  = v(3);

% ส่งค่า phi ออกไปดูใน Scope (ถ้า state 4 = phi)
if numel(x) >= 4
    Phee_value = x(4);
else
    Phee_value = 0;
end

%% 3) คุมแกน z แบบ PD + feedforward
ez  = zt  - z;       % error ตำแหน่ง z
evz = dzt - vz;      % error ความเร็ว z

% ใช้ element (3,3) ของ Kp,Kv คุมแกน z
kpz = Kp(3,3);
kvz = Kv(3,3);

az_des = ddzt + kvz*evz + kpz*ez;   % desired vertical acc

% รวม gravity
T = m*(g - az_des);

% ป้องกัน NaN/Inf
if ~isfinite(T)
    T = m*g;
end

% Limit thrust: ไม่ให้ติดลบหรือสูงเกินไป
T = max(0, min(T, 4*m*g));

%% 4) ยังไม่คุม attitude → ตั้ง moment = 0 ไปเลย
Mx = 0;
My = 0;
Mz = 0;

U = [T; Mx; My; Mz];

%% 5) แปลงเป็นแรงมอเตอร์ f = K\U
if any(~isfinite(K(:))) || abs(det(K)) < 1e-9
    f = zeros(4,1);
else
    f = K \ U;
end

% กันค่า negative / NaN / Inf
for i = 1:4
    if ~isfinite(f(i)) || f(i) < 0
        f(i) = 0;
    end
end

end
