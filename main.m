HA = human_arm();
robot = ur5robot();

q0 = [-1.5692 -0.9318 1.211 -1.85 -1.5708 -0.522];

q = zeros(1000,6);
q(1,:) = q0;
L1 = -robot.links(2).a;
L2 = -robot.links(3).a;
lu = 0.3;
lf = 0.3;
disp("L1 ="+ L1);
disp("L2 ="+L2);
theta = zeros(100,6);
G = SE3.empty(1000,0);
g06 = zeros(4,4,1000);
p_ww = zeros(1000,3);
T = SE3.empty(1000,0);
j = 1;

[p_w , Q_w, dotp_w, omega_w] = HA.get_arm_posture(0);
g06 = robot.fkine(q0);
R0w = quat2rotm(transpose(Q_w));
g0w = SE3([R0w p_w; 0 0 0 1]);
g6w = inv(g06)*g0w;
gwe = SE3([1 0 0 -lu; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
for i= 0:0.01:10
    [p_w , Q_w, dotp_w, omega_w] = HA.get_arm_posture(i);
    g0w = SE3([quat2rotm(transpose(Q_w)) p_w; 0 0 0 1]);
    g06(j) = g0w*inv(g6w); 
    p_e = g06(j)*gwe*[0; 0; 0];
    if(i <= 5)
        d = p_w + i*0.2*(p_e-p_w);
        if(i > 3)
            dp = p_w-p_e;
            ctheta = transpose(p_e)*dp/(norm(p_e)*norm(dp));
            thetak = acos(ctheta);
            Q1 = rotm2quat(g06(j).R);
            %gtarget = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak)*SE3.Rx(-1);
            gtarget = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak);
            Q2 = rotm2quat(gtarget.R);
            Q_slerp = quatinterp(Q1, Q2, (i-3)/4, 'slerp');
            G(j) = [quat2rotm(Q_slerp) d; 0 0 0 1];
        else
            G(j) = SE3([g06(j).R d; 0 0 0 1]);
        end
    else
        dp = p_w-p_e;
        ctheta = transpose(p_e)*dp/(norm(p_e)*norm(dp));
        thetak = acos(ctheta);
        d = p_e - (i-5)*0.2*p_e;
        if(i < 7)
            dp = p_w-p_e;
            ctheta = transpose(p_e)*dp/(norm(p_e)*norm(dp));
            thetak = acos(ctheta);
            Q1 = rotm2quat(g06(j).R);
            %gtarget = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak)*SE3.Rx(-1);
            gtarget = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak);
            Q2 = rotm2quat(gtarget.R);
            Q_slerp = quatinterp(Q1, Q2, (i-3)/4, 'slerp');
            G(j) = [quat2rotm(Q_slerp) d; 0 0 0 1];
        else
        %G(j) = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak)*SE3.Rx(-1);
        G(j) = SE3([g06(j).R d; 0 0 0 1])*SE3.Ry(-thetak);
        end
        disp(j);
        %trplot(g06(j));
        %trplot(G);
        % disp("dp="+dp);
        % disp("d="+d);
        % n = p_e./norm(p_e);
        % % % disp(Rk(:,3));
        % % a = Rk(:,3);
        % % o = -cross(n,a);
        % % o = [R(1,2,j) R(2,2,j) R(3,2,j)];
        % % o = o./norm(o);
        % % a = cross(n,o);
        % % a = [0 0 -1];
        % % R(:,:,j) = [n(1) o(1) a(1);n(2) o(2) a(2);n(3) o(3) a(3)];

    end
    if(j>1)
    vp = (G(j).t - G(j-1).t)/0.01;
    dRdt = (G(j).R - G(j-1).R)./0.01;
    omegaphat = dRdt*transpose(G(j).R);
    omega = [omegaphat(3,2); omegaphat(1,3); omegaphat(2,1)];
    V = [vp; omega];
    J = robot.jacob0(q(j-1,:));
    qdot = transpose(inv(J)*V);
    q(j,:) = q(j-1,:) + qdot*0.01;
    %q(j,:) = robot.ikcon(G(j));
    end
    % disp(p_w);
    % disp(d);
    % disp(i);
    % p_ww(j,:) = p_w;
    % theta(j,:) = robot.ikine(G);
    % theta(j,1) = atan2(pb(2)-d(2),pb(1)-d(1));
    % D1 = cos(theta(j,1))*(pb(1)-d(1)) + sin(theta(j,1))*(pb(2)-d(2));
    % D2 = d(3)-pb(3);
    % D3 = (D1^2+D2^2-L1^2-L2^2)/(2*L1*L2);
    % theta(j,3) = atan2(sqrt(1-D3^2),D3);
    % ro = sqrt((sin(theta(j,3))*L2)^2+(cos(theta(j,3))*L2+L1)^2);
    % thetab = atan2(cos(theta(j,3))*L2+L1,-sin(theta(j,3))*L2);
    % theta(j,2) = atan2(D1/ro,sqrt(1-(D1/ro)^2)) - thetab;
    % 
    % c5 = sin(theta(j,1))*R(1,3,j)-cos(theta(j,1))*R(2,3,j);
    % theta(j,5) = atan2(sqrt(1-(c5)^2),c5);
    % theta(j,6) = atan2(sin(theta(j,1))*R(1,2,j)-cos(theta(j,1))*R(2,2,j),cos(theta(j,1))*R(2,1,j)-sin(theta(j,1))*R(1,1,j));
    % cc1 = cos(theta(j,1))*sin(theta(j,2)+theta(j,3))*R(1,3,j)+sin(theta(j,1))*sin(theta(j,2)+theta(j,3))*R(2,3,j)-cos(theta(j,2)+theta(j,3))*R(3,3,j);
    % cc2 = -cos(theta(j,1))*cos(theta(j,2)+theta(j,3))*R(1,3,j)-sin(theta(j,1))*cos(theta(j,2)+theta(j,3))*R(2,3,j)-sin(theta(j,2)+theta(j,3))*R(3,3,j);
    % theta(j,4) = atan2(cc1,cc2);

    % g0b = [1 0 0 pb(1); 0 1 0 pb(2); 0 0 1 pb(3); 0 0 0 1];
    % gba = [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];
    % ga1 = [cos(theta(j,1)) -sin(theta(j,1)) 0 0; sin(theta(j,1)) cos(theta(j,1)) 0 0; 0 0 1 0; 0 0 0 1];
    % g12 = [cos(theta(j,2)) -sin(theta(j,2)) 0 0; 0 0 1 0; -sin(theta(j,2)) -cos(theta(j,2)) 0 0; 0 0 0 1];
    % g23 = [cos(theta(j,3)) -sin(theta(j,3)) 0 L1; sin(theta(j,3)) cos(theta(j,3)) 0 0; 0 0 1 0; 0 0 0 1];
    % g34 = [cos(theta(j,4)) -sin(theta(j,4)) 0 L2; sin(theta(j,4)) cos(theta(j,4)) 0 0; 0 0 1 0; 0 0 0 1];
    % g45 = [cos(theta(j,5)) -sin(theta(j,5)) 0 0; 0 0 1 0;-sin(theta(j,5)) -cos(theta(j,5)) 0 0; 0 0 0 1];
    % g56 = [cos(theta(j,6)) -sin(theta(j,6)) 0 0; 0 0 -1 0; sin(theta(j,6)) cos(theta(j,6)) 0 0; 0 0 0 1];
    % g06(:,:,j) = g0b*gba*ga1*g12*g23*g34*g45*g56;
    % g06 = robot.fkine(q0);
    %k = g06(:,:,j)*[0;0;0;1];
    %positions(j,:) = transpose(k(1:3));
    T(j) = robot.fkine(q(j,:));
    robot.plot(q(j,:));
    hold on;
    x = [p_e(1), p_w(1)];
    y = [p_e(2), p_w(2)];
    z = [p_e(3), p_w(3)];
    plot3(x,y,z,'-o');
    xa = [0, p_e(1)];
    xb = [0, p_e(2)];
    xc = [0, p_e(3)];
    plot3(xa,xb,xc,'-o');
    hold on;
    %trplot(quat2rotm(transpose(Q_w)));
    %trplot(R(:,:,j));
    %plot3(0,0,0,'ro', 'MarkerSize', 15, 'LineWidth', 2);
    j = j + 1;
end
figure();
t= 0:0.01:10;
subplot(3,3,1);
plot(t,q(:,1));

subplot(3,3,2);
plot(t,q(:,2));

subplot(3,3,3);
plot(t,q(:,3));

subplot(3,3,4);
plot(t,q(:,4));

subplot(3,3,5);
plot(t,q(:,5));

subplot(3,3,6);
plot(t,q(:,6));

x_coords = zeros(1, 1000);
y_coords = zeros(1, 1000);
z_coords = zeros(1, 1000);

x_d = zeros(1, 1000);
y_d = zeros(1, 1000);
z_d = zeros(1, 1000);

for i = 1:1001
    x_coords(i) = T(i).t(1);
    y_coords(i) = T(i).t(2);
    z_coords(i) = T(i).t(3);
    x_d(i) = G(i).t(1);
    y_d(i) = G(i).t(2);
    z_d(i) = G(i).t(3);
end

figure();
t= 0:0.01:10;

subplot(3,1,1);
plot(t,x_d,'Color','b','LineWidth',2);
hold on;
plot(t,x_coords,'Color','r');


subplot(3,1,2);
plot(t,y_d,'Color','b','LineWidth',2);
hold on;
plot(t,y_coords,'Color','r');


subplot(3,1,3);
plot(t,z_d,'Color','b','LineWidth',2);
hold on;
plot(t,z_coords,'Color','r');


