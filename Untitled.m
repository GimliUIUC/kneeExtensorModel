%% test fcn_inv

q1 = -pi/12;
q2 = -pi+pi/6;
s = 0.12;
p_KNEE = rot(q1)*[s;0];
p_FOOT = p_KNEE + rot(q1+q2)*[s;0];


plot([0 p_KNEE(1) p_FOOT(1)],[0 p_KNEE(2) p_FOOT(2)])

z = -p_FOOT(2);
q = fcn_inv([0,z])