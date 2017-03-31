
k = [0.7 0.8 0.8 0.8 0.9 0.9 1;
     0.7 0.8 0.9 1.0 0.9 1.0 1];
GR = 23.3594;
speed_norm = 7451*2*pi/(60*GR);% rpm to rad/s
tor_norm = 0.42*GR;

speed_max = zeros(length(k),1);
tor_max = zeros(length(k),1);
 for i = 1:length(k)
     k_w = k(1,i);
     k_t = k(2,i);
     speed_max(i) = k_w*speed_norm;
     tor_max(i) = k_w*tor_norm;
 end
 
 