function plotCircle(P,r)
    len = 40;
    x = zeros(len,0);
    y = zeros(len,0);
    th = linspace(0,2*pi,len);
    for i = 1:len
        x(i) = r*cos(th(i));
        y(i) = r*sin(th(i));
    end
    x = x + P(1);
    y = y + P(2);
    
    plot(x,y,'linewidth',2)
    axis([-r r -r r])

        



