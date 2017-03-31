
%% getParams function is for storing all the constants
function [g,GR,N,l,Mass,Nq] = getParams()
    g = 9.81;
    GR = 23.3594;
    N = 10;         % length for q/dq
    l = 0.12;
    Mass = 0.74;
%     Mass = 0.8;
    Nq = 8;         % starting index of q/dq in optVar

end