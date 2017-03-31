X
z = X(:,1);
q = zeros(0,2);
for i = 1:size(z,1)
    q(1,:) = fcn_inv([0;z(i)],l1,l2);
    
end


