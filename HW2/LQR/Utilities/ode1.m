function x = ode1(odeFun, tSpan, x0)
    x = zeros(length(tSpan),length(x0));

    x(1,:) = x0';
    
    for i = 1:length(tSpan)-1
        x(i+1,:) = x(i,:) + (tSpan(i+1) - tSpan(i))*odeFun(i,x(i,:)')';
    end
end