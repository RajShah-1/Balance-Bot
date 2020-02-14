function cost, Q = Cost2(data)
    x = data(:, 1);
    xDot = data(:, 2);
    theta = data(:, 3);
    thetaDot = data(:, 4);
    torque = data(:, 5);
    cost = 10*x'*x + 10*xDot'*xDot + 10*theta'*theta + 5*thetaDot'*thetaDot + 2*torque*torque;
endfunction
