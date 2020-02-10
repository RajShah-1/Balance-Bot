function cost = Cost(Q)
  R = 1;
  n = 1000;
  x = [0, 0, 1, 0]';
  R1 = 1;
  Q1 = [1, 0, 0, 0; 0, 1, 0, 0; 0 0 3 0;0 0 0 4];
  [Ad, Bd, G] = DiscreteK(Q, R);
  Anew = Ad-Bd*G;
  cost = 0;
  for i = 1:n
    T = -G*x;
    cost += x'*Q1*x + T'*R1*T;
    x = Anew*x;
  endfor
endfunction
