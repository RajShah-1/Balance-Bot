function Q = Optimize2(Q)
  d = 0.01;
  R = 1;
 
  for i = 1:1000
      Q(1, 1) -=  0.001*(Cost(add(Q, 1)) - Cost(Q));
      Q(2, 2) -=  0.01*(Cost(add(Q, 2)) - Cost(Q));
      Q(3, 3) -=  0.01*(Cost(add(Q, 3)) - Cost(Q));
      Q(4, 4) -=  0.001*(Cost(add(Q, 4)) - Cost(Q));
      Q
  endfor
endfunction
