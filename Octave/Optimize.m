function [minQ, minR,minCost] = Optimize() 
 
n = 1000;
minCost = +inf;
  for i = 1:2
    for j = 1:2
      for k = 1:10
        for l = 1:2
          for m = 1:10
            Q = [i, 0, 0, 0; 
                  0, j, 0, 0;
                  0, 0, k, 0;
                  0, 0, 0, l];
            R = m;
            J = Cost(Q, R, n);
            if J < minCost 
              minCost = J;
              minQ = Q;
              minR = R;
            endif
          endfor
        endfor
      endfor
    endfor
  endfor
Plot(minQ, minR, n);
endfunction
