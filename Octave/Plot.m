function x = Plot(Q, n)
  
   R = 1;
   [Ad, Bd, G] = DiscreteK(Q, R);
   x = [1, 1, 1, 1]';
   px = zeros(n, 1);
   pxDot = zeros(n, 1);
   ptheta = zeros(n, 1);
   pthetaDot = zeros(n, 1);
   
   A = Ad - Bd*G;
   for i = 1:n
     x = A*x;
     px(i) = x(1);
     pxDot(i) = x(2);
     ptheta(i) = x(3);
     pthetaDot(i) = x(4);
   endfor
 
   p = 1:n;
   figure()
   plot(p, px);
   figure()
   plot(p, pxDot);
   figure()
   plot(p, ptheta);
   figure()
   plot(p, pthetaDot);
   
endfunction
