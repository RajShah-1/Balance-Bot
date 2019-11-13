global A = csvread('csv_matter.csv');  #do not change this line

################################################
#######Declare your global variables here#######
################################################

global ax ay az gx gy gz pitch roll;

function a = combine(ah, al)
  a = bitshift(ah, 8) + al;
  a = a - 65536*(bitget(a, 16) == 1);
endfunction


function read_accel(axl,axh,ayl,ayh,azl,azh)  
  
  #################################################
  global ax ay az;
  
  ax = combine(axh, axl) / 16384;
  ay = combine(ayh, ayl) / 16384;
  az = combine(azh, azl) / 16384;
  #################################################


  ####################################################
  lowpassfilter(ax,ay,az,5);
  ####################################################

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  
  #################################################
  global gx gy gz;
  
  gx = combine(gxh, gxl) / 131;
  gy = combine(gyh, gyl) / 131;
  gz = combine(gzh, gzl) / 131;
  #################################################


  #####################################################
  highpassfilter(gx, gy, gz, 5);
  #####################################################;

endfunction


function lowpassfilter(ax,ay,az,f_cut)
  global ax ay az;
  dT = 0.01;  #time in seconds
  Tau = 1 / (2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  a = [1, -alpha];
  b = [1-alpha];
  ax = filter(b , a, ax);
  ay = filter(b , a, ay);
  az = filter(b , a, az);
  ################################################
  
endfunction


function highpassfilter(gx,gy,gz,f_cut)
  global gx gy gz;
  dT = 0.01;  #time in seconds
  Tau= 1 / (2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  a = [1, alpha-1];
  b = [1-alpha, alpha-1];
  gx = filter(b , a, gx);
  gy = filter(b , a, gy);
  gz = filter(b , a, gz);
  ################################################
  
endfunction
  
function comp_filter_pitch(ax,ay,az,gx,gy,gz)
   
  ##############################################
  global pitch;
  alpha = 0.03;
  dt = 0.01;
  
  apitch = atan(ay ./ abs(az)) * 180/pi ;
  gpitch =  -dt*gx;
  x = (1-alpha)*gpitch + alpha*apitch;
  a = [1, alpha-1];
  b = [1];
  pitch = filter(b, a, x);
  ##############################################
  
endfunction 
  
function comp_filter_roll(ax,ay,az,gx,gy,gz)

  ##############################################
  global roll
  dt = 0.01;
  alpha = 0.03;
      
  aroll = atan(ax ./ abs(az)) * 180/pi;
  groll = -dt*gy;
  x = (1-alpha)*groll + alpha*aroll;
  a = [1, alpha-1];
  b = [1];
  roll = filter(b, a, x);
  ##############################################
       
endfunction 

function execute_code
  global ax ay az gx gy gz pitch roll A;

%  for n = 1:rows(A)                    #do not change this line
%    
%    ###############################################
%    
%    ###############################################
%   
%  endfor
  axh = A(:, 1);
  axl = A(:, 2);
  ayh = A(:, 3);
  ayl = A(:, 4);
  azh = A(:, 5);
  azl = A(:, 6);
  gxh = A(:, 7);
  gxl = A(:, 8);
  gyh = A(:, 9);
  gyl = A(:, 10);
  gzh = A(:, 11);
  gzl = A(:, 12);
  read_gyro(gxl, gxh, gyl, gyh, gzl, gzh);
  read_accel(axl, axh, ayl, ayh, azl, azh);
  comp_filter_pitch(ax, ay, az, gx, gy, gz);
  comp_filter_roll(ax, ay, az, gx, gy, gz);
  %csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line