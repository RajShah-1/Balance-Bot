global A = csvread('sensor_data.csv');  #do not change this line
global ax ay az gx gy gz pitch roll;

################################################
#######Declare your global variables here#######
################################################

function ax = combine(axh, axl)
    ax = bitshift(axh, 8)+axl;
    ax = ax - 65536*(bitget(ax, 16) == 1);
endfunction

function read_accel(axl,axh,ayl,ayh,azl,azh)  
  
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####
  #################################################
  global ax ay az;
  % Code to combine axl and axh
  ax = combine(axl, axh);
  ay = combine(ayl, ayh);
  az = combine(azl, azh);

  lowpassfilter(ax, ay, az, 5);
  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  ####################################################

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  #################################################
  global gx gy gz;
  gx = combine(gxl, gxh);
  gy = combine(gyl, gyh);
  gz = combine(gzl, gzh);

  highpassfilter(gx, gy, gz, 5);


  #####################################################
  # Call function highpassfilter(ax,ay,az,f_cut) here #
  #####################################################;

endfunction

function lowpassfilter(ax,ay,az,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  afx = afy = afz = [];
  afx(1)=(1-alpha)*ax(1);
  afy(1)=(1-alpha)*ay(1);
  afz(1)=(1-alpha)*az(1);
  for n = 2:rows(ax)
    afx(n) = (1-alpha)*ax(n) + alpha*afx(n-1);
    afy(n) = (1-alpha)*ay(n) + alpha*afy(n-1);
    afz(n) = (1-alpha)*az(n) + alpha*afz(n-1);
  endfor
  ax=afx;
  ay=afy;
  az=afz;
  
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  gfx = gfy = gfz = [];
  gfx(1)=(1-alpha)*gx(1);
  gfy(1)=(1-alpha)*gy(1);
  gfz(1)=(1-alpha)*gz(1);
  for n = 2:rows(gx)
    gfx(n) = (1-alpha)*gfx(n-1) + (1-alpha)*(gx(n)-gx(n-1));
    gfy(n) = (1-alpha)*gfy(n-1) + (1-alpha)*(gy(n)-gy(n-1));
    gfz(n) = (1-alpha)*gfz(n-1) + (1-alpha)*(gz(n)-gz(n-1));
  endfor
  gx=gfx;
  gy=gfy;
  gz=gfz;
  
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)
  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  ##############################################
  global pitch;
  alpha = 0.03;
  apitch = atan2(ax, sqrt(ay.*ay + az.*az));
  pitch(1) = (1-alpha)*gx(1)*0.01*pi/180 + alpha*apitch(1);
  for n = 2:length(ax) 
    pitch(n) = (1-alpha)*(pitch(n-1) + gx(n)*0.01*pi/180) + alpha*apitch(n);
  endfor
endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)

  ##############################################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  ##############################################
  global roll;
  alpha = 0.03;
  aroll = atan2(ay, sqrt(ax.*ax + az.*az));
  roll(1) = (1-alpha)*gy(1)*0.01*pi/180 + alpha*aroll(1);
  for n = 2:length(ax)
    roll(n) = (1-alpha)*(roll(n-1) + gy(n)*0.01*pi/180) + alpha*aroll(n);
  endfor
endfunction

function execute_code()
  global A ax ay az gx gy gz pitch roll;

  for n = 1:rows(A)                    #do not change this line
    
    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    ###############################################                  
  endfor
  axl = A(:, 1);
  axh = A(:, 2);
  ayl = A(:, 3);
  ayh = A(:, 4);
  azl = A(:, 5);
  azh = A(:, 6);
  gxl = A(:, 7);
  gxh = A(:, 8);
  gyl = A(:, 9);
  gyh = A(:, 10);
  gzl = A(:, 11);
  gzh = A(:, 12);
  read_gyro(gxl, gxh, gyl, gyh, gzl, gzh);
  read_accel(axl, axh, ayl, ayh, azl, azh);
  comp_filter_pitch(ax, ay, az, gx, gy, gz);
  comp_filter_roll(ax, ay, az, gx, gy, gz);\

  % Scaling
  



  % csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
