%Fuzzy Tunning PID Control
clear all;
close all;

a=newfis('fuzzpid');

a=addvar(a,'input','e',[-3,3]);                        %Parameter e
a=addmf(a,'input',1,'NB','zmf',[-3,-1]);
a=addmf(a,'input',1,'NM','trimf',[-3,-2,0]);
a=addmf(a,'input',1,'NS','trimf',[-3,-1,1]);
a=addmf(a,'input',1,'Z','trimf',[-2,0,2]);
a=addmf(a,'input',1,'PS','trimf',[-1,1,3]);
a=addmf(a,'input',1,'PM','trimf',[0,2,3]);
a=addmf(a,'input',1,'PB','smf',[1,3]);

a=addvar(a,'input','ec',[-3,3]);                       %Parameter ec
a=addmf(a,'input',2,'NB','zmf',[-3,-1]);
a=addmf(a,'input',2,'NM','trimf',[-3,-2,0]);
a=addmf(a,'input',2,'NS','trimf',[-3,-1,1]);
a=addmf(a,'input',2,'Z','trimf',[-2,0,2]);
a=addmf(a,'input',2,'PS','trimf',[-1,1,3]);
a=addmf(a,'input',2,'PM','trimf',[0,2,3]);
a=addmf(a,'input',2,'PB','smf',[1,3]);

a=addvar(a,'output','kp',[-0.3,0.3]);                   %Parameter kp
a=addmf(a,'output',1,'NB','zmf',[-0.3,-0.1]);
a=addmf(a,'output',1,'NM','trimf',[-0.3,-0.2,0]);
a=addmf(a,'output',1,'NS','trimf',[-0.3,-0.1,0.1]);
a=addmf(a,'output',1,'Z','trimf',[-0.2,0,0.2]);
a=addmf(a,'output',1,'PS','trimf',[-0.1,0.1,0.3]);
a=addmf(a,'output',1,'PM','trimf',[0,0.2,0.3]);
a=addmf(a,'output',1,'PB','smf',[0.1,0.3]);

a=addvar(a,'output','ki',[-0.06,0.06]);                 %Parameter ki
a=addmf(a,'output',2,'NB','zmf',[-0.06,-0.02]);
a=addmf(a,'output',2,'NM','trimf',[-0.06,-0.04,0]);
a=addmf(a,'output',2,'NS','trimf',[-0.06,-0.02,0.02]);
a=addmf(a,'output',2,'Z','trimf',[-0.04,0,0.04]);
a=addmf(a,'output',2,'PS','trimf',[-0.02,0.02,0.06]);
a=addmf(a,'output',2,'PM','trimf',[0,0.04,0.06]);
a=addmf(a,'output',2,'PB','smf',[0.02,0.06]);

a=addvar(a,'output','kd',[-3,3]);                       %Parameter kp
a=addmf(a,'output',3,'NB','zmf',[-3,-1]);
a=addmf(a,'output',3,'NM','trimf',[-3,-2,0]);
a=addmf(a,'output',3,'NS','trimf',[-3,-1,1]);
a=addmf(a,'output',3,'Z','trimf',[-2,0,2]);
a=addmf(a,'output',3,'PS','trimf',[-1,1,3]);
a=addmf(a,'output',3,'PM','trimf',[0,2,3]);
a=addmf(a,'output',3,'PB','smf',[1,3]);

rulelist=[1 1 7 1 5 1 1;
          1 2 7 1 3 1 1;
          1 3 6 2 1 1 1;
          1 4 6 2 1 1 1;
          1 5 5 3 1 1 1;
          1 6 4 4 2 1 1;
          1 7 4 4 5 1 1;
          
          2 1 7 1 5 1 1;
          2 2 7 1 3 1 1;
          2 3 6 2 1 1 1;
          2 4 5 3 2 1 1;
          2 5 5 3 2 1 1;
          2 6 4 4 3 1 1;
          2 7 3 4 4 1 1;
          
          3 1 6 1 4 1 1;
          3 2 6 2 3 1 1;
          3 3 6 3 2 1 1;
          3 4 5 3 2 1 1;
          3 5 4 4 3 1 1;
          3 6 3 5 3 1 1;
          3 7 3 5 4 1 1;
          
          4 1 6 2 4 1 1;
          4 2 6 2 3 1 1;
          4 3 5 3 3 1 1;
          4 4 4 4 3 1 1;
          4 5 3 5 3 1 1;
          4 6 2 6 3 1 1;
          4 7 2 6 4 1 1;
          
          5 1 5 2 4 1 1;
          5 2 5 3 4 1 1;
          5 3 4 4 4 1 1;
          5 4 3 5 4 1 1;
          5 5 3 5 4 1 1;
          5 6 2 6 4 1 1;
          5 7 2 7 4 1 1;
          
          6 1 5 4 7 1 1;
          6 2 4 4 5 1 1;
          6 3 3 5 5 1 1;
          6 4 2 5 5 1 1;
          6 5 2 6 5 1 1;
          6 6 2 7 5 1 1; 
          6 7 1 7 7 1 1;

          7 1 4 4 7 1 1; 
          7 2 4 4 6 1 1;
          7 3 2 5 6 1 1;
          7 4 2 6 6 1 1;
          7 5 2 6 5 1 1;
          7 6 1 7 5 1 1;
          7 7 1 7 7 1 1];
       
a=addrule(a,rulelist);
a=setfis(a,'DefuzzMethod','centroid');
writefis(a,'fuzzpid');

a=readfis('fuzzpid');

%PID Controller
ts=0.001;
sys=tf(5.235e005,[1,87.35,1.047e004,0]);
dsys=c2d(sys,ts,'tustin');
[num,den]=tfdata(dsys,'v');

u_1=0.0;u_2=0.0;u_3=0.0;

y_1=0;y_2=0;y_3=0;

x=[0,0,0]';

error_1=0;
e_1=0.0;
ec_1=0.0;

kp0=0.40;
kd0=1.0;
ki0=0.0;

for k=1:1:500
time(k)=k*ts;

rin(k)=1;
%Using fuzzy inference to tunning PID
k_pid=evalfis([e_1,ec_1],a);
kp(k)=kp0+k_pid(1);
ki(k)=ki0+k_pid(2);
kd(k)=kd0+k_pid(3);
u(k)=kp(k)*x(1)+kd(k)*x(2)+ki(k)*x(3);

if k==300     % Adding disturbance(1.0v at time 0.3s)
   u(k)=u(k)+1.0;
end
if u(k)>=10
   u(k)=10;
end
if u(k)<=-10
   u(k)=-10;
end

yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
error(k)=rin(k)-yout(k);
%%%%%%%%%%%%%%Return of PID parameters%%%%%%%%%%%%%%%
   u_3=u_2;
   u_2=u_1;
   u_1=u(k);
   
   y_3=y_2;
   y_2=y_1;
   y_1=yout(k);
   
   x(1)=error(k);                % Calculating P
   x(2)=error(k)-error_1;        % Calculating D
   x(3)=x(3)+error(k);           % Calculating I

   e_1=x(1);
   ec_1=x(2);
   
   error_2=error_1;
   error_1=error(k);
end
showrule(a)
figure(1);plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);plot(time,error,'r');
xlabel('time(s)');ylabel('error');
figure(3);plot(time,u,'r');
xlabel('time(s)');ylabel('u');
figure(4);plot(time,kp,'r');
xlabel('time(s)');ylabel('kp');
figure(5);plot(time,ki,'r');
xlabel('time(s)');ylabel('ki');
figure(6);plot(time,kd,'r');
xlabel('time(s)');ylabel('kd');
figure(7);plotmf(a,'input',1);
figure(8);plotmf(a,'input',2);
figure(9);plotmf(a,'output',1);
figure(10);plotmf(a,'output',2);
figure(11);plotmf(a,'output',3);
plotfis(a);
fuzzy fuzzpid