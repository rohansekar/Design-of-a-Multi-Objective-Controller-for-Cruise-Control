from matplotlib import pyplot as plt
import numpy as np
from scipy.integrate import odeint
from matplotlib import rc

def vehicle(v,t,u):                      #modelling the vehicle
    CD=0.3;
    RHO=1.2;
    AF=2;
    F=1000;
    M=1200;
    CRR=0.02;
    G=9.81;
    dv_dt=(F*u-(CRR*M*G)-(0.5*RHO*CD*AF*v**2))/M;
    return dv_dt;
 
TF=20;
TIMESTEPS=21;
DELTA_T=0.1;
KC=11;
KI=3;
KD=0.1;
DISTLEADSTART=650;                       #lead vehicle starts at 650m
DISTEGOSTART=0;                          #ego vehicle starts at origin


distEgo=0;                               #distance traveled by the ego vehicle
distLead=0;                              #distance travelled by the lead vehicle
vLead=15;                                #speed of the lead vehicle   
relativeDistance=np.zeros(TIMESTEPS);
ts=np.linspace(0,TF,TIMESTEPS);
step=np.zeros(TIMESTEPS)
vEgoTemp=50;                             # starting speed of the ego vehicle
vEgo=np.zeros(TIMESTEPS);
sumIntegralError=0;
e=np.zeros(TIMESTEPS);
integralError=np.zeros(TIMESTEPS);
derivativeError=np.zeros(TIMESTEPS);
speedStore=np.zeros(TIMESTEPS);
step=np.zeros(TIMESTEPS);
i=0;


for i in range(TIMESTEPS-1):
    
    distEgo= DISTEGOSTART+vEgoTemp*i;
    distLead=DISTLEADSTART+vLead*i;
    relDistTemp=distLead-distEgo
    relativeDistance[i+1]=relDistTemp;
    speed=50;                           #set-point speed
    speedStore[i+1]=speed;
    error=speed-vEgoTemp;
    sumIntegralError=sumIntegralError+error*DELTA_T;
    derivativeError=error/DELTA_T;
    u=KC*error+KI*sumIntegralError+KD*derivativeError;
    integralError[i+1]=sumIntegralError;
    step[i+1]=u;
    v=odeint(vehicle,vEgoTemp,[0,DELTA_T],args=(u,))    
    vEgoTemp=v[-1];
    vEgo[i+1]=vEgoTemp;                  #speed of the ego vehicle
    
    if vEgo[i+1]>60:                     #upper and lower bounds of speed
        vEgo[i+1]=60;
    if vEgo[i+1]<0:
        vEgo[i+1]=0;
           

    if (distLead-distEgo<=600) and (distLead-distEgo>=200):        
        speed=25;                        #set-point speed
        speedStore[i+1]=speed;
        error=speed-vEgoTemp;
        KC1=10.7;
        KI1=4;
        KD1=0.1;
        sumIntegralError=sumIntegralError+error*DELTA_T;
        derivativeError=error/DELTA_T
        u=KC1*error+KI1*sumIntegralError+KD1*derivativeError
        integralError[i+1]=sumIntegralError;
        step[i+1]=u;
        v=odeint(vehicle,vEgoTemp,[0,DELTA_T],args=(u,))    
        vEgoTemp=v[-1];
        vEgo[i+1]=vEgoTemp;              #speed of the ego vehicle
        
        if vEgo[i+1] >60:               #upper and lower bounds of speed
           vEgo[i+1]=60;
        if vEgo[i+1]<0:
           vEgo[i+1]=0;
           

    elif (distLead-distEgo<200) and (distLead-distEgo>0):
        speed=15;                       #set-point speed
        speedStore[i+1]=speed;
        error=speed-vEgoTemp;
        KC2=10.8;
        KI2=3;
        KD2=0.1;
        sumIntegralError=sumIntegralError+error*DELTA_T;
        derivativeError=error/DELTA_T
        u=KC2*e+KI2*sumIntegralError+KD2*derivativeError;
        integralError[i+1]=sumIntegralError;
        step[i+1]=u;
        v=odeint(vehicle,vEgoTemp,[0,DELTA_T],args=(u,))
        vEgoTemp=v[-1];
        vEgo[i+1]=vEgoTemp;              #speed of the ego vehicle
        
        if vEgo[i+1]>60:                #upper and lower bounds of speed 
           vEgo[i+1]=60;
        if vEgo[i+1]<0:
           vEgo[i+1]=0;
           

    if distLead-distEgo<=0:
          vEgo[i+1]=0;
          speedStore[i+1]=0;
          exit(0);
          

plt.figure()
rc('axes', linewidth=2)
rc('font', weight='bold')
plt.subplot(2,1,1)
plt.xlabel('Time',fontweight='bold',fontsize=19.0)
plt.ylabel('Speed',fontweight='bold',fontsize=19.0)
plt.plot(ts, vEgo,linewidth=6, label ='Speed',color='red')
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.subplot(2,1,2)
plt.xlabel('Time',fontweight='bold',fontsize=19.0)
plt.ylabel('Distance',fontweight='bold',fontsize=19.0)
plt.plot(ts,relativeDistance,linewidth=6, label ='Distance',color='blue')
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.tight_layout()
plt.show()
         
        

    

    



