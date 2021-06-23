# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 15:08:10 2017

@author: DIYwalkers.com

Diagrams of TrotBot's linkage are here:
https://www.diywalkers.com/trotbot-linkage-plans.html

Diagrams of Strider's linkage are here:
https://www.diywalkers.com/strider-linkage-plans.html

Diagrams of Strandbeest's linkage are here:
https://www.diywalkers.com/strandbeest-optimizer-for-lego.html

NOTE: to animate the plot in Anaconda I needed to 
run "%matplotlib qt" in the console before running this code 
"""
#to animate first type in console:
#%matplotlib qt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy
import math


#create 2 dictionaries to store the joints' X and Y coordinates
xdict={}
ydict={}

#this sim has 4 mechanisms, so we'll make sub-dictionaries for each mechanism
mech="strandbeest"
xdict[mech]={}
ydict[mech]={}
#do the same for the Bar lengths
bar={}
bar[mech]={}

#make a dictionary for the center of rotation of each mech
xcenter={}
ycenter={}
#make a dictionary for each mech's frame connections' distances from the center of rotation
frameX={}
frameY={}

#TrotBot's front and rear cranks should be separated to give it a longer leg-base, which will use "crank_separation" as the variable
crank_separation={}

#In order to more quickly animate a graph of a mechanism we'll want to send all 
#joint values for a given crank rotation to the graphing function at once
#We can do this by saving all calculated joint values for a given crank rotation to a List variable.
#Lists are another data structure, useful for saving a list of things that don't require keys.
#First, make 2 dictionaries to store each mech's list
xlist={}
ylist={}
#The lists are added to these dictionaries after the joint calc function 

#we'll also want to save the entire mechanism (all joints) to a dictionary with crank rotation as key.  
#create one for X and one for Y here:
entire_mech_x={}
entire_mech_y={} 
mech="strandbeest"
entire_mech_x[mech]={}
entire_mech_y[mech]={} 

#and make a dict for each mech's foot-path
footpath_x={}
footpath_y={}
footpath_x[mech]={}
footpath_y[mech]={} 
#Strandbeest's linkage plans can be found at the following link (but are divided by 2 in this code)
"https://www.diywalkers.com/strandbeest-optimizer-for-lego.html"
scale_jansen=1.5
bar[mech,1]=1.5*scale_jansen  #crank
bar[mech,2]=3.8*scale_jansen
bar[mech,3]=4.15*scale_jansen
bar[mech,4]=3.93*scale_jansen
bar[mech,5]=4.01*scale_jansen
bar[mech,6]=5.58*scale_jansen
bar[mech,7]=3.94*scale_jansen
bar[mech,8]=3.67*scale_jansen
bar[mech,9]=6.57*scale_jansen
bar[mech,10]=4.9*scale_jansen
bar[mech,11]=5*scale_jansen
bar[mech,12]=6.19*scale_jansen
bar[mech,13]=.78*scale_jansen
xcenter[mech]=10
ycenter[mech]=10   
#below are the frame connections' distances from the center of rotation
frameX[mech]=3.8
frameY[mech]=0.75

#TrotBot section    
mech="trotbot"
#TrotBot's heel/toe linkage plans and background can be found at the below link
"https://www.diywalkers.com/trotbot-linkage-plans.html"
xdict[mech]={}
ydict[mech]={}
bar[mech]={}
entire_mech_x[mech]={}
entire_mech_y[mech]={} 

footpath_x[mech]={}
footpath_y[mech]={} 

scale_trotbot=1.
crank_separation[mech]=5                
bar[mech,0]=4*scale_trotbot #crank
bar[mech,1]=6*scale_trotbot
bar[mech,2]=8*scale_trotbot
bar[mech,3]=2*scale_trotbot
bar[mech,4]=6*scale_trotbot
bar[mech,5]=2*scale_trotbot
bar[mech,6]=11*scale_trotbot
bar[mech,7]=3*scale_trotbot
bar[mech,8]=9*scale_trotbot
bar[mech,9]=8*scale_trotbot
bar[mech,10]=1*scale_trotbot
bar[mech,17]=7.55*scale_trotbot# 7.616*scale_trotbot
xcenter[mech]=10
ycenter[mech]=10  
#below are the frame connections' distances from the center of rotation
frameX[mech]=7
frameY[mech]=6

mech="klann"
xdict[mech]={}
ydict[mech]={}
bar[mech]={}
entire_mech_x[mech]={}
entire_mech_y[mech]={} 

footpath_x[mech]={}
footpath_y[mech]={} 

scale_klann=.9
#Klann's patented linkage plans can be found at the following link
"https://www.diywalkers.com/klanns-linkage-plans.html"
bar[mech,0]=3*scale_klann 
bar[mech,2]=6.6*scale_klann
bar[mech,3]=3.59*scale_klann#4*scale_klann
bar[mech,4]=5.84*scale_klann
bar[mech,5]=10.04*scale_klann
bar[mech,6]=5.79*scale_klann
bar[mech,7]=10.04*scale_klann
xcenter[mech]=10
ycenter[mech]=10 
#the following are the frame connections distances from the center of the crank
low_frameX=6.6
low_frameY=1.97
high_frameX=2.6
high_frameY=6.9
B7_angle=30.
B6_angle=12.8

#Strider's section
mech="strider"
"https://www.diywalkers.com/strider-linkage-plans.html"
xdict[mech]={}
ydict[mech]={}
bar[mech]={}
entire_mech_x[mech]={}
entire_mech_y[mech]={} 

footpath_x[mech]={}
footpath_y[mech]={} 

scale_strider=1     
bar[mech,0]=4*scale_strider #crank
bar[mech,1]=5*scale_strider
bar[mech,2]=14*scale_strider
bar[mech,3]=13*scale_strider
bar[mech,4]=10*scale_strider
bar[mech,5]=6*scale_strider
bar[mech,7]=1*scale_strider
xcenter[mech]=10
ycenter[mech]=10  
#below are the frame connections' distances from the center of rotation
frameX[mech]=11
frameY[mech]=8
    

#Circle algo calcs for X and Y coordinates     
def jnt_x(ax, ay, bx, by, al, bl):
    dist = ((ax-bx)**2 + (ay-by)**2)**.5
    sidea = (al**2 - bl**2 + dist**2)/2/dist
    if al - sidea > 0:
        height = (al**2 - sidea**2)**.5
    else:
        height = 0
    Dpointx = (ax+sidea*(bx-ax)/dist)
    x1 = Dpointx + height*(ay-by)/dist
    x2 = Dpointx-height*(ay-by)/dist
    return x1,x2         

def jnt_y(ax, ay, bx, by, al, bl):
    dist = ((ax-bx)**2 + (ay-by)**2)**.5
    sidea = (al**2 - bl**2 + dist**2)/2/dist
    if al - sidea > 0:
        height = (al**2 - sidea**2)**.5
    else:
        height = 0
    Dpointy = (ay+sidea*(by-ay)/dist)
    y1 = Dpointy - height*(ax-bx)/dist
    y2 = Dpointy+height*(ax-bx)/dist
    return y1,y2    

#define 4 solutions for circle algo
high = 0
low = 1
left = 2
right = 3

def circle_algo(mech,j1,j2,b1,b2,i,solution):    
    x1,x2=jnt_x(xdict[mech,j1,i], ydict[mech,j1,i], xdict[mech,j2,i], ydict[mech,j2,i], bar[mech,b1], bar[mech,b2])
    y1,y2=jnt_y(xdict[mech,j1,i], ydict[mech,j1,i], xdict[mech,j2,i], ydict[mech,j2,i], bar[mech,b1], bar[mech,b2])
  #  print x1,y1
  #  print x2,y2
    if solution==high:
        if y1>y2:
            return x1,y1
        else:
            return x2,y2
    elif solution==low:
        if y1<y2:
            return x1,y1
        else:
            return x2,y2
    elif solution==right:
        if x1>x2:
            return x1,y1
        else:
            return x2,y2
    elif solution==left:
        if x1<x2:
            return x1,y1
        else:
            return x2,y2

def lineextend(X1, Y1, X2, Y2, Length):
    #print "change in X=",X2 - X1,"change in Y=",Y2 - Y1
    slopeangle = np.arctan2(Y2 - Y1,X2 - X1 )
    #print slopeangle
    myXlineextend = X1 - Length * np.cos(slopeangle)
    myYlineextend = Y1 - Length * np.sin(slopeangle)
    return myXlineextend,myYlineextend

def lineextendBentDegrees(X1, Y1, X2, Y2, Length,Angle):
    Angle2=Angle/180*np.pi
    slopeangle = np.arctan2(Y2 - Y1,X2 - X1 )#+Angle/180*np.pi
    myXlineextend = X1 - Length * np.cos(slopeangle+Angle2)
    myYlineextend = Y1 - Length * np.sin(slopeangle+Angle2)
    return myXlineextend,myYlineextend
    
 
#define how many simulations per rotation of the crank
#for example, to calculate every degree of crank rotation 
#you would set "rotationIncrements" to 360
#the higher rotationIncrements, the slower the sim
rotationIncrements=120
#since we want the mechs to walk farther than 1 rotation, set a loop_count variable to to walk farther
loop_count=rotationIncrements*8

#and make a dict for the speed and starting position
    
avgspeed={}
xstart={}
xchange={}
#Strandbeest's speed and starting position
mech='strandbeest'
avgspeed[mech]=-.116*180/rotationIncrements*scale_jansen/1.4
xstart[mech]=35*scale_jansen
Jansen_Shift_dn=-1.05
xchange[mech]=xstart[mech]

#TrotBot's speed and starting position
mech='trotbot'  
avgspeed[mech]=-.1595*180/rotationIncrements*scale_trotbot
xstart[mech]=75*scale_trotbot
TrotBot_Shift_Up=21
xchange[mech]=xstart[mech]

#Strider's speed and starting position
mech='strider'
avgspeed[mech]=-.151*180/rotationIncrements*scale_strider
xstart[mech]=130*scale_strider
strider_shift=0  
xchange[mech]=xstart[mech] 

#Klann's speed and starting position
mech='klann'
avgspeed[mech]=-0.186*180/rotationIncrements*scale_klann
xstart[mech]=150*scale_klann
klann_shift=20  
xchange[mech]=xstart[mech] 

def calc_joints(): 
#Your function code will loop thru one complete rotation of the crank
#You can step thru 2 Pi rotations in any increment you choose.  Say you start out with 100 increments
#This requires a step of 0.02 to go from 0 to 2. 
    for i in range(rotationIncrements):
        theta = (i / (rotationIncrements - 0.0)) * 2 * math.pi #creates rotationIncrements angles from 0 - 2pi

#Strandbeest code    
        mech="strandbeest"    
        joint=1
        xdict[mech,joint,i]=xcenter[mech]*scale_jansen
        ydict[mech,joint,i]=ycenter[mech]*scale_jansen+Jansen_Shift_dn

        joint=2
        xdict[mech,joint,i]=(xcenter[mech] - frameX[mech])*scale_jansen
        ydict[mech,joint,i]=(ycenter[mech] - frameY[mech])*scale_jansen+Jansen_Shift_dn

        joint=3
        xdict[mech,joint,i]=xcenter[mech]*scale_jansen+bar[mech,1]*np.cos(theta)
        ydict[mech,joint,i]=ycenter[mech]*scale_jansen+bar[mech,1]*np.sin(theta)+Jansen_Shift_dn
    
        joint=4
        j1=2
        j2=3
        b1=3
        b2=11        
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)

        joint=5
        j1=4
        j2=2
        b1=6
        b2=5
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,left)
        
        joint=6
        j1=3
        j2=2
        b1=12
        b2=4
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=7
        j1=5
        j2=6
        b1=7
        b2=8
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)        

        joint=8
        j1=6
        j2=7
        b1=10
        b2=9
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)
        
#backlegs section

        joint=102
        xdict[mech,joint,i]=(xcenter[mech] + frameX[mech])*scale_jansen
        ydict[mech,joint,i]=(ycenter[mech] - frameY[mech])*scale_jansen+Jansen_Shift_dn
               
        joint=104
        j1=102
        j2=3
        b1=3
        b2=11   
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)

        joint=105
        j1=104
        j2=102
        b1=6
        b2=5
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,right)

        joint=106
        j1=3
        j2=102
        b1=12
        b2=4
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=107
        j1=105
        j2=106
        b1=7
        b2=8
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=108
        j1=106
        j2=107
        b1=10
        b2=9
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)
        
#TrotBot code
        mech='trotbot'
        joint=0
        xdict[mech,joint,i]=xcenter[mech]*scale_trotbot
        ydict[mech,joint,i]=ycenter[mech]*scale_trotbot+TrotBot_Shift_Up

        joint=3
        xdict[mech,joint,i]=(xcenter[mech] - frameX[mech])*scale_trotbot
        ydict[mech,joint,i]=(ycenter[mech] + frameY[mech])*scale_trotbot+TrotBot_Shift_Up

        joint=1
        xdict[mech,joint,i]=xcenter[mech]*scale_trotbot+bar[mech,0]*np.cos(theta)
        ydict[mech,joint,i]=ycenter[mech]*scale_trotbot+bar[mech,0]*np.sin(theta)+TrotBot_Shift_Up

        joint=2
        j1=1
        j2=3
        b1=1
        b2=2  
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)

 
        joint=4
        j1=3
        j2=2
        b1=3
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])

        joint=5
        j1=4
        j2=1
        b1=4
        b2=6
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=6
        j1=5
        j2=4
        b1=5
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])
        
        joint=9
        j1=1
        j2=2
        b1=10
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])
                
        joint=8
        j1=1
        j2=2
        b1=7
        b2=17    
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,left)

        joint=7
        j1=8
        j2=6
        b1=8
        b2=9 
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

#backlegs section
        joint=100
        xdict[mech,joint,i]=(xcenter[mech] +crank_separation[mech])*scale_trotbot
        ydict[mech,joint,i]=ycenter[mech]*scale_trotbot+TrotBot_Shift_Up

        joint=103
        xdict[mech,joint,i]=(xcenter[mech] + frameX[mech]+crank_separation[mech])*scale_trotbot
        ydict[mech,joint,i]=(ycenter[mech] + frameY[mech])*scale_trotbot+TrotBot_Shift_Up

        joint=101
        xdict[mech,joint,i]=(xcenter[mech] +crank_separation[mech])*scale_trotbot+bar[mech,0]*np.cos(theta)
        ydict[mech,joint,i]=ycenter[mech]*scale_trotbot+bar[mech,0]*np.sin(theta)+TrotBot_Shift_Up
             
        joint=102
        j1=101
        j2=103
        b1=1
        b2=2  
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)
 
        joint=104
        j1=103
        j2=102
        b1=3
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])

        joint=105
        j1=104
        j2=101
        b1=4
        b2=6
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=106
        j1=105
        j2=104
        b1=5
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])

        joint=109
        j1=101
        j2=102
        b1=10
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextend(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1])
                
        joint=108
        j1=109
        j2=102
        b1=7
        b2=17    
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,right)

        joint=107
        j1=108
        j2=106
        b1=8
        b2=9 
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

#Strider code      
        mech='strider' 
        joint=0
        xdict[mech,joint,i]=xcenter[mech]*scale_strider
        ydict[mech,joint,i]=ycenter[mech]*scale_strider+strider_shift
             
        joint=2
        xdict[mech,joint,i]=(xcenter[mech] - frameX[mech])*scale_strider
        ydict[mech,joint,i]=(ycenter[mech] + frameY[mech])*scale_strider+strider_shift
             
        joint=6
        xdict[mech,joint,i]=(xcenter[mech] + frameX[mech])*scale_strider
        ydict[mech,joint,i]=(ycenter[mech] + frameY[mech])*scale_strider+strider_shift
             
        joint=1
        xdict[mech,joint,i]=xcenter[mech]*scale_strider+bar[mech,0]*(np.cos(theta))
        ydict[mech,joint,i]=ycenter[mech]*scale_strider+bar[mech,0]*(np.sin(theta))+strider_shift
             
        joint=3
        j1=2
        j2=1
        b1=1              
        b2=2   
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=9
        j1=1
        j2=3
        b1=5
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],0.)
        
        joint=10
        j1=9
        j2=1
        b1=7
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],90.)

        joint=7
        j1=1
        j2=6
        b1=2              
        b2=1  
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=5
        j1=1
        j2=7
        b1=5
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],0.)

        joint=11
        j1=5
        j2=1
        b1=7
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],-90.)

        joint=4
        j1=3
        j2=11
        b1=3              
        b2=4   
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        joint=8
        j1=7
        j2=10
        b1=3              
        b2=4 
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,low)

        mech="klann"
        joint=0
        xdict[mech,joint,i]=xcenter[mech]
        ydict[mech,joint,i]=ycenter[mech]+klann_shift

        joint=3
        xdict[mech,joint,i]=xcenter[mech]-low_frameX*scale_klann
        ydict[mech,joint,i]=ycenter[mech]-low_frameY*scale_klann+klann_shift
        
        joint=5
        xdict[mech,joint,i]=xcenter[mech]-high_frameX*scale_klann
        ydict[mech,joint,i]=ycenter[mech]+high_frameY*scale_klann+klann_shift
      
        joint=1
        xdict[mech,joint,i]=xcenter[mech]+bar[mech,0]*np.cos(-1*np.float64(theta))
        ydict[mech,joint,i]=ycenter[mech]+bar[mech,0]*np.sin(-1*np.float64(theta))+klann_shift

        joint=4
        j1=1
        b1=2
        j2=3      
        b2=3   
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)
            
        joint=6
        j1=4
        j2=1
        b1=4
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],-1*B6_angle)
        
        joint=7
        j1=5
        b1=6
        j2=6       
        b2=5  
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)

        joint=8
        j1=6
        j2=7
        b1=7
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],B7_angle)
    
        joint=103
        xdict[mech,joint,i]=xcenter[mech]+low_frameX*scale_klann
        ydict[mech,joint,i]=ycenter[mech]-low_frameY*scale_klann+klann_shift
        
        joint=105
        xdict[mech,joint,i]=xcenter[mech]+high_frameX*scale_klann
        ydict[mech,joint,i]=ycenter[mech]+high_frameY*scale_klann+klann_shift
        
        joint=104
        j1=1
        b1=2
        j2=103
        b2=3     
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)
            
        joint=106
        j1=104
        j2=1
        b1=4
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],B6_angle)
        
        joint=107
        j1=105
        b1=6
        j2=106       
        b2=5       
        xdict[mech,joint,i], ydict[mech,joint,i]=circle_algo(mech,j1,j2,b1,b2,i,high)

        joint=108
        j1=106
        j2=107
        b1=7
        xdict[mech,joint,i],ydict[mech,joint,i]=lineextendBentDegrees(xdict[mech,j1,i],ydict[mech,j1,i],xdict[mech,j2,i],ydict[mech,j2,i],bar[mech,b1],-1*B7_angle)

        
#add joints to lists for plotting
    #create dict to hold mech's foot-path lists
    x_foot_path_list={}
    y_foot_path_list={}
    mech='trotbot'
    x_foot_path_list[mech]=[]
    y_foot_path_list[mech]=[]
    mech='strandbeest'
    x_foot_path_list[mech]=[]
    y_foot_path_list[mech]=[]
    mech='strider'
    x_foot_path_list[mech]=[]
    y_foot_path_list[mech]=[]
    mech='klann'
    x_foot_path_list[mech]=[]
    y_foot_path_list[mech]=[]    
    
    for i in range(loop_count):
#TrotBot's lists
        mech='trotbot'    
        xlist[mech]=[]
        ylist[mech]=[]        
        t_joint_plot=[0,1,2,3,4,6,7,8,1,5,1,0,100, 101   ,102,103,104,106,107,108,101,105,101]
        for joint in  t_joint_plot:
            #print i,rotationIncrements, np.mod(i,rotationIncrements)
            #print 1+np.mod(i,rotationIncrements),xchange[mech]
            xlist[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])          
        for joint in  t_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)])         
        for joint in  t_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)])          
        joint=7#108
        x_foot_path_list[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
        y_foot_path_list[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])
        footpath_x[mech,i]=copy.deepcopy(x_foot_path_list[mech])
        footpath_y[mech,i]  =copy.deepcopy(y_foot_path_list[mech]) 
        xchange[mech]=xchange[mech]+avgspeed[mech]
        entire_mech_x[mech,i]=xlist[mech]
        entire_mech_y[mech,i]=ylist[mech]

#Strandbeest's lists       
        mech='strandbeest'
        xlist[mech]=[]
        ylist[mech]=[] 
        j_joint_plot=[1,3,4,2,4,5,2,6,3,6,8,7,6,7,5,4,3,1,3,104,102,104,105,102,106,3,106,108,107,106,107,105,104,3,1]
        #j_joint_plot=[1,3,4,2,4,5,2,6,3,6,8,7,6,7,5   ,1,3,104,102,104,105,102,106,3,106,108,107,106,107,105]
        for joint in  j_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])          
        for joint in  j_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)])         
        for joint in  j_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)]) 
        joint=8
        x_foot_path_list[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
        y_foot_path_list[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])   
        footpath_x[mech,i]=copy.deepcopy(x_foot_path_list[mech])
        footpath_y[mech,i]=copy.deepcopy(y_foot_path_list[mech])
        xchange[mech]=xchange[mech]+avgspeed[mech]    
        entire_mech_x[mech,i]=xlist[mech]
        entire_mech_y[mech,i]=ylist[mech]
                     
#Strider's lists        
        mech='strider'
        xlist[mech]=[]
        ylist[mech]=[] 
        s_joint_plot=[0,1,11,4, 3,2,0,2,3,1,10,8,7,6,0,6,2,6,7,1,0]
        for joint in  s_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])        
        for joint in  s_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements/3,rotationIncrements)])  
        for joint in  s_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements*2/3,rotationIncrements)])              
        joint=4
        x_foot_path_list[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
        y_foot_path_list[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])         
        footpath_x[mech,i]=copy.deepcopy(x_foot_path_list[mech])
        footpath_y[mech,i]=copy.deepcopy(y_foot_path_list[mech]) 
        xchange[mech]=xchange[mech]+avgspeed[mech]
        entire_mech_x[mech,i]=xlist[mech]
        entire_mech_y[mech,i]=ylist[mech]        

#Klann's lists        
        mech='klann'
        xlist[mech]=[]
        ylist[mech]=[] 
        k_joint_plot=[0,1,4,3,4,6,7,5,7,6,8,6,4,1,0,1,104,103,104,106,107,105,5,3,0,103,105,107,106,108,106,104,1,0]

        for joint in  k_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])        
        for joint in  k_joint_plot:
            xlist[mech].append(xdict[mech,joint,np.mod(i+rotationIncrements/2,rotationIncrements)]+xchange[mech])
            ylist[mech].append(ydict[mech,joint,np.mod(i+rotationIncrements/2,rotationIncrements)])            
        joint=8
        x_foot_path_list[mech].append(xdict[mech,joint,np.mod(i,rotationIncrements)]+xchange[mech])
        y_foot_path_list[mech].append(ydict[mech,joint,np.mod(i,rotationIncrements)])         
        footpath_x[mech,i]=copy.deepcopy(x_foot_path_list[mech])
        footpath_y[mech,i]=copy.deepcopy(y_foot_path_list[mech]) 
        xchange[mech]=xchange[mech]+avgspeed[mech]
        entire_mech_x[mech,i]=xlist[mech]
        entire_mech_y[mech,i]=ylist[mech] 

                 
fig1 = plt.figure(figsize = (21.7, 12))
#fig1 = plt.figure(figsize = (25.3, 14))
ax = fig1.add_subplot(111)
ax = plt.axes(xlim=(-38, 37.88), ylim=(0, 42))
ax.set_title("TrotBot, Klann, Strandbeest and Strider Linkages", fontsize=18)

fig1.set_facecolor('white')
line, = ax.plot([], [], '-o', ms=round(7*scale_trotbot), lw=2,color='b', mfc='red') #linkage line
line2, = ax.plot([], [], '-o', ms=round(4.5*scale_jansen), lw=2,color='b', mfc='red') #linkage line
line3, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red',mec='red') #foot-path line
line4, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red',mec='red') #foot-path line
line5, = ax.plot([], [], '-o', ms=round(7*scale_strider), lw=2,color='b', mfc='red') #linkage line
line6, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red',mec='red') #foot-path line
line7, = ax.plot([], [], '-o', ms=round(7*scale_strider), lw=2,color='b', mfc='red') #linkage line
line8, = ax.plot([], [], '-o', lw=0, ms=2, alpha=.5, mfc='red',mec='red') #foot-path line
                
def init():   
    line.set_data([], [])
    return line,

def animate(i):
    mech='trotbot'
    line.set_data(entire_mech_x[mech,i] , entire_mech_y[mech,i])
    line3.set_data(footpath_x[mech,i],footpath_y[mech,i])
    mech='strandbeest'
    line2.set_data(entire_mech_x[mech,i],entire_mech_y[mech,i])    
    line4.set_data(footpath_x[mech,i],footpath_y[mech,i])
    mech='strider'
    line5.set_data(entire_mech_x[mech,i],entire_mech_y[mech,i])
    line6.set_data(footpath_x[mech,i],footpath_y[mech,i])
    mech='klann'
    line7.set_data(entire_mech_x[mech,i],entire_mech_y[mech,i])
    line8.set_data(footpath_x[mech,i],footpath_y[mech,i])
    return line8,line7,line6,line5,line4,line3, line2, line,

calc_joints()

ani = animation.FuncAnimation(fig1, animate, frames=loop_count,interval=0, blit=False, init_func=init)
#to make the animation run faster, set blit=True
#ani = animation.FuncAnimation(fig1, animate, frames=loop_count,interval=0, blit=True, init_func=init)
plt.show() 

