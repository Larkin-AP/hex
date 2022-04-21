 V=[-0.8130   -0.0358   -0.5812;
    0.5816   -0.0000   -0.8135;
   -0.0291    0.9994   -0.0208];
 D = [0.002,0.0069,0.0079]; 
gtEllip=ellipsoidalFit.groundtruth([],[0,0,0],D,[0,0,0]);
gtEllip.R=V;
plot(gtEllip);
view(25,10);