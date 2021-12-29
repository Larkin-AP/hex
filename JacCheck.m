%% check 
clear all
clc

log = readmatrix('hex_forward(check).txt');
m4 = log(:,1);
m5 = log(:,2);
m6 = log(:,3);
ee_fx = log(:,4);
ee_fy = log(:,5);
ee_fz = log(:,6);
ee_bx = log(:,7);
ee_by = log(:,8);
ee_bz = log(:,9);
t=0.001:0.001:size(log(:,1))/1000;
t1=t(1:end-1);

diff_m4 = m4(2:end)-m4(1:end-1);
diff_m5 = m5(2:end)-m5(1:end-1);
diff_m6 = m6(2:end)-m6(1:end-1);

ee_relx = ee_fx-ee_bx;
ee_rely = ee_fy-ee_by;
ee_relz = ee_fz-ee_bz;

dif_relx = ee_relx(2:end)-ee_relx(1:end-1);
dif_rely = ee_rely(2:end)-ee_rely(1:end-1);
dif_relz = ee_relz(2:end)-ee_relz(1:end-1);

J = Jacob([m4(100),m5(100),m6(100)]);
xx = J*[diff_m4(100),diff_m5(100),diff_m6(100)]'
dif = [dif_relx(100),dif_rely(100),dif_relz(100)]

