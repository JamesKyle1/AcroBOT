function writeAcroBOTData(data,params,filePath)


header1 = ["l1 [m]" "l2 [m]" "l3 [m]" "c1 [m]" "c2 [m]" "c3 [m]" "m1 [kg]" "m2 [kg]" "m3 [kg]"];
header2 = ["t [s]" "th1 [rad]" "th2 [rad]" "th3 [rad]" "th1d [rad/s]" "th2d [rad/s]" "th3d [rad/s]"];

writematrix("=========================== Parameters =====================================",filePath);
writematrix(header1,filePath,'Delimiter','tab','WriteMode','append');
writematrix(params,filePath,'WriteMode','append','Delimiter','tab');
writematrix("============================== Data ========================================",filePath,'WriteMode','append');

writematrix(header2,filePath,'Delimiter','tab','WriteMode','append');
writematrix(data,filePath,'WriteMode','append','Delimiter','tab');

end