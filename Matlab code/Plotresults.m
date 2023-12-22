
figure(1);
percentage = 0.35:0.1:20;

subplot(1,3,1)

%Time with mitigation
p1 =   0.0003429;
p2 =    0.006743;
p3 =       18.58;
Time_with_mitigation = p1*percentage.^2 + p2*percentage + p3;
plot(percentage,Time_with_mitigation)
hold on

%Time without mitigation
p1 =     -0.2092;
p2 =       8.061;
p3 =       15.66;
Time_without_mitigation = p1*percentage.^2 + p2*percentage + p3;
plot(percentage,Time_without_mitigation)
hold on



xlabel("Percentage of Spoofed CAVs")
ylabel("Ave. Travel Time")
legend("With-mitigation","Without-mitigation")



subplot(1,3,2)
percentage = 0:0.1:20;
%Energy with mitigation
p1 =    0.004143 ;
p2 =     0.02454 ;
p3 =       14.57;
L2u_with_mitigation = p1*percentage.^2 + p2*percentage + p3;
plot(percentage,L2u_with_mitigation)
hold on



%Energy without mitigation
       p1 =     -0.3188;
       p2 =       11.09;
       p3 =        14.5;
L2u_without_mitigation = p1*percentage.^2 + p2*percentage + p3;
plot(percentage,L2u_without_mitigation)
hold on
xlabel("Percentage of Spoofed CAVs")
ylabel("Ave. Energy Consumption")
legend("With-mitigation","Without-mitigation")



subplot(1,3,3)
% Fuel_withoutmitigation

percentage = 0.3:0.1:20;
p1 =  -0.0008333;
p2 =     0.03306;
p3 =     -0.5183;
p4 =       29.26;
Fuel_with_mitigation = p1*percentage.^3 + p2*percentage.^2 + p3*percentage + p4;
plot(percentage,Fuel_with_mitigation)
hold on


p1 =     -0.0019;
p2 =     0.0344;
p3 =      0.3269;
p4 =       29.02;
Fuel_without_mitigation = p1*percentage.^3 + p2*percentage.^2 + p3*percentage + p4;
plot(percentage,Fuel_without_mitigation)
xlabel("Percentage of Spoofed CAVs")
ylabel("Fuel Consumption")


% Fuel_withmitigation
xlabel("Percentage of Spoofed CAVs")
ylabel("Ave. Fuel Consumption")
legend("With-mitigation","Without-mitigation")

