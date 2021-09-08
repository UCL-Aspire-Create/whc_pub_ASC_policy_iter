function art5_plot(Box4)

%load rotation matrices
P = whc_parameters1();

%def
lw = 3; %plot: line width 
color = 'b';

%def
RLoptitrUmatl = P.Rx(deg2rad(90));

%change of coord: optitr -> matl
Box4.pL1Umatl = RLoptitrUmatl*Box4.pL1Uoptitr; 
Box4.pL2Umatl = RLoptitrUmatl*Box4.pL2Uoptitr; 
Box4.pL4Umatl = RLoptitrUmatl*Box4.pL4Uoptitr; 

%arbitrarily rotate the box iot get a better viewing angle
angle_deg = 90; %[deg]
Box4.pL1Umatl = P.Rz(deg2rad(angle_deg)) * Box4.pL1Umatl;
Box4.pL2Umatl = P.Rz(deg2rad(angle_deg)) * Box4.pL2Umatl;
Box4.pL4Umatl = P.Rz(deg2rad(angle_deg)) * Box4.pL4Umatl;

p_temp14 = Box4.pL4Umatl - Box4.pL1Umatl; %vector of the difference betw 2 other vectors  
p_temp12 = Box4.pL2Umatl - Box4.pL1Umatl; %vector of the difference betw 2 other vectors  
Box4.pL4theorUmatl = P.Rz(deg2rad(-90)) * (p_temp12/norm(p_temp12,2)) * norm(p_temp14,2) + Box4.pL1Umatl;
Box4.pL3theorUmatl = P.Rz(deg2rad(-90)) * (p_temp12/norm(p_temp12,2)) * norm(p_temp14,2) + Box4.pL2Umatl;

%{
%plot points p1 to p4
plot3(Box4.pL1Umatl(1),Box4.pL1Umatl(2),Box4.pL1Umatl(3), 'bo', 'linewidth',lw, 'displayname','p_1^{matl}'); hold on; grid on
plot3(Box4.pL2Umatl(1),Box4.pL2Umatl(2),Box4.pL2Umatl(3), 'go', 'linewidth',lw, 'displayname','p_2^{matl}');
plot3(Box4.pL3theorUmatl(1),Box4.pL3theorUmatl(2),Box4.pL3theorUmatl(3), 'bo', 'linewidth',lw, 'displayname','p_{3,theor}^{matl}'); hold on; grid on
plot3(Box4.pL4theorUmatl(1),Box4.pL4theorUmatl(2),Box4.pL4theorUmatl(3), 'go', 'linewidth',lw, 'displayname','p_{4,theor}^{matl}');
%}

%plot line segments
plot3([Box4.pL1Umatl(1) Box4.pL2Umatl(1)],[Box4.pL1Umatl(2) Box4.pL2Umatl(2)],[Box4.pL1Umatl(3) Box4.pL2Umatl(3)],[color '-'], 'linewidth',lw, 'displayname','line seg' ); hold on; grid on; 
plot3([Box4.pL1Umatl(1) Box4.pL4theorUmatl(1)],[Box4.pL1Umatl(2) Box4.pL4theorUmatl(2)],[Box4.pL1Umatl(3) Box4.pL4theorUmatl(3)],[color '-'], 'linewidth',lw, 'displayname','line seg' );
plot3([Box4.pL3theorUmatl(1) Box4.pL4theorUmatl(1)],[Box4.pL3theorUmatl(2) Box4.pL4theorUmatl(2)],[Box4.pL3theorUmatl(3) Box4.pL4theorUmatl(3)],[color '-'], 'linewidth',lw, 'displayname','line seg' );
plot3([Box4.pL2Umatl(1) Box4.pL3theorUmatl(1)],[Box4.pL2Umatl(2) Box4.pL3theorUmatl(2)],[Box4.pL2Umatl(3) Box4.pL3theorUmatl(3)],[color '-'], 'linewidth',lw, 'displayname','line seg' );

%legend('show');
