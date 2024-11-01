function Ang2 = rot2euler(R)
x = atan2(R(3,2),R(3,3));
y = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
z = atan2(R(2,1),R(1,1));
Ang2 = [x y z];
end
