% Hexagon 

Sides=0.5;
Origin=[0,0];
Hex1{1}=[Origin(1),Origin(2)]
Hex1{2}=[Sides+Origin(1),Origin(2)]
Hex1{3}=[Sides+Origin(1)+0.5*Sides,sin(pi/3)*Sides+Origin(2)]; 
Hex1{4}=[Sides+Origin(1),sin(pi/3)*2*Sides+Origin(2)]
Hex1{5}=[Origin(1),sin(pi/3)*2*Sides+Origin(2)]
Hex1{6}=[Origin(1)-0.5*Sides,sin(pi/3)*Sides+Origin(2)]

for i=1:6
plot(Hex1{i}(1),Hex1{i}(2),'r*')
hold on
end


Sides=1.0;
Origin=[-0.25,-0.433];
Hex2{1}=[Origin(1),Origin(2)]
Hex2{2}=[Sides+Origin(1),Origin(2)]
Hex2{3}=[Sides+Origin(1)+0.5*Sides,sin(pi/3)*Sides+Origin(2)]; 
Hex2{4}=[Sides+Origin(1),sin(pi/3)*2*Sides+Origin(2)]
Hex2{5}=[Origin(1),sin(pi/3)*2*Sides+Origin(2)]
Hex2{6}=[Origin(1)-0.5*Sides,sin(pi/3)*Sides+Origin(2)]

for i=1:6
plot(Hex2{i}(1),Hex2{i}(2),'bo')
hold on
end

%%


