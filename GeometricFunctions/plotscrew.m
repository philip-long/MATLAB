%% 
function plotscrew(S)
% This function plots a 0 or infinite pitch
% screw 
% case one if the screw is infinite pitch
tol=0.00001;%tolerance
if abs(S(1))<tol && abs(S(2))<tol && abs(S(3))<tol
    %infinite pitch screw plot direction only
    quiver3(0,0,0,S(4),S(5),S(6),10,'r-')
else
    % 0 pitch screw 
    % Extract point on screw access
    A=[S(4);S(5);S(6)]*[S(1) S(2) S(3)];
    P(1)=A(3,2);
    P(2)=A(1,3);
    P(3)=A(2,1);
    quiver3(P(1),P(2),P(3),S(1),S(2),S(3),10,'b')
end
