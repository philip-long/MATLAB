% (********************************************)
% (** SYMORO+ : SYmbolic MOdelling of RObots **)
% (**========================================**)
% (**      IRCCyN-ECN - 1, rue de la Noe     **)
% (**      B.P.92101                         **)
% (**      44321 Nantes cedex 3, FRANCE      **)
% (**      www.irccyn.ec-nantes.fr           **)
% (********************************************)

% Copyright (c) 2012 Philip Long
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.
%    Name of file : D:/Documents/KUKA/SymoroFiles/KukaRed.fgm




%      Geometric parameters   


% j        ant      mu       sigma    gamma    b        alpha    d        theta    r


% 1        0        1        0        0        0        0        0        t1       r1

%                                                       Pi                -Pi
% 2        1        1        0        0        0        --       0        --- + t2 0
%                                                       2                  2
%                                                       -Pi
% 3        2        1        0        0        0        ---      0        t3       r3
%                                                        2
%                                                       -Pi
% 4        3        1        0        0        0        ---      0        t4       0
%                                                        2



%   Transformation Matrix From  R -1  to  R  4 :
% -------------------------------------------
% Equations:

% Declaration of the function
function TFT4=T40(u)




global r1 r3 r5 r7 Joint2Offset
% Declaration of global output variables


% Function description:
t1=u(1);
t2=u(2)+Joint2Offset;
t3=u(3);
t4=u(4);



% Function description:

	U1T211=cos(t1).*sin(t2);
	U1T212=cos(t1).*cos(t2);
	U1T221=sin(t1).*sin(t2);
	U1T222=cos(t2).*sin(t1);
	U1T311=U1T211.*cos(t3) - sin(t1).*sin(t3);
	U1T312=-(cos(t3).*sin(t1)) - U1T211.*sin(t3);
	U1T314=r3.*U1T212;
	U1T321=U1T221.*cos(t3) + cos(t1).*sin(t3);
	U1T322=cos(t1).*cos(t3) - U1T221.*sin(t3);
	U1T324=r3.*U1T222;
	U1T331=-(cos(t2).*cos(t3));
	U1T332=cos(t2).*sin(t3);
	U1T334=r1 + r3.*sin(t2);
	U1T411=U1T311.*cos(t4) - U1T212.*sin(t4);
	U1T412=-(U1T212.*cos(t4)) - U1T311.*sin(t4);
	U1T421=U1T321.*cos(t4) - U1T222.*sin(t4);
	U1T422=-(U1T222.*cos(t4)) - U1T321.*sin(t4);
	U1T431=U1T331.*cos(t4) - sin(t2).*sin(t4);
	U1T432=-(cos(t4).*sin(t2)) - U1T331.*sin(t4);
	TFT4(1,1)= U1T411;
	TFT4(2,1)= U1T421;
	TFT4(3,1)= U1T431;
	TFT4(1,2)= U1T412;
	TFT4(2,2)= U1T422;
	TFT4(3,2)= U1T432;
	TFT4(1,3)= U1T312;
	TFT4(2,3)= U1T322;
	TFT4(3,3)= U1T332;
	TFT4(1,4)= U1T314;
	TFT4(2,4)= U1T324;
	TFT4(3,4)= U1T334;
    TFT4(4,:)=[0 0 0 1];
end

% *=*
% Number of operations : 11 '+' or '-', 29 '*' or '/'
