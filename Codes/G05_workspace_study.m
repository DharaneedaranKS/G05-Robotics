clear

% EE430
% G05

% D-H Table  
% Format: Link([theta,   d,    a,    alpha,  R/P ])
% R - Revolute (0); P - Prismatic (1)

L(1)= Link ( [0,   426.07,         0,       pi/2,    0 ] );
L(2)= Link ( [0,      138,      -290,         pi,    0 ] );  
L(3)= Link ( [0,      138,         0,       pi/2,    0 ] );  
L(4)= Link ( [0,   333.93,          0,      -pi/2,   0 ] );  
L(5)= Link ( [0,       0,          0,       pi/2,    0 ] );  
L(6)= Link ( [0,      65,          0,          0,    0 ] );  

robot = SerialLink (L);
robot.name = 'G05';
% robot.teach

% Visualizing the robot using the "plot" method

for q1 = 0 : 1 : 2*pi  % 0 to 360
    for q2 = -0.611*pi : 1 : 0.611*pi  % -110 to 110
        for q3 = -0.805*pi : 1 : 0.805*pi  % -145 to 145
            for q4 = 0 : 1 : 2*pi  % 0 to 360
                for q5 = -0.611*pi : 1 : 0.611*pi  % -110 to 110
                    for q6 = 0 : 1 : 2*pi  % 0 to 360
                        robot.plot ([q1, q2, q3, q4, q5, q6])
                        hold on
                        [T,A] = robot.fkine([q1, q2, q3, q4, q5, q6]);
                        for i=1:6
                            trplot(A(i), 'frame', num2str(i))
                        end
                    end
                end
            end
        end
    end
end