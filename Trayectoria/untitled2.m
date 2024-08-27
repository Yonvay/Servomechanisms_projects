Home = [0, 0, 0, 0, 0, 0];
l = 10;
DHparameters = [[0    0    l    0     0    0];
                [0    0    0    -pi/2 0    0];
                [0    0    2*l  0     0    pi/6];
                [0    0    2*l  0     0    -pi/2];
                [0    0    0    pi/2  0    pi/3];
                [0    0    0    0     1    2*l];
                [0    0    0    0     0    0]];
L(1) = Link(DHparameters(1,:));
L(2) = Link(DHparameters(2,:));
L(3) = Link(DHparameters(3,:));
L(4) = Link(DHparameters(4,:));
L(5) = Link(DHparameters(5,:));
L(6) = Link(DHparameters(6,:));
L(7) = Link(DHparameters(7,:));
phantomX = SerialLink(L);
phantomX.tool = trotx(0) * transl(0, 0, 11.19)
phantomX.name = 'Home';
figure()
phantomX.plot(Home);
view([-22.3 7.9])
zlim([-30 45])
%Funciones
function [H] = MTH(rowDH)
    theta = rowDH(1);
    d = rowDH(2);
    a = rowDH(3);
    alpha = rowDH(4);
    offset = rowDH(5);
    H = trotz(theta + offset)*transl(0,0,d)*transl(a,0,0)*trotx(alpha);
end