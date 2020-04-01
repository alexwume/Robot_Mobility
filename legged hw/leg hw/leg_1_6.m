close all
clear all
yo=1;
g=9.81;
vv=linspace(0,3,50);
pp=linspace(0.2,-0.05,50);
[p,vo]=meshgrid(pp,vv);
xt=-p+sqrt(p.^2+(vo.^2).*yo/g);
surface(vo,p,xt);
view(3)
xlabel('V')
ylabel('P')
zlabel('Xt')


