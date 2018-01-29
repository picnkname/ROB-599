function [hourglass_points] = gen_hourglass(scale,grain)
%
t = 0:pi/grain:2*pi;
[X,Y,Z] = cylinder(1+cos(t));
%
x_lin = scale*X(:);
y_lin = scale*Y(:);
z_lin = scale*Z(:);
%
%
hourglass_points = cat(2, x_lin, y_lin, z_lin, ones(size(x_lin)) );
%
% %% create plot of original hourglass at starting location in world space
% s = 20;%S(:); % want all markers uniform size
% hc = 100*Z(:);%color gradient in z direction
% figure;
% scatter3(x_lin, y_lin, z_lin, s,100*hc,'filled'); hold on;
% xlabel('X','Color','red','FontSize',14)
% ylabel('Y','Color','red','FontSize',14)
% zlabel('Z','Color','red','FontSize',14)
% title('Starting Location of Hourglass in World Frame')
% % plot starting coordinate axes
% arrow3D([0,0,0], [0,0,30],'r',0.9);hold on;
% arrow3D([0,0,0], [0,50,0],'r',0.9);hold on;
% arrow3D([0,0,0], [50,0,0],'r',0.9);hold on;
%
% EOF
end