function plotrobot( xpos, ypos, theta, color, filled, fillColor)
% PLOTROBOT

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on
end

radius = 0.05;
d_radius = 0.025;
plotcircle( [xpos ypos], radius, 100, color, filled, fillColor);

orientationLine = [ xpos xpos+cos(theta)*(radius+d_radius);
		            ypos ypos+sin(theta)*(radius+d_radius)];

plot( orientationLine(1,:), orientationLine(2,:),'Color','black',...
      'LineWidth', 2);

if ~WAS_HOLD
    hold off
end
