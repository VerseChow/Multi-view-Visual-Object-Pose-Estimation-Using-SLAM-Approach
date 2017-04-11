function plotObj(origin_x, origin_y, x_dims, y_dims, color)
if (nargin <4),
 error('Please see help for INPUT DATA.');
elseif (nargin==4)
    style='b-';
    color='b';
end;
line([origin_x-x_dims*0.5, origin_x+x_dims*0.5], [origin_y-y_dims*0.5, origin_y-y_dims*0.5], 'Color', color);
line([origin_x-x_dims*0.5, origin_x+x_dims*0.5], [origin_y+y_dims*0.5, origin_y+y_dims*0.5], 'Color', color);
line([origin_x-x_dims*0.5, origin_x-x_dims*0.5], [origin_y-y_dims*0.5, origin_y+y_dims*0.5], 'Color', color);
line([origin_x+x_dims*0.5, origin_x+x_dims*0.5], [origin_y-y_dims*0.5, origin_y+y_dims*0.5], 'Color', color);

end