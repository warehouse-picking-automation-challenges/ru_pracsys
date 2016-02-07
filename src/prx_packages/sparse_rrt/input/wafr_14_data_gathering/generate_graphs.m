function generate_graphs(plant,planner1,planner2,planner3,x_index,y_index,data_size)
% close all;
width = 3.5;     % Width in inches
height = 1.75;    % Height in inches
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);

% plant = 'point';
% planner1 = 'rrt';
% planner2 = 'rrt_star';
% planner3 = 'drain_rrt';
planner1_index = [x_index,y_index];
planner2_index = [x_index,y_index];
planner3_index = [x_index,y_index];
if strcmp(planner1,'rrt_star') | strcmp(planner1,'rrt_star_shooting')
    if x_index==6
        planner1_index(1) = x_index-1;
    end
    if y_index==6
        planner1_index(2) = y_index-1;
    end
end
if strcmp(planner2,'rrt_star') | strcmp(planner2,'rrt_star_shooting')
    if x_index==6
        planner2_index(1) = x_index-1;
    end
    if y_index==6
        planner2_index(2) = y_index-1;
    end
end
if strcmp(planner3,'rrt_star') | strcmp(planner3,'rrt_star_shooting')
    if x_index==6
        planner3_index(1) = x_index-1;
    end
    if y_index==6
        planner3_index(2) = y_index-1;
    end
end
x_limit = 500000;
% data_size = 1000;
start_of_data = 5;
inc_of_data = 35;
error_bar_flag = 0;



planner_1_x = zeros(50,data_size,1);
planner_2_x = zeros(50,data_size,1);
planner_3_x = zeros(50,data_size,1);
planner_1_y = zeros(50,data_size,1);
planner_2_y = zeros(50,data_size,1);
planner_3_y = zeros(50,data_size,1);
for k = 1:50
    filename = strcat('./data/',plant,'/',plant,'_',planner1,'_',num2str(k-1),'_',plant,'_',planner1,'_',num2str(k-1),'.txt');
    data = load(filename);
    planner_1_x(k,:) = data(:,planner1_index(1));
    planner_1_y(k,:) = data(:,planner1_index(2));
    filename = strcat('./data/',plant,'/',plant,'_',planner2,'_',num2str(k-1),'_',plant,'_',planner2,'_',num2str(k-1),'.txt');
    data = load(filename);
    planner_2_x(k,:) = data(:,planner2_index(1));
    planner_2_y(k,:) = data(:,planner2_index(2));
    if ~strcmp(planner3,'')
        filename = strcat('./data/',plant,'/',plant,'_',planner3,'_',num2str(k-1),'_',plant,'_',planner3,'_',num2str(k-1),'.txt');
        data = load(filename);
        planner_3_x(k,:) = data(:,planner3_index(1));
        planner_3_y(k,:) = data(:,planner3_index(2));
    end
end
%compute the data
planner1_x_axis = mean(planner_1_x,1);
planner2_x_axis = mean(planner_2_x,1);
planner3_x_axis = mean(planner_3_x,1);

planner1_mean = mean(planner_1_y,1);
planner2_mean = mean(planner_2_y,1);
planner3_mean = mean(planner_3_y,1);

if strcmp(planner1,'drain_rrt')
    if y_index==3
        planner1_mean = planner1_mean.*2;
    end
end
if strcmp(planner2,'drain_rrt')
    if y_index==3
        planner2_mean = planner2_mean.*2;
    end
end
if strcmp(planner3,'drain_rrt')
    if y_index==3
        planner3_mean = planner3_mean.*2;
    end
end

planner1_var = sqrt(var(planner_1_y,1));
planner2_var = sqrt(var(planner_2_y,1));
planner3_var = sqrt(var(planner_3_y,1));

f = figure( 'Visible', 'off' );
set(findall(f,'type','text'),'fontSize',8,'fontWeight','bold')
hold on;
if error_bar_flag
    e1 = errorbar(planner1_x_axis(start_of_data:inc_of_data:end),...
                  planner1_mean(start_of_data:inc_of_data:end),...
                  planner1_var(start_of_data:inc_of_data:end),':o');
    set(e1,'Color',[.7 .7 .7]);
    if ~strcmp(planner3,'')
        e3 = errorbar(planner2_x_axis(start_of_data:inc_of_data:end),...
                      planner2_mean(start_of_data:inc_of_data:end),...
                      planner2_var(start_of_data:inc_of_data:end),'--x');
        set(e3,'Color',[.4 .4 .4]);
        e4 = errorbar(planner3_x_axis(start_of_data:inc_of_data:end),...
                      planner3_mean(start_of_data:inc_of_data:end),...
                      planner3_var(start_of_data:inc_of_data:end),'-s');
        set(e4,'Color',[0 0 0]);
    else
        e3 = errorbar(planner2_x_axis(start_of_data:inc_of_data:end),...
                      planner2_mean(start_of_data:inc_of_data:end),...
                      planner2_var(start_of_data:inc_of_data:end),'--s');
        set(e3,'Color',[0 0 0]);
    end
else
    e1 = plot(planner1_x_axis(start_of_data:inc_of_data:end),...
                  planner1_mean(start_of_data:inc_of_data:end),':o');
    set(e1,'Color',[.7 .7 .7]);
    if ~strcmp(planner3,'')
        e3 = plot(planner2_x_axis(start_of_data:inc_of_data:end),...
                      planner2_mean(start_of_data:inc_of_data:end),'--x');
        set(e3,'Color',[.4 .4 .4]);
        e4 = plot(planner3_x_axis(start_of_data:inc_of_data:end),...
                      planner3_mean(start_of_data:inc_of_data:end),'-s');
        set(e4,'Color',[0 0 0]);
    else
        e3 = plot(planner2_x_axis(start_of_data:inc_of_data:end),...
                      planner2_mean(start_of_data:inc_of_data:end),'--s');
        set(e3,'Color',[0 0 0]);
    end
end
% set(e4,'Color',[1.0000    0.1034    0.7241]);
xlim([0,x_limit]);

legend_name1 = '';
legend_name2 = '';
legend_name3 = '';

if strcmp(planner1,'rrt')
    legend_name1 = 'RRT';
elseif strcmp(planner1,'rrt_star')
    legend_name1 = 'RRT*';
elseif strcmp(planner1,'drain_rrt')
    legend_name1 = 'SST';
elseif strcmp(planner1,'rrt_star_shooting')
    legend_name1 = 'Shooting Variant';
end
if strcmp(planner2,'rrt')
    legend_name2 = 'RRT';
elseif strcmp(planner2,'rrt_star')
    legend_name2 = 'RRT*';
elseif strcmp(planner2,'drain_rrt')
    legend_name2 = 'SST';
elseif strcmp(planner2,'rrt_star_shooting')
    legend_name2 = 'Shooting Variant';
end

if ~strcmp(planner3,'')
if strcmp(planner3,'rrt')
    legend_name3 = 'RRT';
elseif strcmp(planner3,'rrt_star')
    legend_name3 = 'RRT*';
elseif strcmp(planner3,'drain_rrt')
    legend_name3 = 'SST';
elseif strcmp(planner3,'rrt_star_shooting')
    legend_name3 = 'Shooting Variant';
end
end

l = legend(legend_name1,legend_name2,legend_name3,'Location','NorthEast');
set([l, gca]             , ...
    'FontSize'   , 8           );

x_label_text = '';
y_label_text = '';
pretty_plant = '';
pretty_x = '';
pretty_y = '';

if strcmp(plant,'point')
    pretty_plant = 'Point System';
elseif strcmp(plant,'acrobot')
    pretty_plant = 'Two-Link Acrobot';
elseif strcmp(plant,'cart_pole')
    pretty_plant = 'Cart-Pole';
elseif strcmp(plant,'quadrotor')
    pretty_plant = 'Quadrotor';
elseif strcmp(plant,'pendulum')
    pretty_plant = 'Simple Pendulum';
elseif strcmp(plant,'double_integrator')
    pretty_plant = 'Two-Dimensional Double Integrator';
elseif strcmp(plant,'physics_car')
    pretty_plant = 'Physically-Simulated Car';
 
end


if planner1_index(1)==1
    x_label_text = ' Time (s)';
    pretty_x = 'time';
elseif planner1_index(1)==2
    x_label_text = ' Iterations';
    pretty_x = 'iterations';
elseif planner1_index(1)==3
    x_label_text = ' Nodes';
    pretty_x = 'nodes';
elseif planner1_index(1)==4
    x_label_text = ' Solution Length (s)';
    pretty_x = 'quality';
elseif planner1_index(1)==6
    x_label_text = ' Average Solution Length (s)';
    pretty_x = 'avg_quality';
end

if planner1_index(2)==1
    y_label_text = 'Time (s)';
    pretty_y = 'time';
elseif planner1_index(2)==2
    y_label_text = 'Iterations';
    pretty_y = 'iterations';
elseif planner1_index(2)==3
    y_label_text = 'Nodes';
    pretty_y = 'nodes';
elseif planner1_index(2)==4
    y_label_text = 'Solution Length (s)';
    pretty_y = 'quality';
elseif planner1_index(2)==6
    y_label_text = 'Average Solution Length (s)';
    pretty_y = 'avg_quality';
end
    

xlabel(x_label_text);
ylabel(y_label_text);
temp = ' v.s. ';
title( strcat(y_label_text,temp,x_label_text,' (',pretty_plant,')'));
print(strcat('figures/',pretty_y,'_',pretty_x,'_',plant),'-dpng','-r300');

end
