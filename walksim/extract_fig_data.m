%get the handle of the current figure
R7run1 = gcf;

%get the handles of the active axes
axes = get(R7run1, 'Children');

%get the handles of the data objects associated with each axis
data = get(axes, 'Children');

%these cell variables will hold the extracted data
x_data = {};
y_data = {};
z_data = {};

%go through all line objects and extract X and Y data
for i=1:length(data)
    object_type = get(data{i}, 'Type');
    current_data = data{i};
        for j=1:length(object_type)
            for k=1:length(object_type)
                x_data = vertcat(x_data, get(current_data(i)));
                y_data = vertcat(y_data, get(current_data(j)));
                z_data = vertcat(z_data, get(current_data(k)));
            end
        end
end

