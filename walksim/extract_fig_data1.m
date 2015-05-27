open('R7run1.fig');
h = gcf;
      axesObjs = get(h, 'Children');  %axes handles

      %dataObjs = get(axesObjs, 'Children');

%objTypes = get(dataObjs, 'Surface');
dataObjs = axesObjs.Children;

xdata = get(dataObjs, 'XData');  %data from low-level grahics objects

ydata = get(dataObjs, 'YData');

zdata = get(dataObjs, 'ZData');
