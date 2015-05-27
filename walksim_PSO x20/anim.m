function anim(x, force);
% anim.m: make a movie of a simulation result x(t)

	% x can be a series of column vectors, or a file containing such data
	if isstr(x)
		load(x);
		x = Result.x;
		% if this was a half gait cycle, create a full one
		if Result.problem.symmetry
			x2 = x(Result.problem.vmx,:);
			x2(1,:) = x2(1,:) + Result.dur * Result.problem.speed;
			x = [x x2];
		end

		% create another gait cycle
		x2 = x;
		x2(1,:) = x2(1,:) + 2*Result.dur * Result.problem.speed;
		x = [x x2];
		
	end
	nframes = size(x,2);	

	fps = 25;
	
	% initialize movie file
	avi = VideoWriter('anim.avi');
	open(avi);

	% initialize figure window
	close all
	figure(1);
	clf;
	set(gcf,'Position',[10 10 550 550]);
	set(gcf, 'color', 'white');
	
	% determine size of ground
	xmin = min(x(1,:)) - 1;
	xmax = max(x(1,:)) + 1;
	
	% determine how much the model moves per frame
	speed = (max(x(1,:)) - min(x(1,:))) / nframes;
	
	% create ground points (random dots)
	np = 5000*round(xmax-xmin);
	xg = rand(np,1);
	yg = rand(np,1);
	xg = xmin + (xmax-xmin)*[xg ; 2-xg];
	yg = -0.15*[yg ; yg];
	
	% make the movie
	R = [1:6 4];			% right stick points
	L = [2 7:10 8];			% left stick points
	u = zeros(9,1);
	for i=0:nframes-1
		plot(xg,yg,'.','Color',[0.7 0.7 0.7],'MarkerSize',4);
		hold on
		[xdot, grf, d] = gait2dem(x(:,i+1),u);
		d = reshape(d',2,10)';
		plot(d(R,1),d(R,2),'r',d(L,1),d(L,2),'b','LineWidth',2);
		if (nargin == 2)
			plot([d(2,1) d(2,1)+force(i+1)/100], [d(2,2) d(2,2)],'k');
		end
		axis('equal');
        axis([-1.5+x(1,i+1) 1.5+x(1,i+1) -0.2 1.5]);
		axis('off');
		if (i==0)
			F = getframe(gca);
			frame = [1 1 size(F.cdata,2) size(F.cdata,1)];
		else
			F = getframe(gca,frame);
		end
		writeVideo(avi,F);
		cla;
	end

	close(avi);
	hold off;
	close all
end