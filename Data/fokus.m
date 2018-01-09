A = importdata('Log_mads_kicked_it.dat',',');

ax1 = subplot(2,1,1);
ax2 = subplot(2,1,2);

plot(ax1,A(:,2),A(:,3));
xlabel(ax1,'X');
ylabel(ax1,'Y');
title(ax1, 'Y-position as a function of X-position');

plot(ax2,A(:,2),A(:,3));
xlabel(ax2,'X');
ylabel(ax2,'Y');
title(ax2, 'Same, zoomed');
ylim(ax2, [-0.6 2.3]);
xlim(ax2, [1.8 2.2]);

print('DrivingCorrection','-dpng');
