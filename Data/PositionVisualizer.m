A = importdata('Log.dat',',');

ax1 = subplot(2,2,1);
ax2 = subplot(2,2,2);
ax3 = subplot(2,2,3);
ax4 = subplot(2,2,4);

plot(ax1,A(:,1),A(:,2));
xlabel(ax1,'Time');
ylabel(ax1,'X');
title(ax1, 'X-position as a function of time');

plot(ax2,A(:,1),A(:,3));
xlabel(ax2,'Time');
ylabel(ax2,'Y');
title(ax2, 'Y-position as a function of time');

plot(ax3,A(:,1),A(:,4));
xlabel(ax3,'Time');
ylabel(ax3,'Theta');
title(ax3, 'Theta as a function of time');

plot(ax4,A(:,2),A(:,3));
xlabel(ax4,'X');
ylabel(ax4,'Y');
title(ax4, 'Y-position as a function of X');
