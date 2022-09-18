%% Plotting demo with trivial solution
load("no_admittance.mat");

f = figure;
f.Renderer = 'painters';
grid on
hold on
plot(time(1,:),pos_nom(3,:),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z_{r}/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)

f = figure;
f.Renderer = 'painters';
grid on
hold on
plot(time(1,:),pos_nom(3,:),'LineWidth',2);
xlabel('$t/\mathrm{s}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z_{r}/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
