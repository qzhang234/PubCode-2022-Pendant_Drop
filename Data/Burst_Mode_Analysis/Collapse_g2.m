

clear all
close all

load('D100_Burst_avg.mat');

ROI_index = 5;

g2 = squeeze(viewresultinfo.result.g2Batchavg);
g2_err = squeeze(viewresultinfo.result.g2BatchavgErr);
t_el = viewresultinfo.result.delay{1,1}';


burst_plot = zeros(numel(t_el),3);

burst_plot(:,1) = t_el;
burst_plot(:,2) = g2(ROI_index,:);
burst_plot(:,3) = g2_err(ROI_index,:);

writematrix(burst_plot,'burst_plot.csv');