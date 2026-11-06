%% Best Result v17: Final Synchronized Dual-Axis
% Fixed Zoom Synchronization using Property Links and Listeners

clear; clc; close all;

%% USER ADJUSTABLE PARAMETERS
USER_OFFSET_M = 3;  
c = 3e8;            
fs = 200e6;         

%% Section 1: Load and Preprocess Data
% Work based on academic projects regarding autonomous vehicle safety [cite: 3, 22]
load('raw_data_anechoic_chamber.mat');
data = all_data_trunc;
[n_scans_full, n_time, n_cols] = size(data);

trt = (1:n_time) / fs;
range_rt_orig = trt * c;  
data_avg = mean(data, 3);

% Signal Alignment
ref_scan = data_avg(1, :);
data_aligned_full = data_avg;
for k = 2:n_scans_full
    xc = xcorr(abs(data_avg(k,:)), abs(ref_scan));
    [~, lag_idx] = max(xc);
    lag(k) = lag_idx - n_time;
    F = fft(data_avg(k, :));
    freqs = (0:n_time-1) - floor(n_time/2);
    freqs = ifftshift(freqs);
    phase_ramp = exp(-1j * 2*pi * lag(k) * freqs / n_time);
    data_aligned_full(k, :) = ifft(F .* phase_ramp);
end
n_scans = 20; 
data_aligned = data_aligned_full(1:n_scans, :);

%% Section 2: Processing
mask_clean = true(1, n_time);
mask_clean(100:200) = false; 
energy_per_scan = sum(abs(data_aligned(:, mask_clean)).^2, 2);
median_e = median(energy_per_scan);
energies_norm = sqrt(energy_per_scan / median_e);
data_B = data_aligned ./ energies_norm;
data_B_sub = data_B - mean(data_B, 1); 
kernel = ones(3, 5) / 15;
env_smooth = conv2(abs(data_B_sub), kernel, 'same');

%% Section 3 & 4: Calibration and Axis Transformation
mean_env = mean(env_smooth, 1);
[~, peak_idx] = max(mean_env);
zero_ref_range = range_rt_orig(peak_idx) - 0.5;

range_shifted_rt = range_rt_orig - zero_ref_range;
range_shifted_ow = range_shifted_rt / 2;       
fast_time_ns = 2 * range_shifted_ow / c * 1e9; 

%% Section 5: Professional Visualization
to_dB = @(x) 20*log10(abs(x) + 1e-10)-100;
fig = figure('Name','Synchronized Radar Analysis','Position', [50 50 1700 600]);
scan_axis = 1:n_scans;

zoom_idx = (range_shifted_ow >= -8) & (range_shifted_ow <= 22);
z_time = fast_time_ns(zoom_idx);

% Subplot (a)
ax1 = subplot(1, 3, 1);
render_dual_axis_plot(ax1, scan_axis, z_time, to_dB(data_B_sub(:, zoom_idx)'), c);
title('(a) Background Removed');
% ground truth

% Subplot (b)
ax2 = subplot(1, 3, 2);
render_dual_axis_plot(ax2, scan_axis, z_time, to_dB(env_smooth(:, zoom_idx)'), c);
title('(b) Moving Average Filtered');

% Subplot (c)
Ninterp = 4;
z_range = range_shifted_ow(zoom_idx);
[Xz, Yz] = meshgrid(z_range, 1:n_scans);
[Xzi, Yzi] = meshgrid(linspace(z_range(1), z_range(end), Ninterp*length(z_range)), ...
                      linspace(1, n_scans, Ninterp*n_scans));
roi_interp = interp2(Xz, Yz, env_smooth(:, zoom_idx), Xzi, Yzi, 'cubic');
interp_t_ns = 2 * Xzi(1,:) / c * 1e9;

ax3 = subplot(1, 3, 3);
render_dual_axis_plot(ax3, Yzi(:,1), interp_t_ns, to_dB(roi_interp'), c);
title('(c) 4x Interpolated');

sgtitle('Range - Slow Time Map', 'FontSize', 16, 'FontWeight','bold');

% Get current positions
p1 = ax1.Position;
p2 = ax2.Position;
p3 = ax3.Position;

gap = 0.1;    % smaller gap

w = (1 - 2*0.08 - 2*gap)/3;   % width of each subplot

ax1.Position = [0.08 0.15 w 0.75];
ax2.Position = [0.08+w+gap 0.15 w 0.75];
ax3.Position = [0.08+2*(w+gap) 0.15 w 0.75];
%% RMSE calculation
slope_p = 6 / 10;
slope_n = -6 / 9;
loc_p = slope_p * (1:11) ;
loc_n = slope_n * (12:20) + 12;
ground_truth = [loc_p, loc_n];
scan_axis_ip = linspace(scan_axis(1), scan_axis(end), Ninterp*length(scan_axis));
z_range_ip = linspace(z_range(1), z_range(end), Ninterp*length(z_range));
ground_truth_ip = interp1(scan_axis ,ground_truth, scan_axis_ip);
range_shifted_ow_ip = interp1(z_range, range_shifted_ow(zoom_idx), z_range_ip);

for i = 1:20

    peak1(i) = find(to_dB(data_B_sub(i,:)) == max(to_dB(data_B_sub(i,:))));
    peak2(i) = find(to_dB(env_smooth(i,:)) == max(to_dB(env_smooth(i,:))));

end
for i = 1: 80
    peak3(i) = find(to_dB(roi_interp(i,:)) == max(to_dB(roi_interp(i,:))));
end 

e1 = ground_truth - range_shifted_ow(peak1);
e2 = ground_truth - range_shifted_ow(peak2);
e3 = ground_truth_ip - range_shifted_ow_ip(peak3);

rmse1 = sqrt(sum((e1).^2)/20)
rmse2 = sqrt(sum((e2).^2)/20)
rmse3 = sqrt(sum((e3).^2)/80)

std1 = std(e1)
std2 = std(e2)
std3 = std(e3)

max1 = max(e1)
max2 = max(e2)
max3 = max (e3)


%% Delay stats
mean_delay = mean(lag) * 5e-9
var_lag = var(lag) * 5e-9
figure
hl = histogram(lag)
ylabel('Frequency','FontName','Arial', 'FontSize',14);
xlabel('Delay (samples)', 'FontName','Arial', 'FontSize',14);
 title('Histogram of Processing Delay Misalignment','FontSize',14)
    h1.FontSize = 14;
    h1.FontName = 'Arial'
%% Helper Function with Real-Time Synchronization
function render_dual_axis_plot(ax, x, y, z, c_speed)
    % 1. Setup Left Side (Time)
    yyaxis(ax, 'left');
    imagesc(ax, x, y, z);
    axis(ax, 'xy');
    ylabel('ًRoundtrip Fast Time (ns)','FontName','Arial', 'FontSize',14);
    xlabel('Scan Index', 'FontName','Arial', 'FontSize',14);
    ax.YColor = 'k';
    colormap(ax, 'jet');
    hold(ax, 'on');
    yline(ax, 0, 'k--', 'LineWidth', 1.5);
    yline(ax, 40, 'k--', 'LineWidth', 1.5);
    line([1 11],[0 40], 'LineStyle','--','Color','w','LineWidth',3);
    line([11 20], [40 0], 'LineStyle','--','Color','w','LineWidth',3);
    ax.FontSize = 14;
    ax.FontName = 'Arial'


    % 2. Setup Right Side (Distance)
    yyaxis(ax, 'right');
    ylabel('Range (m)', 'FontName','Arial', 'FontSize',14);
    ax.YColor = 'k';

    % 3. HARD LINK: This forces the zoom to happen on both rulers simultaneously
    % We link the ruler objects themselves
    leftRuler = ax.YAxis(1);
    rightRuler = ax.YAxis(2);
    linkprop([leftRuler, rightRuler], 'Limits');

    % 4. DYNAMIC TICK UPDATE: Update labels whenever the zoom changes
    addlistener(ax, 'YLim', 'PostSet', @(src, event) sync_labels(ax, c_speed));
    
    % Initial sync
    sync_labels(ax, c_speed);
    colorbar
end

function sync_labels(ax, c_speed)
    % Synchronize tick positions to match the left axis exactly
    ax.YAxis(2).TickValues = ax.YAxis(1).TickValues;
    % Calculate and update labels
    ticks = ax.YAxis(1).TickValues;
    labels = arrayfun(@(t) sprintf('%.1f', t * c_speed / 2 / 1e9), ticks, 'UniformOutput', false);
    ax.YAxis(2).TickLabels = labels;
end


