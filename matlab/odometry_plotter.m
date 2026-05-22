% Real-time robot odometry receiver and plotter
% Packet format (ASCII, newline-terminated):
%   "x_mm,y_mm,theta_deg,d_mm,da_deg\n"
% Requirements: Any MATLAB with JVM — uses Java DatagramSocket, no toolbox needed
% ---------------------------------------------------------------------------

%% Config
UDP_PORT    = 5005;
MAX_HISTORY = 4000;    % maximum 
ARROW_LEN   = 80;     % mm, heading arrow visual length

%% Open UDP socket (Java DatagramSocket — no toolbox required)
try
    socket  = java.net.DatagramSocket(UDP_PORT);
    socket.setSoTimeout(5);              % 5 ms receive timeout so the loop stays responsive
catch ME
    error('Cannot open UDP port %d: %s', UDP_PORT, ME.message);
end
fprintf('Listening on port %d ...\n', UDP_PORT);

pkt_buf = zeros(1, 512, 'int8');
pkt     = java.net.DatagramPacket(pkt_buf, numel(pkt_buf));
cleanup = onCleanup(@() socket.close());  % auto-close on Ctrl-C or error

%% Circular buffers
xs     = nan(1, MAX_HISTORY);
ys     = nan(1, MAX_HISTORY);
ths    = nan(1, MAX_HISTORY);
ts_buf = nan(1, MAX_HISTORY);
n = 0;                               % a counter of total packet recieve 
t0 = tic;                            % stopwatch 

%% Figure : RGB triplets from 0.0 to 1.0 
BG  = [0.10 0.10 0.10]; % very dark grey : background 
BG2 = [0.14 0.14 0.14]; % slightly lighter grey : axes
GR  = [0.28 0.28 0.28]; % grid line color 
W   = [1.00 1.00 1.00]; % white (text, labels)

% At pos 80, 80 , opens a 1150*620 window panel
fig = figure('Name','Robot Odometry Gr:6 — Live','NumberTitle','off', ...
             'Color', BG, 'Position', [80 80 1150 620]);

% XY trajectory [left, bottom, width, height] (left)

ax_xy = subplot('Position',[0.05 0.09 0.44 0.84]);
hold(ax_xy,'on');

% Robot path history : blue
h_trail = plot(ax_xy, NaN, NaN, '-',  'Color',[0.30 0.70 1.00], 'LineWidth',1.3); 

% Robot current position: red filled circle 
h_dot   = plot(ax_xy, NaN, NaN, 'o',  'MarkerSize',9, ...
               'MarkerFaceColor',[1.0 0.30 0.30], 'MarkerEdgeColor',W);

% Robot current heading direction: yellow arrow
h_arrow = quiver(ax_xy, NaN, NaN, NaN, NaN, 0, ...
                 'Color',[1.0 0.88 0.20], 'LineWidth',2, 'MaxHeadSize',2.5);
xlabel(ax_xy,'X (mm)','Color',W); ylabel(ax_xy,'Y (mm)','Color',W);
title(ax_xy,'XY Trajectory  (KF fused pose)','Color',W);
grid(ax_xy,'on'); axis(ax_xy,'equal');
set(ax_xy,'Color',BG2,'XColor',W,'YColor',W,'GridColor',GR,'GridAlpha',0.5);

% Heading over time (top-right) : Purple line
ax_th = subplot('Position',[0.57 0.55 0.41 0.38]);
h_theta = plot(ax_th, NaN, NaN, '-', 'Color',[0.75 0.40 1.00], 'LineWidth',1.3);
xlabel(ax_th,'Time (s)','Color',W); ylabel(ax_th,'Theta (deg)','Color',W);
title(ax_th,'Heading over Time','Color',W);
grid(ax_th,'on'); ylim(ax_th,[-195 195]);
set(ax_th,'Color',BG2,'XColor',W,'YColor',W,'GridColor',GR,'GridAlpha',0.5);

% Live status (bottom-right)
ax_st = subplot('Position',[0.57 0.06 0.41 0.41]);
axis(ax_st,'off'); set(ax_st,'Color',BG);

% Canvas for text 
h_status = text(ax_st, 0.04, 0.95, 'Waiting for first packet...', ...
    'Color',[0.85 0.85 0.85], 'FontSize',11, 'FontName','Courier New', ...
    'VerticalAlignment','top', 'Units','normalized', 'Interpreter','none');
drawnow;

%% Receive loop
fprintf('Close the figure or press Ctrl-C to stop.\n');


while ishandle(fig)
    % block up to 5 ms waiting for a packet; timeout silently loops
    try
        socket.receive(pkt);
    catch
        continue;
    end
    raw = char(pkt.getData())';
    raw = raw(1:pkt.getLength());

    parts = str2double(strsplit(strtrim(raw), ','));

    % corrupt data guard
    if numel(parts) < 5 || any(isnan(parts))
        continue;
    end

    x_mm = parts(1);  y_mm = parts(2);  th_d = parts(3);
    d_mm = parts(4);  da_d = parts(5);
    t_now = toc(t0);

    % write into circular buffer
    n    = n + 1;
    slot = mod(n-1, MAX_HISTORY) + 1; % overwrites oldest 
    xs(slot) = x_mm;  ys(slot) = y_mm;
    ths(slot) = th_d; ts_buf(slot) = t_now;

    % ordered indices
    if n <= MAX_HISTORY
        ord = 1:n;
    else
        ord = [slot+1:MAX_HISTORY, 1:slot];
    end
    ok = ~isnan(xs(ord));
    px = xs(ord(ok));  py = ys(ord(ok));
    pt = ths(ord(ok)); pts = ts_buf(ord(ok));

    % update XY plot
    set(h_trail, 'XData', px,   'YData', py);
    set(h_dot,   'XData', x_mm, 'YData', y_mm);
    set(h_arrow, 'XData', x_mm, 'YData', y_mm, ...
                 'UData', ARROW_LEN*cosd(th_d), ...
                 'VData', ARROW_LEN*sind(th_d));
    if numel(px) > 1
        xp = max(200, 0.13*(max(px)-min(px)+1));
        yp = max(200, 0.13*(max(py)-min(py)+1));
        xlim(ax_xy, [min(px)-xp, max(px)+xp]);
        ylim(ax_xy, [min(py)-yp, max(py)+yp]);
    end

    % update heading plot
    set(h_theta, 'XData', pts, 'YData', pt);
    xlim(ax_th, [max(0, t_now-60), t_now+1]);

    % update status
    set(h_status, 'String', sprintf([ ...
        'X        : %8.2f mm\n'  ...
        'Y        : %8.2f mm\n'  ...
        'Theta    : %8.2f deg\n' ...
        'dDist    : %8.3f mm\n'  ...
        'dAngle   : %8.3f deg\n' ...
        'Elapsed  : %8.1f s\n'   ...
        'Packets  : %d'],         ...
        x_mm, y_mm, th_d, d_mm, da_d, t_now, n));

    drawnow limitrate;
end

fprintf('Stopped.\n');
