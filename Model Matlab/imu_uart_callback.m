% imu_uart_callback.m
port = "/dev/ttyACM0";
      % adapter au port
baud = 115200;
bufSeconds = 30;      % durée du buffer en secondes (affichage glissant)
fs_est = 100;         % estimation fréquence d'échantillonnage pour buffer length

% Initialisation serial
s = serialport(port, baud);
configureTerminator(s,"LF");

% --- Figure et layout ---
fig = figure('Name','IMU Live','NumberTitle','off');
t = tiledlayout(fig,3,4,'TileSpacing','compact','Padding','compact'); % 4x2

% Left column : 3 lignes pour ACC, GYR, MAG
axAcc = nexttile(t,1);
hAccX = animatedline(axAcc,'Color','r','DisplayName','ACC x');
hAccY = animatedline(axAcc,'Color','g','DisplayName','ACC y');
hAccZ = animatedline(axAcc,'Color','b','DisplayName','ACC z');
title(axAcc,'Accelerometer'); xlabel(axAcc,'Time (s)'); ylabel(axAcc,'m/s^2'); legend(axAcc,'show'); grid(axAcc,'on')

axGyr = nexttile(t,5);
hGyrX = animatedline(axGyr,'Color','r','DisplayName','GYR x');
hGyrY = animatedline(axGyr,'Color','g','DisplayName','GYR y');
hGyrZ = animatedline(axGyr,'Color','b','DisplayName','GYR z');
title(axGyr,'Gyroscope'); xlabel(axGyr,'Time (s)'); ylabel(axGyr,'rad/s'); legend(axGyr,'show'); grid(axGyr,'on')

axMag = nexttile(t,9);
hMagX = animatedline(axMag,'Color','r','DisplayName','MAG x');
hMagY = animatedline(axMag,'Color','g','DisplayName','MAG y');
hMagZ = animatedline(axMag,'Color','b','DisplayName','MAG z');
title(axMag,'Magnetometer'); xlabel(axMag,'Time (s)'); ylabel(axMag,'uT'); legend(axMag,'show'); grid(axMag,'on')

% Right column : poseplot spanning 3 rows
axPose = nexttile(t,2,[3 1]); % span 3 rows, 1 col
QuaternionPosition = poseplot(axPose, quaternion(1,0,0,0));
axis(axPose,'equal'); xlabel(axPose,"North-x (m)"); ylabel(axPose,"East-y (m)");
zlabel(axPose,"Down-z (m)"); grid(axPose,'on'); view(axPose,3)


axPos = nexttile(t,3,[3 1]);   % 3e colonne, 3 lignes
hold(axPos,'on');

hEst = plot3(axPos, NaN, NaN, NaN, 'b', 'LineWidth', 1.5);
hTrue = plot3(axPos, NaN, NaN, NaN, '--r', 'LineWidth', 1.2);

axis(axPos,'equal');
grid(axPos,'on');
view(axPos,3);

xlabel(axPos,"North (m)");
ylabel(axPos,"East (m)");
zlabel(axPos,"Down (m)");
title(axPos,"Position (meters)");
legend(axPos,{'Estimated','True'});

% --- Right column: GPS axis occupying entire right column (tiles 3,6,9) ---
axGPS = nexttile(t,4,[3 1]); % starts at tile 3, spans 3 rows
hLat = animatedline(axGPS,'Color','m','DisplayName','Latitude (°)');
hLon = animatedline(axGPS,'Color','c','DisplayName','Longitude (°)');
hAlt = animatedline(axGPS,'Color','k','DisplayName','Altitude (m)');
title(axGPS,'GPS: Latitude / Longitude / Altitude'); xlabel(axGPS,'Time (s)');
legend(axGPS,'show'); grid(axGPS,'on')



% Store handles (ud) for Processu usage
ud = struct();
ud.h = [hAccX,hAccY,hAccZ,hGyrX,hGyrY,hGyrZ,hMagX,hMagY,hMagZ];
ud.hGPS = [hLat,hLon,hAlt];
% ud.hPos = [hPosN,hPosE,hPosD];
ud.tStart = tic;
ud.t0 = datetime('now');
fig.UserData = ud;

s.UserData = ud;
s.UserData.lastACC = nan(1,3);
s.UserData.lastGYR = nan(1,3);
s.UserData.lastMAG = nan(1,3);

% Polling à 10 Hz
pollHz = 10;
pollDt = 1/pollHz;
nextPoll = tic;

pEstHist = zeros(0,3);

Fuse = ahrsfilter("SampleRate", pollHz);   % créer le filtre une seule fois
FuseGPS = insfilterAsync;

while ishandle(fig)
    % attend pour tenir 10 Hz sans bloquer inutilement
    elapsed = toc(nextPoll);
    if elapsed < pollDt
        pause(pollDt - elapsed);
    end
    nextPoll = tic;

    % Lire toutes les lignes disponibles (sans bloquer)
    while s.NumBytesAvailable > 0
        line = readline(s);   % lit jusqu'au LF (terminator)
        uartProcessLine(s, line); % même parsing que ton callback
    end

   ud = s.UserData;            % snapshot
    accel = ud.lastACC;        % 1x3 double
    gyro  = ud.lastGYR;        % 1x3 double
    mag   = ud.lastMAG;        % 1x3 double


    % GPS LLA
    if isfield(ud,"lla")
        lla = ud.lla;
    elseif isfield(ud,"lastGPS")  % exemple possible
        lla = ud.lastGPS;
    else
        lla = [];
    end
    
    % GPS Velocity
    if isfield(ud,"gpsvel")
        gpsvel = ud.gpsvel;
    elseif isfield(ud,"gpsVel")
        gpsvel = ud.gpsVel;
    else
        gpsvel = [];
    end


    %Imu sensor fusion accel,mag, gyro
    Q= Fuse(accel,gyro,mag);
    QuaternionPosition.Orientation = Q;

        FuseGPS.predict(1/pollHz);

        Rmag = 0.4;
        Racc = 90;
        Rgyro = 0.75e-5;
        Rvel = 0.01;
    
        FuseGPS.fuseaccel(accel, Racc);
        FuseGPS.fusegyro (gyro, Rgyro);
        FuseGPS.fusemag  (mag, Rmag);
    
        % GPS: fusion seulement quand une nouvelle mesure est "passée"
        if ~isempty(lla) && ~isempty(gpsvel)
            FuseGPS.fusegps(lla, Rpos, gpsvel, Rvel);
        end
    
        [p,q] = pose(FuseGPS);

        % pose INS (attitude + position)
        % QuaternionPosition.Position    = p;
        QuaternionPosition.Orientation = q;
        
        pEst = FuseGPS.State(1:3)';    % [North East Down]
        % ajoute une nouvelle ligne
pEstHist = [pEstHist; pEst];    % pEst est 1x3


        set(hEst, ...
            'XData', pEstHist(:,1), ...
            'YData', pEstHist(:,2), ...
            'ZData', pEstHist(:,3));

        % Temps écoulé
        tt = toc(fig.UserData.tStart);


end


% Nettoyage à la fermeture de la figure
fig.CloseRequestFcn = @(src,ev)cleanup(src, s);

%% Uart process reception
function uartProcessLine(src, line)
    ud = src.UserData;
    t = seconds(datetime('now') - ud.t0);

    vals = regexp(line,'x=([-\d\.eE]+)\s+y=([-\d\.eE]+)\s+z=([-\d\.eE]+)','tokens','once');
    if ~isempty(vals)
        vec = [str2double(vals{1}), str2double(vals{2}), str2double(vals{3})]; % 1x3 double

        if contains(line,"ACC")
            ud.lastACC = vec;
            addpoints(ud.h(1), t, vec(1));
            addpoints(ud.h(2), t, vec(2));
            addpoints(ud.h(3), t, vec(3));
        elseif contains(line,"GYR")
            ud.lastGYR = vec;
            addpoints(ud.h(4), t, vec(1));
            addpoints(ud.h(5), t, vec(2));
            addpoints(ud.h(6), t, vec(3));
        elseif contains(line,"MAG")
            ud.lastMAG = vec;
            addpoints(ud.h(7), t, vec(1));
            addpoints(ud.h(8), t, vec(2));
            addpoints(ud.h(9), t, vec(3));
        end

        ud.lastT = t;
    end

    gpsTokens = regexp(line,'NEO7M:\s*GPS\s+lat=([-\d]+)\s+lon=([-\d]+)\s+alt_mm=([-\d]+)\s+fix=(\d+)\s+sats=(\d+)','tokens','once');
    if ~isempty(gpsTokens)
        % Convertir
        lat_raw = str2double(gpsTokens{1});
        lon_raw = str2double(gpsTokens{2});
        alt_mm  = str2double(gpsTokens{3});

        % Option: mettre en unités utiles
        ud.lastGPS.lat_raw = lat_raw / 1e7;
        ud.lastGPS.lon_raw = lon_raw/ 1e7;
        ud.lastGPS.alt_mm = alt_mm/1000;

        % Exemple d'affichage / plots (adapter handles si présents)
        if isfield(ud,'hGPS') && ~isempty(ud.hGPS)
            % suppose ud.hGPS(1)=lat, (2)=lon, (3)=alt
            addpoints(ud.hGPS(1), t, lat_raw);
            addpoints(ud.hGPS(2), t, lon_raw);
            addpoints(ud.hGPS(3), t, alt_mm);
        end

        ud.lastT = t;
    end


    src.UserData = ud;        % affectation atomique
    drawnow limitrate
end

%% Cleanup
function cleanup(figHandle, serialObj)
    try
        configureCallback(serialObj,"off");
        clear serialObj;
    catch
    end
    delete(figHandle);
end