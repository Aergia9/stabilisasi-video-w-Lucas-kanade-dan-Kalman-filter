%% SISTEM STABILISASI VIDEO BERBASIS MOTION ESTIMATION - VERSI DIPERBAIKI
% Metode: Optical Flow Lucas-Kanade + Transformasi Affine + Kalman Filter
% Mempertahankan resolusi asli video selama proses

clc; clear; close all;
warning('off', 'images:initSize:adjustingMag');

%% ======== BAGIAN 1: LOAD VIDEO INPUT ========
% Pilih file video
[file, path] = uigetfile({'*.mp4;*.avi;*.mov','File Video'});
if isequal(file,0)
    error('Tidak ada video yang dipilih');
end
videoFile = fullfile(path, file);

% Buat video reader
videoReader = VideoReader(videoFile);

% Tampilkan info video
fprintf('=== STABILISASI VIDEO DIPERBAIKI ===\n');
fprintf('File input  : %s\n', file);
fprintf('Frame rate  : %.2f fps\n', videoReader.FrameRate);
fprintf('Durasi      : %.2f detik\n', videoReader.Duration);
fprintf('Resolusi    : %d x %d\n', videoReader.Width, videoReader.Height);

%% ======== BAGIAN 2: INISIALISASI PARAMETER YANG DIPERBAIKI ========
% Parameter optical flow yang lebih sensitif
opticFlow = opticalFlowLK('NoiseThreshold', 0.03, 'NumPyramidLevels', 3);

% Buffer untuk smoothing yang lebih responsif
bufferSize = 50;  % Dikurangi untuk respons lebih cepat
trajectoryX_buf = zeros(1, bufferSize);
trajectoryY_buf = zeros(1, bufferSize);
trajectoryTheta_buf = zeros(1, bufferSize);
ptr = 1;

% Kalman Filter untuk smoothing yang lebih baik
% State: [x, vx, y, vy, theta, omega]
F = [1 1 0 0 0 0;  % State transition matrix
     0 1 0 0 0 0;
     0 0 1 1 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 1;
     0 0 0 0 0 1];
H = [1 0 0 0 0 0;  % Observation matrix
     0 0 1 0 0 0;
     0 0 0 0 1 0];
Q = eye(6) * 0.01;  % Process noise
R = eye(3) * 0.1;   % Measurement noise
P = eye(6);         % Error covariance
x_state = zeros(6,1);  % Initial state

% Penyimpanan trajectory penuh
fullTrajectoryX = [];   
fullTrajectoryY = [];   
fullTrajectoryTheta = []; 
fullSmoothX = [];       
fullSmoothY = [];       
fullSmoothTheta = [];   

% Parameter stabilisasi yang ditingkatkan
smoothFactor = 0.3;  % Dinaikkan untuk koreksi lebih agresif
adaptiveFactor = 0.0001; % Faktor adaptif untuk gerakan besar

% Frame pertama
prevFrame = readFrame(videoReader);
prevGray = im2gray(prevFrame);

% Simpan ukuran asli video dengan margin untuk crop
originalSize = [size(prevFrame, 1), size(prevFrame, 2)];
cropMargin = 20; % Margin untuk cropping yang lebih besar

% Inisialisasi penyimpanan trajectory
fullTrajectoryX(1) = 0;
fullTrajectoryY(1) = 0;
fullTrajectoryTheta(1) = 0;
fullSmoothX(1) = 0;
fullSmoothY(1) = 0;
fullSmoothTheta(1) = 0;

% Inisialisasi penyimpanan video
videoAsli = {};
videoStabil = {};
videoAsli{1} = prevFrame;
videoStabil{1} = prevFrame;

%% ======== BAGIAN 3: PROSES STABILISASI YANG DIPERBAIKI ========
hWait = waitbar(0, 'Memproses stabilisasi video...');
totalFrames = floor(videoReader.Duration * videoReader.FrameRate);
frameCount = 1;

% Deteksi region of interest untuk optical flow
roi = [cropMargin+1, cropMargin+1, originalSize(2)-2*cropMargin, originalSize(1)-2*cropMargin];

while hasFrame(videoReader)
    currFrame = readFrame(videoReader);
    currGray = im2gray(currFrame);
    frameCount = frameCount + 1;
    
    % Estimasi optical flow pada ROI
    flow = estimateFlow(opticFlow, currGray);
    
    % Hitung motion vector dengan pembobotan berbasis magnitude
    validFlow = ~isnan(flow.Vx) & ~isnan(flow.Vy);
    magnitude = sqrt(flow.Vx.^2 + flow.Vy.^2);
    
    % Gunakan median untuk mengurangi outlier
    medianVx = median(flow.Vx(validFlow & magnitude > 0.1), 'omitnan');
    medianVy = median(flow.Vy(validFlow & magnitude > 0.1), 'omitnan');
    
    % Hitung rotasi dengan metode yang lebih robust
    [hFlow, wFlow] = size(flow.Vx);
    [X, Y] = meshgrid(1:wFlow, 1:hFlow);
    centerX = wFlow/2;
    centerY = hFlow/2;
    
    % Gunakan hanya flow dengan magnitude signifikan
    strongFlow = magnitude > quantile(magnitude(:), 0.7);
    
    if sum(strongFlow(:)) > 10  % Pastikan ada cukup point
        dx = X(strongFlow) - centerX;
        dy = Y(strongFlow) - centerY;
        vx_strong = flow.Vx(strongFlow);
        vy_strong = flow.Vy(strongFlow);
        
        rotationVectors = (vx_strong .* dy - vy_strong .* dx);
        medianRotation = median(rotationVectors, 'omitnan') / 1000;
    else
        medianRotation = 0;
    end
    
    % Kalman Filter Update
    z = [medianVx; medianVy; medianRotation];  % Measurement
    
    % Predict
    x_state = F * x_state;
    P = F * P * F' + Q;
    
    % Update
    S = H * P * H' + R;
    K = P * H' / S;
    x_state = x_state + K * (z - H * x_state);
    P = (eye(6) - K * H) * P;
    
    % Ekstrak smoothed values dari Kalman state
    smoothX = x_state(1);
    smoothY = x_state(3);
    smoothTheta = x_state(5);
    
    % Simpan trajectory
    fullTrajectoryX(frameCount) = medianVx;
    fullTrajectoryY(frameCount) = medianVy;
    fullTrajectoryTheta(frameCount) = medianRotation;
    fullSmoothX(frameCount) = smoothX;
    fullSmoothY(frameCount) = smoothY;
    fullSmoothTheta(frameCount) = smoothTheta;
    
    % Motion compensation dengan faktor adaptif
    motionMagnitude = sqrt(medianVx^2 + medianVy^2);
    currentAdaptiveFactor = min(adaptiveFactor, motionMagnitude * 10);
    
    compX = currentAdaptiveFactor * smoothX;
    compY = currentAdaptiveFactor * smoothY;
    compTheta = currentAdaptiveFactor * smoothTheta;
    
    % Batasi kompensasi untuk menghindari overcorrection
    compX = max(-5, min(5, compX));
    compY = max(-5, min(5, compY));
    compTheta = max(-0.05, min(0.05, compTheta));
    
    % Buat transformasi affine
    tform = affine2d([cos(compTheta)  -sin(compTheta)  0;
                     sin(compTheta)   cos(compTheta)  0;
                     compX            compY           1]);
    
    % Aplikasikan transformasi dengan padding untuk menghindari black borders
    padSize = 30;
    paddedFrame = padarray(currFrame, [padSize padSize], 'replicate');
    paddedOutputView = imref2d([originalSize(1)+2*padSize, originalSize(2)+2*padSize]);
    
    stabilizedPadded = imwarp(paddedFrame, tform, 'OutputView', paddedOutputView, ...
        'FillValues', [0 0 0], 'SmoothEdges', true);
    
    % Crop kembali ke ukuran asli
    stabilized = stabilizedPadded(padSize+1:end-padSize, padSize+1:end-padSize, :);
    
    % Pastikan ukuran sama dengan asli
    if size(stabilized, 1) ~= originalSize(1) || size(stabilized, 2) ~= originalSize(2)
        stabilized = imresize(stabilized, originalSize);
    end
    
    % Simpan frame
    videoAsli{frameCount} = currFrame;
    videoStabil{frameCount} = stabilized;
    
    % Update untuk frame berikutnya
    prevGray = currGray;
    
    % Update waitbar
    waitbar(frameCount/totalFrames, hWait, ...
        sprintf('Memproses frame %d/%d (%.1f%%)', frameCount, totalFrames, frameCount/totalFrames*100));
end

close(hWait);
fprintf('Proses stabilisasi selesai! Total frame diproses: %d\n', frameCount);

%% ======== BAGIAN 4: TAMPILKAN VIDEO DENGAN SIDE-BY-SIDE ========
% Konversi cell array ke 4D array
videoAsli_4D = cat(4, videoAsli{:});
videoStabil_4D = cat(4, videoStabil{:});

% Buat video side-by-side untuk perbandingan visual
videoComparison = zeros(originalSize(1), originalSize(2)*2, 3, frameCount, 'uint8');
for i = 1:frameCount
    videoComparison(:, 1:originalSize(2), :, i) = videoAsli{i};
    videoComparison(:, originalSize(2)+1:end, :, i) = videoStabil{i};
end

% Tampilkan video perbandingan
playerComparison = implay(videoComparison, videoReader.FrameRate);
set(playerComparison.Parent, 'Name', 'Perbandingan: Asli (Kiri) vs Stabilisasi (Kanan)');

%% ======== BAGIAN 5: ANALISIS HASIL YANG DIPERBAIKI ========
% Hitung metrik stabilisasi
reduksiX = 100 * (1 - std(fullSmoothX)/std(fullTrajectoryX));
reduksiY = 100 * (1 - std(fullSmoothY)/std(fullTrajectoryY));
reduksiTheta = 100 * (1 - std(fullSmoothTheta)/std(fullTrajectoryTheta));

% Hitung Inter-frame Transformation Fidelity (ITF)
itf_original = 0;
itf_stabilized = 0;
for i = 2:min(50, frameCount)  % Sample 50 frame untuk efisiensi
    % Hitung perubahan antar frame
    diff_orig = abs(fullTrajectoryX(i) - fullTrajectoryX(i-1)) + ...
                abs(fullTrajectoryY(i) - fullTrajectoryY(i-1));
    diff_stab = abs(fullSmoothX(i) - fullSmoothX(i-1)) + ...
                abs(fullSmoothY(i) - fullSmoothY(i-1));
    
    itf_original = itf_original + diff_orig;
    itf_stabilized = itf_stabilized + diff_stab;
end
itf_improvement = 100 * (1 - itf_stabilized/itf_original);

% Hitung PSNR untuk beberapa frame
psnr_values = [];
ssim_values = [];
for i = 10:10:min(100, frameCount)  % Sample setiap 10 frame
    orig_gray = im2double(rgb2gray(videoAsli{i}));
    stab_gray = im2double(rgb2gray(videoStabil{i}));
    
    mse_val = immse(orig_gray, stab_gray);
    if mse_val > 0
        psnr_val = 10*log10(1/mse_val);
        psnr_values(end+1) = psnr_val;
    end
    
    ssim_val = ssim(orig_gray, stab_gray);
    ssim_values(end+1) = ssim_val;
end

avg_psnr = mean(psnr_values);
avg_ssim = mean(ssim_values);

% Tampilkan hasil analisis
fprintf('\n=== ANALISIS HASIL DIPERBAIKI ===\n');
fprintf('Reduksi Getaran Horizontal (X)    : %.1f%%\n', reduksiX);
fprintf('Reduksi Getaran Vertikal (Y)      : %.1f%%\n', reduksiY);
fprintf('Reduksi Getaran Rotasi            : %.1f%%\n', reduksiTheta);
fprintf('Peningkatan Stabilitas Antar Frame: %.1f%%\n', itf_improvement);
fprintf('\n=== METRIK KUALITAS ===\n');
fprintf('PSNR Rata-rata : %.2f dB\n', avg_psnr);
fprintf('SSIM Rata-rata : %.4f\n', avg_ssim);

%% ======== BAGIAN 6: VISUALISASI YANG DIPERBAIKI ========
hFig = figure('Name', 'Analisis Stabilisasi Video', 'Position', [50 50 1200 900], 'Color', 'white');

% Plot Translasi X
subplot(2,2,1);
plot(1:frameCount, fullTrajectoryX, 'r-', 'LineWidth', 1.5); hold on;
plot(1:frameCount, fullSmoothX, 'b-', 'LineWidth', 2);
title('TRAJECTORY TRANSLASI X', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frame'); ylabel('Pergeseran (pixel)');
legend('Asli', 'Stabilisasi', 'Location', 'best');
grid on; grid minor;

% Plot Translasi Y
subplot(2,2,2);
plot(1:frameCount, fullTrajectoryY, 'r-', 'LineWidth', 1.5); hold on;
plot(1:frameCount, fullSmoothY, 'b-', 'LineWidth', 2);
title('TRAJECTORY TRANSLASI Y', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frame'); ylabel('Pergeseran (pixel)');
legend('Asli', 'Stabilisasi', 'Location', 'best');
grid on; grid minor;

% Plot Rotasi
subplot(2,2,3);
plot(1:frameCount, fullTrajectoryTheta*1000, 'r-', 'LineWidth', 1.5); hold on;
plot(1:frameCount, fullSmoothTheta*1000, 'b-', 'LineWidth', 2);
title('TRAJECTORY ROTASI', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frame'); ylabel('Rotasi (miliradians)');
legend('Asli', 'Stabilisasi', 'Location', 'best');
grid on; grid minor;

% Plot Magnitude Gerakan
subplot(2,2,4);
mag_orig = sqrt(fullTrajectoryX.^2 + fullTrajectoryY.^2);
mag_stab = sqrt(fullSmoothX.^2 + fullSmoothY.^2);
plot(1:frameCount, mag_orig, 'r-', 'LineWidth', 1.5); hold on;
plot(1:frameCount, mag_stab, 'b-', 'LineWidth', 2);
title('MAGNITUDE GERAKAN TOTAL', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frame'); ylabel('Magnitude (pixel)');
legend('Asli', 'Stabilisasi', 'Location', 'best');
grid on; grid minor;

% Tambahkan summary box
summary_text = sprintf(['HASIL STABILISASI:\n' ...
                       'Reduksi X: %.1f%% | Y: %.1f%% | Rotasi: %.1f%%\n' ...
                       'Stabilitas Antar Frame: %.1f%%\n' ...
                       'PSNR: %.2f dB | SSIM: %.3f'], ...
                       reduksiX, reduksiY, reduksiTheta, itf_improvement, avg_psnr, avg_ssim);

annotation(hFig, 'textbox', [0.02 0.02 0.96 0.08], ...
    'String', summary_text, ...
    'EdgeColor', 'black', 'LineWidth', 1.5, ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'BackgroundColor', [0.95 0.95 0.95]);

% Pesan sukses yang diperbaiki
msg = sprintf(['Stabilisasi Video Berhasil!\n\n' ...
               'Metrik Peningkatan:\n' ...
               '• Reduksi Getaran X: %.1f%%\n' ...
               '• Reduksi Getaran Y: %.1f%%\n' ...
               '• Reduksi Rotasi: %.1f%%\n' ...
               '• Stabilitas Antar Frame: %.1f%%\n\n' ...
               'Kualitas Video:\n' ...
               '• PSNR: %.2f dB\n' ...
               '• SSIM: %.3f\n\n' ...
               'Lihat video perbandingan side-by-side\n' ...
               'untuk melihat perbedaan visual!'], ...
               reduksiX, reduksiY, reduksiTheta, itf_improvement, avg_psnr, avg_ssim);
msgbox(msg, 'Hasil Stabilisasi', 'help');

%% ======== BAGIAN 7: EXPORT VIDEO (OPSIONAL) ========
% Uncomment bagian ini jika ingin menyimpan video hasil
% outputVideo = VideoWriter('video_stabilized.mp4', 'MPEG-4');
% outputVideo.FrameRate = videoReader.FrameRate;
% open(outputVideo);
% for i = 1:frameCount
%     writeVideo(outputVideo, videoStabil{i});
% end
% close(outputVideo);
% fprintf('Video hasil disimpan sebagai: video_stabilized.mp4\n');