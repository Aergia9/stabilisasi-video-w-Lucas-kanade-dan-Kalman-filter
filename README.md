# Sistem Stabilisasi Video Berbasis Motion Estimation

Sistem stabilisasi video yang menggunakan kombinasi **Optical Flow Lucas-Kanade**, **Transformasi Affine**, dan **Kalman Filter** untuk mengurangi getaran dan guncangan pada video dengan tetap mempertahankan resolusi asli.

## 🎯 Fitur Utama

- **Stabilisasi Multi-Dimensi**: Mengoreksi gerakan translasi (X,Y) dan rotasi
- **Preservasi Resolusi**: Mempertahankan ukuran dan kualitas video asli
- **Algoritma Robust**: Menggunakan Lucas-Kanade optical flow dengan deteksi outlier
- **Smoothing Adaptif**: Kalman Filter untuk prediksi dan koreksi gerakan
- **Analisis Komprehensif**: Metrik kualitas lengkap (PSNR, SSIM, ITF)
- **Visualisasi Real-time**: Perbandingan side-by-side video asli vs hasil stabilisasi


## 🚀 Cara Penggunaan

### 1. Persiapan
```matlab
% Pastikan semua toolbox terinstall
clc; clear; close all;
```

### 2. Menjalankan Program
1. Buka MATLAB dan navigasi ke folder program
2. Jalankan script utama:
   ```matlab
   run('video_stabilization_system.m')
   ```
3. Pilih file video melalui dialog box yang muncul
4. Tunggu proses stabilisasi selesai (progress bar akan menampilkan kemajuan)

### 3. Hasil Output
Program akan menghasilkan:
- **Video perbandingan side-by-side** (asli vs stabilisasi)
- **Grafik analisis trajectory** untuk X, Y, dan rotasi
- **Laporan metrik kualitas** dalam command window
- **Summary statistik** dalam message box

## 📊 Metrik Evaluasi

### Stabilization Metrics
- **Reduksi Getaran X/Y**: Persentase pengurangan gerakan translasi
- **Reduksi Rotasi**: Pengurangan gerakan rotasional
- **Inter-frame Transformation Fidelity (ITF)**: Konsistensi antar frame

### Quality Metrics
- **PSNR (Peak Signal-to-Noise Ratio)**: Kualitas preservasi gambar
- **SSIM (Structural Similarity Index)**: Kesamaan struktural

## ⚙️ Parameter Konfigurasi

### Optical Flow Settings
```matlab
% Sensitivitas deteksi gerakan
opticFlow = opticalFlowLK('NoiseThreshold', 0.03, 'NumPyramidLevels', 3);
```

### Kalman Filter Tuning
```matlab
Q = eye(6) * 0.01;  % Process noise (stabilitas)
R = eye(3) * 0.1;   % Measurement noise (responsivitas)
```

### Stabilization Parameters
```matlab
smoothFactor = 0.3;        % Agresivitas koreksi (0.1-0.5)
adaptiveFactor = 0.0001;   % Adaptasi gerakan besar
cropMargin = 20;           % Margin cropping (pixel)
```

## 🔬 Algoritma yang Digunakan

### 1. Motion Estimation
- **Lucas-Kanade Optical Flow** dengan pyramid levels
- **Median filtering** untuk mengurangi outlier
- **ROI-based processing** untuk efisiensi

### 2. Motion Smoothing
- **Kalman Filter** 6-state (posisi + velocity untuk X,Y,θ)
- **Predictive modeling** untuk trajectory
- **Adaptive compensation** berdasarkan magnitude gerakan

### 3. Frame Transformation
- **Affine transformation** untuk koreksi geometrik
- **Padding technique** untuk menghindari black borders
- **Smooth edge blending** untuk transisi halus

## 🛠️ Troubleshooting

### Error: "Tidak ada video yang dipilih"
**Solusi**: Pastikan memilih file video yang valid saat dialog muncul

### Warning: "adjustingMag"
**Solusi**: Normal, warning ini di-suppress otomatis oleh program

### Memory Error
**Solusi**: 
- Gunakan video dengan durasi lebih pendek (< 2 menit)
- Tutup aplikasi lain untuk mengosongkan RAM
- Kurangi `bufferSize` dalam kode

### Hasil Stabilisasi Kurang Optimal
**Solusi**:
- Tingkatkan `smoothFactor` (0.4-0.5) untuk koreksi lebih agresif
- Turunkan `NoiseThreshold` (0.01-0.02) untuk sensitivitas lebih tinggi
- Adjust `cropMargin` sesuai tingkat guncangan video


## 📝 Export Video (Opsional)

Untuk menyimpan video hasil stabilisasi, uncomment bagian export di akhir kode:

```matlab
outputVideo = VideoWriter('video_stabilized.mp4', 'MPEG-4');
outputVideo.FrameRate = videoReader.FrameRate;
open(outputVideo);
for i = 1:frameCount
    writeVideo(outputVideo, videoStabil{i});
end
close(outputVideo);
```
