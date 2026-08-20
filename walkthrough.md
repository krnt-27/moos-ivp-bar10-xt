# Walkthrough: Konversi Pipeline IMU Python ke C++

Migrasi sistem dari prototype Python (`pipeline_imu`) ke C++ MOOS-IvP (`moos-ivp-wit-motion`) telah berhasil diselesaikan dan dicompile tanpa error.

## Ringkasan Perubahan

Berikut adalah komponen yang telah ditambahkan dan diubah:

### 1. Library Baru (Filter & Sensor Fusion)
- **`MadgwickAHRS`**: Di-porting dari algoritma standar untuk menghasilkan nilai Roll, Pitch, dan Yaw yang stabil berdasarkan sensor Accelerometer, Gyroscope, dan Magnetometer.
- **`SimpleKalman`**: Di-porting dari class Python `SimpleKalman1D` dan `SensorKalmanFilter` untuk menghaluskan (smoothing) data sensor dan mengatasi masalah *angle wrapping* (lompatan nilai 360 ke 0).

### 2. Modbus RTU Protocol (`HWT9053` Library)
- Struktur `HWT9053Data` ditambahkan untuk menyimpan data konversi dalam tipe `float` (g, °/s, µT, dll.).
- Ditambahkan support untuk **Modbus RTU** (`parseModbusResponse` dan `calculateCRC`). Sekarang pembacaan dilakukan via blok memori dengan CRC check, bukan *streaming* byte per byte.
- Menambahkan fungsi `buildReadCommand()` untuk membuat request Modbus ke sensor.

### 3. Integrasi Aplikasi (`iSerialWitMotion`)
- **Proses Polling**: Meminta data block utama (0x34), quaternion (0x51), dan suhu (0x43) setiap siklus.
- **Auto-Kalibrasi**: Merekam 265 sampel pertama saat startup untuk menentukan nilai offset Accelerometer dan Yaw statis (persis seperti versi Python).
- **Pemrosesan Data**: Mengurutkan eksekusi mulai dari pembacaan *raw*, kalibrasi, Madgwick AHRS, hingga Kalman Filter.
- **Publish Data**:
  - Mempublikasikan variabel individu seperti `INS_ROLL`, `INS_ACCEL_X`, `INS_TEMPERATURE`, dll.
  - Mempublikasikan **satu string JSON utuh** di variabel `INS_DATA` yang formatnya sama persis dengan yang dihasilkan oleh Python.
- Nilai Suhu: Suhu mentah (`raw_temperature`) dan yang sudah terkonversi (`temperature`) keduanya diekstrak dan disiapkan di `INS_DATA` (sesuai permintaan).

### 4. Konfigurasi (`witmotion.moos`)
- Parameter `HEADING_CORRECTION` telah ditambahkan di blok konfigurasi `.moos` dengan default `20.0`. Ini bisa diubah sesuai kebutuhan saat deployment kapal.

## Verifikasi
- ✅ Pembuatan semua dependensi (C++ headers/source code)
- ✅ Pembaruan `CMakeLists.txt`
- ✅ **Kompilasi sukses (`./build.sh`) tanpa error.**

Anda sekarang dapat menjalankan aplikasi menggunakan `pAntler witmotion.moos` untuk menguji hasilnya secara langsung dengan hardware HWT9053-RS485.
