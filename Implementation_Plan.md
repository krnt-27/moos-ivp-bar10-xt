# Konversi Pipeline IMU Python → C++ MOOS-IvP

Mengkonversi fungsionalitas `app_simplify.py` (Python) ke aplikasi `iSerialWitMotion` (C++ MOOS-IvP), mengambil bagian yang production-ready dan membuang yang bermasalah.

## User Review Required

> [!IMPORTANT]
> **Bagian Python yang TIDAK akan dikonversi** karena tidak production-ready:
> - **Dead Reckoning** (`inertial_navigation.py`) — integrasi akselerasi murni tanpa koreksi GPS akan drift sangat cepat, posisi akan salah dalam hitungan menit
> - **Temperature faking** — kode Python menggunakan `random.uniform()` untuk memalsukan suhu. Di C++ kita akan publish suhu mentah dari register saja
> - **Yaw drift compensation** (logika ad-hoc di `serial_reading_simple.py` L325-349) — terlalu fragile untuk production

> [!IMPORTANT]
> **Heading correction**: Di Python, `heading_correction` di-hardcode `20.0°`. Apakah nilai ini tetap sama untuk deployment C++? Kita akan membuatnya configurable via `.moos` file.

## Open Questions

1. **Slave ID**: Python menggunakan `0x50`. Apakah C++ juga `0x50`? (sudah `0x50` di `pollSensor()`)
2. **AppTick**: Saat ini 10Hz. Apakah cukup atau perlu lebih cepat?
3. **Publish format**: Python publish satu JSON string `INS_DATA`. Apakah di C++ juga ingin format JSON, atau lebih baik publish variabel MOOS terpisah (standar MOOS-IvP)?

## Proposed Changes

### Arsitektur

```
lib/
├── HWT9053/          # [MODIFY] Perbaiki parsing, tambah Modbus RTU protocol
│   ├── HWT9053.h
│   └── HWT9053.cpp
├── MadgwickAHRS/     # [NEW] Algoritma Madgwick AHRS (header-only)
│   ├── MadgwickAHRS.h
│   └── MadgwickAHRS.cpp
└── SimpleKalman/     # [NEW] Simple 1D Kalman filter
    ├── SimpleKalman.h
    └── SimpleKalman.cpp

src/iSerialWitMotion/
├── SerialWitMotion.h           # [MODIFY] Tambah member variables
├── SerialWitMotion.cpp         # [MODIFY] Implementasi utama
├── SerialWitMotion_Info.cpp    # [MODIFY] Update dokumentasi
├── SerialWitMotion_Info.h      # (tidak berubah)
├── main.cpp                    # (tidak berubah)
└── CMakeLists.txt              # [MODIFY] Tambah library baru
```

---

### Library: HWT9053 (Modbus RTU Protocol)

#### [MODIFY] [HWT9053.h](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/lib/HWT9053/HWT9053.h)

Masalah saat ini:
- Parsing byte (`CopeSerialData`) mengasumsikan format serial TTL (byte-by-byte streaming), tapi HWT9053-**RS485** menggunakan **Modbus RTU** (request-response)
- Perlu: struct untuk menyimpan data ter-konversi (bukan hanya raw), dan fungsi `parseModbusResponse()`

Perubahan:
- Tambah struct `HWT9053Data` untuk menyimpan data yang sudah terkonversi ke satuan fisik (g, °/s, µT, °, quaternion, °C)
- Tambah method `parseModbusResponse()` yang memproses response Modbus RTU
- Tambah konstanta skala sesuai datasheet (`SCALE_ACCEL=2048.0`, `SCALE_GYRO=16.384`, `SCALE_MAG=76.923`, `SCALE_ANGLE=1000.0`)
- Tambah method `buildReadCommand()` untuk membuat Modbus RTU command dengan CRC yang benar
- Tetap pertahankan register address defines yang sudah ada

#### [MODIFY] [HWT9053.cpp](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/lib/HWT9053/HWT9053.cpp)

- Implementasi `parseModbusResponse()`: validasi header Modbus (slave, function code, byte count), extract register values, konversi ke satuan fisik
- Implementasi CRC16 Modbus untuk validasi dan pembuatan command
- Implementasi `buildReadCommand()` untuk blok utama (0x34, 15 reg), quaternion (0x51, 4 reg), dan temperatur (0x43, 1 reg)
- Konversi signed int16 dan int32 (little-endian) sesuai Python: `struct.pack("<HH", regs[0], regs[1])`

---

### Library: MadgwickAHRS [NEW]

#### [NEW] lib/MadgwickAHRS/MadgwickAHRS.h
#### [NEW] lib/MadgwickAHRS/MadgwickAHRS.cpp

Implementasi algoritma Madgwick AHRS filter, setara dengan `ahrs.filters.Madgwick` di Python.

Fitur:
- `updateMARG(q, gyro, accel, mag)` — update quaternion dengan data 9-axis
- `getEuler()` — konversi quaternion ke Roll/Pitch/Yaw (derajat)
- Quaternion format scalar-first `[w, x, y, z]` (sama dengan Python)
- Parameter `beta` (gain) yang configurable

Sumber referensi: Algoritma Madgwick standard (paper 2011, open source, banyak implementasi C/C++ yang tersedia)

---

### Library: SimpleKalman [NEW]

#### [NEW] lib/SimpleKalman/SimpleKalman.h
#### [NEW] lib/SimpleKalman/SimpleKalman.cpp

Port langsung dari [kalman_simple.py](file:///home/kurnia/work/PAL/KSOT/Services/50.4_DCS/pipeline_imu/src/filter/kalman_simple.py):

- Class `SimpleKalman1D`: 1D Kalman filter (predict + update) — 1:1 dari Python `SimpleKalman1D`
- Class `SensorKalmanFilter`: Berisi 12 instance `SimpleKalman1D`:
  - 3x Accel (X,Y,Z)
  - 3x Gyro (X,Y,Z)
  - 3x Mag (X,Y,Z)
  - 3x Angle sin/cos pairs untuk Roll/Pitch/Yaw Madgwick (menggunakan atan2 untuk angle wrapping, sama persis dengan Python)
- Method `update(HWT9053Data&)` → return filtered data

---

### Aplikasi: iSerialWitMotion

#### [MODIFY] [SerialWitMotion.h](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/src/iSerialWitMotion/SerialWitMotion.h)

Tambah member variables:
- `MadgwickAHRS m_madgwick` — instance filter Madgwick
- `SensorKalmanFilter m_kalman` — instance Kalman filter
- `HWT9053Data m_sensor_data` — data sensor terbaru
- `double m_heading_correction` — koreksi heading (configurable via .moos)
- `int m_calibration_counter` — counter untuk fase kalibrasi (265 sample)
- `double m_acc_correction[3]` — koreksi offset accelerometer (dihitung saat kalibrasi)
- `double m_yaw_static` — yaw rata-rata saat kalibrasi
- `std::vector<double> m_calib_ax, m_calib_ay, m_calib_az, m_calib_yaw` — buffer kalibrasi
- `double m_yaw_mean_buffer[3]` — buffer untuk moving average yaw (3 sample)
- `bool m_calibrated` — flag status kalibrasi

Tambah method:
- `void processCalibration()` — logika kalibrasi 265 sample pertama
- `void publishSensorData()` — publish semua data ke MOOSDB

#### [MODIFY] [SerialWitMotion.cpp](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/src/iSerialWitMotion/SerialWitMotion.cpp)

**OnStartUp()**: Tambah parsing config dari `.moos`:
- `HEADING_CORRECTION` (default: 20.0)
- `PORT`, `BAUDRATE` (sudah ada)

**Iterate()**: Ubah flow menjadi:
1. Poll sensor → 3 Modbus commands (main block, quaternion, temperature)
2. Parse response via `HWT9053::parseModbusResponse()`
3. Konversi ke satuan fisik
4. Jalankan kalibrasi (jika belum calibrated, 265 sample pertama)
5. Jalankan Madgwick AHRS → dapatkan roll/pitch/yaw madgwick + quaternion
6. Jalankan Kalman filter → smooth semua data
7. Hitung yaw_mean (moving average 3 sample)
8. Publish ke MOOSDB

**publishSensorData()**: Publish variabel MOOS:
- `INS_ROLL` (double) — roll madgwick ter-filter
- `INS_PITCH` (double) — pitch madgwick ter-filter
- `INS_YAW` (double) — yaw madgwick ter-filter
- `INS_YAW_RAW` (double) — yaw sebelum filter
- `INS_YAW_MEAN` (double) — moving average yaw
- `INS_ACCEL_X/Y/Z` (double) — akselerasi ter-filter (g)
- `INS_GYRO_X/Y/Z` (double) — gyro ter-filter (°/s)
- `INS_MAG_X/Y/Z` (double) — magnetometer ter-filter (µT)
- `INS_TEMPERATURE` (double) — suhu sensor (°C)
- `INS_QUAT_W/X/Y/Z` (double) — quaternion
- `INS_STATUS` (string) — "Calibrating" / "Calibrated"
- `INS_DATA` (string) — JSON string berisi semua data (kompatibel dengan format Python)

**readSerialPort()**: Ubah dari byte-by-byte parsing ke Modbus RTU response buffering

**pollSensor()**: Ganti hardcoded command dengan `HWT9053::buildReadCommand()` yang menghitung CRC dinamis. Tambah polling untuk register quaternion (0x51) dan temperature (0x43).

**buildReport()**: Update untuk menampilkan data ter-konversi (bukan raw), status kalibrasi, heading correction

#### [MODIFY] [SerialWitMotion_Info.cpp](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/src/iSerialWitMotion/SerialWitMotion_Info.cpp)

Update dokumentasi:
- Synopsis
- Example config (dengan HEADING_CORRECTION)
- Interface (list semua PUBLICATIONS)

#### [MODIFY] [CMakeLists.txt](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/src/iSerialWitMotion/CMakeLists.txt)

Tambah source files:
- `${CMAKE_SOURCE_DIR}/lib/MadgwickAHRS/MadgwickAHRS.cpp`
- `${CMAKE_SOURCE_DIR}/lib/SimpleKalman/SimpleKalman.cpp`

Tambah include directories:
- `${CMAKE_SOURCE_DIR}/lib/MadgwickAHRS`
- `${CMAKE_SOURCE_DIR}/lib/SimpleKalman`

---

### Mission Config

#### [MODIFY] [witmotion.moos](file:///home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/missions/witmotion.moos)

```
ProcessConfig = iSerialWitMotion
{
    AppTick            = 10
    CommsTick          = 10
    PORT               = /dev/ttyINS
    BAUDRATE           = 9600
    HEADING_CORRECTION = 20.0
}
```

## Verification Plan

### Build Test
```bash
cd /home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion
./build.sh
```

### Unit Verification
- Verifikasi CRC16 Modbus menghasilkan output yang benar (0x08, 0x49 untuk command yang sudah ada)
- Verifikasi konversi skala menghasilkan nilai yang masuk akal (accel ≈ 1g saat diam)
- Verifikasi Madgwick output mendekati output Python untuk input yang sama

### Integration Test
- Jalankan dengan `.moos` file, verifikasi variabel MOOS terpublish di `uMS` (MOOS Scope)
- Verifikasi `buildReport()` menampilkan data yang benar di AppCast
- Verifikasi fase kalibrasi berjalan 265 sample kemudian switch ke "Calibrated"
