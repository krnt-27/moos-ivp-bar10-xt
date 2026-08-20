#!/bin/bash
# Script untuk menjalankan launch.sh dan logging MOOS data ke file

# Masuk ke folder missions
cd /home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/missions

# Buat nama file log dengan timestamp
LOG_FILE="/home/kurnia/work/PAL/KSSR18M/Code/moos-ivp-wit-motion/WIT_MOTION_HWT9053_$(date +%Y%m%d_%H%M%S).log"

echo "Starting MOOS mission with logging..."
echo "Log file: $LOG_FILE"
echo "Press Ctrl+C to stop both mission and logging"

# Jalankan launch.sh di background
./launch.sh &
LAUNCH_PID=$!

# Tunggu sebentar agar MOOSDB siap
sleep 2

# Jalankan uXMS logging di background
uXMS witmotion.moos --all > "$LOG_FILE" &
LOG_PID=$!

echo "Mission PID: $LAUNCH_PID"
echo "Logging PID: $LOG_PID"

# Fungsi cleanup saat Ctrl+C
cleanup() {
    echo ""
    echo "Stopping mission and logging..."
    kill $LAUNCH_PID 2>/dev/null
    kill $LOG_PID 2>/dev/null
    wait $LAUNCH_PID 2>/dev/null
    wait $LOG_PID 2>/dev/null
    echo "Logging complete. Data saved to: $LOG_FILE"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Tunggu sampai salah satu proses selesai
wait -n $LAUNCH_PID $LOG_PID
cleanup
