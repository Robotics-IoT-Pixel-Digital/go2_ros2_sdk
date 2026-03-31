# 🚀 Auto Start ROS2 Sistem

Setup ini memungkinkan Jetson:

* Auto source ROS saat boot
* Auto setup network (CycloneDDS)
* Auto menjalankan node ROS
* Stabil walaupun network belum siap saat boot

---

# 📦 Prasyarat (ROS2 Humble - CycloneDDS)

Pastikan:

* ROS2 Humble sudah terinstall
* Workspace sudah di-build:

```bash
cd ~/Projects/UnitreeGo2/ros2_ws
colcon build
```

* CycloneDDS config tersedia:

```
/root/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/config/cyclonedds.xml
```

---

# ⚙️ 1. Buat Script Startup (sh/bash file - penjelasan dengan comment line)

```bash
sudo nano /root/env_start_ros.sh
```

Isi:

```bash
#!/bin/bash
set -e

echo "[START] ROS2 Environment Setup"

# ===== Disable conda (jika ada) =====
conda deactivate 2>/dev/null || true

# ===== Source ROS2 global =====
source /opt/ros/humble/setup.bash

# ===== Source workspace =====
source /root/Projects/UnitreeGo2/ros2_ws/install/setup.bash

# ===== Set environment ROS2 DDS =====
export ROS_DOMAIN_ID=0                # ID domain ROS (harus sama antar device)
export ROS_LOCALHOST_ONLY=0           # allow komunikasi network
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/config/cyclonedds.xml

echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# ===== WAIT NETWORK READY =====
# tunggu sampai device mendapatkan IP
echo "[WAIT] Waiting for network..."
while true; do
    IP=$(hostname -I | awk '{print $1}')
    if [ -n "$IP" ]; then
        echo "[OK] IP detected: $IP"
        break
    fi
    sleep 2
done

# delay tambahan untuk stabilisasi DDS
sleep 2

# ===== RUN NODE (AUTO RETRY) =====
while true; do
    echo "[START] Running Go2 Node..."

    ros2 run go2_robot_sdk go2_gstreamer_node

    echo "[ERROR] Node stopped, retrying..."
    sleep 3
done
```

Beri permission:

```bash
chmod +x /root/env_start_ros.sh
```

---

# ⚙️ 2. Buat Systemd Service (beri penjelasan dengan comment line)

```bash
sudo nano /etc/systemd/system/start_ros.service
```

Isi:

```ini
[Unit]
Description=ROS2 Auto Start Service
After=network-online.target        # jalankan setelah network siap
Wants=network-online.target

[Service]
Type=simple
ExecStart=/root/env_start_ros.sh   # script utama
Restart=always                     # auto restart jika gagal
RestartSec=3                       # delay restart
User=root
Environment="HOME=/root"           # environment penting untuk ROS

[Install]
WantedBy=multi-user.target         # start saat boot
```

---

# ⚙️ 3. Command Service

#### 🔄 Reload config (WAJIB setelah edit service)

```bash
sudo systemctl daemon-reload
```

#### ▶️ Start service

```bash
sudo systemctl start start_ros.service
```

#### ⏹ Stop service

```bash
sudo systemctl stop start_ros.service
```

#### 🔁 Restart service

```bash
sudo systemctl restart start_ros.service
```

#### ✅ Enable auto start saat boot

```bash
sudo systemctl enable start_ros.service
```

#### ❌ Disable auto start

```bash
sudo systemctl disable start_ros.service
```

#### 📊 Cek status

```bash
systemctl status start_ros.service
```

#### 📜 Debug log realtime

```bash
journalctl -u start_ros.service -f
```

---

# 🌐 4. Setup di Device Lain (WAJIB)

Agar ROS2 bisa komunikasi antar device (Jetson ↔ Laptop):

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

Catatan:

* `ROS_DOMAIN_ID` harus sama
* `CYCLONEDDS_URI` harus konsisten (interface/network config)
* Pastikan device berada di network yang sama

---

# 🧪 5. Test ROS Network antar Device

## Di Jetson:

```bash
ros2 topic list
```

## Di Laptop:

```bash
ros2 topic list
```

Jika berhasil:

* topic dari Jetson akan muncul di laptop

---

# ⚠️ Troubleshooting

## ❌ Topic tidak muncul di device lain

* Pastikan satu jaringan (WiFi/LAN)
* Pastikan `ROS_DOMAIN_ID` sama
* Pastikan `CYCLONEDDS_URI` sudah di-set
* Disable firewall:

```bash
sudo ufw disable
```

---

## ❌ Error: failed to create domain

Penyebab:

* DDS start sebelum network siap

Solusi:

* gunakan mekanisme `wait network` di script

---

## ❌ Service jalan tapi node tidak start

Cek log:

```bash
journalctl -u start_ros.service -f
```

---

## ❌ No executable found

Cek executable ROS:

```bash
ros2 pkg executables go2_robot_sdk
```

---

## ❌ Topic ada di Jetson tapi tidak di laptop

Penyebab:

* CycloneDDS salah pilih interface

Solusi:

* cek `cyclonedds.xml`
* gunakan interface WiFi atau konfigurasi multi-interface

---

# 🎯 Kesimpulan

Dengan setup ini:

* Jetson boot → ROS langsung jalan
* Node auto restart jika crash
* Network stabil walaupun boot delay
* Komunikasi antar device berjalan via DDS

---