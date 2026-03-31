# 🟢 ROS2 Multi-Machine Setup (Jetson + Laptop) dengan CycloneDDS

Dokumentasi ini menjelaskan cara setup **ROS2 Humble** agar dapat komunikasi antar mesin (Jetson Orin Nano dan Laptop) di **jaringan yang sama**, menggunakan **CycloneDDS**, termasuk konfigurasi URI, troubleshooting, dan testing dengan demo nodes.

---

## 1️⃣ Prasyarat

* ROS2 Humble sudah terinstall di semua mesin
* Python 3 tersedia (`ROS_PYTHON_VERSION=3`)
* Semua mesin berada dalam **satu subnet**, bisa ping / ssh satu sama lain
* Tidak ada firewall blocking UDP multicast (port 7400, dsb)
* IP statis atau DHCP dengan alamat yang dapat diakses antar mesin

---

## 2️⃣ Konfigurasi Lingkungan

### 2.1 Variabel ROS

```bash
export ROS_DOMAIN_ID=0           # pastikan sama di semua node
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2.2 CycloneDDS URI

#### a) Talker (Jetson)

* Bind ke interface lokal Jetson
* Tentukan peer minimal listener agar discovery berhasil

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="10.1.33.208"/></Interfaces></General><Discovery><Peers><Peer address="10.1.2.33"/><Peer address="10.1.2.50"/></Peers></Discovery></Domain>'
```

#### b) Listener (Laptop)

* Bind ke interface lokal Laptop
* Peer list opsional jika multicast discovery tersedia

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="10.1.2.33"/></Interfaces></General></Domain>'
```

> ⚠️ Catatan: `NetworkInterface` harus pakai **address**, bukan nama interface (`wlP1p1s0` / `wlp3s0`) di ROS Humble / Jetson karena versi DDS belum sepenuhnya support.

---

## 3️⃣ Testing Dasar

### 3.1 Demo Nodes

* **Talker (Jetson)**

```bash
ros2 run demo_nodes_cpp talker
```

* **Listener (Laptop)**

```bash
ros2 run demo_nodes_cpp listener
```

### 3.2 Mengecek Topic & Node

```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

### 3.3 Multicast Test (Opsional)

Pastikan UDP multicast dapat lewat:

```bash
# Jetson listen
nc -lu 7400

# Laptop send
echo "Hello" | nc -u 239.255.0.1 7400
```

* Jika berhasil → jaringan multicast tidak terblok

---

## 4️⃣ Troubleshooting & Insight

| Masalah                                | Gejala                                                   | Solusi / Insight                                          |
| -------------------------------------- | -------------------------------------------------------- | --------------------------------------------------------- |
| Node tidak muncul                      | `talker` tidak bisa create node, `rmw handle is invalid` | Pastikan CycloneDDS URI bind ke IP yang valid             |
| Deprecated warning                     | `'NetworkInterfaceAddress' deprecated`                   | Gunakan `<Interfaces><NetworkInterface address="..."/>`   |
| Talker tidak connect ke listener       | Listener sudah jalan tapi talker gagal                   | Minimal talker harus tahu peer list, listener bisa kosong |
| Banyak interface di Jetson             | `auto` binding kadang gagal                              | Gunakan IP spesifik, jangan `auto`                        |
| Domain ID sama tapi node tidak connect | Node tidak discover                                      | Periksa IP interface + peer + multicast bisa lewat        |
| Multicast test                         | `ros2 multicast receive` berhasil tapi demo nodes tidak  | Artinya UDP multicast ok, tapi DDS bind / peer list salah |

---

## 5️⃣ Tips dan Insight

1. **Peers minimal di talker** → listener akan auto-discover talker
2. **IP statis lebih stabil** daripada `auto` interface binding di multi-NIC Jetson/Laptop
3. **Multiple peers** bisa ditambahkan di URI talker untuk beberapa listener / robot:

```xml
<Discovery>
  <Peers>
    <Peer address="10.1.2.33"/>
    <Peer address="10.1.2.50"/>
  </Peers>
</Discovery>
```

4. Domain ID sama → **tidak menjamin node connect** jika peer atau interface salah
5. Multicast bisa digunakan untuk quick testing, tapi **static peer lebih deterministik** di jaringan multi-interface

---

## 6️⃣ Contoh URI Satu Baris untuk Multi-Peer Talker

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="10.1.33.208"/></Interfaces></General><Discovery><Peers><Peer address="10.1.2.33"/><Peer address="10.1.2.50"/></Peers></Discovery></Domain>'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

* Listener cukup:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="10.1.2.33"/></Interfaces></General></Domain>'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 7️⃣ Kesimpulan

* ROS2 multi-machine stabil di **satu subnet**, **IP presisi**, dan minimal talker tahu peer
* CycloneDDS URI harus **bind ke interface aktif**, gunakan IP
* Auto-discovery bisa pakai multicast, tapi **static peer lebih deterministik** di jaringan multi-interface
* Testing pakai `demo_nodes_cpp` + multicast UDP → cara cepat pastikan network, node, topic, dan peer config benar
