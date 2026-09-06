# Robotaksi BEE1 — Araç Bağlantı Adımları (ROS2 / Foxy)

> Kaynak: 2026 Robotaksi Hazır Araç **Kullanıcı Dokümanı**, **Mimari Tanımlama** ve **Genel Bilgilendirme** dokümanları.
> **Bu proje ROS2 (Foxy) ile çalışacaktır.** Araçta ROS1 (Noetic) ve ROS2 (Foxy) birlikte kurulu gelir; aşağıdaki adımlar ROS2 tarafına göre yazılmıştır.

---

## 0. Araç Bilgileri (Referans)

| Bilgi | Değer |
|---|---|
| İşletim Sistemi | Ubuntu 20.04 LTS (focal) |
| ROS | ROS1 Noetic **&** ROS2 **Foxy** (seçimli) |
| Sanal Ortam | Docker |
| Araç Bilgisayarı | Advantech MIC-770V3H + MIC-75G20, RTX 3060, i9-12900TE, 32GB RAM |
| İletişim Altyapısı | CANBus |
| Sensörler | Velodyne VLP-16 Lidar, ZED2 Stereo Kamera, XSENS MTI-680 GPS/IMU |
| WiFi | WAVLINK AC1200 (2.4 + 5 GHz) |
| Kullanıcı adı (SSH) | `smart` |

### WiFi / IP Bilgileri

| Araç | WiFi SSID | IP Adresi |
|---|---|---|
| Robotaxi 1 | `robotaxi_1` | `192.168.30.100` |
| Robotaxi 2 | `robotaxi_2` | `192.168.10.100` |

- **WiFi Şifresi:** `robotaxi`
- **SSH/SFTP kullanıcı:** `smart`
- **SFTP Port:** `22`

> ⚠️ Aşağıdaki örneklerde **Robotaxi 1 (192.168.30.100)** IP'si kullanılmıştır. Robotaxi 2'ye bağlanıyorsan IP'yi `192.168.10.100` yap.

---

## 1. Fiziksel / Ağ Bağlantısı

1. **Aracın kontağını aç.** (Kontak açılmadan WiFi ağı yayınlanmaz.)
2. Bir süre bekle; bilgisayarının WiFi listesinde **`robotaxi_1`** (veya `robotaxi_2`) ağı görünecek.
3. Bu ağa bağlan — şifre: **`robotaxi`**.
4. Araç bilgisayarının açık olduğundan emin ol: **power tuşu YEŞİL yanmalı.**

---

## 2. SSH ile Araç Bilgisayarına Bağlanma

Terminalden:

```bash
sudo ssh smart@192.168.30.100
```

- SSH → araçtaki kontrol bilgisayarının **terminaline** erişim sağlar.
- Bağlantı sonrası tüm ROS2 komutları bu SSH oturumu üzerinden çalıştırılır.

### SFTP ile Dosya Erişimi

Dosya sistemine erişim için SFTP kullanılır. İki yöntem var:

**Yöntem A — Terminal:**
```bash
sudo sftp smart@192.168.30.100
```

**Yöntem B — Files (Nautilus) ile görsel erişim (önerilen):**

Komut satırına veya ayrı bir programa gerek yok; Ubuntu'nun **Files** dosya yöneticisi SFTP'yi doğrudan bir ağ konumu gibi bağlar (GVFS).

1. **Files**'ı aç, sol menüde **"Other Locations / Diğer Konumlar"** tıkla (veya `Ctrl+L`).
2. Adres / "Connect to Server" kutusuna yaz:
   ```
   sftp://smart@192.168.30.100
   ```
   (Robotaxi 2 için `sftp://smart@192.168.10.100`)
3. **Connect** → `smart` kullanıcısının **SSH parolasını** gir (WiFi şifresi değil). "Remember password" işaretlenebilir.
4. Araç dosya sistemi sol tarafta bir ağ konumu olarak görünür; klasörleri normal klasör gibi gezersin.
5. Belirli bir klasöre gitmek için path ekleyebilirsin, örn:
   ```
   sftp://smart@192.168.30.100/Projects/ros2_ws
   ```
6. Bağlantıyı kesmek için konumun yanındaki **⏏ (eject)** ikonuna tıkla.

> Dosya kopyalamak için (ör. Docker `.tar` dosyası) dosyayı bu pencereye sürükle-bırak yeterli.

---

## 3. ROS2 Ortamını Aktif Etme (KRİTİK)

Araç varsayılan olarak ROS1 (Noetic) kaynaklı gelebilir. **ROS2 kullanmak için `~/.bashrc` düzenlenmelidir:**

1. SSH ile bağlandıktan sonra home klasöründeki gizli **`.bashrc`** dosyasını aç.
2. Dosyanın **en alt 3 satırında**:
   - `source /opt/ros/foxy/s ...` satırının **yorumunu kaldır** (baştaki `#` işaretini sil).
   - `source /opt/ros/noetic/ ...` satırını **yoruma al** (başına `#` koy).
3. ROS2 çalışma alanı için:
   - `source /Projects/ros2_ws ...` satırının **yorumda olmadığından** emin ol.
4. **Değişiklikten sonra terminaller yeniden açılmalıdır** (yeni terminal aç veya `source ~/.bashrc`).

> Özet: ROS2 = `foxy` source aktif + `noetic` source yorumda + `ros2_ws` source aktif.

### 3.1 DDS Middleware — Fast-DDS (araç varsayılanı)

**Araç, Foxy'nin VARSAYILAN DDS'ini (Fast-DDS = `rmw_fastrtps_cpp`) kullanıyor.** Container da natif Foxy olduğu için aynı Fast-DDS 2.0.x sürümünü kullanır (image'da `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` yine açıkça gömülü).

- **Araç tarafında hiçbir şey değiştirmeye gerek yok** — kendi varsayılanını kullanıyor. `~/.bashrc`'ye ekstra DDS export'u eklemeye gerek yok.
- Sadece `ROS_DOMAIN_ID`'nin iki tarafta aynı olduğundan emin ol (varsayılan 0):
  ```bash
  export ROS_DOMAIN_ID=0
  ```
- Container zaten Fast-DDS + `ROS_DOMAIN_ID=0` ile geliyor.

> ⚠️ **KRİTİK:** İki taraf farklı RMW kullanırsa (örn. biri Fast-DDS, diğeri CycloneDDS) topic'ler **birbirini görmez**. Araç Fast-DDS kullandığı için container da Fast-DDS. Değişirse container'ı `-e RMW_IMPLEMENTATION=<...>` ile çalıştırıp eşitleyebilirsin.
>
> **Doğrulama:** Container içinden `ros2 topic list` araç topic'lerini (`/beemobs/...`) görüyorsa RMW eşleşmesi tamam.

---

## 4. CAN Hattını ve Sensörleri Başlatma

Araç kontrolcüsü ile CAN üzerinden haberleşmek ve sensör verilerini almak için **tek launch yeterlidir:**

```bash
ros2 launch smart_can_stuff can_launch.xml
```

- Bu komuttan sonra CAN hattı aktif olur ve tüm sensör bilgileri (Lidar, Kamera, GPS/IMU) ile CAN topic'leri sisteme yayınlanır.
- ROS1 için verilen tüm topic isimleri ve içerikleri **ROS2'de de aynen geçerlidir.**

---

## 5. (Opsiyonel) Joystick / Uzaktan Kontrol

CAN aktif edildikten sonra:

```bash
ros2 run smart_can_stuff beemobs.py
```

---

## 6. (Opsiyonel) PID Kontrolcülerini Başlatma

```bash
ros2 launch smart_can_stuff pid_launch.py
```

- Teker açısı → `/beemobs/steering_target_value` topic'ine değer verilince devreye girer.
- Araç hızı → `/beemobs/speed_target_value` topic'i ile kontrol edilir.
- **Not:** Bu topic'lere 3 loop boyunca mesaj gelmezse PID kontrolcü devreden çıkar.
- PID parametreleri (Kp, Ki, Kd): `Projects/ros2_ws/src_smart_can_stuff/launch/pid_params.yaml`

---

## 7. Kendi Docker Image'ımızı Araca Aktarma ve Çalıştırma

> **Container da artık NATİF FOXY tabanlıdır** (araçla birebir aynı: Ubuntu 20.04, ROS 2 Foxy, Fast-DDS 2.0.x, Python 3.8). Çapraz-dağıtım (Humble↔Foxy) köprüsü kaldırıldı; eski Humble tabanı `Dockerfile.humble` olarak yedekte.
> İki tarafın konuşması için aynı `ROS_DOMAIN_ID` ve birebir aynı `smart_can_msgs` mesaj tanımları şart. (Bkz. Bölüm 3.1)

> 🔴 **KALICILIK (KRİTİK — komite bilgisi):** Araç resetlendiğinde **yalnızca `permanent` klasörü** kalır; geri kalan her şey (yüklenen docker image'ları dahil, çünkü `/var/lib/docker` silinir) sıfırlanır.
> Bu yüzden:
> - `bee1_autonom.tar` dosyasını **`permanent` klasörüne** koy.
> - **Her reset sonrası `docker load` tekrar çalıştırılmalı** (image kalıcı değil, ama `.tar` kalıcı).
> - Config, model, bag, log gibi kalıcı olması gereken her çıktı `permanent`'a yazılmalı / bind-mount edilmeli.
> - Kalıcı klasörün **teyitli tam yolu:** `/home/smart/Permanent` (büyük P — aşağıdaki tüm komutlarda bu yol kullanılır).

### 7.1 Yerelde image'i derle ve paketle
```bash
# saha_ws dizininde (Dockerfile burada)
docker build -t bee1_autonom .

# Tek .tar dosyasına paketle
docker save -o bee1_autonom.tar bee1_autonom
```

### 7.2 `.tar` dosyasını araca (permanent klasörüne) aktar
- **Files (Nautilus) SFTP** ile: `sftp://smart@192.168.30.100`'a bağlan, `bee1_autonom.tar`'ı **`/home/smart/Permanent/` klasörüne** sürükle-bırak (Bkz. Bölüm 2).
- Bu transfer **bir kez** yapılır; `.tar` permanent'ta kaldığı için sonraki oturumlarda tekrar aktarım gerekmez.

### 7.3 Araçta image'i yükle (SSH üzerinden) — HER RESET SONRASI
```bash
docker load -i /home/smart/Permanent/bee1_autonom.tar
```
> Araç her resetlendiğinde image silinir; bu komut her oturum başında tekrar çalıştırılmalıdır (~1-2 dk, tekrar transfer yok).

### 7.4 Container'ı Fast-DDS + GPU ile çalıştır
```bash
sudo docker run -it --rm \
  --network host \
  --gpus all \
  -e ROS_DOMAIN_ID=0 \
  -v /home/smart/Permanent/data:/data \
  --name bee1_autonom bee1_autonom
```
- **RMW:** Image zaten `rmw_fastrtps_cpp` ile geliyor (araç varsayılanıyla eşleşir), ekstra `-e` gerekmez. Değiştirmek istersen `-e RMW_IMPLEMENTATION=<...>` ekle.
- **SHM kapalı (UDP-only):** Image, Fast-DDS'i `/fastdds_udp_only.xml` profiliyle UDPv4'e zorlar (`FASTRTPS_DEFAULT_PROFILES_FILE` gömülü). Container'ın `/dev/shm`'i host'tan ayrı olduğu için shared-memory transportu "topic görünüyor ama veri akmıyor" sorununa yol açıyordu; bu profil onu kökten çözer, `--ipc=host` gerekmez.
- **`-v /home/smart/Permanent/data:/data`**: container çıktıları (bag, log, kayıt) reset'e dayanması için permanent'a yazılır. Container içinde `/data` altına kaydet.
- **`--network host`**: aracın node'ları ile aynı ağda keşif için (bridge yerine host).
- **`--gpus all`**: YOLO (`best.pt`, RTX 3060) için — host'ta NVIDIA Container Toolkit kurulu olmalı.

### 7.5 Container içinde node'ları çalıştır
Ortam entrypoint ile otomatik yüklenir (`ros2` komutları hazır). Örnek:
```bash
# Trafik işareti / ışık algılama
ros2 launch yolo_traffic_detector yolo_detector.launch.py

# Mevcut topic'leri doğrula (araç node'ları görünmeli)
ros2 topic list
```

### 7.6 HER OTURUM ÖZET AKIŞ (permanent klasörü ile)

> Kalıcı klasör: `/home/smart/Permanent` (teyitli, büyük P).
> `.tar` bir kez aktarılır ve orada kalır; her reset sonrası sadece **load + run** yapılır.

```bash
# 1) SSH ile araca bağlan
sudo ssh smart@192.168.30.100

# 2) Image zaten yüklü mü kontrol et (reset olmadıysa yüklü olabilir)
docker images | grep bee1_autonom

# 3) Yüklü DEĞİLSE image'i permanent'taki .tar'dan yükle (~1-2 dk)
docker load -i /home/smart/Permanent/bee1_autonom.tar

# 4) Domain ID'yi kontrol et (araç Fast-DDS varsayılanını kullanıyor, DDS export gerekmez)
export ROS_DOMAIN_ID=0

# 5) CAN + sensörleri başlat (araç host'unda)
ros2 launch smart_can_stuff can_launch.xml

# 6) Container'ı başlat (Fast-DDS gömülü; GPU + host network + permanent bind-mount)
sudo docker run -it --rm \
  --network host \
  --gpus all \
  -e ROS_DOMAIN_ID=0 \
  -v /home/smart/Permanent/data:/data \
  --name bee1_autonom bee1_autonom
```

### 7.7 İkinci terminalden çalışan container'a girmek

`docker run` bir terminali meşgul eder. Aynı container içinde başka node çalıştırmak için **yeni bir SSH terminali** aç ve:
```bash
# Çalışan container'a interaktif shell ile gir
docker exec -it bee1_autonom bash

# Örn. içeride:
ros2 launch yolo_traffic_detector yolo_detector.launch.py
ros2 topic list
```

### 7.8 Faydalı Docker yönetim komutları

```bash
# Yüklü image'ları listele
docker images

# Çalışan container'ları listele
docker ps

# Container loglarını izle
docker logs -f bee1_autonom

# Container'ı durdur (--rm ile başlatıldıysa durunca otomatik silinir)
docker stop bee1_autonom

# Takılı kalan / eski container'ı zorla sil
docker rm -f bee1_autonom

# GPU'nun container içinde görünüp görünmediğini doğrula (CUDA 13 sürücü kontrolü)
docker run --rm --gpus all bee1_autonom \
  python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"

# Disk şişerse: kullanılmayan image/container/cache temizle (DİKKAT: bee1_autonom yüklü kalsın)
docker container prune -f
docker image prune -f
```

> ⚠️ `docker run` komutunda `--rm` var: container durduğunda otomatik silinir (temiz kalır). Image (`bee1_autonom`) reset olana kadar durur; container silinse bile tekrar `docker run` ile başlatılır, `load` gerekmez. Reset sonrası ise image de gider → 7.6 adım 3'teki `docker load` şarttır.

---

## 8. Hızlı Bağlantı Checklist (Araç Başına Geçince)

- [ ] Aracın kontağı açık
- [ ] Bilgisayar `robotaxi_1`/`robotaxi_2` WiFi'ına bağlı (şifre: `robotaxi`)
- [ ] Araç bilgisayarı power tuşu **yeşil**
- [ ] `sudo ssh smart@192.168.30.100` ile bağlanıldı
- [ ] `.bashrc` içinde **foxy** source aktif, **noetic** yoruma alınmış, `ros2_ws` source aktif
- [ ] Yeni terminal açıldı / `source ~/.bashrc` yapıldı
- [ ] **Araç host'unda** `export ROS_DOMAIN_ID=0` (araç Fast-DDS varsayılanını kullanıyor, DDS export gerekmez)
- [ ] `ros2 launch smart_can_stuff can_launch.xml` çalıştırıldı → CAN + sensörler aktif
- [ ] `ros2 topic list` ile topic'ler (`/beemobs/...`) doğrulandı
- [ ] (Docker) `bee1_autonom.tar` **`/home/smart/Permanent` klasöründe** (reset'e dayanır)
- [ ] (Docker) **Her reset sonrası** `docker load -i /home/smart/Permanent/bee1_autonom.tar` çalıştırıldı
- [ ] (Docker) `docker run ... --gpus all --network host -e ROS_DOMAIN_ID=0 -v /home/smart/Permanent/data:/data` (Fast-DDS gömülü)
- [ ] Container içinde `ros2 topic list` araç topic'lerini görüyor (RMW/Fast-DDS eşleşmesi doğrulandı)

---

## 9. Aracı Hareket Ettirme — Temel Kontrol Sırası

CAN aktif olduktan sonra aracı hareket ettirmek için sıra:

1. **Kontağı ver (Ignition):** `/beemobs/rc_unittoOmux` → `RC_Ignition = 1`
   - Doğrula: `/beemobs/FB_OMUX_to_AUTONOMOUS` → `FB_IGNITION = 1`
2. **Vites seç:** `/beemobs/rc_unittoOmux` → `RC_SelectionGear` (0=Boş, 1=İleri, 2=Geri)
3. **Frenleri bırak:**
   - El freni: `/beemobs/AUTONOMOUS_HB_MotorControl` → `AUTONOMOUS_HB_MotEN=1`, `AUTONOMOUS_HB_MotState=1` (indir), `PWM≈200`
   - Ayak freni: `/beemobs/AUTONOMOUS_BrakePedalControl` → `AUTONOMOUS_BrakePedalMotor_PER=0`
4. **Gaz ver (Throttle):** `/beemobs/RC_THRT_DATA` (periyodik gönderilmeli)
   - `RC_THRT_PEDAL_PRESS = 0` (0 = güç verilebilir)
   - `RC_THRT_PEDAL_POSITION = 50–250` (kademeli artır; şu an ~100–250 aktif aralık)
5. **Direksiyon:** `/beemobs/AUTONOMOUS_SteeringMot_Control`
   - `AUTONOMOUS_SteeringMot_EN = 1` (önce aktif et)
   - `AUTONOMOUS_SteeringMot_PWM`: `0–127` = SOL, `128–255` = SAĞ

> ⚠️ **Önemli:** Gönderilen her mesaj periyodik olarak sürekli gönderilir; değerler son değişikliği koruyarak yayınlanmaya devam eder.
> Bu yüzden gaz verdikten sonra fren yapacaksan **önce gazı kes**, sonra fren yap; frenden çıkarken de değerleri tekrar `0`'a çek.

---

## 10. Önemli ROS2 Topic'leri (Referans)

### Araca Komut Gönderme (ROS → CAN)
| Topic | Amaç |
|---|---|
| `/beemobs/rc_unittoOmux` | Ignition, vites, farlar, sinyaller, kapı, acil stop |
| `/beemobs/RC_THRT_DATA` | Gaz / throttle (periyodik) |
| `/beemobs/AUTONOMOUS_BrakePedalControl` | Ayak freni |
| `/beemobs/AUTONOMOUS_SteeringMot_Control` | Direksiyon (EN + PWM) |
| `/beemobs/AUTONOMOUS_HB_MotorControl` | El freni |

### Araçtan Veri Okuma (CAN → ROS)
| Topic | İçerik |
|---|---|
| `/beemobs/FB_MotorDriver` | Güncel vites, tekerlek RPM |
| `/beemobs/FB_VehicleSpeed` | Araç hızı (km/h ve m/s) |
| `/beemobs/FB_OMUX_to_AUTONOMOUS` | Ignition, acil stop, farlar, kapı, batarya, hata durumları |
| `/beemobs/snd_RCUnit_SteeringData` | Direksiyon limit/PWM/yön |
| `/beemobs/snd_RCUnit_BrakeData` | Fren pedal pozisyon/ivme |
| `/beemobs/snd_RCUnit_HandBrakeData` | El freni durumu |
| `/beemobs/FeedbackSteeringAngle` | Teker dönüş açısı + fren pedal açısı |

---

## 11. Kamera Görüntüsü — ROS2 Bag Kaydı

ZED2 kamerasının RGB görüntüsünü kaydetmek için. Kamera topic'leri, CAN launch'ı (Bölüm 4) çalıştırıldıktan sonra yayınlanır.

### 11.1 Önce doğru topic'i bul
```bash
ros2 topic list | grep -i zed
# veya
ros2 topic list | grep -i image
```
- Bilinen RGB topic'i: **`/zed/zed_node/rgb_raw/image_raw_color`** (`sensor_msgs/Image`)
- Yayınlanıyor mu / FPS kontrolü:
  ```bash
  ros2 topic hz /zed/zed_node/rgb_raw/image_raw_color
  ```

### 11.2 Bag kaydı

> **NEREDE kaydetmeli:** Container da artık Foxy olduğu için bag formatı iki tarafta birebir aynıdır — araç host'unda da container içinde de (kalıcılık için `/data` altına) kaydedebilirsin.

Sadece RGB görüntü:
```bash
ros2 bag record /zed/zed_node/rgb_raw/image_raw_color -o camera_bag
```

RGB + kamera kalibrasyonu (kullanışlı — CameraInfo ile birlikte):
```bash
ros2 bag record -o camera_bag \
  /zed/zed_node/rgb_raw/image_raw_color \
  /zed/zed_node/rgb_raw/camera_info
```
> `camera_info` topic'inin tam adını `ros2 topic list | grep camera_info` ile doğrula.

### 11.3 Boyut kontrolü (ÖNEMLİ)

1280x720 ham görüntü ~30 FPS ≈ **80 MB/s** (~5 GB/dk). Uzun kayıtlarda:

- **Zstd sıkıştırma** ile kaydet:
  ```bash
  ros2 bag record -o camera_bag \
    --compression-mode file --compression-format zstd \
    /zed/zed_node/rgb_raw/image_raw_color
  ```
- **Süreye göre böl** (ör. her 60 sn'de yeni dosya):
  ```bash
  ros2 bag record -o camera_bag -d 60 /zed/zed_node/rgb_raw/image_raw_color
  ```
- Kayıt öncesi disk boşluğunu kontrol et: `df -h`

### 11.4 Kaydı bitirme ve doğrulama
- Kaydı durdur: **`Ctrl+C`**
- İçeriği görüntüle:
  ```bash
  ros2 bag info camera_bag
  ```
- Geri oynat (test için):
  ```bash
  ros2 bag play camera_bag
  ```

### 11.5 Bag'i bilgisayarına çekme
- **Files (Nautilus) SFTP** ile `sftp://smart@192.168.30.100`'a bağlanıp `camera_bag/` klasörünü sürükle-bırak (Bkz. Bölüm 2).

---

## 12. Haritalama (SLAM) — VLP-16 Lidar + rtabmap

Aracın **Velodyne VLP-16** lidarı ile 3D SLAM. Bizim `vehicle_mapping` paketimizle (container içinde) çalışır.

> **Neden lidar?** Araç dış mekanda ~30 km/sa gidiyor; RGB-D (ZED derinlik) SLAM güneş/hız/kısa menzil nedeniyle kırılgan. Lidar dış mekan için çok daha sağlam.
> Boru hattı: `/velodyne_points` → `icp_odometry` (odometri) → `rtabmap` (harita + loop closure).

### 12.1 Ön koşullar
- CAN + sensörler ayağa kalkmış olmalı (Bölüm 4: `ros2 launch smart_can_stuff can_launch.xml`) → lidar `/velodyne_points` yayınlıyor.
- Container çalışıyor olmalı (Bölüm 7.6) ve içine girilmiş olmalı (Bölüm 7.7: `docker exec -it bee1_autonom bash`).

### 12.2 Topic ve TF doğrulama (İLK İŞ)
```bash
# Lidar topic'i gerçekten bu mu?
ros2 topic list | grep -i velodyne          # beklenen: /velodyne_points
ros2 topic hz /velodyne_points              # veri akıyor mu (5-20 Hz)

# IMU topic'i (ZED2 varsayılan; XSENS kullanacaksan adını not al)
ros2 topic list | grep -iE "imu"

# Araçtaki mevcut TF ağacı (velodyne frame'i zaten var mı?)
ros2 run tf2_tools view_frames
```

### 12.3 Haritalamayı başlat
```bash
# Varsayılanlar: scan_cloud=/velodyne_points, imu=/zed/zed_node/imu/data,
#                frame_id=base_link, lidar_frame=velodyne
ros2 launch vehicle_mapping lidar_mapping.launch.py
```

Topic/frame farklıysa argümanla değiştir (örnek — XSENS IMU + araç kendi TF'sini veriyorsa):
```bash
ros2 launch vehicle_mapping lidar_mapping.launch.py \
    scan_cloud_topic:=/velodyne_points \
    imu_topic:=/imu/data \
    use_imu:=true \
    frame_id:=base_link \
    lidar_frame:=velodyne \
    publish_lidar_tf:=false        # araç zaten base_link->velodyne TF veriyorsa
```

**Tüm argümanları görmek için:**
```bash
ros2 launch vehicle_mapping lidar_mapping.launch.py --show-args
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `scan_cloud_topic` | `/velodyne_points` | Lidar PointCloud2 |
| `imu_topic` | `/zed/zed_node/imu/data` | Füzyonlu IMU (XSENS için değiştir) |
| `frame_id` | `base_link` | Robot taban frame |
| `lidar_frame` | `velodyne` | Lidar frame_id |
| `use_imu` | `true` | IMU füzyonu aç/kapa |
| `publish_lidar_tf` | `true` | base_link→lidar statik TF yayınla |
| `localization` | `false` | true: kayıtlı haritada lokalizasyon |
| `delete_db` | `true` | Başta DB sil (yeni harita) |
| `qos_scan` / `qos_imu` | `0` / `1` | 0=reliable, 1=best_effort |

### 12.4 Haritanın oluştuğunu doğrula
```bash
# Odometri ve harita frame'leri yayınlanıyor mu?
ros2 topic hz /odom
ros2 topic list | grep -iE "rtabmap|map|grid"     # /map, /rtabmap/... görünmeli

# TF zinciri tam mı: map -> odom -> base_link -> velodyne
ros2 run tf2_tools view_frames
```
- Aracı **yavaşça** sür; harita `map` frame'inde büyümeli.

### 12.5 Haritayı kaydetme ve dışa aktarma
- rtabmap veritabanı varsayılan olarak `~/.ros/rtabmap.db` altına yazılır. **Kalıcılık için** container'ı `-v /home/smart/Permanent/data:/data` ile başlatıp DB'yi oraya yönlendir:
  ```bash
  ros2 launch vehicle_mapping lidar_mapping.launch.py \
      --ros-args -p database_path:=/data/rtabmap.db
  ```
- Kaydı bitir: `Ctrl+C` (DB otomatik yazılır).
- Bilgisayara çek: **Files/SFTP** ile `rtabmap.db` dosyasını al; yerelde `rtabmap-databaseViewer rtabmap.db` ile incele.

### 12.6 Kaydedilmiş haritada lokalizasyon (sonraki turlar)
```bash
ros2 launch vehicle_mapping lidar_mapping.launch.py \
    localization:=true delete_db:=false \
    --ros-args -p database_path:=/data/rtabmap.db
```

> ⚠️ **Sahada doğrulanacak varsayımlar:** (1) lidar topic `/velodyne_points`, (2) IMU topic adı ve QoS, (3) `base_link→velodyne` TF — Mimari dokümandaki offset (~x=-0.177, z=0.62) *ön aks merkezine* göredir, `base_link` tanımına göre kalibre et. Araç kendi TF'sini yayınlıyorsa `publish_lidar_tf:=false`.

---

## Notlar

- Araç maksimum hızı fiziksel olarak 55 km/sa, yazılımsal olarak **30 km/sa** ile limitli.
- Acil Stop: araç içi/dışı butonlar veya CAN sinyali ile tetiklenebilir. Aktif olunca araç çalışır ama hareket edemez (tahrik motoru durur, vites boşa alınır).
- Sensör montaj konumları (ön aks merkezine göre X/Y/Z) için Genel Bilgilendirme dokümanına bakılabilir (Lidar, Kamera, GPS/IMU offset'leri).
