#!/bin/bash
# =====================================================================
#  ARAC TESHIS SCRIPTI - park algoritmasi sorun tespiti
#  Kullanim (ARACTA, SSH ile baglandiktan sonra):
#      bash ARAC_TESHIS.sh
#  Cikti: ~/teshis_<tarih>.txt  (tek dosya, bana gonderebilirsiniz)
#
#  NOT: Bu script SADECE OKUR, hicbir sey degistirmez / arac hareket etmez.
#  DOCKER TAR'INA GEREK YOK - aracin kendi sistemini kontrol eder.
# =====================================================================

OUT="$HOME/teshis_$(date +%Y%m%d_%H%M%S).txt"
exec > >(tee "$OUT") 2>&1

echo "==================== ARAC TESHIS RAPORU ===================="
echo "Tarih : $(date)"
echo "Host  : $(hostname)"
echo "Dosya : $OUT"
echo ""

source /opt/ros/foxy/setup.bash 2>/dev/null
# Aracin kendi workspace'i (yol farkliysa duzeltin)
for WS in ~/Projects/ros2_ws ~/ros2_ws /home/smart/Projects/ros2_ws; do
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" && echo "[ws] $WS yuklendi" && break
done
echo "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-<bos>}"
echo ""

echo "########## 1) TUM TOPIC LISTESI ##########"
ros2 topic list
echo ""

echo "########## 2) CALISAN NODE'LAR ##########"
ros2 node list
echo ""

echo "########## 3) SENSORLER YAYINDA MI (Hz) ##########"
for t in /gnss /imu/data /velodyne_points; do
  echo "--- $t ---"
  timeout 5 ros2 topic hz "$t" 2>&1 | head -3
done
echo ""

echo "########## 4) GPS FIX KALITESI ##########"
echo "(status.status >= 0 olmali; negatif = FIX YOK)"
timeout 6 ros2 topic echo /gnss 2>&1 | head -25
echo ""

echo "########## 5) ARAC GERI BESLEMESI (kapali cevrim buna BAGLI) ##########"
echo "--- /beemobs/FeedbackSteeringAngle (YOKSA park guvenli durusa gecer!) ---"
timeout 6 ros2 topic echo /beemobs/FeedbackSteeringAngle 2>&1 | head -8
echo "--- /beemobs/FB_VehicleSpeed ---"
timeout 6 ros2 topic echo /beemobs/FB_VehicleSpeed 2>&1 | head -8
echo ""

echo "########## 6) FREN BASILI MI? (ARACIN HAREKET ETMEMESININ 1 NUMARALI SEBEBI) ##########"
echo "(autonomous_brakepedalmotor_per: 100 surekli geliyorsa BIRI FRENI TUTUYOR)"
timeout 6 ros2 topic echo /beemobs/AUTONOMOUS_BrakePedalControl 2>&1 | head -20
echo ""
echo "--- freni KIM yayinliyor? (birden fazla ise CAKISMA) ---"
ros2 topic info /beemobs/AUTONOMOUS_BrakePedalControl
echo ""

echo "########## 7) GAZ KOMUTU ##########"
timeout 6 ros2 topic echo /beemobs/RC_THRT_DATA 2>&1 | head -12
echo "--- gazi KIM yayinliyor? ---"
ros2 topic info /beemobs/RC_THRT_DATA
echo ""

echo "########## 8) GOREV KAPILARI (park baslamadiysa sebep burada) ##########"
echo "--- /gorev_ok  ('start' ise gorev_stop %100 FREN basiyor!) ---"
timeout 5 ros2 topic echo /gorev_ok 2>&1 | head -5
echo "--- /park_gorev  ('start' gelmezse park HIC baslamaz) ---"
timeout 5 ros2 topic echo /park_gorev 2>&1 | head -5
echo "--- /current_zone  (PARK gorunmezse master park komutu vermez) ---"
timeout 5 ros2 topic echo /current_zone 2>&1 | head -5
echo "--- /vehicle_command (path_planner -> master) ---"
timeout 5 ros2 topic echo /vehicle_command 2>&1 | head -5
echo ""

echo "########## 9) LOKALIZASYON (cep koordinatlari buna BAGLI) ##########"
echo "--- /odometry/filtered ---"
timeout 5 ros2 topic echo /odometry/filtered 2>&1 | head -12
echo "--- /odometry/filtered_map (YOKSA cepler YANLIS cercevede kalir!) ---"
timeout 5 ros2 topic echo /odometry/filtered_map 2>&1 | head -12
echo ""

echo "########## 10) KAMERA (YOLO icin) ##########"
ros2 topic list | grep -i zed
echo "--- goruntu genisligi (cam_image_width icin) ---"
for t in $(ros2 topic list | grep -iE "zed.*image" | head -2); do
  echo "[$t]"
  timeout 5 ros2 topic echo "$t" --field width 2>&1 | head -2
done
echo ""

echo "########## 11) DOCKER DURUMU ##########"
docker images 2>&1 | head -10
echo "--- calisan container'lar ---"
docker ps -a 2>&1 | head -10
echo ""

echo "########## 12) PARK CALISIYORSA DURUMU ##########"
timeout 5 ros2 topic echo /parking_state 2>&1 | head -5
timeout 5 ros2 topic echo /parking/slot_status 2>&1 | head -5
echo ""

echo "==================== RAPOR BITTI ===================="
echo "Rapor dosyasi: $OUT"
echo "Bana gondermek icin: bu dosyanin icerigini kopyalayin."
