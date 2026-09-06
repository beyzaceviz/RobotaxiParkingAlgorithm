# =============================================================================
#  Teknofest Robotaksi — PARK ALGORITMASI (Arac Entegre) — ROS 2 FOXY
#
#  Arac natif ROS 2 Foxy (Ubuntu 20.04, Fast-DDS 2.0.x, Python 3.8) calistirir.
#  Bu image de ros:foxy tabanlidir -> capraz-dagitim DDS koprusu YOK; aracla
#  birebir ayni dagitim + ayni DDS surumu.
#
# -----------------------------------------------------------------------------
#  ICERDIGI PAKETLER (2026-07-25 saha_ws birlestirmesi)
# -----------------------------------------------------------------------------
#    - parking_safety        : TEK aktif park algoritmasi (ament_python)
#         + beemobs_actuator.py        (SINIF, node degil: CAN aktuator katmani;
#                                       DOGRUDAN PWM + kapali cevrim; aractaki
#                                       steering_pid/speed_pid KULLANILMAZ)
#         + parking_gps_zone_trigger   (/gnss -> /parking_zone_reached geofence)
#         + parking_mission_planner    (/velodyne_points + /yolo_detections
#                                       -> /parking/slot_status)
#         + parking_decision_and_control_node (GPS kapisi + pure-pursuit
#                                       -> BeemobsActuator ile /beemobs/*)
#    - robotaxi_localization : wheel_odometry_node + robot_localization cift-UKF
#         -> /odometry/filtered (surekli/puruzsuz, kontrol icin)
#         -> /odometry/filtered_map (GPS-mutlak, cep koordinatlari icin)
#         park_bringup bunu dahil eder ve /odometry/filtered'i /odom'a remap eder.
#    - smart_can_msgs        : /beemobs/... mesaj tipleri (ZORUNLU)
#    - yolo_traffic_detector : /yolo_detections uretir (2026-07-25 BU IMAJA
#         TASINDI - gerekcesi asagida). models/best.pt paket icinde gelir.
#
#  KALDIRILDI (saha_ws karari): cmd_vel_to_beemobs_bridge. Ara /cmd_vel + ayri
#    kopru yerine kontrol dugumu araci DOGRUDAN suruyor (BeemobsActuator.tick()).
#    Gorev pasifken aktuator tamamen susar -> /beemobs/* sahibi lane_following
#    kalir, aktuator cakismasi yapisal olarak cozulmus olur.
#  KALDIRILDI (2026-07-24): parking_controller_pkg (C++/PCL, kullanilmayan
#    ikinci park implementasyonu; ayni anda calissa /cmd_vel'de cakisirdi).
#    PCL/lifecycle bagimliliklari da bu yuzden burada YOK.
#
#  NEDEN YOLO ARTIK AYRI IMAJ DEGIL (2026-07-25):
#    parking_mission_planner /yolo_detections'a abone; tabela<->cep esleme
#    onsuz calismaz. Iki ayri container -> iki ayri /dev/shm + iki ayri DDS
#    katilimcisi demekti (discovery calisip veri akmama riski, iki tar, iki
#    ayri calistirma komutu). Tek imaj: torch katmani zaten paylasiliyordu,
#    toplam boyut ~ayni; saha komutu tek. Kamera yoksa/bench testinde
#    launch argumaniyla kapatilir: park_bringup.launch.py start_yolo:=false
# -----------------------------------------------------------------------------
#  HIZLI KULLANIM
#    Derle   : docker build -t park_algoritmasi_foxy .
#    Paketle : docker save -o park_foxy.tar park_algoritmasi_foxy
#    Aracta  : docker load -i ~/park_foxy.tar
#    Calistir: sudo docker run -it --rm --network host --gpus all \
#                -e ROS_DOMAIN_ID=0 --name park park_algoritmasi_foxy
#    Icinde  : ros2 launch parking_safety park_bringup.launch.py
#
#  SURUM PINLERINI DEGISTIRMEDEN ONCE ILGILI KATMANIN YORUMUNU OKUYUN —
#  her pin bilinen bir kirilmanin cozumudur.
# =============================================================================
FROM ros:foxy

# ----------------------------------------------------------------------------
# Ortam degiskenleri
# ----------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0
ENV ROS_DISTRO=foxy

# ----------------------------------------------------------------------------
# Haziran 2025 ROS imzalama anahtari rotasyonu duzeltmesi
# (HER apt-get update'ten ONCE gelmeli — eski ros:foxy imajindaki anahtarin
#  suresi dolmus durumda ve apt'yi komple kirar.)
# ros2-apt-source 1.2.0, EOL dagitimlar icin focal build icerir.
# ----------------------------------------------------------------------------
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list \
          /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
          /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.focal_all.deb && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Sistem + ROS bagimliliklari (hepsi Foxy deposunda mevcut)
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-sensor-msgs-py \
    ros-foxy-robot-localization \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-vision-opencv \
    ros-foxy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*
# NOT: sensor_msgs_py, parking_mission_planner + parking_decision LiDAR
# islemesi icin ZORUNLU (pc2.read_points). rosdep'e birakilmadi, ACIKCA kuruldu.
# Foxy'de read_points VAR ama read_points_numpy YOK (Humble ile geldi) — kod
# read_points + np.array kullanir (Foxy-uyumlu).
# cv_bridge / image_transport / vision_opencv: yolo_traffic_detector icin.

# ----------------------------------------------------------------------------
# PyTorch — AYRI agir katman (ultralytics re-pin'i torch'u tekrar indirmesin)
# torch 2.4.1 = Python 3.8 destekleyen SON surum (2.5.0'da cp38 wheel yok).
# PyPI wheel'i cu121 -> RTX 3060 (sm_86) uyumlu; CUDA runtime wheel icinde,
# host'ta yalnizca NVIDIA surucusu (>=525) + nvidia-container-toolkit gerekir.
# ----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --timeout 120 --retries 10 torch==2.4.1 torchvision==0.19.1

# ----------------------------------------------------------------------------
# YOLO + lokalizasyon Python bagimliliklari (py3.8 pinleri)
#  - ultralytics 8.3.186: py3.8'de kurulabilen SON surum (8.3.60 import kirik,
#    >=8.3.187 kurulum kirik). Sorun cikarsa fallback: ultralytics==8.3.59
#  - opencv-python 4.10 pini: pin'siz gelen OpenCV 5.0, Foxy cv_bridge
#    (OpenCV 4.2'ye derlenmis) ile uyusmuyor (KeyError: 16).
#  - pyproj 3.5.0 / scipy 1.10.1: py3.8'i destekleyen son surumler
#    (robotaxi_localization GPS->ENU donusumu ve UKF icin).
# ----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir --timeout 120 --retries 10 \
    ultralytics==8.3.186 \
    opencv-python==4.10.0.84 \
    "numpy<2" \
    pyyaml \
    pyproj==3.5.0 \
    scipy==1.10.1

# setuptools pini TUM pip kurulumlarindan SONRA: 58.2.0 = ROS 2 standart pini
# (ultralytics setuptools'u yukseltir; colcon ament_python + rosdep
# pkg_resources icin bu surume geri cekilir)
RUN pip3 install --no-cache-dir setuptools==58.2.0

# ----------------------------------------------------------------------------
# Workspace + ROS bagimliliklari
# COPY src BU NOKTADA -> src degisikligi yalnizca rosdep+colcon'u tetikler,
# agir torch katmanini yeniden indirmez.
# ----------------------------------------------------------------------------
WORKDIR /ros2_ws

# Paketleri kopyala (host build/install/log ve *.tar .dockerignore ile haric)
COPY ./src ./src

# rosdep: Foxy EOL -> --include-eol-distros SART (yoksa "Unsupported distro").
# -r + --skip-keys: eksik/test-only anahtar build'i durdurmasin (kritik
# bagimliliklar ustteki apt katmaninda zaten kurulu).
RUN apt-get update && \
    rosdep update --include-eol-distros --rosdistro foxy && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "python3-pytest ament_copyright ament_flake8 ament_pep257" \
    && rm -rf /var/lib/apt/lists/*

# Derleme (colcon bagimlilik sirasini kendi cozer: smart_can_msgs once uretilir)
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

# DDS: Foxy'nin varsayilani zaten Fast-DDS; aracla eslesme acikca sabitlenir.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ----------------------------------------------------------------------------
# Ortam otomatik yuklensin (interaktif shell icin)
# ----------------------------------------------------------------------------
RUN echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc

# ----------------------------------------------------------------------------
# Fast-DDS UDP-only profili: container /dev/shm'i host'tan AYRI oldugu icin
# varsayilan shared-memory transportu container<->arac veri akisini koparir
# (discovery calisir, veri AKMAZ — en sinsi saha hatasi). SHM kapatilip
# UDPv4'e zorlanir. XML semasi Fast-DDS 2.0.x (Foxy) ile uyumlu.
# ----------------------------------------------------------------------------
COPY fastdds_udp_only.xml /fastdds_udp_only.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml

# Entrypoint her komutta ROS + workspace + RMW yukler
# (`docker exec` ile acilan ikinci terminalde de ortam hazir olsun diye)
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
