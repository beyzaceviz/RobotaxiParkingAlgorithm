#!/usr/bin/env python3
"""Park Gorev Planlayici (Parking Mission Planner).

Otoparktaki 8 cebin doluluk (LiDAR) ve izin (YOLO tabela) durumunu
haritalandirir, kullanilabilir cepleri yayinlar.

Mimari:
  - /velodyne_points (PointCloud2)      -> cep bazli ROI doluluk kontrolu
  - /odom (nav_msgs/Odometry)           -> arac pozu (LiDAR -> dunya donusumu icin)
  - /yolo_detections (std_msgs/String, JSON dizisi) -> frame'deki TUM tespitler
        Ornek: [{"class": "park_yeri", "conf": 0.91, "distance": 8.5,
                 "cx": 0.45, "cy": 0.52}, ...]
  - /parking/slot_status (std_msgs/String, JSON, 10 Hz)   -> durum matrisi yayini
  - /perception_active (std_msgs/Bool)  -> Algi Kilidi (Stop-and-Stare):
        kontrol dugumu arac hareketsizken (dur-ve-tara fazlari) True,
        hareket halindeyken False yayinlar. False iken YOLO ve LiDAR
        guncellemeleri ISLENMEZ (hareket bulanikligi/odometri gecikmesi
        kararsiz eslesme uretir); hafizadaki son stabil matris
        /parking/slot_status uzerinden yayinlanmaya devam eder.

ONEMLI - YOLO tabela -> cep eslestirme (ACISAL, mesafeden bagimsiz):
  YOLO'nun mesafe tahmini kutu boyutu + sinifin VARSAYILAN fiziksel
  boyutuna dayanir; Gazebo mesh boyutlari varsayimdan farkli oldugu icin
  siniflar arasi sistematik olcek hatasi olusur (ornek: ayni hizadaki
  park_yasak 8.5 m, park_yeri 11.4 m olculdu). Bu yuzden mesafe SADECE
  kaba menzil kapisinda kullanilir; konumlandirma tamamen acisaldir:
    1. Olculen aci: bearing_cam = atan2((cx-0.5)*goruntu_genisligi, odak_px)
       -> dunya acisi = yaw - bearing_cam
    2. Beklenen aci her cep icin CEP MERKEZINE DEGIL, beklenen TABELA
       konumuna hesaplanir (tabelalar cep merkezlerinin ~1.65 m
       arkasindaki sirada durur, sign_row_offset_x). Cep merkezi
       kullanilsaydi egik bakista eslesme sistematik olarak komsu cebe
       kayardi (Cep 2 -> Cep 3 hatasinin kokeni).
    3. En kucuk aci farkli cep secilir; IKI kapi birden gecilmeli:
       - best_diff <= sign_assoc_max_deg (mutlak hata siniri)
       - (second_diff - best_diff) >= min_angle_diff (belirsizlik MARJI):
         en iyi aday ikinciden en az bu kadar daha iyi olmali. Egik/uzak
         bakista cepler acisal olarak sikisir; marj yetersizse belirsiz
         okuma ISLENMEZ, arac yaklastikca ayrisip islenir. (Onceki oran
         kapisi kucuk acilarda asiri seciciydi; marj kapisi dar
         perspektifteki gecerli okumalari kurtarir.)
    4. Ayni frame'in tespitleri ceplere ORTAK atanir (1 cep <= 1 tespit):
       en net eslesme once yerlesir; sonraki tespit KALAN cepler icinden
       secer ve belirsizlik marji da yalnizca serbest ceplere karsi
       olculur. Tek tek atama, yan yana iki tabela goruldugunde ikisinin
       ayni cebe yazilmasina / birinin komsu cebi 'calmasina' izin
       veriyordu (wp3'te Cep 3 tabelasinin Cep 4'e kaymasi). NOT: doluluk
       (LiDAR) bilgisi atamada BILEREK kullanilmaz - dolu ama izinli cep
       (araba park etmis P cebi) mesru bir durumdur; dolulukla filtrelemek
       Cep 4'un kendi tabelasini reddedip yanlis cebe itebilirdi.
  /yolo_detections ham per-frame veridir; koruma COK KATMANLIDIR: guven
  esigi (min_sign_conf) + menzil kapisi + belirsizlik kapisi + DURAK BAZLI
  COGUNLUK OYLAMASI (majority voting, LiDAR havuzuyla senkron sifirlanir).

  KILIT YUMUSATMA (overwrite, kalite bazli):
    Permit bir kez commit edilse de KATI degildir. Her durak temiz oy
    havuzu toplar; kalite skoru KAZANAN SINIFIN kendi tespitlerinden
    hesaplanir: kalite = o sinifin en yuksek guveni * en net acisi
    (gate'e gore normalize). Kalite izleri sinif bazlidir; yoksa azinlik
    sinifin yuksek guvenli tespiti cogunlugun kararina kalite
    kazandirirdi (kalite hirsizligi). Overwrite kapisi yalnizca
    DURAKLAR ARASI calisir: yeni durak eski karari ancak
    sign_overwrite_margin kadar yuksek kaliteyle ezebilir (histerezis).
    AYNI DURAK icinde havuz buyudukce hukum serbestce guncellenir;
    erken (az oyla) atilan yanlis commit, ayni durakta cogunluk
    degisince kendini duzeltir. permit degerinin kendisi durak
    gecislerinde SILINMEZ (gecmis tecrube korunur); yalnizca oy
    havuzlari sifirlanir.

  YOL GURULTUSU TEMIZLIGI:
    Arac hareket halindeyken (ALGI KILITLENDI) YOLO tespitleri ISLENMEZ
    ve her algi-kilidi gecisinde TUM oy havuzlari sifirlanir. Boylece
    yolda birikebilecek 'hayalet oylar' arac durdugundaki temiz algiyi
    zehirleyemez; her durus sifir havuzla, taze oylar toplar.

ONEMLI - Koordinat cercevesi:
  /velodyne_points verisi velodyne_link (arac) cercevesindedir, kuresel DEGILDIR.
  Bu yuzden her nokta once sensor montaj ofseti (chassis'e gore x=0.3, z=0.8,
  AUtomotion1.world'den) ve /odom pozu (x, y, yaw) ile dunya cercevesine
  donusturulur; ROI karsilastirmasi ondan sonra yapilir. Ackermann eklentisi
  odom'u dunya pozu olarak yayinladigi icin odom = Gazebo koordinati kabulu
  gecerlidir (parking_decision_and_control_node ile ayni kabul).

ONEMLI - Gozlemlenebilirlik:
  LiDAR'in yatay FOV'u sadece on +/-90 derecedir (world dosyasindan). Bir cep
  o anda gorus alaninda / menzilde degilse ROI'ye nokta dusmemesi cebin bos
  oldugunu KANITLAMAZ. Bu yuzden occupied sadece cep gozlemlenebilirken
  guncellenir; gozlemlenmemis cepler 'unknown' kalir.

ROI (her cep merkezine gore, dunya cercevesinde):
  x: +/-1.0 m (cep derinligi yonu)
  y: +/-0.55 m (cep genisligi; komsu cep araligi min 1.13 m oldugu icin
     +/-1.5 m KULLANILAMAZ - komsu cepteki arac yanlis pozitif verir)
  z: [0.1, 1.0] m (zemin haric, arac govdesi dahil)

ONEMLI - Frame Biriktirme (Temporal Aggregation, Dur-Tara ile senkron):
  Doluluk karari TEK frame'den verilmez. Algi acikken (arac sabit) her
  cebin ROI nokta sayisi havuza eklenir; en az min_pool_frames kare
  biriktikten sonra kare basina ORTALAMA >= min_avg_points ise DOLU.
  Dik bakis acisindaki uzak cep (orn. baslangictan Cep 7) karede sadece
  2-4 nokta uretir -> ortalama ~3, esik 1.5 ile yakalanir; tek karelik
  hayalet parlama ise havuzda seyrelir (30 karede ort. ~0.1) ve elenir.
  DIKKAT: ortalama birikimle buyumez; esik 5-10'a cekilirse Cep 7 yine
  kacar. Havuz, algi kilidi her degistiginde sifirlanir (yeni durus
  noktasi = yeni perspektif = temiz havuz).

ONEMLI - DOLU Kilidi (Object Permanence, cok durakli tarama ile senkron):
  Otopark statiktir: bir kez DOLU tespit edilen cep, sonraki duraklardan
  yapilan taramalarda dusuk sayim verse bile (gorus hatti baska engelle
  kesildi = occlusion; slot_observable bunu BILEMEZ, sadece FOV/menzil
  bakar) ASLA BOS'a dondurulmez. Ornek kaza senaryosu: baslangictan Cep 6
  ort. 51 nokta ile DOLU; wp2'den bakista occlusion yuzunden ort. 1.4 <
  1.5 -> kilit olmasa BOS'a doner ve arac dolu cebe park etmeye kalkardi.
  BOS -> DOLU yonu aciktir (yeni tespit guvenlik kazanci). permit latch'i
  ile ayni felsefe: ogrenilen guvenlik bilgisi unutulmaz.

Kullanilabilirlik kurali:
  occupied == False VE permit == 'park_edilebilir' VE cep en az bir kez
  gozlemlenmis (status != 'unknown') ise cep kullanilabilirdir.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class ParkingMissionPlanner(Node):

    # YOLO sinif adi -> planner permit degeri.
    # 'park_yeri_engelli' normal arac icin YASAK kabul edilir; engelli
    # araci senaryosu gelirse burasi degistirilir.
    SIGN_TO_PERMIT = {
        'park_yeri': 'park_edilebilir',
        'park_edilebilir': 'park_edilebilir',
        'park_yasak': 'park_yasak',
        'park_yeri_engelli': 'park_yasak',
    }

    def __init__(self):
        super().__init__('parking_mission_planner')

        # --- Cep Geometrileri (GERCEK SAHA - GPS'ten ENU donusumu) ----------
        # 2026-07-24: kullanicinin verdigi gercek GPS (lat, lon) olcumlerinden
        # hesaplandi. Burada saklanan x,y DEGERLERI ENU-MUTLAK (Origin =
        # Start noktasi Lat 40.7901429, Lon 29.5092136; x=Dogu, y=Kuzey [m]).
        #
        # DUZELTME (2026-07-24, 2. tur): bu dosyanin ONCEKI yorumu
        # "beemobs_odometry_node'un use_imu=True calismasi yeterli" diyordu -
        # BU ARTIK GECERSIZ: beemobs_odometry_node SILINDI, yerine
        # robotaxi_localization (robot_localization cift-UKF) geldi ve onun
        # /odometry/filtered (park_bringup.launch.py'de /odom'a remap edilen)
        # ciktisi yaw'i IMU'nun MUTLAK yonelimiyle DEGIL, sadece acisal HIZ
        # (vyaw) entegrasyonuyla uretiyor (bkz. ukf_fusion.yaml ukf_local.
        # imu0_config[5]=false). Yani /odom'un yaw=0'i aracin ACILIS ANINDAKI
        # yonudur, Dogu DEGIL - bu ENU degerleriyle DOGRUDAN karsilastirilamaz.
        #
        # COZUM: capture_enu_transform() / enu_to_local() (asagida). Park
        # bolgesine girildigi an (/parking_zone_reached), /odometry/filtered
        # (yerel, puruzsuz - kontrol icin) ile /odometry/filtered_map (GPS
        # ile duzeltilmis, ENU'ya yakin - navsat_transform+IMU+manyetik sapma
        # sayesinde) arasindaki SABIT yaw farki (yaw_offset) BIR KEZ olculur;
        # bu ceplerin ENU koordinatlarini /odometry/filtered'in yerel
        # cercevesine donusturmek icin kullanilir (self.slot_xy_local).
        # LiDAR ROI kontrolu ve YOLO acisal eslestirmesi artik slot['x']/['y']
        # (ham ENU) DEGIL, self.slot_xy_local[sid] (donusturulmus) kullanir.
        # Donusum yakalanamazsa (ör. /odometry/filtered_map hic gelmezse)
        # slot_xy_local ham ENU degerlerine DUSER (eski/hatali davranis) ve
        # bir UYARI loglanir - sessiz basarisizlik YOK.
        #
        # Eski Gazebo simulasyon koordinatlari (x~-24, y~-6.6..0.8) GECERSIZ,
        # KALDIRILDI.
        # status : 'unknown' (henuz gozlemlenmedi) | 'bos' | 'dolu'
        # permit : 'unknown' | 'park_edilebilir' | 'park_yasak'
        # ===================================================================
        #  2026-07-28 GUNCELLEME - ceplere PARK EDILEREK alinan bag kayitlari
        # ===================================================================
        # ESKI degerler kullanicinin ELLE OKUDUGU tek atislik GPS gosterimlerinden
        # geliyordu. Yeni degerler, arac cebe park edilip 20-40 s DURDURULARAK
        # kaydedilen bag'lerdeki /filter/positionlla akisinin (2000-3900 ornek)
        # son %40'inin ortalamasi. Fark ~2.8 m'ye kadar cikiyordu - goal_tolerance
        # 0.35 m oldugu icin eski degerlerle arac cebi TAMAMEN ISKALARDI.
        #
        # Kaynak bag'ler (cep_bagleri/):
        #   cep1_0727_1746  cep3_0727_1749  cep5_0727_1751
        #   cep7_0727_1753  cep9_0727_1756
        # TEK sayili cepler OLCULDU. CIFT sayililar (2,4,6,8) ARA-DEGERLEME:
        #   28 Temmuz'da alinan cep2/4/6/8 bag'lerinde Xsens surucusu
        #   calismadigi icin /gnss ve /filter/* HIC kaydedilmemis (sadece
        #   /beemobs/* CAN var) -> konum cikarilamadi.
        #
        # 2026-07-28 REVIZE - koordinatlar WGS84 ELIPSOIT ile yeniden hesaplandi.
        # ONCEKI degerler kuresel R=6378137 kullaniyordu; 40.79 derece enlemde
        # dogru yaricap N=6387268, kuzey yaricapi M=6362688 -> eski hesap
        # kuzeyde %0.243 uzun, doguda %0.143 kisa. Cep1-cep9 arasi 20.6 m'de
        # ~5 cm ve ayrica sekil bozulmasi. Simdi N/M ayri kullaniliyor.
        #
        # Ara-degerleme yontemi: 5 olculen cebe en kucuk kareler dogrusu
        # (aci -47.771 deg ENU), komsu cep araligi 2.5660 m. Cift cepler
        # komsularinin tam ortasina konuldu.
        #
        # !! FIT KALITESI - ONCEKI IDDIA DUZELTILDI !!
        #   Daha once "fit artigi max 0.095 m" yazmistim; DOGRULANMADI.
        #   Gercek deger: dik artik max 26.2 cm, rms 15.9 cm.
        #   goal_tolerance 0.35 m ile ayni mertebede -> cep haritasi
        #   sanildigi kadar kesin DEGIL. slot_offset_x/y ile saha basinda
        #   kaydirma bu yuzden onemli (asagi bkz.).
        #
        # !! COZULMEMIS: CEP ARALIGI 3 FARKLI CEVAP VERIYOR !!
        #   serit metre (2.47 + 0.15 cizgi) : 2.620 m -> cep1-cep9 20.96 m
        #   serit metre (2.47 duz)          : 2.470 m -> 19.76 m
        #   GPS regresyon (5 nokta)         : 2.566 m -> 20.53 m
        #   GPS uc-uca (cep1->cep9)         : 2.578 m -> 20.63 m
        #   Ardisik olcumler de tutarsiz: 2.602 / 2.497 / 2.539 / 2.688 m.
        #   KARAR: GPS regresyonu kullanildi (tek tutarli kaynak). Sahada
        #   cep1 orta cizgisinden cep9 orta cizgisine TEK serit metre olcumu
        #   bunu kesin cozer - o olcum gelene kadar cift cepler SUPHELI.
        #
        # NOT: bu koordinatlar GPS ANTENININ konumudur, aracin merkezi degil.
        # Araç ceplere 32.85..62.85 derece arasi degisen aciyla park edilmis
        # (cep1 en kotusu), yani anten kol boyu her cepte farkli yone kaydirir.
        #
        # !! BU HIPOTEZ TEST EDILDI VE REDDEDILDI (2026-07-28) !!
        #   Kol boyu d, 0..3 m araliginda tarandi; her cep koordinatindan
        #   d*[cos(yaw), sin(yaw)] cikarilip dogru yeniden fit edildi:
        #       d=0.00 -> artik rms 18.0 cm   <- EN IYI
        #       d=0.50 -> 20.0 cm     d=1.00 -> 24.5 cm     d=1.50 -> 30.4 cm
        #   Duzeltme fiti TEK YONLU kotulestiriyor. Yani 16-26 cm'lik artik
        #   anten geometrisinden DEGIL, surucunun goz karari park etmesinden
        #   geliyor. (Testin gucu zayif: 5 cepten sadece cep1'in yaw'i
        #   digerlerinden belirgin farkli, yani pratikte tek noktaya dayaniyor.)
        #
        #   AYRICA: 1.44 m anten offset'i fusion_localization.launch.py'de
        #   static TF (base_link->gps_link) olarak VAR, ama xsens_odometry_node
        #   onu UYGULAMIYOR - GPS konumunu dogrudan base_link sayiyor. Cep
        #   haritasi da ham anten koordinatindan uretildigi icin iki taraf
        #   TUTARLI. Ancak localization_source:=ukf yapilirsa UKF zinciri TF'i
        #   uygular ve poz 1.44 m SICRAR - o modda bu harita GECERSIZ.
        #
        # Koordinatlar ENU-MUTLAK, orijin = Start (40.7901429, 29.5092136),
        # x = Dogu, y = Kuzey [m]. (enu_to_local() ile yerel cerceveye tasinir.)
        self.parking_slots = {
            1: {'x': -5.815565, 'y': 26.791973, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # OLCULDU
            2: {'x': -4.054538, 'y': 24.876439, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # ara-deger
            3: {'x': -2.293510, 'y': 22.960906, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # OLCULDU
            4: {'x': -0.672327, 'y': 21.061196, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # ara-deger
            5: {'x':  0.948856, 'y': 19.161487, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # OLCULDU
            6: {'x':  2.584474, 'y': 17.219297, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # ara-deger
            7: {'x':  4.220092, 'y': 15.277107, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # OLCULDU
            8: {'x':  6.194659, 'y': 13.453145, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # ara-deger
            9: {'x':  8.169226, 'y': 11.629182, 'status': 'unknown', 'occupied': False, 'permit': 'unknown'},  # OLCULDU
        }

        # --- REFERANS KAYDIRMA (saha basinda olculur, rebuild GEREKMEZ) ------
        # GPS mutlak konumu ~0.4 m (1 sigma, status=0 RTK YOK) belirsiz, ama
        # haritanin SEKLI cm mertebesinde dogru. Bu yuzden harita KATI tutulup
        # tek bir bilinen fiziksel noktadan olculen farkla otelenir.
        # Kullanim:  ros2 launch ... slot_offset_x:=0.62 slot_offset_y:=-0.41
        # Olcum: araci cep1'e (boyali cizgiler kesin referans) park et, 20 s
        # GPS oku, fark = (olculen) - (yukaridaki cep1 koordinati).
        self.declare_parameter('slot_offset_x', 0.0)
        self.declare_parameter('slot_offset_y', 0.0)
        _ox = float(self.get_parameter('slot_offset_x').value)
        _oy = float(self.get_parameter('slot_offset_y').value)
        if _ox or _oy:
            for _s in self.parking_slots.values():
                _s['x'] += _ox
                _s['y'] += _oy
            self.get_logger().info(
                f'Cep haritasi referans kaydirmasi uygulandi: '
                f'({_ox:+.3f}, {_oy:+.3f}) m')

        # Cep sirasinin ENU acisi (cep1 -> cep9 yonu). ROI'nin DONDURULMESI
        # icin sart: cepler eksene hizali DEGIL, -48 derecelik bir sirada.
        # Cep ekseni (park yonu, siraya dik) = -47.771 + 90 = +42.23 deg.
        # Capraz kontrol: ceplerde park edilmis aracin quaternion yaw
        # ortalamasi +41.69 deg (ham) - yaw_offset +3.8 = +37.9 deg; sira
        # dikligiyle 4.6 deg fark, ki bu suruculerin park acisi sacilmasi
        # (cep1 +63 deg gibi kotu bir park dahil) icinde kaliyor.
        # 2026-07-28 REVIZE: -47.968 -> -47.771 (WGS84 elipsoit ENU ile).
        self.declare_parameter('slot_row_angle_deg', -47.771)
        self.slot_row_angle = math.radians(
            self.get_parameter('slot_row_angle_deg').value)

        # --- ROI sinirlari (cep merkezine gore, cep eksenine DONDURULMUS) ----
        # 2026-07-28: serit metre olcumune gore boyutlandirildi.
        #   cep derinligi 4.94-5.12 m (ort. 5.00) -> yari boy 2.50
        #   cep genisligi 2.44-2.48 m (ort. 2.47) -> yari boy 1.235
        # ESKI degerler (1.0 / 0.55) cep alaninin ancak %36'sini tariyordu ve
        # ustelik eksene HIZALI idi; cepler -48 derece egik oldugu icin
        # dondurulmemis bir dikdortgen komsu cebe tasip yanlis DOLU verirdi.
        #
        # !! 2026-07-28 KRITIK: half_x 2.10 -> 1.50 (DUVAR VAR) !!
        # lidar_gozlem_0728_1022 kus bakisi haritasindan: cep merkezlerinin
        # 3.0-3.5 m ARKASINDA surekli bir bariyer/duvar uzaniyor. ROI derinligi
        # tarandiginda (gercek kayit, yer gercegi cep7+cep9 dolu):
        #     half_x=1.50 -> yanlis-DOLU yok
        #     half_x=2.10 -> yanlis-DOLU yok  ama duvara sadece 0.9 m var
        #     half_x=3.50 -> [1,2,3,4,5,6,8] HEPSI yanlis DOLU (duvar ROI'de)
        # Poz biraz kayinca 2.10 duvari yakaliyor. Yaw hatasi taramasinda:
        #     half_x=2.10, dyaw=+3.0 deg -> cep1 yanlis DOLU
        #     half_x=1.50, dyaw=+4.0 deg -> yanlis DOLU YOK
        # Gercek tespitte kayip yok: cep7 27.7 -> 27.7, cep9 61.4 -> 56.9.
        self.declare_parameter('roi_half_x', 1.50)   # cep derinligi yari boyu [m]
        self.declare_parameter('roi_half_y', 1.05)   # cep genisligi yari boyu [m]
        # z sinirlari ZEMINE GORE (bkz. ground_z_in_lidar asagida).
        # 0.15: bordur/duba tabani ustu (asfalt yansimasi elenir)
        # 2.00: arac tavani + tabela ustu
        self.declare_parameter('roi_z_min', 0.15)    # zeminden yukseklik [m]
        self.declare_parameter('roi_z_max', 2.00)
        # Frame Biriktirme (Temporal Aggregation) esikleri.
        # Dur-Tara mimarisi sayesinde arac sabitken LiDAR frame'leri
        # havuzlanir; karar TEK frame'e degil kare basina ORTALAMAYA dayanir.
        # min_avg_points 1.5 -> 8.0 (2026-07-28). ESKI 1.5 degeri ESKI, KUCUK
        #   ve YANLIS YUKSEKLIKTEKI ROI icin secilmisti (2.2 m^2 taban x 0.9 m
        #   bant). Yeni ROI 8.8 m^2 x 1.85 m = yaklasik 8 KAT hacim; ayni esik
        #   birakilirsa tek bir sapkin yansima cebi DOLU ilan eder ve 'DOLU
        #   kilidi' bunu bir daha GERI ALMAZ (kalici yanlis pozitif).
        #
        #   OLCUM (lidar_gozlem_0728_1022 + lidar_cepte_gozlem_0728_1029,
        #   2 x 20 tarama): zeminden 0.15-2.00 m bandinda, 2.10 x 1.05 m
        #   kutu sahne boyunca 5-18 m'de gezdirildi ->
        #       BOS kutu     : medyan 0 nokta/kare (kutularin %57-76'si TAM BOS)
        #       DOLU kutu    : 18 m'de ~24-64, 11 m'de ~100, 5 m'de ~1000+
        #   Zemin filtresi calistigi icin bos/dolu ayrimi cok keskin. 8.0
        #   esigi gurultunun (0-2) ~4 kati ustunde, en zayif gercek tespitin
        #   (~24) ~3 kati altinda.
        #   NOT: bu olcum GERCEK cep konumlarinda degil, sahnede genel tarama
        #   ile yapildi - LiDAR bag'lerinde GPS/poz kaydedilmedigi icin cepler
        #   dunya cercevesine oturtulamadi. Ilk saha kosusunda
        #   /parking/slot_status log'undan avg_points degerleri okunup esik
        #   teyit edilmeli.
        # min_pool_frames = 5: karar icin gereken minimum havuz derinligi;
        #   tek karelik yanilgiyla DOLU/BOS ilan edilmez.
        # YOLO tabela izni SART mi? (bkz. slot_available). Yaris kosusunda
        # TRUE; tabelasiz/kamerasiz denemede launch argumaniyla false yapilir.
        self.declare_parameter('require_sign_permit', True)
        self.declare_parameter('min_avg_points', 8.0)
        self.declare_parameter('min_pool_frames', 5)
        self.roi_half_x = self.get_parameter('roi_half_x').value
        self.roi_half_y = self.get_parameter('roi_half_y').value
        self.roi_z_min = self.get_parameter('roi_z_min').value
        self.roi_z_max = self.get_parameter('roi_z_max').value
        self.require_sign_permit = self.get_parameter('require_sign_permit').value
        self.min_avg_points = self.get_parameter('min_avg_points').value
        self.min_pool_frames = self.get_parameter('min_pool_frames').value

        # --- Sensor montaji --------------------------------------------------
        # sensor_dx: base_link->velodyne statik TF'inden (ekipce olculmus).
        self.sensor_dx = -0.177  # [m] base_link -> velodyne x ofseti

        # 2026-07-28 OLCUM - LiDAR YERDEN YUKSEKLIGI:
        # lidar_gozlem_0728_1022 ve lidar_cepte_gozlem_0728_1029 bag'lerinde
        # 20'ser tarama (~215.000 nokta) birlestirilip 20 m yaricapta zemine
        # en kucuk kareler duzlemi oturtuldu:
        #     bag 1: z = +0.0072x -0.0038y -1.366   (artik sd 0.037 m)
        #     bag 2: z = -0.0007x +0.0126y -1.365   (artik sd 0.075 m)
        # Iki bagimsiz olcum 1 mm icinde ayni: LiDAR zeminden 1.366 m yukarida.
        #
        # ESKI sensor_dz = 0.62 -> 0.75 m HATALI IDI. Sonucu: roi_z 0.1..1.0
        # bandi gercekte YERDEN 0.85..1.75 m'yi tariyordu; bir aracin govdesinin
        # ancak ust kismini goruyor, duba/bordur/alcak engelleri TAMAMEN
        # kaciriyordu. Yukseklik dagilimi olcumu, engellerin asil yogunlastigi
        # bandin 0.40-1.40 m oldugunu gosterdi.
        self.declare_parameter('ground_z_in_lidar', -1.366)  # zeminin ham z'si
        self.sensor_dz = -self.get_parameter('ground_z_in_lidar').value  # 1.366

        # --- Kamera modeli ----------------------------------------------------
        # cam_dx: base_link->zed2_left_camera_frame statik TF'inden (gercek).
        # cam_focal_px/cam_image_width HALA GAZEBO DEGERI (HFOV 1.91986 rad,
        # 1920px varsayimi) - gercek ZED2 VGA cozunurlukte (can_launch.xml:
        # grab_resolution=VGA) ve KD Doc'taki gercek HFOV (max 110° H) ile
        # YENIDEN HESAPLANMALI. Su an tabela-cep acisal eslestirmesi (YOLO)
        # icin YANLIS OLABILIR - saha testinde kontrol edin.
        self.declare_parameter('cam_focal_px', 672.2)
        self.declare_parameter('cam_image_width', 1920.0)
        self.declare_parameter('cam_dx', -0.205)         # base_link -> kamera [m]
        self.cam_focal_px = self.get_parameter('cam_focal_px').value
        self.cam_image_width = self.get_parameter('cam_image_width').value
        self.cam_dx = self.get_parameter('cam_dx').value

        # --- Tabela -> cep eslestirme esikleri (ACISAL) -----------------------
        # ===================================================================
        # MUHURLENDI (parking_custom1.world, saha dogrulamasi):
        #   Yeni harita duzeninde tabelalar 0.1-1.4 deg acisal sapmayla ve
        #   %76-90 kalite skoruyla dogru ceplere eslesti. Asagidaki iki deger
        #   (sign_row_offset_x = -1.10, sign_assoc_max_deg = 10.0) bu basarili
        #   duruma AITTIR ve DEGISTIRILMEMELIDIR. Harita/tabela/cep konumlari
        #   yeniden kaydirilmadikca bu degerlere dokunma; algi ve oylama
        #   mantigi da bu esiklere gore dogrulanmistir.
        # ===================================================================
        # sign_min/max_range: mesafe SADECE bu kaba kapida kullanilir; sinif
        #   bazli olcek hatasi tasidigi icin konumlandirmada KULLANILMAZ.
        # sign_row_offset_x: tabela sirasinin cep merkezlerine gore x ofseti
        #   (parking_custom1.world: tabela x ~ -25.1, cep x ~ -24.0 -> ~ -1.10 m;
        #   olculen gercek fark cep-cep ort. ~ -1.06 m, 10 deg kapinin cok
        #   icinde). Beklenen aci tabela konumuna hesaplanir, cep merkezine degil.
        # sign_assoc_max_deg: olculen ile beklenen aci arasi ust sinir [deg].
        #   10 deg: wp3 saha testinde 6 deg dar perspektifte gecerli
        #   okumalari eliyordu; komsu sizinti riskini artik esas olarak
        #   1'e-1 ortak atama + oylama + kalite kapilari tasidigi icin
        #   mutlak sinir gevsetildi. Guncel harita 0.1-1.4 deg sapmayla
        #   bu kapinin cok altinda kaldigi icin marj boldur.
        # min_angle_diff: belirsizlik MARJI [deg]. En iyi ve ikinci en iyi
        #   adayin aci HATALARI arasindaki fark (second_diff - best_diff) bu
        #   degerden kucukse okuma belirsizdir ve atlanir. wp3 dar acisinda
        #   1.5 deg bile gecerli okumalari eledigi icin 0.2 deg'e cekildi:
        #   ayrim gucunu artik 1'e-1 ortak atama sagliyor, bu kapi yalnizca
        #   fiilen ayirt edilemez (esdeger acili) okumalari suzuyor.
        self.declare_parameter('sign_min_range', 1.0)
        self.declare_parameter('sign_max_range', 15.0)
        self.declare_parameter('sign_row_offset_x', -1.10)
        self.declare_parameter('sign_assoc_max_deg', 10.0)
        self.declare_parameter('min_angle_diff', 0.2)
        self.sign_min_range = self.get_parameter('sign_min_range').value
        self.sign_max_range = self.get_parameter('sign_max_range').value
        self.sign_row_offset_x = self.get_parameter('sign_row_offset_x').value
        self.sign_assoc_max_rad = math.radians(
            self.get_parameter('sign_assoc_max_deg').value)
        self.min_angle_diff_rad = math.radians(
            self.get_parameter('min_angle_diff').value)

        # --- Tabela Zamansal Havuzlama (Temporal Aggregation) ----------------
        # LiDAR doluluk havuzunun tabela karsiligi. Tek karelik YOLO yanlis
        # siniflandirmasi (halusinasyon) veya dusuk guvenli sallama matrisi
        # bozamaz:
        #   min_sign_conf: bu esigin altindaki tespitler oylamaya ALINMAZ
        #     (0.60 varsayilan; daha temkinli icin 0.65 verilebilir).
        #   min_sign_votes: bir cebe permit ATANMADAN once o durakta
        #     toplanmasi gereken minimum gecerli oy; tek okumayla karar
        #     verilmez (LiDAR'daki min_pool_frames ile ayni felsefe).
        #   sign_overwrite_margin: commit edilmis bir permitin uzerine
        #     yazabilmek icin yeni oy havuzunun eski karardan en az bu kadar
        #     yuksek kaliteli olmasi gerekir (histerezis; gurultülu esdeger
        #     okuma saglam karari sarsmasin). Kalite [0..~1] araligindadir.
        self.declare_parameter('min_sign_conf', 0.45)
        self.declare_parameter('min_sign_votes', 3)
        self.declare_parameter('sign_overwrite_margin', 0.05)
        self.min_sign_conf = self.get_parameter('min_sign_conf').value
        self.min_sign_votes = self.get_parameter('min_sign_votes').value
        self.sign_overwrite_margin = self.get_parameter('sign_overwrite_margin').value

        # --- Gozlemlenebilirlik sinirlari ------------------------------------
        # LiDAR: yatay FOV +/-90 deg, menzil 0.9-100 m (world dosyasindan).
        # FOV kenarlari ve asiri uzak mesafe guvenilmez; biraz iceriden kirp.
        self.fov_half = math.radians(80.0)
        self.obs_min_range = 1.2
        self.obs_max_range = 30.0

        # --- Ic durum ---------------------------------------------------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Algi Kilidi: kontrol dugumu sinyal gonderene kadar algi ACIK kalir
        # (baslangic taramasi kontrolcu baslamadan da calisabilsin diye).
        self.perception_active = True

        # Gorev Kapisi: master /park_gorev='start' demeden planner pasiftir
        # (10 Hz LiDAR/oylama dongusu calismaz, /parking/slot_status yayinlanmaz).
        self.mission_active = False

        # Frame Biriktirme havuzu (Dur-Tara: arac sabitken doldurulur).
        # Cep bazli toplam ROI nokta sayisi ve cebin gozlemlendigi kare
        # sayisi ayri tutulur; ortalama = toplam / gozlem_karesi. Havuz her
        # algi kilidi degisiminde sifirlanir (yeni durus = temiz havuz).
        self.lidar_accumulated_points = {sid: 0 for sid in self.parking_slots}
        self.slot_observed_frames = {sid: 0 for sid in self.parking_slots}
        self.aggregation_frame_count = 0

        # Tabela Oylama Havuzu (Temporal Aggregation, LiDAR havuzuyla senkron).
        # Her durakta her cep icin permit siniflarinin oylari sayilir; en cok
        # oyu alan sinif (min_sign_votes toplandiktan sonra) permite islenir.
        # Havuzun KALITE izleri SINIF BAZLI tutulur (overwrite karari icin):
        #   sign_vote_best_diff[sid][permit]: o sinifin en NET acisi [rad]
        #   sign_vote_best_conf[sid][permit]: o sinifin en yuksek guveni
        # Cep bazli tek iz tutulsaydi azinlikta kalan sinifin yuksek guveni
        # KAZANAN sinifin kalitesini sisirirdi (kalite hirsizligi): 0.91'lik
        # park_yeri tespiti, 2 oyluk park_yasak kararina kalite kazandirirdi.
        # sign_vote_classes: ham YOLO sinif adi sayaci (saf teshis/log icin;
        # 1E/2Y gibi bir havuzun oylarinin HANGI siniflardan geldigini gosterir).
        # sign_committed_this_stop: bu duraktaki havuz permite yazdi mi?
        # True ise ayni havuz buyudukce hukmu SERBESTCE guncellenir (durak ici
        # cogunluk degisimi overwrite kapisina TAKILMAZ; kapi duraklar ARASI
        # calisir). Havuzlar algi kilidi her degistiginde birlikte sifirlanir.
        self.sign_votes = {sid: {'park_edilebilir': 0, 'park_yasak': 0}
                           for sid in self.parking_slots}
        self.sign_vote_best_diff = {
            sid: {'park_edilebilir': float('inf'), 'park_yasak': float('inf')}
            for sid in self.parking_slots}
        self.sign_vote_best_conf = {
            sid: {'park_edilebilir': 0.0, 'park_yasak': 0.0}
            for sid in self.parking_slots}
        self.sign_vote_classes = {sid: {} for sid in self.parking_slots}
        self.sign_committed_this_stop = {sid: False
                                         for sid in self.parking_slots}

        # permit_quality: permit'in commit edildigi oy havuzunun kalite skoru
        # (yuksek = daha net aci + yuksek guven). Yeni bir durak ancak bundan
        # sign_overwrite_margin kadar daha yuksek kaliteyle karari EZEBILIR.
        # 'unknown' cepte -inf: ilk commit her zaman serbest. permit degeri
        # durak gecisinde silinmez (gecmis tecrube korunur).
        for slot in self.parking_slots.values():
            slot['permit_quality'] = float('-inf')

        # --- ENU -> yerel /odom donusumu (bkz. yukaridaki DUZELTME notu) -----
        # Baslangicta HAM ENU degerlerine DUSER (donusum hic yakalanamazsa
        # bile calismaya devam eder, ama o zaman eski/hatali davranista kalir
        # - bu yuzden capture olunca UYARI DEGIL BILGI, capture olmazsa
        # ayrica UYARI loglanir, bkz. maybe_capture_enu_transform).
        self.slot_xy_local = {sid: (slot['x'], slot['y'])
                              for sid, slot in self.parking_slots.items()}
        self.map_x = self.map_y = self.map_yaw = 0.0
        self.map_pose_received = False
        self.gps_zone_reached = False
        self.enu_transform_captured = False
        self.yaw_offset = 0.0
        # Donusum yakalanana kadar ROI ham ENU acisinda durur (slot_xy_local
        # de ham ENU'ya dustugu icin ikisi tutarli kalir).
        self.slot_row_angle_local = self.slot_row_angle
        self._local_x0 = self._local_y0 = 0.0
        self._map_x0 = self._map_y0 = 0.0

        # --- Abonelikler ve yayincilar ----------------------------------------
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # GPS-duzeltmeli MUTLAK poz (robotaxi_localization/ukf_global) -
        # SADECE yaw_offset'i BIR KEZ yakalamak icin kullanilir, surekli
        # kontrol icin DEGIL (bkz. yukaridaki DUZELTME notu).
        self.map_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered_map', self.map_odom_callback, 10)
        self.zone_sub = self.create_subscription(
            Bool, '/parking_zone_reached', self.zone_callback, 10)
        self.yolo_detections_sub = self.create_subscription(
            String, '/yolo_detections', self.yolo_detections_callback, 10)
        self.perception_sub = self.create_subscription(
            Bool, '/perception_active', self.perception_callback, 10)
        # master_kararci Gorev Kapisi (start/stop)
        self.park_gorev_sub = self.create_subscription(
            String, '/park_gorev', self.park_gorev_callback, 10)
        self.status_pub = self.create_publisher(String, '/parking/slot_status', 10)
        self.slot_markers_pub = self.create_publisher(
            MarkerArray, '/parking/slot_markers', 10)

        # Durum matrisi 10 Hz yayinlanir; log 1 Hz'e kisilir (spam onleme)
        self.status_timer = self.create_timer(0.1, self.status_loop)

        # --- YOLO canlilik izleme -------------------------------------------
        # Amac: 'kamera/YOLO OLU' ile 'YOLO calisiyor ama tabela gormuyor'
        # ayrimini yapabilmek. Ikisi de ceplerin permit='unknown' kalmasina yol
        # acar ama sebepleri ve mudahalesi TAMAMEN farklidir; log'da ayirt
        # edilemezse sahada bos yere durak taranarak dakikalar kaybedilir.
        # Sayac yolo_detections_callback'in EN BASINDA, gorev/algi kapilarindan
        # ONCE artar - kapilar kapaliyken de topic'in aktigini gorebilmek icin.
        # NOT: yolo_detector_node tespit yokken bos dizi [] yayinlar, yani
        # mesaj akisi 'tabela var mi' sorusundan bagimsizdir.
        self.yolo_msg_count = 0
        self.last_yolo_msg_time = None
        self.yolo_health_timer = self.create_timer(1.0, self.yolo_health_check)

        self.get_logger().info(
            'Parking Mission Planner baslatildi. '
            f'{len(self.parking_slots)} cep izleniyor | '
            f'ROI: x±{self.roi_half_x} y±{self.roi_half_y} '
            f'z[{self.roi_z_min}, {self.roi_z_max}]')

    # ------------------------------------------------------------------- Odom
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        # Quaternion -> yaw (tf_transformations kurulu degil, dogrudan formul)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True
        self.maybe_capture_enu_transform()

    def map_odom_callback(self, msg):
        """GPS-duzeltmeli MUTLAK poz (/odometry/filtered_map, ukf_global).

        SADECE yaw_offset'i bir kez yakalamak icin kullanilir; surekli
        kontrolde HALA /odom (/odometry/filtered, puruzsuz) kullanilir.
        """
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.map_pose_received = True
        self.maybe_capture_enu_transform()

    def zone_callback(self, msg):
        """parking_gps_zone_trigger'dan GPS geofence durumu."""
        self.gps_zone_reached = bool(msg.data)
        self.maybe_capture_enu_transform()

    def maybe_capture_enu_transform(self):
        """ENU(Dogu/Kuzey) <-> yerel /odom (/odometry/filtered) donusumunu
        park bolgesine girildigi an BIR KEZ yakala.

        NEDEN: /odometry/filtered'in yaw'i IMU'nun MUTLAK yoneliminden degil
        sadece acisal hizdan (vyaw) geliyor (bkz. __init__ DUZELTME notu) -
        yani yaw=0 aracin ACILIS yonudur, Dogu degil. /odometry/filtered_map
        ise GPS+IMU+manyetik-sapma ile duzeltilmis, ENU'ya yakin mutlak poz
        veriyor. Ikisi arasindaki SABIT aci farkini (yaw_offset) bir kez
        olcup, ceplerin ENU koordinatlarini /odometry/filtered'in yerel
        cercevesine donusturmek icin kullaniyoruz.
        """
        if self.enu_transform_captured:
            return
        if not (self.gps_zone_reached and self.odom_received and
                self.map_pose_received):
            return

        self._local_x0 = self.current_x
        self._local_y0 = self.current_y
        self._map_x0 = self.map_x
        self._map_y0 = self.map_y
        self.yaw_offset = math.atan2(
            math.sin(self.map_yaw - self.current_yaw),
            math.cos(self.map_yaw - self.current_yaw))
        self.enu_transform_captured = True

        for sid, slot in self.parking_slots.items():
            self.slot_xy_local[sid] = self.enu_to_local(slot['x'], slot['y'])

        # Cep sirasi acisi da AYNI donusumle tasinmali - enu_to_local()
        # noktalari -yaw_offset kadar donduruyor (bkz. asagisi), dolayisiyla
        # ROI'nin yonu de -yaw_offset kadar doner. Bu yapilmazsa dondurulmus
        # ROI yerel cercevede YANLIS ACIDA durur ve komsu cebi tarar.
        self.slot_row_angle_local = self.slot_row_angle - self.yaw_offset

        self.get_logger().info(
            'ENU->yerel /odom donusumu YAKALANDI: '
            f'yaw_offset={math.degrees(self.yaw_offset):.1f} deg '
            f'(map_yaw={math.degrees(self.map_yaw):.1f} deg, '
            f'local_yaw={math.degrees(self.current_yaw):.1f} deg). '
            'Cep koordinatlari yerel cerceveye donusturuldu; '
            f'cep sirasi acisi {math.degrees(self.slot_row_angle):.1f} -> '
            f'{math.degrees(self.slot_row_angle_local):.1f} deg (yerel).')

    def enu_to_local(self, px_enu, py_enu):
        """Bir ENU (Dogu,Kuzey) noktasini yakalanan donusumle yerel
        /odometry/filtered cercevesine cevirir. Donusum yakalanmadiysa
        HAM ENU degerini dondurur (eski/hatali ama sessizce cokmez)."""
        if not self.enu_transform_captured:
            return px_enu, py_enu
        dx = px_enu - self._map_x0
        dy = py_enu - self._map_y0
        c = math.cos(self.yaw_offset)
        s = math.sin(self.yaw_offset)
        lx = self._local_x0 + dx * c + dy * s
        ly = self._local_y0 - dx * s + dy * c
        return lx, ly

    # ------------------------------------------------------------ Gorev Kapisi
    def park_gorev_callback(self, msg):
        """master_kararci Gorev Kapisi (start/stop). Pasifken planner hicbir
        sey yayinlamaz ve LiDAR/YOLO oylamasini islemez."""
        cmd = str(msg.data).strip().lower()
        if cmd == 'start' and not self.mission_active:
            self.mission_active = True
            self.reset_perception_pools()
            self.get_logger().info('PARK GOREVI: planner aktif (/park_gorev start)')
        elif cmd == 'stop' and self.mission_active:
            self.mission_active = False
            self.get_logger().info('PARK GOREVI: planner pasif (/park_gorev stop)')

    # ------------------------------------------------------------ Algi Kilidi
    def perception_callback(self, msg):
        """Kontrol dugumunden gelen Algi Kilidi sinyali (Stop-and-Stare).

        False -> YOLO/LiDAR guncellemeleri islenmez, matris dondurulur.
        Sinyal 20 Hz geldigi icin log sadece durum DEGISIMINDE yazilir.
        """
        if msg.data != self.perception_active:
            if msg.data:
                self.get_logger().info(
                    'ALGI ACILDI: arac hareketsiz, cep guncellemeleri isleniyor')
            else:
                self.get_logger().info(
                    'ALGI KILITLENDI: arac hareket halinde; son stabil matris '
                    'yayinlanmaya devam ediyor, yeni okuma islenmiyor')
            # Yeni durus noktasi = temiz havuz + temiz oylama: eski konumdan
            # biriken LiDAR sayimlari ve tabela oylari yeni perspektifle
            # karistirilmaz
            self.reset_perception_pools()
        self.perception_active = msg.data

    def yolo_health_check(self):
        """1 Hz: /yolo_detections akmiyorsa SAHADA HEMEN gorunur sekilde uyar.

        Gorev aktifken YOLO susarsa cepler sonsuza kadar permit='unknown'
        kalir. Son care mekanizmasi (slot_available_fallback) bunu kurtarir
        ama ancak tum duraklar tukendikten sonra - yani ~1 dakika sonra.
        Bu uyari, o dakikayi beklemeden kamera/YOLO'nun olu oldugunu
        soyler ki mudahale edilebilsin.
        """
        if not self.mission_active:
            return
        if self.last_yolo_msg_time is None:
            self.get_logger().error(
                '/yolo_detections HIC gelmedi - yolo_detector_node calisiyor '
                'mu, kamera acik mi? Cepler permit=unknown kalacak; park '
                'ancak SON CARE olcutuyle mumkun olur.',
                throttle_duration_sec=5.0)
            return
        age = (self.get_clock().now() - self.last_yolo_msg_time).nanoseconds * 1e-9
        if age > 3.0:
            self.get_logger().error(
                f'/yolo_detections {age:.1f} s sessiz (toplam '
                f'{self.yolo_msg_count} mesaj alinmisti) - YOLO durmus olabilir.',
                throttle_duration_sec=5.0)

    def reset_perception_pools(self):
        """Frame Biriktirme havuzunu ve tabela oy havuzunu TAMAMEN sifirla.

        Her algi-kilidi gecisinde (durus<->hareket) cagirilir. TUM cepler
        icin (kilitli/kilitsiz ayrimi YOK) LiDAR sayimlari ve tabela oylari
        + kalite izleri sifirlanir. Iki nedenle:
          - YOL GURULTUSU: hareket halinde (ALGI KILITLENDI) birikebilecek
            hayalet oylar yeni durusu zehirlemesin.
          - OVERWRITE: her durus sifir havuzla taze oy toplasin ki daha net
            bir durak, eski (kirli) karari kaliteye gore ezebilsin.
        permit degeri ve permit_quality KORUNUR (gecmis tecrube); karar
        yalnizca yeni oylarin kalitesi eskisini asarsa commit_sign_votes'ta
        guncellenir.
        """
        for sid in self.parking_slots:
            self.lidar_accumulated_points[sid] = 0
            self.slot_observed_frames[sid] = 0
            self.sign_votes[sid] = {'park_edilebilir': 0, 'park_yasak': 0}
            self.sign_vote_best_diff[sid] = {'park_edilebilir': float('inf'),
                                             'park_yasak': float('inf')}
            self.sign_vote_best_conf[sid] = {'park_edilebilir': 0.0,
                                             'park_yasak': 0.0}
            self.sign_vote_classes[sid] = {}
            self.sign_committed_this_stop[sid] = False
        self.aggregation_frame_count = 0

    # ------------------------------------------------------------------ LiDAR
    def lidar_callback(self, msg):
        # Gorev Kapisi: park gorevi aktif degilse hic isleme
        if not self.mission_active:
            return
        # Algi Kilidi: arac hareket halindeyken okuma islenmez
        if not self.perception_active:
            return
        # Pozu bilmeden noktalar dunya cercevesine tasinamaz
        if not self.odom_received:
            self.get_logger().warn('Odom bekleniyor, LiDAR verisi atlandi.',
                                   throttle_duration_sec=2.0)
            return

        # Sensor cercevesindeki noktalar (N, 3) numpy dizisi olarak okunur.
        # NOT: pc2.read_points_numpy() Foxy'de YOK (Humble ile geldi). Foxy'de de
        # calisan read_points() generator'i (N,3) float32 diziye cevrilir.
        pts = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            return
        points = np.array(pts, dtype=np.float32)
        if points.size == 0:
            return

        # Sensor -> chassis (sadece oteleme) -> dunya (yaw dondurme + oteleme).
        # Roll/pitch ihmal edilir (duz otopark zemini).
        px = points[:, 0] + self.sensor_dx
        py = points[:, 1]
        pz = points[:, 2] + self.sensor_dz

        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        wx = self.current_x + cos_y * px - sin_y * py
        wy = self.current_y + sin_y * px + cos_y * py
        wz = self.current_z + pz

        # z filtresi tum cepler icin ortak, bir kez uygulanir
        z_ok = (wz >= self.roi_z_min) & (wz <= self.roi_z_max)
        wx, wy = wx[z_ok], wy[z_ok]

        # Frame Biriktirme: anlik sayimlar dogrudan karar URETMEZ; havuza
        # eklenir. Karar, yeterli kare biriktikten sonra kare basina dusen
        # ORTALAMA nokta sayisina gore verilir (tek karelik parlama/eksik
        # okuma karari degistiremez).
        self.aggregation_frame_count += 1

        for slot_id, slot in self.parking_slots.items():
            slot_x, slot_y = self.slot_xy_local[slot_id]
            if not self.slot_observable(slot_x, slot_y):
                continue  # gorus alaninda degil: mevcut durum korunur

            # DONDURULMUS ROI (2026-07-28). Cepler ENU'da -47.97 derecelik bir
            # sirada; eksene hizali dikdortgen kullanmak iki hata birden yapar:
            #   (a) cep alaninin buyuk kismi ROI disinda kalir (kacan DOLU),
            #   (b) ROI komsu cebe tasar (yanlis DOLU).
            # Noktalar cep yerel eksenine (u = sira boyu, v = cep derinligi)
            # izdusurulur; boylece ROI cebin kendi acisinda durur.
            dx = wx - slot_x
            dy = wy - slot_y
            c_r = math.cos(self.slot_row_angle_local)
            s_r = math.sin(self.slot_row_angle_local)
            du = dx * c_r + dy * s_r        # sira yonunde  -> cep GENISLIGI
            dv = -dx * s_r + dy * c_r       # siraya dik    -> cep DERINLIGI
            in_roi = ((np.abs(dv) <= self.roi_half_x) &
                      (np.abs(du) <= self.roi_half_y))
            self.lidar_accumulated_points[slot_id] += int(np.count_nonzero(in_roi))
            self.slot_observed_frames[slot_id] += 1

            # Karar ani: havuz yeterince derinlesince ortalama esikle kiyasla
            frames = self.slot_observed_frames[slot_id]
            if frames < self.min_pool_frames:
                continue
            avg_points = self.lidar_accumulated_points[slot_id] / frames
            occupied = avg_points >= self.min_avg_points

            # DOLU Kilidi (nesne surekliligi / object permanence):
            # Otopark statik oldugu icin bir kez DOLU tespit edilen cep
            # ASLA BOS'a dondurulmez. Farkli duraktan bakista gorus hatti
            # baska bir engelle kesilirse (occlusion) ROI'ye az nokta
            # duser ve dusuk ortalama 'bos' izlenimi verir; oysa
            # slot_observable() isin kesilmesini bilemez. Ters yon
            # (BOS -> DOLU) aciktir: yeni tespit guvenlik kazancidir.
            if slot['occupied'] and not occupied:
                self.get_logger().warn(
                    f'Cep {slot_id}: dusuk sayim (ort. {avg_points:.1f} < '
                    f'{self.min_avg_points}) YOKSAYILDI - DOLU kilidi aktif '
                    '(occlusion suphesi, statik engel unutulmaz)',
                    throttle_duration_sec=5.0)
                continue

            if occupied != slot['occupied'] or slot['status'] == 'unknown':
                self.get_logger().info(
                    f"Cep {slot_id}: {'DOLU' if occupied else 'BOS'} "
                    f"(ort. {avg_points:.1f} nokta/kare, {frames} kare)")
            slot['occupied'] = occupied
            slot['status'] = 'dolu' if occupied else 'bos'

    def slot_observable(self, slot_x, slot_y):
        """Cep su an LiDAR'in gorus alaninda mi?

        FOV sadece on +/-90 deg oldugu icin gozlemlenmeyen cebe
        'bos' demek yanlis olur; bu kontrol onu engeller.
        slot_x/slot_y YEREL /odom cercevesinde olmali (self.slot_xy_local).
        """
        dx = slot_x - self.current_x
        dy = slot_y - self.current_y
        rng = math.hypot(dx, dy)
        if not (self.obs_min_range < rng < self.obs_max_range):
            return False
        bearing = math.atan2(dy, dx) - self.current_yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return abs(bearing) < self.fov_half

    # ------------------------------------------------------------------- YOLO
    def yolo_detections_callback(self, msg):
        """/yolo_detections: frame'deki TUM tespitler (JSON dizisi).

        Her eleman: {"class": str, "conf": float, "distance": float|null,
                     "cx": float (0-1), "cy": float (0-1)}
        Park ile ilgili her tespit ayri ayri cebe eslenir; boylece arac
        durus noktasindan birden fazla cebin tabelasi tek frame'de islenir.
        """
        # Canlilik sayaci: TUM kapilardan ONCE. Bu satirin amaci mesaji
        # islemek degil, 'YOLO dugumu hayatta mi' sorusunu cevaplamak.
        self.yolo_msg_count += 1
        self.last_yolo_msg_time = self.get_clock().now()

        # Gorev Kapisi: park gorevi aktif degilse hic isleme
        if not self.mission_active:
            return
        # Algi Kilidi: arac hareket halindeyken okuma islenmez.
        # Tasarim geregi (spam olmasin diye warn degil, seyrek info)
        if not self.perception_active:
            self.get_logger().info(
                'YOLO tespitleri islenmiyor: ALGI KILITLI (arac hareketli)',
                throttle_duration_sec=10.0)
            return
        if not self.odom_received:
            self.get_logger().warn(
                'YOLO tespiti atlandi: henuz odom alinmadi',
                throttle_duration_sec=5.0)
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Gecersiz YOLO JSON atlandi: {exc}')
            return
        if not isinstance(detections, list):
            self.get_logger().warn(
                f'YOLO mesaji liste degil ({type(detections).__name__}), atlandi',
                throttle_duration_sec=5.0)
            return

        valid = []
        for det in detections:
            try:
                name = str(det['class'])
                distance = det.get('distance')
                cx_norm = float(det['cx'])
                conf = float(det.get('conf', 0.0))
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(
                    f'YOLO tespiti bozuk, atlandi: {det!r} ({exc})',
                    throttle_duration_sec=5.0)
                continue  # bozuk eleman digerlerini engellemesin
            if distance is None:
                self.get_logger().warn(
                    f'YOLO tabela ({name}) mesafesiz geldi, '
                    'konumlandirilamadigi icin atlandi',
                    throttle_duration_sec=5.0)
                continue
            # Guvenlik Filtresi (Confidence Threshold): dusuk guvenli
            # tespitler oylamaya hic alinmaz (anlik sallamalar elenir)
            if conf < self.min_sign_conf:
                self.get_logger().warn(
                    f'YOLO tabela ({name}) dusuk guven {conf:.2f} < '
                    f'{self.min_sign_conf:.2f}, yoksayildi',
                    throttle_duration_sec=2.0)
                continue
            permit = self.SIGN_TO_PERMIT.get(name)
            if permit is None:
                # park ile ilgisiz tabela (dur, yaya_gecidi, ...): beklenen
                # durum, debug seviyesinde iz birakilir
                self.get_logger().debug(
                    f'YOLO tabela ({name}) park disi, yoksayildi')
                continue
            # Kaba menzil kapisi: mesafenin tek kullanim yeri (konumlandirma
            # tamamen acisal; mesafe sinif bazli olcek hatasi tasir)
            if not (self.sign_min_range <= float(distance) <= self.sign_max_range):
                self.get_logger().warn(
                    f'YOLO tabela ({name}, {float(distance):.1f} m) menzil '
                    f'disi [{self.sign_min_range:.1f}-{self.sign_max_range:.1f} m], '
                    'atlandi', throttle_duration_sec=2.0)
                continue
            valid.append({'name': name, 'permit': permit, 'conf': conf,
                          'cx': cx_norm})

        # Frame'in TUM gecerli tespitleri ceplere ORTAK atanir (1'e-1);
        # tek tek atama yan yana tabelalarda komsu cebe kaymaya yol acar
        if valid:
            self.assign_and_vote(valid)

    def assign_and_vote(self, dets):
        """Frame'in gecerli tespitlerini ceplere ORTAK (1'e-1) ata ve oyla.

        Tek tek 'en iyi cep' secimi, ayni frame'de gorunen komsu tabelalarin
        AYNI cebe yazilmasina / birinin komsu cebi calmasina izin veriyordu
        (wp3: Cep 3 tabelasi Cep 4'e kayiyor, Cep 3 '?' kaliyordu). Ortak
        atamada her cep en fazla BIR tespit alir: global en net eslesme once
        yerlesir, sonraki tespitler KALAN cepler icinden secer. Cep 4'un
        kendi tabelasi Cep 4'u kapinca Cep 3'un tabelasi Cep 3'e oturur.

        Doluluk (LiDAR) bilgisi atamada BILEREK kullanilmaz: 'dolu ama
        izinli' cep mesru bir durumdur; dolulukla filtrelemek dolu cebin
        kendi tabelasini reddedip komsuya iterdi (matris bozulur).

        Kilitli/kilitsiz TUM cepler adaydir (overwrite karari kaliteye gore
        commit_sign_votes'ta verilir).
        """
        # Ayni fiziksel tabelanin cift kutusu (NMS kacagi) 1'e-1 atamada
        # komsu cebe TASAR; ayni sinifta cx'i cakisan tespitler once
        # teklestirilir (en yuksek guvenli kalir)
        unique = []
        for d in sorted(dets, key=lambda d: -d['conf']):
            if any(u['name'] == d['name'] and abs(u['cx'] - d['cx']) < 0.005
                   for u in unique):
                self.get_logger().warn(
                    f"YOLO tabela ({d['name']}, cx={d['cx']:.3f}, "
                    f"conf={d['conf']:.2f}) cift kutu (NMS kacagi) olarak "
                    'teklestirildi', throttle_duration_sec=5.0)
                continue
            unique.append(d)

        # Olculen aci: goruntude saga kayma = yaw'dan saat yonunde sapma.
        # Beklenen acilar kameranin dunya konumundan, cebin BEKLENEN TABELA
        # konumuna (cep merkezi + sira ofseti) hesaplanir.
        cam_x = self.current_x + self.cam_dx * math.cos(self.current_yaw)
        cam_y = self.current_y + self.cam_dx * math.sin(self.current_yaw)
        # NOT: slot_xy_local artik YEREL /odom cercevesinde (donusturulmus).
        # sign_row_offset_x, ESKI (Gazebo) koordinat sisteminin x-ekseni
        # yonune gore kalibre edilmisti; yeni yerel cercevede x-ekseni
        # aracin ACILIS yonu oldugu icin bu ofsetin fiziksel anlami
        # (tabela sirasinin hangi eksende kaydigi) SAHADA YENIDEN
        # DOGRULANMALI - ayri bir kalibrasyon konusu, bu duzeltmenin kapsami
        # sadece cep/durak konumlarinin dogru cercevede olmasi.
        expected = {
            sid: math.atan2(xy[1] - cam_y, (xy[0] + self.sign_row_offset_x) - cam_x)
            for sid, xy in self.slot_xy_local.items()}
        for d in unique:
            bearing_cam = math.atan2(
                (d['cx'] - 0.5) * self.cam_image_width, self.cam_focal_px)
            d['measured'] = self.current_yaw - bearing_cam

        def ang_diff(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

        pending = list(unique)
        free = set(self.parking_slots.keys())
        while pending and free:
            # Her bekleyen tespit icin SERBEST cepler arasinda en iyi ve
            # ikinci en iyi aday; global en net eslesme once yerlesir
            ranked = []
            for d in pending:
                diffs = sorted((ang_diff(d['measured'], expected[sid]), sid)
                               for sid in free)
                second = diffs[1][0] if len(diffs) > 1 else None
                ranked.append((diffs[0][0], diffs[0][1], second, d))
            ranked.sort(key=lambda r: r[0])
            best_diff, slot_id, second_diff, det = ranked[0]

            # Mutlak aci kapisi: global en net aday bile gecemiyorsa
            # kalanlar hic gecemez, frame biter
            if best_diff > self.sign_assoc_max_rad:
                self.get_logger().warn(
                    f"YOLO tabela ({det['name']}) eslesmedi: en yakin cep "
                    f'{slot_id}, aci farki {math.degrees(best_diff):.1f} deg '
                    f'> {math.degrees(self.sign_assoc_max_rad):.1f} deg',
                    throttle_duration_sec=2.0)
                break

            pending.remove(det)

            # Belirsizlik kapisi (MARJ, yalnizca serbest ceplere karsi):
            # ikinci aday cok yakinsa bu tespit bu frame'de islenmez
            if (second_diff is not None and
                    (second_diff - best_diff) < self.min_angle_diff_rad):
                self.get_logger().warn(
                    f"YOLO tabela ({det['name']}) belirsiz, atlandi: cep "
                    f'{slot_id} ({math.degrees(best_diff):.1f} deg) ile '
                    f'ikinci aday ({math.degrees(second_diff):.1f} deg) '
                    f'arasi marj yetersiz',
                    throttle_duration_sec=2.0)
                continue

            free.discard(slot_id)
            self._register_sign_vote(slot_id, det['name'], det['permit'],
                                     det['conf'], best_diff)

    def _register_sign_vote(self, slot_id, name, permit, conf, best_diff):
        """Eslesen tespiti oy havuzuna isle ve commit'i dene.

        Okuma dogrudan permite YAZILMAZ; cebin oy sayacina eklenir ve KENDI
        SINIFININ kalite izleri guncellenir (kazanmayan sinifin guveni
        kazanan sinifin kalitesine karisamaz). Ham sinif adi teshis
        sayacina islenir. Karar duraktaki cogunluga dayanir.
        """
        self.sign_votes[slot_id][permit] += 1
        self.sign_vote_best_diff[slot_id][permit] = min(
            self.sign_vote_best_diff[slot_id][permit], best_diff)
        self.sign_vote_best_conf[slot_id][permit] = max(
            self.sign_vote_best_conf[slot_id][permit], conf)
        self.sign_vote_classes[slot_id][name] = (
            self.sign_vote_classes[slot_id].get(name, 0) + 1)
        self.commit_sign_votes(slot_id)

    def commit_sign_votes(self, slot_id):
        """Oy havuzu yeterince dolduysa cogunluk sinifini permite isle.

        LiDAR karar aniyla ayni felsefe: yeterli oy (min_sign_votes)
        birikmeden karar verilmez. Cogunluk (majority) kazanir; BERABERLIKTE
        guvenli taraf (park_yasak) secilir.

        KALITE: kalite skoru KAZANAN SINIFIN kendi tespitlerinden hesaplanir
        (o sinifin en yuksek guveni * en net acisi). Cep bazli tek iz
        kullanilsaydi azinlik sinifin yuksek guvenli tespiti (orn. 0.91'lik
        park_yeri) cogunlugun (park_yasak) kararina kalite kazandirirdi ve
        yanlis karar dogru tespitin kalitesiyle kilitlenirdi.

        DURAK ICI / DURAKLAR ARASI ayrimi:
          - Ayni duragin havuzu permite bir kez yazdiysa
            (sign_committed_this_stop), havuz buyudukce hukmu SERBESTCE
            guncellenir: 3. oyda 1E/2Y ile yanlis baslayan karar, havuz
            10E/2Y olunca ayni durakta duzelir. (Onceden overwrite kapisi
            burada da islediginden durak ici cogunluk degisimi bloklanirdi.)
          - Farkli durak (havuz henuz yazmadi) eski karari ancak
            sign_overwrite_margin kadar daha yuksek kaliteyle ezebilir
            (histerezis: esdeger gurultu saglam karari sarsamaz).
        Ilk commit ('unknown') her zaman serbesttir.
        """
        votes = self.sign_votes[slot_id]
        total = votes['park_edilebilir'] + votes['park_yasak']
        if total < self.min_sign_votes:
            return  # yeterli tanik yok, tek/az okumayla karar verme

        # Cogunluk sinifi; beraberlik -> guvenli taraf (park_yasak)
        permit = ('park_edilebilir'
                  if votes['park_edilebilir'] > votes['park_yasak']
                  else 'park_yasak')

        slot = self.parking_slots[slot_id]

        # Kalite [0..~1]: KAZANAN sinifin en net acisi + en yuksek guveni
        clarity = max(0.0, 1.0 - self.sign_vote_best_diff[slot_id][permit] /
                      self.sign_assoc_max_rad)
        quality = self.sign_vote_best_conf[slot_id][permit] * clarity

        # Overwrite kapisi SADECE duraklar arasi: bu duragin havuzu daha
        # once yazdiysa hukum serbestce guncellenir (durak ici duzelme)
        if (not self.sign_committed_this_stop[slot_id] and
                slot['permit'] != 'unknown' and
                quality <= slot['permit_quality'] + self.sign_overwrite_margin):
            return

        if slot['permit'] != permit:
            level = self.get_logger().warn if slot['permit'] != 'unknown' \
                else self.get_logger().info
            prev = slot['permit'] if slot['permit'] != 'unknown' else '-'
            classes = ', '.join(f'{k}x{v}' for k, v in
                                sorted(self.sign_vote_classes[slot_id].items()))
            level(
                f'Cep {slot_id} tabela: {prev} -> {permit} '
                f'(oylama {votes["park_edilebilir"]}E/{votes["park_yasak"]}Y '
                f'[{classes}], kalite {quality:.2f} '
                f'[onceki {slot["permit_quality"]:.2f}], '
                f'aci {math.degrees(self.sign_vote_best_diff[slot_id][permit]):.1f} deg)')
        slot['permit'] = permit
        slot['permit_quality'] = quality
        self.sign_committed_this_stop[slot_id] = True

    # --------------------------------------------------------- Durum Matrisi
    def slot_available(self, slot):
        """Cep ancak gozlemlenmis + bos + izinli ise kullanilabilir.

        2026-07-28 NOTU (masabasi bag testinde ortaya cikti): izin sarti YOLO
        tabela tespitine bagli. Kamera/YOLO calismazsa permit TUM cepler icin
        'unknown' kalir ve '-> kullanilabilir: YOK' sonsuza kadar surer; arac
        BOS cepleri dogru gormesine ragmen HIC PARK ETMEZ. Bu sessiz degil
        (log her saniye 'YOK' yazar) ama sahada dakikalarca beklendikten sonra
        fark edilirse zaman kaybi olur.

        require_sign_permit=False SADECE tabelasiz bench/saha denemesi icin:
        cep salt LiDAR dolulugu ile kullanilabilir sayilir. YARIS KOSUSUNDA
        TRUE OLMALI - 'park_yasak' tabelasi yok sayilirsa engelli/yasak cebe
        park edilir ve puran gider.
        """
        if not (slot['status'] == 'bos' and not slot['occupied']):
            return False
        if not self.require_sign_permit:
            return True
        return slot['permit'] == 'park_edilebilir'

    def slot_available_fallback(self, slot):
        """SON CARE olcutu: izni COZULEMEMIS cebi de kabul et.

        2026-08-02 gerekcesi: strict olcut (slot_available) izni
        'park_edilebilir' OLARAK DOGRULANMIS cep ister. Kamera/YOLO olurse ya
        da tabelalar hicbir duraktan okunamazsa TUM cepler 'unknown' kalir,
        hicbir cep secilmez ve gorev PARK_IPTAL ile biter - yani hic park
        edilmez. Bu mekanizma o durumda arac tum kesif duraklarini tuketttikten
        SONRA devreye girer (bkz. decision node, son durak dali).

        KRITIK AYRIM - bu bir 'tabelayi yoksay' modu DEGILDIR:
            permit == 'park_yasak'      -> HALA REDDEDILIR. Acikca yasak
                                           gorulmus cebe fallback'te de
                                           GIRILMEZ.
            permit == 'unknown'         -> kabul edilir (hic okuma yok).
            doluluk (LiDAR) kapisi      -> aynen korunur, gevsetilmez.

        Yani sadece 'okuyamadim' durumu 'girilebilir'e cevrilir; 'yasak
        okudum' asla. Yine de bu bir RISK KABULUDUR: okunamayan cep gercekte
        yasak olabilir. Hic park etmemeye (kesin 0) karsi tercih edilmistir.
        """
        if not (slot['status'] == 'bos' and not slot['occupied']):
            return False
        return slot['permit'] != 'park_yasak'

    def status_loop(self):
        """10 Hz: durum matrisini JSON olarak yayinla, 1 Hz loglama.

        DIKKAT: 'x'/'y' burada YEREL /odom (/odometry/filtered) cercevesinde
        yayinlanir (slot_xy_local) - parking_decision_and_control_node bu
        degerleri KENDI /odom'uyla (ayni cerceve) karsilastirip pure-pursuit
        hedefi olarak kullaniyor. HAM ENU degeri (slot['x']/['y']) DEGIL.
        """
        # Gorev Kapisi: park gorevi aktif degilse matris yayinlanmaz
        if not self.mission_active:
            return
        matrix = {
            str(slot_id): {
                'x': self.slot_xy_local[slot_id][0],
                'y': self.slot_xy_local[slot_id][1],
                'status': slot['status'],
                'occupied': slot['occupied'],
                'permit': slot['permit'],
                'available': self.slot_available(slot),
                # SON CARE olcutu: kontrol dugumu bunu SADECE tum kesif
                # duraklari tukendiginde okur (bkz. slot_available_fallback).
                'available_fallback': self.slot_available_fallback(slot),
            }
            for slot_id, slot in self.parking_slots.items()
        }
        msg = String()
        msg.data = json.dumps(matrix)
        self.status_pub.publish(msg)

        available = [sid for sid, s in self.parking_slots.items()
                     if self.slot_available(s)]
        rows = ' | '.join(
            f"{sid}:{s['status'][:3]}/"
            f"{s['permit'] if s['permit'] != 'unknown' else '?'}"
            for sid, s in self.parking_slots.items())
        # Son care adaylari da loglanir: strict liste bosken hangi ceplerin
        # 'sadece izni okunamadi' yuzunden elendigi sahada aninda gorunsun.
        fb = [sid for sid, s in self.parking_slots.items()
              if self.slot_available_fallback(s)]
        yolo_tag = ('YOLO:YOK' if self.last_yolo_msg_time is None
                    else f'YOLO:{self.yolo_msg_count}')
        self.get_logger().info(
            f'[{rows}] -> kullanilabilir: {available if available else "YOK"} '
            f'| son-care adayi: {fb if fb else "YOK"} | {yolo_tag}',
            throttle_duration_sec=1.0)

        # RViz gorsellestirilmesi: cep kutulari
        self.publish_slot_markers()

    def publish_slot_markers(self):
        """RViz'de cepleri kutu olarak goster: yesil (izinli), kirmizi (yasak),
        gri (okunamadi), capraz cizgili (dolu)."""
        if not self.mission_active:
            return
        arr = MarkerArray()
        for slot_id, slot in self.parking_slots.items():
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = slot_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = self.slot_xy_local[slot_id][0]
            m.pose.position.y = self.slot_xy_local[slot_id][1]
            m.pose.position.z = 0.0
            m.scale.x = 0.5  # [m] cep eni
            m.scale.y = 2.3  # [m] cep derinligi
            m.scale.z = 0.01
            # Renk: izin durumuna gore
            if not (slot['status'] == 'bos' and not slot['occupied']):
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # kirmizi: dolu
            elif slot['permit'] == 'park_edilebilir':
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # yesil: izinli
            elif slot['permit'] == 'park_yasak':
                m.color.r, m.color.g, m.color.b = 0.5, 0.0, 0.5  # mor: yasak
            else:  # unknown
                m.color.r, m.color.g, m.color.b = 0.7, 0.7, 0.7  # gri: bilinmiyor
            m.color.a = 0.6
            m.text = f'{slot_id}'
            arr.markers.append(m)
        self.slot_markers_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMissionPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
