#!/usr/bin/env python3
"""BeemobsActuator - Robotaksi Hazir Arac CAN Aktuator Katmani (paylasimli).

Park (parking_decision_and_control_node) ve durak (stop_decision_and_control_node)
kontrol dugumleri, hesapladiklari (ileri hiz [m/s], direksiyon acisi [rad]) ciftini
DOGRUDAN araca yazmak icin bu sinifi kullanir. Eski cmd_vel_to_beemobs_bridge
dugumu KALDIRILDI (araya /cmd_vel + ayri kopru dugumu koymak yerine kontrol
dugumu araca dogrudan komut verir). Iki misyon paketi TEK KAYNAKTAN (bu sinif)
ayni arac mantigini paylasir - durak paketi bunu parking_safety'den import eder.

KONTROL YOLU: DOGRUDAN PWM + KAPALI CEVRIM  (araca gomulu PID KULLANILMAZ)
    Geri beslemeyi (FeedbackSteeringAngle derece, FB_VehicleSpeed km/h) biz okuyup
    aktuator komutunu (SteeringMot PWM, throttle pedal) biz uretiriz. Gerekce:
    aractaki steering_pid_node/speed_pid_node yalniz pid_launch ile baslar,
    parametreleri yanlis (buyuk/kucuk harf) topic'lere bagli ve steering min_output
    dogrulanamiyor -> sahada calismaz. Saha-kanitli desen: lane_following_controller.

PWM KONVANSIYONU (dbc + Kullanici Dok. + lane_following ile dogrulandi):
    128 = notr; >128 SAGA, <128 SOLA. 128'den uzaklik motorun DONME HIZI. Konumu,
    geri beslemeyi okuyup hedefe varinca motoru durdurarak (en=0, pwm=128) saglariz.

ISARET (SIGN): ROS/REP-103 angular.z POZITIF=SOLA; arac POZITIF=SAGA. steer_sign
    (varsayilan -1.0) ile cevrilir. Sahada ters kirarsa +1.0 yapin.

===========================================================================
 SAHA KALIBRASYONU - 2026-07-28, direksiyon_0727_1735 bag kaydindan OLCULDU
===========================================================================
Kaynak: hareket iceren 8 bag (direksiyon_0727_1735, cep2kayit, durak_0727_1138,
durak_0727_1141, duzters_0727_1758, cep1duzkayit_0727_1541, hiz_0727_1156,
duz_0727_1152) - 26502 gecerli ornek, R^2 = 0.936.

Yontem (2026-07-28 REVIZE): egrilik 1 saniyelik kayan pencerede
kappa = delta_yaw / yay_uzunlugu olarak hesaplandi. ONCEKI yontem
(kappa = yaw_rate / hiz, yaw_rate = np.gradient) HATALIYDI: /filter/quaternion
200 Hz ve yinelenen zaman damgalari iceriyor, gradyan patliyor, filtre
orneklerin %99'unu atiyordu (n=78 kaliyordu). Konum->metre donusumu de kuresel
R=6378137 yerine WGS84 elipsoit yaricaplariyla (N=6387268, M=6362688 @40.79 deg)
yapildi; eskisi kuzeyde %0.243 olcek hatasi veriyordu.

  1) GERI BESLEME BIRIMI DERECE DEGIL - HAM SAYAC.
     Olculen baginti:   kappa [1/m] = -0.005813 * fb - 0.005939
                        (yani kappa = -0.005813 * (fb + 1.02))
     Tam kilit fb=-37 -> R = +4.78 m (SOLA);  fb=+33 -> R = -5.06 m (SAGA)
     Bisiklet modeli (L=1.86) ile:  fb=-37 -> +21.26 deg, fb=+33 -> -20.20 deg
     => 1 sayac ~ 0.58 derece.  ESKI KOD 1 sayac = 1 derece VARSAYIYORDU.
     steer_deg_to_counts() ile TAM (tanjantli) donusum yapiliyor.

     ONCEKI (HATALI) DEGERLER: egim -0.006850, merkez +1.25. Eğim %18 fazlaydi;
     fb=-37'de 25.98 deg diyordu, gercegi 21.26 deg - 4.7 derece HATA.

  2) DUZ KONUM fb = 0 DEGIL, fb = -1.02.  (kappa=0 kesisimi, 26502 ornek.)
     DIKKAT - OLU BANT: sol taraf ekstrapolasyonu merkezi -2.5, sag taraf +4.2
     veriyor. Aradaki ~6.8 sayac mekanik bosluk/histerezis. Bu yuzden
     steer_tolerance_counts 2.0 -> 4.0 yapildi; 2.0 bosluktan kucuk oldugu
     icin kontrolcu merkez civarinda salinirdi.

  3) MEKANIK LIMITLER ASIMETRIK: fb -37 (sol) .. +33 (sag).
     PWM 70 ile 33 s, PWM 200 ile 49 s zorlandiginda bu degerlerde SABIT
     kaldi => gercek mekanik dayanma. Derece karsiligi +21.26 / -20.20.
     max_steer_deg 22.0 -> 20.0: yeni kalibrasyonla 22.0 IKI YONDE DE
     ULASILAMIYOR. 20.0 ile fb -34.7 / +32.6 -> ikisi de limit icinde.

  4) CALISTIGI KANITLANMIS PWM: sadece 70 ve 200 (notr 128'e gore -58/+72).
     ESKI min/max offset 22/52 -> PWM 106..150 araligi HIC DENENMEDI; motorun
     olu bandinin altinda kalip direksiyonun HIC donmemesi riski vardi.
     Yeni offsetler 58/72 = sahada donmesi kanitlanmis tek aralik.
     Olculen donme hizi: PWM 70 -> -8..-10 sayac/s, PWM 200 -> +12 sayac/s.
     => KILITTEN KILIDE (70 sayac) 6-9 SANIYE. Manevra planlanirken bu
        gecikme hesaba katilmali.
     ACIK RISK: ara PWM degerleri (129..199) araçta HIC denenmedi. Kapali
     cevrim kontrolcu bunlari uretiyor; motorun olu bandi bilinmiyor.

  5) steer_sign = -1.0 DOGRULANDI (varsayim degil, olcum):
     PWM 70 -> fb azaliyor -> yaw_rate POZITIF -> arac SOLA donuyor.
     PWM 200 -> fb artiyor -> yaw_rate NEGATIF -> arac SAGA donuyor.
     ROS'ta angular.z pozitif = SOLA oldugundan isaret cevrilmelidir.

  6) GAZ TAVANI: duz_0727_1152 ve hiz_0727_1156'da pedal pozisyonu en fazla
     76 ve 84 kullanildi; ulasilan hiz 0.98 ve 1.25 m/s. thr_max 120 -> 90.
     FB_VehicleSpeed cozunurlugu 1 km/h oldugu icin park hizinda (0.5 m/s =
     1.8 km/h) entegrator kolayca tavana tirmanir; 120'de ani firlama riski.
===========================================================================

Kullanim (kontrol dugumu icinde):
    self.actuator = BeemobsActuator(self)        # __init__ icinde
    ...                                          # control_loop icinde, GOREV AKTIFKEN:
    self.actuator.tick(linear_ms, angular_rad)   # her komut yerine (cmd_vel yayini yerine)
Gorev aktif DEGILKEN kontrol dugumu tick() cagirmaz -> hicbir /beemobs yazilmaz
(lane_following CAN'in sahibi kalir). Gate mantigi kontrol dugumundedir.
"""

import math

from geometry_msgs.msg import Twist  # noqa: F401  (tip referansi/uyumluluk)
from smart_can_msgs.msg import (
    Rcunittoomux,
    Autonomoussteeringmotcontrol,
    Rcthrtdata,
    Autonomousbrakepedalcontrol,
    Feedbacksteeringangle,
    Fbvehiclespeed,
)


class BeemobsActuator:
    """Bir rclpy Node'a takilan arac CAN aktuator katmani (Node DEGIL)."""

    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.clock = node.get_clock()

        d = node.declare_parameter
        g = node.get_parameter

        # --- Genel -----------------------------------------------------------
        d('enabled', True)
        d('cmd_vel_timeout', 0.5)          # tick akisi kesilince guvenli durus [s]
        d('max_speed_ms', 2.0)             # guvenlik hiz tavani

        # --- Direksiyon (TAMAMI 2026-07-28 saha bag'lerinden olculdu) --------
        # max_steer_deg: mekanik limitler fb -37/+33 = +21.26/-20.20 derece;
        # iki yonde de ULASILABILIR simetrik sinir 20.0 (ust yorum madde 3).
        # 20.0 -> fb -34.7 / +32.6; ikisi de mekanik limitin icinde.
        d('max_steer_deg', 20.0)
        d('steer_sign', -1.0)              # ROS(+=SOL) -> arac(+=SAG); OLCULDU
        d('steer_pwm_neutral', 128)
        # Sahada donmesi KANITLANMIS tek aralik: PWM 70 (128-58) ve 200 (128+72)
        d('steer_pwm_min_offset', 58)
        d('steer_pwm_max_offset', 72)
        d('require_steer_feedback', True)  # feedback yoksa guvenli durus
        d('feedback_timeout', 1.0)

        # Geri besleme HAM SAYAC birimindedir (derece DEGIL) - ust yorum md.1/2.
        # Tolerans ve olcek de bu yuzden sayac cinsinden tutulur.
        # 2026-07-28 REVIZE: 26502 ornek / 8 bag / R^2=0.936 (ust yorum md.1).
        d('steer_fb_center', -1.02)        # kappa=0 kesisimi [sayac]
        d('steer_curv_per_count', 0.005813)  # [1/m] egrilik / sayac
        # 2026-07-28: 1.86 m KULLANICI TARAFINDAN TEYIT EDILDI (onceden tahmindi).
        # Egrilik kalibrasyonu bundan BAGIMSIZ; L sadece derece<->sayac
        # donusumunde kullanilir. Kontrolcu hedefe steer_tolerance_counts kala
        # durdugu icin kilide DAYANMA (stall) ancak L<1.647 m olsaydi olurdu.
        d('wheelbase', 1.86)               # derece<->egrilik donusumu icin [m]
        # 2.0 -> 4.0: olculen mekanik bosluk ~6.8 sayac (ust yorum md.2).
        # Toleransi bosluktan kucuk tutmak merkez civarinda salinim yaratir.
        # YAN ETKI: kontrolcu hedefe 4 sayac kala durur, yani her komut
        # acisinda ~2.3 derece kalici hata kalir. max_steer_deg=20.0 komut
        # edilse bile FIILEN ulasilan aci +-17.8 derece. Park manevrasinin
        # donusu surekli genis kaliyorsa sebebi budur; o zaman max_steer_deg
        # 22-23'e cikarilip tolerans telafi edilir (SAHA: Asama 5).
        d('steer_tolerance_counts', 4.0)   # +-4 sayac ~ +-2.3 derece
        d('steer_full_scale_counts', 15.0)  # bu hatada PWM tavana cikar

        # --- Hiz (gaz pedali) -------------------------------------------------
        d('speed_ctrl_rate_hz', 5.0)       # gaz rampasi kadansi (tick'ten bagimsiz)
        d('thr_min', 50)
        # OLCULDU: sahada en fazla 84 kullanildi (1.25 m/s). 120 -> 90.
        d('thr_max', 90)
        d('speed_deadzone_kmh', 1.0)
        d('speed_stop_threshold_kmh', 0.5)
        d('gear_change_speed_kmh', 1.0)

        # --- Fren / acil ------------------------------------------------------
        d('brake_percent', 100)
        d('brake_acc', 10000)
        d('emergency_on_timeout', False)

        self.enabled = bool(g('enabled').value)
        self.cmd_timeout = float(g('cmd_vel_timeout').value)
        self.max_speed = float(g('max_speed_ms').value)

        self.max_steer_deg = float(g('max_steer_deg').value)
        self.steer_sign = float(g('steer_sign').value)
        self.pwm_neutral = int(g('steer_pwm_neutral').value)
        self.pwm_min_off = int(g('steer_pwm_min_offset').value)
        self.pwm_max_off = int(g('steer_pwm_max_offset').value)
        self.require_steer_fb = bool(g('require_steer_feedback').value)
        self.fb_timeout = float(g('feedback_timeout').value)

        self.fb_center = float(g('steer_fb_center').value)
        self.curv_per_count = float(g('steer_curv_per_count').value)
        self.wheelbase = float(g('wheelbase').value)
        self.steer_tol = float(g('steer_tolerance_counts').value)
        self.steer_full_scale = float(g('steer_full_scale_counts').value)

        self.speed_dt = 1.0 / max(1e-6, float(g('speed_ctrl_rate_hz').value))
        self.thr_min = int(g('thr_min').value)
        self.thr_max = int(g('thr_max').value)
        self.speed_deadzone = float(g('speed_deadzone_kmh').value)
        self.speed_stop_thr = float(g('speed_stop_threshold_kmh').value)
        self.gear_change_speed = float(g('gear_change_speed_kmh').value)

        self.brake_percent = int(g('brake_percent').value)
        self.brake_acc = int(g('brake_acc').value)
        self.emergency_on_timeout = bool(g('emergency_on_timeout').value)

        # --- Geri besleme abonelikleri (kapali cevrim) -----------------------
        self.steer_fb_sub = node.create_subscription(
            Feedbacksteeringangle, '/beemobs/FeedbackSteeringAngle',
            self.steer_fb_callback, 10)
        self.speed_fb_sub = node.create_subscription(
            Fbvehiclespeed, '/beemobs/FB_VehicleSpeed',
            self.speed_fb_callback, 10)

        # --- Cikti (arac arayuzu) --------------------------------------------
        self.omux_pub = node.create_publisher(
            Rcunittoomux, '/beemobs/rc_unittoOmux', 10)
        self.steer_pub = node.create_publisher(
            Autonomoussteeringmotcontrol,
            '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.thrt_pub = node.create_publisher(
            Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.brake_pub = node.create_publisher(
            Autonomousbrakepedalcontrol,
            '/beemobs/AUTONOMOUS_BrakePedalControl', 10)

        # --- Durum ------------------------------------------------------------
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.current_steer_count = 0.0
        self.steer_fb_time = None
        self.current_speed_kmh = 0.0
        self.speed_fb_time = None
        self.thr_value = self.thr_min
        self.current_gear = 0            # 0=N 1=D 2=R
        self.pending_gear = None
        self._last_speed_t = None        # gaz rampasi kadans zaman damgasi

        # DEADMAN: gorev aktifken kontrol dugumu tick() akisini kesince (or.
        # durak BEKLEME fazi, tetik bekliyor) arac GUVENLI DURUSA gecer (eski
        # kopru davranisi). set_active(False) -> aktuator tamamen susar
        # (lane_following CAN'in sahibi kalir).
        self._active = False
        self._last_tick_t = None
        self._deadman_timer = node.create_timer(0.05, self._deadman_check)

        self.log.info(
            f'BeemobsActuator hazir (DOGRUDAN PWM, saha-kalibreli). '
            f'enabled={self.enabled}, max_steer={self.max_steer_deg} deg '
            f'(= fb {self.steer_deg_to_counts(self.max_steer_deg):+.1f} / '
            f'{self.steer_deg_to_counts(-self.max_steer_deg):+.1f} sayac), '
            f'steer_sign={self.steer_sign}, fb_center={self.fb_center}, '
            f'pwm={self.pwm_neutral}+-[{self.pwm_min_off},{self.pwm_max_off}], '
            f'thr=[{self.thr_min},{self.thr_max}], fail-safe AKTIF')

    # ------------------------------------------------------------- Callbacks
    def steer_fb_callback(self, msg: Feedbacksteeringangle):
        # int8 HAM SAYAC (derece DEGIL - ust yorum md.1). Duz = steer_fb_center
        # (+1.25), NEGATIF = SOLA, POZITIF = SAGA. Aralik -37..+33 (mekanik).
        self.current_steer_count = float(msg.feedbacksteeringangle)
        self.steer_fb_time = self.clock.now()

    def speed_fb_callback(self, msg: Fbvehiclespeed):
        # uint8 km/h (ms cozunurlugu park hizlarinda yetersiz)
        self.current_speed_kmh = float(msg.fb_vehiclespeed_kmh)
        self.speed_fb_time = self.clock.now()

    # ----------------------------------------------------------- Aktif/Deadman
    def set_active(self, on: bool):
        """Kontrol dugumu gorev kapisinda cagirir. True: aktuator devrede
        (deadman korumasi acik). False: aktuator TAMAMEN susar, /beemobs'a
        yazmaz (lane_following sahiplenir)."""
        on = bool(on)
        if on and not self._active:
            self._last_tick_t = self.clock.now()   # ilk anda deadman tetiklemesin
        self._active = on

    def _deadman_check(self):
        """Gorev aktifken tick akisi kesilirse guvenli durus (eski kopru deadman)."""
        if not self._active:
            return                                  # pasif: hicbir sey yazma
        if self._age(self._last_tick_t) > self.cmd_timeout:
            self.safe_stop(emergency=self.emergency_on_timeout)

    # -------------------------------------------------------------- Yardimci
    def _age(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.clock.now() - stamp).nanoseconds * 1e-9

    def publish_steer(self, en: int, pwm: int):
        msg = Autonomoussteeringmotcontrol()
        msg.autonomous_steeringmot_en = int(en)
        msg.autonomous_steeringmot_pwm = int(max(0, min(255, pwm)))
        self.steer_pub.publish(msg)

    def stop_steering(self):
        """Direksiyon motorunu durdur (konumu KORUR)."""
        self.publish_steer(0, self.pwm_neutral)

    def stop_throttle(self):
        thr = Rcthrtdata()
        thr.rc_thrt_pedal_press = 1       # 1 = NotPressed -> guc yok
        thr.rc_thrt_pedal_position = self.thr_min
        self.thrt_pub.publish(thr)
        self.thr_value = self.thr_min     # windup engeli

    def release_brake(self):
        brk = Autonomousbrakepedalcontrol()
        brk.autonomous_brakepedalmotor_en = 0
        brk.autonomous_brakemotor_voltage = 0
        brk.autonomous_brakepedalmotor_acc = self.brake_acc
        brk.autonomous_brakepedalmotor_per = 0
        self.brake_pub.publish(brk)

    # ------------------------------------------------------------- Fail-safe
    def safe_stop(self, emergency: bool = False):
        """Gazi kes, mekanik freni bas, direksiyon motorunu durdur, vites N."""
        self.stop_throttle()
        self.stop_steering()

        brk = Autonomousbrakepedalcontrol()
        brk.autonomous_brakepedalmotor_en = 1
        brk.autonomous_brakemotor_voltage = 1
        brk.autonomous_brakepedalmotor_acc = self.brake_acc
        brk.autonomous_brakepedalmotor_per = self.brake_percent
        self.brake_pub.publish(brk)

        omux = Rcunittoomux()
        omux.rc_ignition = 1
        omux.rc_selectiongear = 0
        omux.autonomous_emergency = 1 if emergency else 0
        self.omux_pub.publish(omux)
        self.current_gear = 0

    # --------------------------------------------------- Direksiyon (kapali cevrim)
    def steer_deg_to_counts(self, steer_deg: float) -> float:
        """Istenen tekerlek acisini (derece, ROS: + = SOLA) geri besleme
        SAYACINA cevirir. Bagintý sahada olculdu (ust yorum md.1):

            kappa = tan(delta) / L            (bisiklet modeli)
            fb    = center + sign * kappa / curv_per_count

        steer_sign = -1 iken: delta=+26 deg (sol) -> fb = -37 (olculen kilit).
        Tanjant kullanilir, kucuk aci yaklasimi DEGIL - 22 derecede fark %5.
        """
        kappa = math.tan(math.radians(steer_deg)) / max(1e-6, self.wheelbase)
        return self.fb_center + self.steer_sign * kappa / self.curv_per_count

    def steering_control(self):
        # last_angular: kontrol dugumunun urettigi TEKERLEK ACISI [rad]
        # (bkz. parking_decision_and_control_node ust yorumu - angular.z acisal
        #  hiz degil, hedef direksiyon acisidir).
        target_deg = math.degrees(self.last_angular)
        target_deg = max(-self.max_steer_deg, min(target_deg, self.max_steer_deg))

        # Isaret cevrimi ARTIK BURADA DEGIL - steer_deg_to_counts icinde
        # (egrilik uzerinden), boylece asimetrik mekanik de dogru modellenir.
        target_count = self.steer_deg_to_counts(target_deg)
        err = target_count - self.current_steer_count

        if abs(err) <= self.steer_tol:
            self.stop_steering()          # hedefte -> durdur (titreme engeli)
            return

        span = max(1e-6, self.steer_full_scale - self.steer_tol)
        ratio = min(1.0, (abs(err) - self.steer_tol) / span)
        offset = int(round(self.pwm_min_off +
                           (self.pwm_max_off - self.pwm_min_off) * ratio))
        pwm = self.pwm_neutral + offset if err > 0 else self.pwm_neutral - offset
        self.publish_steer(1, pwm)

    # ------------------------------------------------------------- Hiz rampasi
    def _speed_ramp(self):
        """Artimli gaz kontrolu; speed_ctrl_rate_hz kadansinda (tick'ten bagimsiz)."""
        now = self.clock.now()
        if self._last_speed_t is not None and \
                (now - self._last_speed_t).nanoseconds * 1e-9 < self.speed_dt:
            # Bu tick'te henuz rampa zamani gelmedi; son gaz degerini tekrarla
            self._publish_throttle()
            return
        self._last_speed_t = now

        if self.pending_gear is not None:
            return                        # vites degisimi icin duruyoruz
        target_kmh = min(abs(self.last_linear), self.max_speed) * 3.6
        if target_kmh < self.speed_stop_thr:
            self.stop_throttle()
            return
        err = target_kmh - self.current_speed_kmh
        if abs(err) > self.speed_deadzone:
            step = 3 if abs(err) >= 3.0 else (2 if abs(err) >= 2.0 else 1)
            self.thr_value += step if err > 0 else -step
            self.thr_value = max(self.thr_min, min(self.thr_max, self.thr_value))
        self._publish_throttle()

    def _publish_throttle(self):
        thr = Rcthrtdata()
        thr.rc_thrt_pedal_press = 0       # 0 = Pressed -> guc verilir
        thr.rc_thrt_pedal_position = int(self.thr_value)
        self.thrt_pub.publish(thr)

    # ----------------------------------------------------------------- Ana API
    def tick(self, linear_ms: float, angular_rad: float):
        """Kontrol dugumu her aktif dongude cagirir: (ileri hiz m/s, direksiyon rad)
        -> arac CAN komutlari. Gorev pasifken dugum bunu CAGIRMAZ (gate dugumde)."""
        self.last_linear = float(linear_ms)
        self.last_angular = float(angular_rad)
        self._last_tick_t = self.clock.now()        # deadman'i besle

        if not self.enabled:
            self.safe_stop(emergency=False)
            return

        # Kapali cevrim direksiyon geri beslemesi yoksa KORDUR -> guvenli durus
        if self.require_steer_fb and self._age(self.steer_fb_time) > self.fb_timeout:
            self.log.warn(
                '/beemobs/FeedbackSteeringAngle YOK/bayat -> guvenli durus. '
                'can_launch calisiyor mu? (kapali cevrim direksiyon buna bagli)',
                throttle_duration_sec=2.0)
            self.safe_stop(emergency=False)
            return

        # --- 1) Vites secimi (hareket halindeyken DEGISTIRME) ----------------
        if self.last_linear > 0.05:
            desired_gear = 1              # DRIVE
        elif self.last_linear < -0.05:
            desired_gear = 2              # REVERSE
        else:
            desired_gear = 0              # NEUTRAL

        if desired_gear != self.current_gear and \
                self.current_speed_kmh > self.gear_change_speed:
            # Once tamamen dur, sonra vites degistir (sanziman korumasi)
            self.pending_gear = desired_gear
            self.stop_throttle()
            brk = Autonomousbrakepedalcontrol()
            brk.autonomous_brakepedalmotor_en = 1
            brk.autonomous_brakemotor_voltage = 1
            brk.autonomous_brakepedalmotor_acc = self.brake_acc
            brk.autonomous_brakepedalmotor_per = self.brake_percent
            self.brake_pub.publish(brk)
            gear_to_send = self.current_gear
            self.log.info(
                f'Vites degisimi bekliyor ({self.current_gear}->{desired_gear}), '
                f'hiz={self.current_speed_kmh:.0f} km/h - once duruluyor',
                throttle_duration_sec=1.0)
        else:
            gear_to_send = desired_gear
            self.current_gear = desired_gear
            self.pending_gear = None
            self.release_brake()

        omux = Rcunittoomux()
        omux.rc_ignition = 1
        omux.autonomous_emergency = 0
        omux.rc_selectiongear = gear_to_send
        self.omux_pub.publish(omux)

        # --- 2) Direksiyon (kapali cevrim PWM) -------------------------------
        self.steering_control()

        # --- 3) Gaz (rampali, kendi kadansinda) ------------------------------
        self._speed_ramp()
