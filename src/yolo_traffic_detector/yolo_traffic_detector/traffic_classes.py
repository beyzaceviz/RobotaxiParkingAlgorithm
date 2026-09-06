"""
Traffic class definitions and priority system for YOLO traffic detector.
"""

# Class ID to name mapping
ID2NAME = {
    0: "red_light", 1: "yellow_light", 2: "green_light",
    3: "yaya_gecidi",
    4: "trafik_adasi",
    5: "trafik_isigi_cihazi",
    6: "saga_donulmez", 7: "sola_donulmez",
    8: "giris_yasak",
    9: "sagdan_gidiniz", 10: "soldan_gidiniz",
    11: "saga_mecburi", 12: "sola_mecburi",
    13: "ileri_mecburi",
    14: "ileri_ve_saga_mecburi", 15: "ileri_ve_sola_mecburi",
    16: "ileriden_saga_mecburi", 17: "ileriden_sola_mecburi",
    18: "saga_duzenleme_levha", 19: "sola_duzenleme_levha",
    20: "iki_yonlu_yol",
    21: "park_yasak",
    22: "park_yeri", 23: "park_yeri_engelli",
    24: "tunel", 25: "durak",
    26: "dur",
}

# Priority values for different traffic classes
CLASS_PRIORITY = {
    "red_light": 1.00, "green_light": 0.60, "yellow_light": 0.40,
    "dur": 0.95, "giris_yasak": 0.90,

    "saga_mecburi": 0.75, "sola_mecburi": 0.75, "ileri_mecburi": 0.75,
    "ileri_ve_saga_mecburi": 0.70, "ileri_ve_sola_mecburi": 0.70,
    "ileriden_saga_mecburi": 0.70, "ileriden_sola_mecburi": 0.70,
    "saga_duzenleme_levha": 0.65, "sola_duzenleme_levha": 0.65,
    "sagdan_gidiniz": 0.65, "soldan_gidiniz": 0.65,

    "trafik_isigi_cihazi": 0.60, "yaya_gecidi": 0.60,
    "trafik_adasi": 0.55, "iki_yonlu_yol": 0.55, "tunel": 0.55,

    "park_yasak": 0.50,
    "park_yeri": 0.40, "park_yeri_engelli": 0.40, "durak": 0.40,
}

# Geometric ranges for distance estimation
# Format: ("height"|"width", min_meters, max_meters)
CLASS_GEOM_RANGE = {
    # Traffic lights (full head)
    "red_light": ("height", 0.75, 0.85),
    "yellow_light": ("height", 0.75, 0.85),
    "green_light": ("height", 0.75, 0.85),

    # Circular signs (diameter 0.60 m)
    "dur": ("width", 0.60, 0.60),
    "giris_yasak": ("width", 0.60, 0.60),
    "park_yasak": ("width", 0.60, 0.60),
    "saga_donulmez": ("width", 0.60, 0.60),
    "sola_donulmez": ("width", 0.60, 0.60),
    "sagdan_gidiniz": ("width", 0.60, 0.60),
    "soldan_gidiniz": ("width", 0.60, 0.60),
    "saga_mecburi": ("width", 0.60, 0.60),
    "sola_mecburi": ("width", 0.60, 0.60),
    "ileri_mecburi": ("width", 0.60, 0.60),
    "ileri_ve_saga_mecburi": ("width", 0.60, 0.60),
    "ileri_ve_sola_mecburi": ("width", 0.60, 0.60),
    "ileriden_saga_mecburi": ("width", 0.60, 0.60),
    "ileriden_sola_mecburi": ("width", 0.60, 0.60),
    "saga_duzenleme_levha": ("width", 0.60, 0.60),
    "sola_duzenleme_levha": ("width", 0.60, 0.60),
    "trafik_adasi": ("width", 0.60, 0.60),

    # Rectangular signs
    "park_yeri": ("height", 0.60, 0.60),
    "park_yeri_engelli": ("height", 0.60, 0.60),
    "durak": ("height", 0.60, 0.60),

    # Triangular signs
    "yaya_gecidi": ("height", 0.60, 0.60),
    "trafik_isigi_cihazi": ("height", 0.60, 0.60),
    "iki_yonlu_yol": ("height", 0.60, 0.60),

    # Large rectangular signs
    "tunel": ("height", 0.90, 0.90),
}


def is_light(class_name: str) -> bool:
    """Check if class is a traffic light."""
    return class_name in ("red_light", "yellow_light", "green_light")


def get_class_priority(class_name: str) -> float:
    """Get priority value for a class."""
    return CLASS_PRIORITY.get(class_name, 0.4)


def get_class_geometry(class_name: str) -> tuple:
    """Get geometric range for distance estimation."""
    return CLASS_GEOM_RANGE.get(class_name, None)


def get_class_name(class_id: int) -> str:
    """Get class name from ID."""
    return ID2NAME.get(class_id, f"cls_{class_id}")