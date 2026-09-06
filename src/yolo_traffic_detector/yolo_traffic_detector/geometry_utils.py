"""
Geometry utilities for distance estimation and scoring.
"""
import math
from typing import Optional, Tuple
from .traffic_classes import get_class_geometry


def clip(value: float, min_val: float, max_val: float) -> float:
    """Clip value to range [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def normalize_inverse_distance(distance: float, d_min: float = 1.0, d_max: float = 40.0) -> float:
    """
    Normalize distance to [0, 1] where closer objects get higher scores.
    
    Args:
        distance: Distance in meters
        d_min: Minimum distance for normalization
        d_max: Maximum distance for normalization
        
    Returns:
        Normalized inverse distance score [0, 1]
    """
    distance = clip(distance, d_min, d_max)
    return (d_max - distance) / (d_max - d_min)


def estimate_distance_meters(class_name: str, x1: float, y1: float, x2: float, y2: float, 
                           focal_length_px: float) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """
    Estimate distance using object size and known real-world dimensions.
    
    Args:
        class_name: Name of the detected class
        x1, y1, x2, y2: Bounding box coordinates
        focal_length_px: Camera focal length in pixels
        
    Returns:
        Tuple of (estimated_distance, min_distance, max_distance) in meters
        Returns (None, None, None) if geometry info not available
    """
    geom_info = get_class_geometry(class_name)
    if geom_info is None or focal_length_px is None:
        return None, None, None
    
    width_px = max(1.0, x2 - x1)
    height_px = max(1.0, y2 - y1)
    
    mode, size_min, size_max = geom_info
    
    if mode == "height":
        pixel_size = height_px
    else:  # mode == "width"
        pixel_size = width_px
    
    # Distance = (focal_length * real_world_size) / pixel_size
    distance_min = (focal_length_px * size_min) / pixel_size
    distance_max = (focal_length_px * size_max) / pixel_size
    
    # Ensure min <= max
    if distance_min > distance_max:
        distance_min, distance_max = distance_max, distance_min
    
    estimated_distance = 0.5 * (distance_min + distance_max)
    
    return estimated_distance, distance_min, distance_max


def calculate_off_axis_penalty(x1: float, x2: float, image_width: float) -> float:
    """
    Calculate penalty for objects that are off-center in the image.
    
    Args:
        x1, x2: Bounding box x-coordinates
        image_width: Width of the image
        
    Returns:
        Penalty value [0, 1] where 0 is centered, 1 is at edge
    """
    center_x = 0.5 * (x1 + x2)
    image_center = image_width / 2.0
    
    # Normalized distance from center
    dx = abs(center_x - image_center) / (image_width / 2.0)
    
    return clip(dx, 0.0, 1.0)


def calculate_detection_score(confidence: float, distance: float, class_name: str,
                            x1: float, x2: float, image_width: float,
                            weights: dict, distance_range: tuple) -> float:
    """
    Calculate composite detection score.
    
    Args:
        confidence: Detection confidence [0, 1]
        distance: Estimated distance in meters
        class_name: Name of detected class
        x1, x2: Bounding box x-coordinates
        image_width: Width of the image
        weights: Dictionary with scoring weights
        distance_range: (min_distance, max_distance) for normalization
        
    Returns:
        Composite score
    """
    from .traffic_classes import get_class_priority
    
    # Normalize distance (higher score for closer objects)
    inv_dist_score = normalize_inverse_distance(distance, *distance_range) if distance is not None else 0.0
    
    # Get class priority
    priority_score = get_class_priority(class_name)
    
    # Calculate off-axis penalty
    off_axis_penalty = calculate_off_axis_penalty(x1, x2, image_width)
    
    # Composite score
    score = (weights['weight_confidence'] * confidence +
             weights['weight_distance'] * inv_dist_score +
             weights['weight_priority'] * priority_score -
             weights['weight_off_axis'] * off_axis_penalty)
    
    return score