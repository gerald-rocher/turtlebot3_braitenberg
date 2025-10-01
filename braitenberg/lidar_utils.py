import math
from sensor_msgs.msg import LaserScan

def get_max_range(scan: LaserScan) -> float:
    """
    Return the maximum valid range detected by the LiDAR.
    Ignores NaN, inf, and values outside [range_min, range_max].
    """
    valid_ranges = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    if not valid_ranges:
        return 0.0
    return max(valid_ranges)


def get_min_range(scan: LaserScan) -> float:
    """
    Return the minimum valid range detected by the LiDAR.
    Ignores NaN, inf, and values outside [range_min, range_max].
    """
    valid_ranges = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    if not valid_ranges:
        return 0.0
    return min(valid_ranges)


def get_range_at_angle(scan: LaserScan, angle_deg: float, window_deg: float = 2.0) -> float:
    """
    Compute average range and return indices used in the window.
    """
    angle_rad = math.radians(angle_deg)
    half_window = math.radians(window_deg)

    idx_min = max(0, int(round((angle_rad - half_window - scan.angle_min) / scan.angle_increment)))
    idx_max = min(len(scan.ranges) - 1, int(round((angle_rad + half_window - scan.angle_min) / scan.angle_increment)))

    values = [r for r in scan.ranges[idx_min:idx_max+1] if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    avg_range = float('inf') if not values else sum(values) / len(values)

    return avg_range, (idx_min, idx_max)

def publish_debug_scan(original_scan: LaserScan, left_indices, right_indices, publisher):
    """
    Publish a LaserScan with only the ranges used for left_d and right_d.
    Other values are set to +inf.
    """
    debug_scan = LaserScan()
    debug_scan.header = original_scan.header
    debug_scan.angle_min = original_scan.angle_min
    debug_scan.angle_max = original_scan.angle_max
    debug_scan.angle_increment = original_scan.angle_increment
    debug_scan.time_increment = original_scan.time_increment
    debug_scan.scan_time = original_scan.scan_time
    debug_scan.range_min = original_scan.range_min
    debug_scan.range_max = original_scan.range_max

    # Copy ranges and mask everything except left/right windows
    ranges = [float('inf')] * len(original_scan.ranges)

    li_min, li_max = left_indices
    ri_min, ri_max = right_indices

    for i in range(li_min, li_max + 1):
        ranges[i] = original_scan.ranges[i]

    for i in range(ri_min, ri_max + 1):
        ranges[i] = original_scan.ranges[i]

    debug_scan.ranges = ranges

    publisher.publish(debug_scan)
    
    
