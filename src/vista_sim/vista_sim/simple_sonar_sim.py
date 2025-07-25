import numpy as np
import numpy.typing as npt


def get_range_bearing_pitch_mask(
    relative_ranges: npt.NDArray[np.float_],
    relative_bearings: npt.NDArray[np.float_],
    relative_pitches: npt.NDArray[np.float_],
    min_range: float,
    max_range: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> npt.NDArray[np.bool_]:
    """
    Apply range, bearing, and pitch mask to simulated sonar data.

    :param relative_ranges: 2D array of ranges to each map cell from the sonar.
    :type relative_ranges: numpy.ndarray
    :param relative_bearings: 2D array of bearings to each map cell from the sonar,
        w.r.t vehicle heading.
    :type relative_bearings: numpy.ndarray
    :param relative_pitches: 2D array of pitches to each map cell from the sonar,
        w.r.t effective vehicle steering angle (vehicle pitch + steering angle).
    :type relative_pitches: numpy.ndarray
    :param min_range: The minimum range to consider from the center in the terrain slice.
    :type min_range: float
    :param max_range: The maximum range to consider from the center in the terrain slice.
    :type max_range: float
    :param horizontal_fov_deg: The total horizontal field-of-view of the sonar in degrees.
    :type horizontal_fov_deg: float
    :param vertical_fov_deg: The total vertical field-of-view of the sonar in degrees.
    :type vertical_fov_deg: float
    :return: Mask array indicating visibility region of sonar. True mask values are
        outside of sonar visibility.
    :rtype: npt.NDArray[bool]
    """
    # Create range mask to filter out values outside of range
    range_mask = (relative_ranges < min_range) | (relative_ranges > max_range)

    # Create bearing mask to filter out values outside of horizontal FOV
    max_h_angle_rad = np.deg2rad(horizontal_fov_deg / 2)
    brg_mask = np.abs(relative_bearings) > max_h_angle_rad

    # Create pitch mask to filter out values outside of vertical FOV
    max_v_angle_rad = np.deg2rad(vertical_fov_deg / 2)
    pitch_mask = np.abs(relative_pitches) > max_v_angle_rad

    # Filter out using combined range, bearing, and pitch mask
    total_mask = range_mask | brg_mask | pitch_mask
    return total_mask