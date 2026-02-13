import numpy as np


def deg2rad(deg):
    return deg * np.pi / 180.0

def deg2rad_list(deg_list: list[float]) -> list[float]:
    return [deg2rad(deg) for deg in deg_list]

def rad2deg(rad):
    return rad * 180.0 / np.pi

def rad2deg_list(rad_list: list[float]) -> list[float]:
    return [rad2deg(rad) for rad in rad_list]
