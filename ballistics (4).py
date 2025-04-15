
import math

# Constants
GAS_CONSTANT = 287.05  # J/kg·K
R_AIR = 287.05

def speed_of_sound(temp_c):
    return math.sqrt(1.4 * R_AIR * (temp_c + 273.15))

def air_density(temp_c, pressure_hpa, humidity):
    T = temp_c + 273.15
    P = pressure_hpa * 100
    saturation = 6.1078 * 10**((7.5 * temp_c) / (temp_c + 237.3))
    vapor_pressure = humidity / 100.0 * saturation * 100
    dry_air_pressure = P - vapor_pressure
    return dry_air_pressure / (R_AIR * T)

# Drag tables (ft/s, Cd) converted from G1 and G7
G1_DRAG_TABLE = [
    (137, 0.262), (228, 0.230), (274, 0.211), (366, 0.203), (457, 0.198),
    (549, 0.195), (640, 0.192), (732, 0.188), (823, 0.185), (914, 0.182),
    (1006, 0.179), (1097, 0.176), (1189, 0.172), (1280, 0.169), (1372, 0.165),
    (1463, 0.162), (1555, 0.159), (1646, 0.156), (1738, 0.152), (1829, 0.149),
    (1920, 0.147), (2012, 0.144), (2103, 0.141)
]

G7_DRAG_TABLE = [
    (137, 0.120), (228, 0.112), (274, 0.107), (366, 0.104), (457, 0.102),
    (549, 0.100), (640, 0.098), (732, 0.095), (823, 0.093), (914, 0.091),
    (1006, 0.089), (1097, 0.087), (1189, 0.085), (1280, 0.083), (1372, 0.081),
    (1463, 0.079), (1555, 0.077), (1646, 0.075), (1738, 0.073), (1829, 0.071),
    (1920, 0.069), (2012, 0.067), (2103, 0.065)
]

def interpolate_cd(velocity, drag_table):
    v_ftps = velocity * 3.28084
    for i in range(len(drag_table) - 1):
        v1, cd1 = drag_table[i]
        v2, cd2 = drag_table[i + 1]
        if v1 <= v_ftps <= v2:
            return cd1 + (cd2 - cd1) * ((v_ftps - v1) / (v2 - v1))
    return drag_table[-1][1]

def transonic_correction(cd, mach):
    if 0.8 <= mach <= 1.2:
        return cd * 1.15  # Increase drag by 15% in transonic zone
    return cd

def simulate_trajectory(bullet, range_m, env, wind_speed=0, wind_angle_deg=0, target_elevation=0):
    g = 9.81
    dt = 0.01
    t = 0.0
    x = 0.0
    y = bullet["sight_height"]
    v = bullet["muzzle_velocity"]

    rho = air_density(env["temp"], env["pressure"], env["humidity"])
    sos = speed_of_sound(env["temp"])
    A = math.pi * (bullet["diameter"] / 2) ** 2
    mass = bullet["weight_gr"] * 0.00006479891
    wind_angle_rad = math.radians(wind_angle_deg)
    drift = 0.0

    drag_table = G1_DRAG_TABLE if bullet["drag_model"] == "G1" else G7_DRAG_TABLE

    while x < range_m and v > 100:
        mach = v / sos
        Cd = interpolate_cd(v, drag_table)
        Cd = transonic_correction(Cd, mach)
        Fd = 0.5 * rho * Cd * A * v**2
        a_drag = Fd / mass
        v -= a_drag * dt
        y -= g * dt
       # Split wind into headwind (affects velocity) and crosswind (affects drift)
        headwind = wind_speed * math.cos(wind_angle_rad)
        crosswind = wind_speed * math.sin(wind_angle_rad)

# Apply headwind to velocity decay more aggressively
        speed += headwind * 0.01  # optional scaling for realism
        if speed > 0:
            drag_coef = drag_lookup(speed, drag_model)
            drag_force = drag_coef * air_density * speed**2
            acceleration = -drag_force / mass
            speed += acceleration * dt

# Apply crosswind for drift
        drift += crosswind * dt

        x += v * dt
        t += dt

    drop = bullet["sight_height"] - y - target_elevation
    mils = (drop / range_m) * 1000
    drift_mils = (drift / range_m) * 1000
    inches = drop * 39.3701
    return {
    "time_of_flight": t,
    "drop_m": drop,
    "drop_in": drop * 39.3701,
    "mil_adjustment": drop * 1000 / range_m,
    "wind_drift_mil": drift * 1000 / range_m
}

BULLETS = {
    "308_M80": {
        "name": ".308 M80 Ball",
        "bc": 0.400,
        "muzzle_velocity": 840,
        "weight_gr": 147,
        "sight_height": 0.06985,
        "diameter": 0.00782,
        "drag_model": "G1"
    },
    "308_SUB": {
        "name": ".308 Subsonic",
        "bc": 0.518,
        "muzzle_velocity": 325,
        "weight_gr": 200,
        "sight_height": 0.06985,
        "diameter": 0.00782,
        "drag_model": "G1"
    },
    "300BLK_SUPER": {
        "name": ".300 Blackout Supersonic",
        "bc": 0.305,
        "muzzle_velocity": 670,
        "weight_gr": 125,
        "sight_height": 0.06985,
        "diameter": 0.00782,
        "drag_model": "G7"
    },
    "300BLK_SUB": {
        "name": ".300 Blackout Subsonic",
        "bc": 0.66,
        "muzzle_velocity": 305,
        "weight_gr": 220,
        "sight_height": 0.06985,
        "diameter": 0.00782,
        "drag_model": "G1"
    },
    "556_M193": {
        "name": "5.56 NATO 55gr M193",
        "bc": 0.243,
        "muzzle_velocity": 990,
        "weight_gr": 55,
        "sight_height": 0.06985,
        "diameter": 0.0057,
        "drag_model": "G1"
    },
    "68BLK_SUPER": {
        "name": "6.8 Blackout Supersonic",
        "bc": 0.37,
        "muzzle_velocity": 780,
        "weight_gr": 110,
        "sight_height": 0.06985,
        "diameter": 0.0068,
        "drag_model": "G7"
    },
    "68BLK_SUB": {
        "name": "6.8 Blackout Subsonic",
        "bc": 0.45,
        "muzzle_velocity": 330,
        "weight_gr": 180,
        "sight_height": 0.06985,
        "diameter": 0.0068,
        "drag_model": "G1"
    }
}
