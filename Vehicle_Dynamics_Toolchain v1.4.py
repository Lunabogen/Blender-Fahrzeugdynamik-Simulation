# Vehicle Dynamics Toolchain for Blender
#
# Datenfluss:
#   Blender-Szene
#     -> road_input.csv + vehicle_parameter.json
#     -> Viertelfahrzeug-Solver
#     -> vehicle_response.csv
#     -> Keyframes fuer Karosserie und Radfederung
#
# Das lokale Referenzsystem ist immer Fahrzeug_Pfadsteuerung-local.
# Der Exporter wertet die Szene nur frameweise aus und stellt danach
# den urspruenglichen Frame wieder her. Erst der Baker schreibt Keyframes.
#
# Nach Aenderungen an Fahrspur, Fahrbahn oder Animation muessen die
# Eingabedateien erneut exportiert und die Pipeline erneut ausgefuehrt werden.

import bpy
import csv
import json
import math
from mathutils import Vector, Matrix
from pathlib import Path


bl_info = {
    "name": "Vehicle Dynamics Toolchain",
    "author": "OpenAI / project integration",
    "version": (1, 3, 0),
    "blender": (3, 6, 0),
    "location": "3D View > Sidebar > Road Input",
    "description": "Export road input, solve vehicle response and bake animation",
    "category": "Animation",
}


# -----------------------------------------------------------------------------
# Gemeinsame Konfiguration: Objektnamen, Dateien und Standardparameter
# -----------------------------------------------------------------------------

CAR_OBJECT_NAME = "Fahrzeug_Pfadsteuerung"
FAHRBAHN_OBJECT_NAME = "Fahrbahn_Kollider"

RAD_OBJECT_NAMES = {
    "VL": "Rad_VorneLinks",
    "VR": "Rad_VorneRechts",
    "HL": "Rad_HintenLinks",
    "HR": "Rad_HintenRechts",
}

RAY_START_OFFSET_M = 2.0
RAY_DISTANCE_M = 5.0


RAD_ENVELOPE_OFFSET_RATIO_LIST = [
    -0.8,
    -0.4,
    0.0,
    0.4,
    0.8,
]

AUTO_RAD_RADIUS = True
RAD_RADIUS_MANUAL_M = 0.32

RAD_RADIUS_MIN_PLAUSIBLE_M = 0.10
RAD_RADIUS_MAX_PLAUSIBLE_M = 1.00

ROAD_INPUT_FILENAME = "road_input.csv"
VEHICLE_PARAMETER_FILENAME = "vehicle_parameter.json"
VEHICLE_RESPONSE_FILENAME = "vehicle_response.csv"

KAROSSERIE_OBJECT_NAME = "Karosserie_Steuerung"
RAD_FEDERUNG_OBJECT_NAMES = {
    "VL": "Rad_VorneLinks_Federung_Steuerung",
    "VR": "Rad_VorneRechts_Federung_Steuerung",
    "HL": "Rad_HintenLinks_Federung_Steuerung",
    "HR": "Rad_HintenRechts_Federung_Steuerung",
}

KAROSSERIE_BASIS_Z_LOCAL_M = 0.8
RAD_FEDERUNG_BASIS_Z_LOCAL_M = {
    "VL": 0.32,
    "VR": 0.32,
    "HL": 0.32,
    "HR": 0.32,
}

DEFAULT_SOLVER_PARAMETER = {
    "FAHRZEUGMASSE_KG": 1600.0,
    "GEFEDERTE_MASSE_ANTEIL": 0.85,
    "UNGEFEDERTEMASSE_PRO_RAD_KG": 45.0,
    "FEDERSTEIFIGKEIT_N_PRO_M": 30000.0,
    "DAEMPFERKONSTANTE_N_S_PRO_M": 3000.0,
    "REIFENSTEIFIGKEIT_N_PRO_M": 200000.0,
    "MAX_EINFEDERUNG_M": 0.035,
    "MAX_AUSFEDERUNG_M": 0.035,
    "LASTVERLAGERUNG_AKTIV": True,
    "LASTVERLAGERUNG_LAENGS_AKTIV": True,
    "LASTVERLAGERUNG_QUER_AKTIV": True,
    "SCHWERPUNKT_HOEHE_M": 0.55,
    "LASTVERLAGERUNG_LAENGS_VORZEICHEN": 1.0,
    "LASTVERLAGERUNG_QUER_VORZEICHEN": 1.0,
    "ROLLEN_SKALIERUNG": 0.5,
    "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_FENSTER_FRAMES": 5,
    "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_SCHWELLE_MPS2": 0.5,
    "LAENGS_BESCHLEUNIGUNG_GLAETTUNGSFENSTER_FRAMES": 5,
    "LAENGS_BESCHLEUNIGUNG_TOTZONE_MPS2": 0.10,
    "BERECHNUNG_UNTERSCHRITTE_PRO_FRAME": 20,
}

# -----------------------------------------------------------------------------
# Exporter: Projektpfade, Objekte, Raycasts und Setup-Validierung
# -----------------------------------------------------------------------------

def get_obj(object_name):
    obj = bpy.data.objects.get(object_name)

    if obj is None:
        raise ValueError("Objekt nicht gefunden: " + object_name)

    return obj


def get_project_directory():
    if bpy.data.filepath == "":
        raise RuntimeError(
            "Die .blend-Datei ist noch nicht gespeichert. "
            "Bitte zuerst die Blender-Datei speichern."
        )

    return Path(bpy.path.abspath("//"))


def get_project_path(filename):
    return get_project_directory() / filename


def get_configured_output_path(context, property_name, default_filename):
    parameter = context.scene.road_input_vehicle_parameter
    configured_path = getattr(parameter, property_name).strip()

    if configured_path == "":
        return get_project_path(default_filename)

    return Path(bpy.path.abspath(configured_path))


def collect_mesh_children(obj):
    mesh_obj_list = []

    if obj.type == "MESH":
        mesh_obj_list.append(obj)

    for child_obj in obj.children:
        mesh_obj_list.extend(
            collect_mesh_children(child_obj)
        )

    return mesh_obj_list


def raycast_fahrbahn_world(
    fahrbahn_eval,
    ray_start_world_vec,
    ray_dir_world_vec,
    ray_distance_m,
):
    fahrbahn_to_world_mat = fahrbahn_eval.matrix_world.copy()
    world_to_fahrbahn_mat = fahrbahn_eval.matrix_world.inverted()

    if ray_dir_world_vec.length == 0.0:
        raise ValueError("Raycast-Richtung darf kein Nullvektor sein.")

    ray_dir_world_vec = ray_dir_world_vec.normalized()

    ray_end_world_vec = (
        ray_start_world_vec
        + ray_dir_world_vec * ray_distance_m
    )

    ray_start_fahrbahn_vec = (
        world_to_fahrbahn_mat @ ray_start_world_vec
    )

    ray_end_fahrbahn_vec = (
        world_to_fahrbahn_mat @ ray_end_world_vec
    )

    ray_vec_fahrbahn = (
        ray_end_fahrbahn_vec - ray_start_fahrbahn_vec
    )

    ray_distance_fahrbahn = ray_vec_fahrbahn.length

    if ray_distance_fahrbahn == 0.0:
        raise ValueError(
            "Raycast-Laenge im Fahrbahn-Koordinatensystem ist null."
        )

    ray_dir_fahrbahn_vec = (
        ray_vec_fahrbahn / ray_distance_fahrbahn
    )

    hit_bool, hit_pos_fahrbahn_vec, hit_normal_fahrbahn_vec, hit_face_index = (
        fahrbahn_eval.ray_cast(
            ray_start_fahrbahn_vec,
            ray_dir_fahrbahn_vec,
            distance=ray_distance_fahrbahn,
        )
    )

    if hit_bool:
        hit_world_vec = fahrbahn_to_world_mat @ hit_pos_fahrbahn_vec

        hit_normal_world_vec = (
            world_to_fahrbahn_mat.to_3x3().transposed()
            @ hit_normal_fahrbahn_vec
        ).normalized()
    else:
        hit_world_vec = None
        hit_normal_world_vec = None

    return (
        hit_bool,
        hit_world_vec,
        hit_normal_world_vec,
        hit_face_index,
    )


OLD_RAYCAST_HANDLER_NAMES = {
    "raycast_federweg_handler",
    "raycast_federweg_v2_handler",
    "raycast_federweg_v3_handler",
}


def validate_required_objects():
    required_names = [
        CAR_OBJECT_NAME,
        FAHRBAHN_OBJECT_NAME,
        *RAD_OBJECT_NAMES.values(),
        KAROSSERIE_OBJECT_NAME,
        *RAD_FEDERUNG_OBJECT_NAMES.values(),
    ]
    missing_names = [
        name for name in required_names
        if bpy.data.objects.get(name) is None
    ]

    if missing_names:
        raise ValueError(
            "Pflichtobjekte fehlen: " + ", ".join(missing_names)
        )

    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)
    if fahrbahn_obj.type != "MESH":
        raise ValueError(
            f"{FAHRBAHN_OBJECT_NAME} muss ein Mesh sein. "
            f"Aktueller Typ: {fahrbahn_obj.type}"
        )


def validate_scene_output_and_parameters(context):
    scene = context.scene

    if scene.frame_end < scene.frame_start:
        raise ValueError("Ungueltiger Frame-Bereich.")

    if scene.render.fps_base == 0.0:
        raise ValueError("FPS Base darf nicht 0 sein.")

    fps = scene.render.fps / scene.render.fps_base
    if not math.isfinite(fps) or fps <= 0.0:
        raise ValueError("FPS muss groesser als 0 sein.")

    if bpy.data.filepath == "":
        raise ValueError(
            "Die .blend-Datei ist noch nicht gespeichert."
        )

    output_paths = [
        (
            "Road Input CSV",
            get_configured_output_path(
                context,
                "road_input_csv_output_path",
                ROAD_INPUT_FILENAME,
            ),
            ".csv",
        ),
        (
            "Vehicle Parameter JSON",
            get_configured_output_path(
                context,
                "vehicle_parameter_json_output_path",
                VEHICLE_PARAMETER_FILENAME,
            ),
            ".json",
        ),
        (
            "Vehicle Response CSV",
            get_configured_output_path(
                context,
                "vehicle_response_csv_output_path",
                VEHICLE_RESPONSE_FILENAME,
            ),
            ".csv",
        ),
    ]

    for label, output_path, expected_suffix in output_paths:
        if output_path.suffix.lower() != expected_suffix:
            raise ValueError(
                f"{label} muss auf {expected_suffix} enden: {output_path}"
            )
        if not output_path.parent.is_dir():
            raise ValueError(
                f"Ausgabeordner fuer {label} ist ungueltig: "
                + str(output_path.parent)
            )

    parameter_data = build_vehicle_parameter_dict(context)
    positive_keys = [
        "FAHRZEUGMASSE_KG",
        "UNGEFEDERTEMASSE_PRO_RAD_KG",
        "FEDERSTEIFIGKEIT_N_PRO_M",
        "DAEMPFERKONSTANTE_N_S_PRO_M",
        "REIFENSTEIFIGKEIT_N_PRO_M",
        "MAX_EINFEDERUNG_M",
        "MAX_AUSFEDERUNG_M",
        "SCHWERPUNKT_HOEHE_M",
    ]
    invalid_keys = [
        key for key in positive_keys
        if (
            not math.isfinite(float(parameter_data[key]))
            or parameter_data[key] <= 0.0
        )
    ]

    if invalid_keys:
        raise ValueError(
            "Ungueltige Fahrzeugparameter: "
            + ", ".join(invalid_keys)
        )

    sprung_mass_ratio = parameter_data["GEFEDERTE_MASSE_ANTEIL"]
    if not 0.0 < sprung_mass_ratio < 1.0:
        raise ValueError(
            "GEFEDERTE_MASSE_ANTEIL muss zwischen 0 und 1 liegen."
        )


def get_active_old_raycast_handler_names():
    return sorted({
        getattr(handler, "__name__", "")
        for handler in bpy.app.handlers.frame_change_post
        if getattr(handler, "__name__", "")
        in OLD_RAYCAST_HANDLER_NAMES
    })


def validate_current_frame_raycast():
    scene = bpy.context.scene
    original_frame = scene.frame_current
    warnings = []

    try:
        static_data = read_static_vehicle_geometry(original_frame)

        positive_geometry_keys = [
            "car_spurweite_front_m",
            "car_spurweite_rear_m",
            "car_radstand_left_m",
            "car_radstand_right_m",
        ]
        invalid_geometry = [
            f"{key}={static_data[key]}"
            for key in positive_geometry_keys
            if (
                not math.isfinite(float(static_data[key]))
                or static_data[key] <= 0.0
            )
        ]
        if invalid_geometry:
            raise ValueError(
                "Ungueltige Fahrzeuggeometrie: "
                + ", ".join(invalid_geometry)
            )

        invalid_radii = []
        for rad_key in RAD_OBJECT_NAMES:
            radius = static_data[
                "rad_radius_used_" + rad_key + "_m"
            ]
            if (
                radius is None
                or not math.isfinite(float(radius))
                or not (
                    RAD_RADIUS_MIN_PLAUSIBLE_M
                    <= radius
                    <= RAD_RADIUS_MAX_PLAUSIBLE_M
                )
            ):
                invalid_radii.append(
                    rad_key + "=" + str(radius)
                )

        if invalid_radii:
            raise ValueError(
                "Unplausible Rad-Radien: "
                + ", ".join(invalid_radii)
            )

        depsgraph = bpy.context.evaluated_depsgraph_get()
        car_eval = get_obj(CAR_OBJECT_NAME).evaluated_get(depsgraph)
        fahrbahn_eval = get_obj(
            FAHRBAHN_OBJECT_NAME
        ).evaluated_get(depsgraph)

        center_data = read_rad_center_fahrbahn_current_frame(
            car_eval, fahrbahn_eval, depsgraph
        )
        envelope_data = read_rad_envelope_fahrbahn_current_frame(
            car_eval,
            fahrbahn_eval,
            depsgraph,
            static_data,
        )

        raycast_errors = []
        expected_hits = len(RAD_ENVELOPE_OFFSET_RATIO_LIST)

        for rad_key in RAD_OBJECT_NAMES:
            center_hit = center_data[rad_key][
                "rad_center_hit_bool"
            ]
            envelope_hits = envelope_data[rad_key][
                "rad_envelope_hit_count"
            ]
            required_z = envelope_data[rad_key][
                "rad_envelope_required_max_z_local_m"
            ]

            if not center_hit:
                raycast_errors.append(
                    "Center-Raycast " + rad_key + " ohne Hit"
                )

            if envelope_hits <= 0 or required_z is None:
                raycast_errors.append(
                    "Multi-Raycast " + rad_key + " ohne Hit"
                )
            elif envelope_hits < expected_hits:
                warnings.append(
                    f"Multi-Raycast {rad_key}: "
                    f"nur {envelope_hits}/{expected_hits} Hits."
                )

            print(
                f"Rad {rad_key} | center_hit={center_hit} | "
                f"envelope_hits={envelope_hits} | "
                f"required_z_local_m={required_z}"
            )

        if raycast_errors:
            raise ValueError("; ".join(raycast_errors))

    finally:
        if scene.frame_current != original_frame:
            scene.frame_set(original_frame)

    return warnings


def validate_export_setup(context):
    passed_checks = []
    warnings = []
    errors = []

    checks = [
        ("Pflichtobjekte", validate_required_objects),
        (
            "Szene, Ausgabe und Parameter",
            lambda: validate_scene_output_and_parameters(context),
        ),
        ("Geometrie und Raycast", validate_current_frame_raycast),
    ]

    for check_name, check_function in checks:
        try:
            check_warnings = check_function()
            passed_checks.append(check_name)
            if check_warnings:
                warnings.extend(check_warnings)
        except Exception as error:
            errors.append(check_name + ": " + str(error))

    old_handler_names = get_active_old_raycast_handler_names()
    if old_handler_names:
        warnings.append(
            "Alte Raycast-Federweg-Handler aktiv: "
            + ", ".join(old_handler_names)
            + ". Sie wurden nicht entfernt."
        )
    else:
        passed_checks.append("Raycast-Handler")

    print("============================================================")
    print("Road Input Exporter - Validate Setup")
    print("============================================================")

    for message in passed_checks:
        print("[OK]", message)
    for message in warnings:
        print("[WARNUNG]", message)
    for message in errors:
        print("[FEHLER]", message)

    print(
        f"Ergebnis: {len(passed_checks)} bestanden, "
        f"{len(warnings)} Warnungen, {len(errors)} Fehler."
    )
    print("============================================================")

    if errors:
        raise ValueError(
            "Setup-Validierung fehlgeschlagen:\n- "
            + "\n- ".join(errors)
        )

    return {
        "passed_count": len(passed_checks),
        "warnings": warnings,
    }


# Radmittelpunkt-Raycast fuer einen Frame

def read_rad_center_fahrbahn_current_frame(
    car_eval,
    fahrbahn_eval,
    depsgraph,
):
    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    rad_center_fahrbahn_data = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        rad_center_ray_start_world_vec = (
            rad_center_pos_world_vec
            + car_up_world_vec * RAY_START_OFFSET_M
        )

        rad_center_ray_dir_world_vec = -car_up_world_vec

        (
            rad_center_hit_bool,
            rad_center_hit_world_vec,
            rad_center_hit_normal_world_vec,
            rad_center_hit_face_index,
        ) = raycast_fahrbahn_world(
            fahrbahn_eval,
            rad_center_ray_start_world_vec,
            rad_center_ray_dir_world_vec,
            RAY_DISTANCE_M,
        )

        if rad_center_hit_bool:
            rad_center_hit_local_vec = (
                car_world_to_local_mat @ rad_center_hit_world_vec
            )

            rad_center_fahrbahn_z_local_m = rad_center_hit_local_vec.z
        else:
            rad_center_hit_local_vec = None
            rad_center_fahrbahn_z_local_m = None

        rad_center_fahrbahn_data[rad_key] = {
            "rad_object_name": rad_object_name,

            "rad_center_pos_world_vec": rad_center_pos_world_vec,
            "rad_center_pos_local_vec": rad_center_pos_local_vec,

            "rad_center_ray_start_world_vec": rad_center_ray_start_world_vec,
            "rad_center_ray_dir_world_vec": rad_center_ray_dir_world_vec,

            "rad_center_hit_bool": rad_center_hit_bool,
            "rad_center_hit_world_vec": rad_center_hit_world_vec,
            "rad_center_hit_local_vec": rad_center_hit_local_vec,
            "rad_center_hit_normal_world_vec": rad_center_hit_normal_world_vec,
            "rad_center_hit_face_index": rad_center_hit_face_index,

            "rad_center_fahrbahn_z_local_m": rad_center_fahrbahn_z_local_m,
        }

    return rad_center_fahrbahn_data

# Fuenfpunkt-Reifenhuellen-Raycast fuer einen Frame

def read_rad_envelope_fahrbahn_current_frame(
    car_eval,
    fahrbahn_eval,
    depsgraph,
    static_geometry_data,
):
    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    rad_envelope_fahrbahn_data = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_radius_used_m = static_geometry_data[
            "rad_radius_used_" + rad_key + "_m"
        ]

        if rad_radius_used_m is None or rad_radius_used_m <= 0.0:
            raise Exception(
                "Ungueltiger Rad-Radius fuer "
                + rad_key
                + ": "
                + str(rad_radius_used_m)
            )

        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_center_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_center_pos_local_vec = car_world_to_local_mat @ rad_center_pos_world_vec

        rad_envelope_sample_data_list = []

        rad_envelope_hit_count = 0
        rad_envelope_required_max_z_local_m = None
        rad_envelope_contact_offset_y_local_m = None

        for rad_envelope_offset_ratio in RAD_ENVELOPE_OFFSET_RATIO_LIST:
            rad_envelope_offset_y_local_m = (
                rad_envelope_offset_ratio * rad_radius_used_m
            )

            rad_envelope_vertical_radius_m = math.sqrt(
                max(
                    0.0,
                    rad_radius_used_m * rad_radius_used_m
                    - rad_envelope_offset_y_local_m
                    * rad_envelope_offset_y_local_m,
                )
            )

            rad_envelope_sample_pos_local_vec = (
                rad_center_pos_local_vec
                + Vector((0.0, rad_envelope_offset_y_local_m, 0.0))
            )

            rad_envelope_sample_pos_world_vec = (
                car_local_to_world_mat @ rad_envelope_sample_pos_local_vec
            )

            rad_envelope_ray_start_world_vec = (
                rad_envelope_sample_pos_world_vec
                + car_up_world_vec * RAY_START_OFFSET_M
            )

            rad_envelope_ray_dir_world_vec = -car_up_world_vec

            (
                rad_envelope_hit_bool,
                rad_envelope_hit_world_vec,
                rad_envelope_hit_normal_world_vec,
                rad_envelope_hit_face_index,
            ) = raycast_fahrbahn_world(
                fahrbahn_eval,
                rad_envelope_ray_start_world_vec,
                rad_envelope_ray_dir_world_vec,
                RAY_DISTANCE_M,
            )

            if rad_envelope_hit_bool:
                rad_envelope_hit_count += 1

                rad_envelope_hit_local_vec = (
                    car_world_to_local_mat @ rad_envelope_hit_world_vec
                )

                rad_envelope_fahrbahn_z_local_m = (
                    rad_envelope_hit_local_vec.z
                )

                rad_envelope_required_z_local_m = (
                    rad_envelope_fahrbahn_z_local_m
                    + rad_envelope_vertical_radius_m
                )

                if (
                    rad_envelope_required_max_z_local_m is None
                    or rad_envelope_required_z_local_m
                    > rad_envelope_required_max_z_local_m
                ):
                    rad_envelope_required_max_z_local_m = (
                        rad_envelope_required_z_local_m
                    )

                    rad_envelope_contact_offset_y_local_m = (
                        rad_envelope_offset_y_local_m
                    )

            else:
                rad_envelope_hit_local_vec = None
                rad_envelope_fahrbahn_z_local_m = None
                rad_envelope_required_z_local_m = None

            rad_envelope_sample_data_list.append(
                {
                    "rad_envelope_offset_ratio": rad_envelope_offset_ratio,
                    "rad_envelope_offset_y_local_m": rad_envelope_offset_y_local_m,
                    "rad_envelope_vertical_radius_m": rad_envelope_vertical_radius_m,

                    "rad_envelope_sample_pos_local_vec": rad_envelope_sample_pos_local_vec,
                    "rad_envelope_sample_pos_world_vec": rad_envelope_sample_pos_world_vec,

                    "rad_envelope_ray_start_world_vec": rad_envelope_ray_start_world_vec,
                    "rad_envelope_ray_dir_world_vec": rad_envelope_ray_dir_world_vec,

                    "rad_envelope_hit_bool": rad_envelope_hit_bool,
                    "rad_envelope_hit_world_vec": rad_envelope_hit_world_vec,
                    "rad_envelope_hit_local_vec": rad_envelope_hit_local_vec,
                    "rad_envelope_hit_normal_world_vec": rad_envelope_hit_normal_world_vec,
                    "rad_envelope_hit_face_index": rad_envelope_hit_face_index,

                    "rad_envelope_fahrbahn_z_local_m": rad_envelope_fahrbahn_z_local_m,
                    "rad_envelope_required_z_local_m": rad_envelope_required_z_local_m,
                }
            )

        rad_envelope_fahrbahn_data[rad_key] = {
            "rad_object_name": rad_object_name,

            "rad_center_pos_world_vec": rad_center_pos_world_vec,
            "rad_center_pos_local_vec": rad_center_pos_local_vec,

            "rad_envelope_hit_count": rad_envelope_hit_count,
            "rad_envelope_required_max_z_local_m": rad_envelope_required_max_z_local_m,
            "rad_envelope_contact_offset_y_local_m": rad_envelope_contact_offset_y_local_m,

            "rad_envelope_sample_data_list": rad_envelope_sample_data_list,
        }

    return rad_envelope_fahrbahn_data


# Statische Fahrzeuggeometrie und automatische Radradius-Ermittlung

def estimate_rad_radius_from_mesh(rad_key, rad_object_name, depsgraph):
    rad_drehung_obj = get_obj(rad_object_name)
    rad_drehung_eval = rad_drehung_obj.evaluated_get(depsgraph)

    # Die Objektskalierung muss in die Radiusmessung eingehen. Deshalb
    # verwendet das Mess-Koordinatensystem nur Translation und Rotation;
    # matrix_world.inverted() wuerde die Radskalierung herauskuerzen.
    rad_drehung_pos_world_vec = (
        rad_drehung_eval.matrix_world.translation.copy()
    )

    rad_drehung_rot_world_mat = (
        rad_drehung_eval.matrix_world
        .to_quaternion()
        .to_matrix()
        .to_4x4()
    )

    rad_drehung_pos_world_mat = Matrix.Translation(
        rad_drehung_pos_world_vec
    )

    rad_drehung_to_world_ohne_skala_mat = (
        rad_drehung_pos_world_mat
        @ rad_drehung_rot_world_mat
    )

    world_to_rad_drehung_mat = (
        rad_drehung_to_world_ohne_skala_mat.inverted()
    )

    wheel_mesh_obj_list = collect_mesh_children(rad_drehung_obj)

    if len(wheel_mesh_obj_list) == 0:
        print(
            "WARNUNG: Kein Mesh unter Rad gefunden:",
            rad_key,
            "=",
            rad_object_name,
        )
        return None

    rad_radius_auto_m = None

    for wheel_mesh_obj in wheel_mesh_obj_list:
        wheel_mesh_eval = wheel_mesh_obj.evaluated_get(depsgraph)

        if wheel_mesh_eval.type != "MESH":
            continue

        wheel_mesh_data = wheel_mesh_eval.data

        for vertex in wheel_mesh_data.vertices:
            vertex_pos_world_vec = (
                wheel_mesh_eval.matrix_world @ vertex.co
            )

            vertex_pos_rad_drehung_vec = (
                world_to_rad_drehung_mat @ vertex_pos_world_vec
            )

            radius_candidate_m = math.sqrt(
                vertex_pos_rad_drehung_vec.y
                * vertex_pos_rad_drehung_vec.y
                + vertex_pos_rad_drehung_vec.z
                * vertex_pos_rad_drehung_vec.z
            )

            if (
                rad_radius_auto_m is None
                or radius_candidate_m > rad_radius_auto_m
            ):
                rad_radius_auto_m = radius_candidate_m

    if rad_radius_auto_m is None:
        print(
            "WARNUNG: Rad Radius konnte nicht berechnet werden:",
            rad_key,
            "=",
            rad_object_name,
        )
        return None

    if (
        rad_radius_auto_m < RAD_RADIUS_MIN_PLAUSIBLE_M
        or rad_radius_auto_m > RAD_RADIUS_MAX_PLAUSIBLE_M
    ):
        print(
            "WARNUNG: Rad Radius wirkt unplausibel:",
            rad_key,
            "=",
            round(rad_radius_auto_m, 4),
            "m",
        )

    print(
        "rad_radius_auto_" + rad_key + "_m:",
        round(rad_radius_auto_m, 6),
    )

    return rad_radius_auto_m


def read_static_vehicle_geometry(static_geometry_frame):
    scene = bpy.context.scene
    scene.frame_set(static_geometry_frame)

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    car_eval = car_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()
    car_world_to_local_mat = car_local_to_world_mat.inverted()

    rad_pos_local = {}

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_obj = get_obj(rad_object_name)
        rad_eval = rad_obj.evaluated_get(depsgraph)

        rad_pos_world_vec = rad_eval.matrix_world.translation.copy()
        rad_pos_local_vec = car_world_to_local_mat @ rad_pos_world_vec

        rad_pos_local[rad_key] = rad_pos_local_vec

    vl = rad_pos_local["VL"]
    vr = rad_pos_local["VR"]
    hl = rad_pos_local["HL"]
    hr = rad_pos_local["HR"]

    car_rad_center_x_local_m = (
        vl.x + vr.x + hl.x + hr.x
    ) / 4.0

    car_rad_center_y_local_m = (
        vl.y + vr.y + hl.y + hr.y
    ) / 4.0

    car_spurweite_front_m = abs(vr.x - vl.x)
    car_spurweite_rear_m = abs(hr.x - hl.x)

    car_spurweite_mean_m = (
        car_spurweite_front_m
        + car_spurweite_rear_m
    ) / 2.0

    car_radstand_left_m = abs(vl.y - hl.y)
    car_radstand_right_m = abs(vr.y - hr.y)

    car_radstand_mean_m = (
        car_radstand_left_m
        + car_radstand_right_m
    ) / 2.0

    static_geometry_data = {}

    static_geometry_data["static_geometry_frame"] = static_geometry_frame

    static_geometry_data["car_spurweite_front_m"] = car_spurweite_front_m
    static_geometry_data["car_spurweite_rear_m"] = car_spurweite_rear_m
    static_geometry_data["car_spurweite_mean_m"] = car_spurweite_mean_m

    static_geometry_data["car_radstand_left_m"] = car_radstand_left_m
    static_geometry_data["car_radstand_right_m"] = car_radstand_right_m
    static_geometry_data["car_radstand_mean_m"] = car_radstand_mean_m

    for rad_key, rad_object_name in RAD_OBJECT_NAMES.items():
        rad_radius_auto_m = estimate_rad_radius_from_mesh(
            rad_key,
            rad_object_name,
            depsgraph,
        )

        if AUTO_RAD_RADIUS and rad_radius_auto_m is not None:
            rad_radius_used_m = rad_radius_auto_m
        else:
            rad_radius_used_m = RAD_RADIUS_MANUAL_M

        static_geometry_data[
            "rad_radius_auto_" + rad_key + "_m"
        ] = rad_radius_auto_m

        static_geometry_data[
            "rad_radius_used_" + rad_key + "_m"
        ] = rad_radius_used_m

    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_pos_local_vec = rad_pos_local[rad_key]

        static_geometry_data[
            "rad_offset_x_local_" + rad_key + "_m"
        ] = rad_pos_local_vec.x - car_rad_center_x_local_m

        static_geometry_data[
            "rad_offset_y_local_" + rad_key + "_m"
        ] = rad_pos_local_vec.y - car_rad_center_y_local_m

    print("============================================================")
    print("Road Input Exporter - Static Fahrzeuggeometrie")
    print("============================================================")
    print("static_geometry_frame:", static_geometry_frame)
    print("car_spurweite_front_m:", round(car_spurweite_front_m, 4))
    print("car_spurweite_rear_m:", round(car_spurweite_rear_m, 4))
    print("car_radstand_left_m:", round(car_radstand_left_m, 4))
    print("car_radstand_right_m:", round(car_radstand_right_m, 4))

    for rad_key in RAD_OBJECT_NAMES.keys():
        print(
            "rad_radius_auto_" + rad_key + "_m:",
            static_geometry_data["rad_radius_auto_" + rad_key + "_m"],
        )
        print(
            "rad_radius_used_" + rad_key + "_m:",
            static_geometry_data["rad_radius_used_" + rad_key + "_m"],
        )

    print("============================================================")

    return static_geometry_data


# Vollstaendigen Rohdatensatz fuer einen Frame erfassen

def read_groessen_current_frame(
    frame,
    static_geometry_data,
):
    bpy.context.scene.frame_set(frame)

    depsgraph = bpy.context.evaluated_depsgraph_get()

    car_obj = get_obj(CAR_OBJECT_NAME)
    fahrbahn_obj = get_obj(FAHRBAHN_OBJECT_NAME)

    car_eval = car_obj.evaluated_get(depsgraph)
    fahrbahn_eval = fahrbahn_obj.evaluated_get(depsgraph)

    car_local_to_world_mat = car_eval.matrix_world.copy()

    car_pos_world_vec = car_local_to_world_mat.translation.copy()

    car_forward_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 1.0, 0.0))
    ).normalized()

    car_right_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((1.0, 0.0, 0.0))
    ).normalized()

    car_up_world_vec = (
        car_local_to_world_mat.to_3x3() @ Vector((0.0, 0.0, 1.0))
    ).normalized()

    car_yaw_rad = math.atan2(
        car_forward_world_vec.x,
        car_forward_world_vec.y,
    )

    scene = bpy.context.scene
    fps = scene.render.fps / scene.render.fps_base

    if fps <= 0.0:
        raise Exception("Die Blender-Framerate muss groesser als 0 sein.")

    t_s = (frame - scene.frame_start) / fps

    if frame == scene.frame_start:
        dt_s = 0.0
    else:
        dt_s = 1.0 / fps

    rad_center_fahrbahn_data = read_rad_center_fahrbahn_current_frame(
        car_eval,
        fahrbahn_eval,
        depsgraph,
    )

    rad_envelope_fahrbahn_data = read_rad_envelope_fahrbahn_current_frame(
        car_eval,
        fahrbahn_eval,
        depsgraph,
        static_geometry_data,
    )

    frame_data = {}

    frame_data["frame"] = frame
    frame_data["t_s"] = t_s
    frame_data["dt_s"] = dt_s

    frame_data["car_pos_x_world_m"] = car_pos_world_vec.x
    frame_data["car_pos_y_world_m"] = car_pos_world_vec.y
    frame_data["car_pos_z_world_m"] = car_pos_world_vec.z

    frame_data["car_yaw_rad"] = car_yaw_rad
    frame_data["car_forward_x_world"] = car_forward_world_vec.x
    frame_data["car_forward_y_world"] = car_forward_world_vec.y
    frame_data["car_forward_z_world"] = car_forward_world_vec.z

    frame_data["car_right_x_world"] = car_right_world_vec.x
    frame_data["car_right_y_world"] = car_right_world_vec.y
    frame_data["car_right_z_world"] = car_right_world_vec.z

    frame_data["car_up_x_world"] = car_up_world_vec.x
    frame_data["car_up_y_world"] = car_up_world_vec.y
    frame_data["car_up_z_world"] = car_up_world_vec.z


    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_center_data = rad_center_fahrbahn_data[rad_key]
        rad_envelope_data = rad_envelope_fahrbahn_data[rad_key]

        frame_data[
            "rad_center_hit_" + rad_key + "_bool"
        ] = rad_center_data["rad_center_hit_bool"]

        frame_data[
            "rad_center_fahrbahn_z_local_" + rad_key + "_m"
        ] = rad_center_data["rad_center_fahrbahn_z_local_m"]

        frame_data[
            "rad_envelope_hit_count_" + rad_key
        ] = rad_envelope_data["rad_envelope_hit_count"]

        frame_data[
            "rad_envelope_required_max_z_local_" + rad_key + "_m"
        ] = rad_envelope_data["rad_envelope_required_max_z_local_m"]

        frame_data[
            "rad_envelope_contact_offset_y_local_" + rad_key + "_m"
        ] = rad_envelope_data["rad_envelope_contact_offset_y_local_m"]

    return frame_data

# Rohdaten ueber den gesamten Frame-Bereich erfassen

def read_groessen_all_frames():
    scene = bpy.context.scene

    frame_start = scene.frame_start
    frame_end = scene.frame_end
    frame_current_before_export = scene.frame_current

    frame_data_list = []

    try:
        static_geometry_data = read_static_vehicle_geometry(frame_start)

        print("============================================================")
        print("Road Input Exporter - Read Groessen All Frames")
        print("============================================================")
        print("frame_start:", frame_start)
        print("frame_end:", frame_end)

        for frame in range(frame_start, frame_end + 1):
            frame_data = read_groessen_current_frame(
                frame,
                static_geometry_data,
            )

            frame_data.update(static_geometry_data)

            frame_data_list.append(frame_data)

            print(
                "Frame gelesen:",
                frame,
                "/",
                frame_end,
            )

    finally:
        scene.frame_set(frame_current_before_export)

    print("============================================================")
    print("FERTIG: Read Groessen All Frames funktioniert.")
    print("Gelesene Frames:", len(frame_data_list))
    print("============================================================")

    return frame_data_list

# Geschwindigkeiten, Beschleunigungen und Gierraten berechnen

def unwrap_yaw_angle_rad(yaw_rad, previous_yaw_unwrapped_rad):
    yaw_unwrapped_rad = yaw_rad

    while yaw_unwrapped_rad - previous_yaw_unwrapped_rad > math.pi:
        yaw_unwrapped_rad -= 2.0 * math.pi

    while yaw_unwrapped_rad - previous_yaw_unwrapped_rad < -math.pi:
        yaw_unwrapped_rad += 2.0 * math.pi

    return yaw_unwrapped_rad


def compute_scalar_derivative(frame_data_list, index, value_key):
    frame_count = len(frame_data_list)

    if frame_count <= 1:
        return 0.0

    if index == 0:
        previous_data = frame_data_list[index]
        next_data = frame_data_list[index + 1]

    elif index == frame_count - 1:
        previous_data = frame_data_list[index - 1]
        next_data = frame_data_list[index]

    else:
        previous_data = frame_data_list[index - 1]
        next_data = frame_data_list[index + 1]

    previous_value = previous_data[value_key]
    next_value = next_data[value_key]

    previous_t_s = previous_data["t_s"]
    next_t_s = next_data["t_s"]

    dt_s = next_t_s - previous_t_s

    if dt_s == 0.0:
        return 0.0

    derivative_value = (next_value - previous_value) / dt_s

    return derivative_value


def compute_car_kinematics(frame_data_list):
    print("============================================================")
    print("Road Input Exporter - Compute Car Kinematics")
    print("============================================================")

    previous_yaw_unwrapped_rad = None

    for frame_data in frame_data_list:
        car_yaw_rad = frame_data["car_yaw_rad"]

        if previous_yaw_unwrapped_rad is None:
            car_yaw_unwrapped_rad = car_yaw_rad
        else:
            car_yaw_unwrapped_rad = unwrap_yaw_angle_rad(
                car_yaw_rad,
                previous_yaw_unwrapped_rad,
            )

        frame_data["car_yaw_unwrapped_rad"] = car_yaw_unwrapped_rad
        previous_yaw_unwrapped_rad = car_yaw_unwrapped_rad

    for index, frame_data in enumerate(frame_data_list):
        car_vel_x_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_x_world_m",
        )

        car_vel_y_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_y_world_m",
        )

        car_vel_z_world_mps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_pos_z_world_m",
        )

        car_vel_world_vec = Vector(
            (
                car_vel_x_world_mps,
                car_vel_y_world_mps,
                car_vel_z_world_mps,
            )
        )

        car_vel_abs_mps = car_vel_world_vec.length

        car_forward_world_vec = Vector(
            (
                frame_data["car_forward_x_world"],
                frame_data["car_forward_y_world"],
                frame_data["car_forward_z_world"],
            )
        ).normalized()

        car_right_world_vec = Vector(
            (
                frame_data["car_right_x_world"],
                frame_data["car_right_y_world"],
                frame_data["car_right_z_world"],
            )
        ).normalized()

        car_up_world_vec = Vector(
            (
                frame_data["car_up_x_world"],
                frame_data["car_up_y_world"],
                frame_data["car_up_z_world"],
            )
        ).normalized()

        car_vel_forward_local_mps = car_vel_world_vec.dot(
            car_forward_world_vec
        )

        car_vel_lateral_local_mps = car_vel_world_vec.dot(
            car_right_world_vec
        )

        car_vel_vertical_local_mps = car_vel_world_vec.dot(
            car_up_world_vec
        )

        frame_data["car_vel_x_world_mps"] = car_vel_x_world_mps
        frame_data["car_vel_y_world_mps"] = car_vel_y_world_mps
        frame_data["car_vel_z_world_mps"] = car_vel_z_world_mps

        frame_data["car_vel_abs_mps"] = car_vel_abs_mps

        frame_data["car_vel_forward_local_mps"] = car_vel_forward_local_mps
        frame_data["car_vel_lateral_local_mps"] = car_vel_lateral_local_mps
        frame_data["car_vel_vertical_local_mps"] = car_vel_vertical_local_mps

        car_yaw_rate_radps = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_yaw_unwrapped_rad",
        )

        frame_data["car_yaw_rate_radps"] = car_yaw_rate_radps

    for index, frame_data in enumerate(frame_data_list):
        car_acc_x_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_x_world_mps",
        )

        car_acc_y_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_y_world_mps",
        )

        car_acc_z_world_mps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_vel_z_world_mps",
        )

        car_acc_world_vec = Vector(
            (
                car_acc_x_world_mps2,
                car_acc_y_world_mps2,
                car_acc_z_world_mps2,
            )
        )

        car_forward_world_vec = Vector(
            (
                frame_data["car_forward_x_world"],
                frame_data["car_forward_y_world"],
                frame_data["car_forward_z_world"],
            )
        ).normalized()

        car_right_world_vec = Vector(
            (
                frame_data["car_right_x_world"],
                frame_data["car_right_y_world"],
                frame_data["car_right_z_world"],
            )
        ).normalized()

        car_up_world_vec = Vector(
            (
                frame_data["car_up_x_world"],
                frame_data["car_up_y_world"],
                frame_data["car_up_z_world"],
            )
        ).normalized()

        car_acc_forward_local_mps2 = car_acc_world_vec.dot(
            car_forward_world_vec
        )

        car_acc_lateral_local_mps2 = car_acc_world_vec.dot(
            car_right_world_vec
        )

        car_acc_vertical_local_mps2 = car_acc_world_vec.dot(
            car_up_world_vec
        )

        frame_data["car_acc_x_world_mps2"] = car_acc_x_world_mps2
        frame_data["car_acc_y_world_mps2"] = car_acc_y_world_mps2
        frame_data["car_acc_z_world_mps2"] = car_acc_z_world_mps2

        frame_data["car_acc_forward_local_mps2"] = car_acc_forward_local_mps2
        frame_data["car_acc_lateral_local_mps2"] = car_acc_lateral_local_mps2
        frame_data["car_acc_vertical_local_mps2"] = car_acc_vertical_local_mps2

        car_yaw_acc_radps2 = compute_scalar_derivative(
            frame_data_list,
            index,
            "car_yaw_rate_radps",
        )

        frame_data["car_yaw_acc_radps2"] = car_yaw_acc_radps2

    print("FERTIG: Compute Car Kinematics funktioniert.")
    print("============================================================")

    return frame_data_list

# Strassenanregung relativ zum Referenzframe berechnen

def compute_relative_road_input(frame_data_list):
    print("============================================================")
    print("Road Input Exporter - Compute Relative Road Input")
    print("============================================================")

    if len(frame_data_list) == 0:
        raise Exception("frame_data_list ist leer.")

    first_frame_data = frame_data_list[0]

    for rad_key in RAD_OBJECT_NAMES.keys():
        rad_center_z_key = (
            "rad_center_fahrbahn_z_local_" + rad_key + "_m"
        )

        rad_center_z0_key = (
            "rad_center_fahrbahn_z0_local_" + rad_key + "_m"
        )

        rad_center_z_rel_key = (
            "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m"
        )

        rad_envelope_z_key = (
            "rad_envelope_required_max_z_local_" + rad_key + "_m"
        )

        rad_envelope_z0_key = (
            "rad_envelope_required_max_z0_local_" + rad_key + "_m"
        )

        rad_envelope_z_rel_key = (
            "rad_envelope_required_max_z_rel_local_" + rad_key + "_m"
        )

        rad_center_z0_local_m = first_frame_data[rad_center_z_key]
        rad_envelope_z0_local_m = first_frame_data[rad_envelope_z_key]

        for frame_data in frame_data_list:
            rad_center_z_local_m = frame_data[rad_center_z_key]
            rad_envelope_z_local_m = frame_data[rad_envelope_z_key]

            if (
                rad_center_z_local_m is None
                and rad_envelope_z_local_m is None
            ):
                raise Exception(
                    "Kein Fahrbahn-Ray-Hit fuer Rad "
                    + rad_key
                    + " in Frame "
                    + str(frame_data["frame"])
                    + "."
                )

            frame_data[rad_center_z0_key] = rad_center_z0_local_m
            frame_data[rad_envelope_z0_key] = rad_envelope_z0_local_m

            if (
                rad_center_z_local_m is not None
                and rad_center_z0_local_m is not None
            ):
                frame_data[rad_center_z_rel_key] = (
                    rad_center_z_local_m
                    - rad_center_z0_local_m
                )
            else:
                frame_data[rad_center_z_rel_key] = None

            if (
                rad_envelope_z_local_m is not None
                and rad_envelope_z0_local_m is not None
            ):
                frame_data[rad_envelope_z_rel_key] = (
                    rad_envelope_z_local_m
                    - rad_envelope_z0_local_m
                )
            else:
                frame_data[rad_envelope_z_rel_key] = None

    print("FERTIG: Compute Relative Road Input funktioniert.")
    print("============================================================")

    return frame_data_list

# road_input.csv aufbauen und sicher schreiben

def build_csv_fieldnames():
    fieldnames = [
        "frame",
        "t_s",
        "dt_s",

        "car_pos_x_world_m",
        "car_pos_y_world_m",
        "car_pos_z_world_m",

        "car_yaw_rad",
        "car_yaw_unwrapped_rad",
        "car_yaw_rate_radps",
        "car_yaw_acc_radps2",

        "car_forward_x_world",
        "car_forward_y_world",
        "car_forward_z_world",

        "car_right_x_world",
        "car_right_y_world",
        "car_right_z_world",

        "car_up_x_world",
        "car_up_y_world",
        "car_up_z_world",

        "static_geometry_frame",

        "car_spurweite_front_m",
        "car_spurweite_rear_m",
        "car_spurweite_mean_m",

        "car_radstand_left_m",
        "car_radstand_right_m",
        "car_radstand_mean_m",

        "car_vel_x_world_mps",
        "car_vel_y_world_mps",
        "car_vel_z_world_mps",
        "car_vel_abs_mps",

        "car_vel_forward_local_mps",
        "car_vel_lateral_local_mps",
        "car_vel_vertical_local_mps",

        "car_acc_x_world_mps2",
        "car_acc_y_world_mps2",
        "car_acc_z_world_mps2",

        "car_acc_forward_local_mps2",
        "car_acc_lateral_local_mps2",
        "car_acc_vertical_local_mps2",
    ]

    for rad_key in RAD_OBJECT_NAMES.keys():
        fieldnames.extend(
            [
                "rad_offset_x_local_" + rad_key + "_m",
                "rad_offset_y_local_" + rad_key + "_m",

                "rad_radius_auto_" + rad_key + "_m",
                "rad_radius_used_" + rad_key + "_m",

                "rad_center_hit_" + rad_key + "_bool",

                "rad_center_fahrbahn_z_local_" + rad_key + "_m",
                "rad_center_fahrbahn_z0_local_" + rad_key + "_m",
                "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m",

                "rad_envelope_hit_count_" + rad_key,

                "rad_envelope_required_max_z_local_" + rad_key + "_m",
                "rad_envelope_required_max_z0_local_" + rad_key + "_m",
                "rad_envelope_required_max_z_rel_local_" + rad_key + "_m",

                "rad_envelope_contact_offset_y_local_" + rad_key + "_m",
            ]
        )

    return fieldnames

def write_road_input_csv(frame_data_list, context):
    if len(frame_data_list) == 0:
        raise ValueError("frame_data_list ist leer. CSV kann nicht geschrieben werden.")

    csv_path = get_configured_output_path(
        context,
        "road_input_csv_output_path",
        ROAD_INPUT_FILENAME,
    )

    fieldnames = build_csv_fieldnames()

    with open(csv_path, mode="w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )

        writer.writeheader()

        for frame_data in frame_data_list:
            writer.writerow(frame_data)

    print("============================================================")
    print("Road Input Exporter - CSV Export")
    print("============================================================")
    print("CSV geschrieben:")
    print(csv_path)
    print("Anzahl Frames:", len(frame_data_list))
    print("Anzahl Spalten:", len(fieldnames))
    print("============================================================")

    return csv_path

# Exporter-Pipeline

def export_road_input_csv(context):
    print("============================================================")
    print("Road Input Exporter - Full Export Pipeline")
    print("============================================================")

    frame_data_list = read_groessen_all_frames()

    frame_data_list = compute_car_kinematics(frame_data_list)

    frame_data_list = compute_relative_road_input(frame_data_list)

    csv_path = write_road_input_csv(frame_data_list, context)

    print("============================================================")
    print("FERTIG: Road Input Export abgeschlossen.")
    print("CSV:", csv_path)
    print("============================================================")

    return csv_path


# -----------------------------------------------------------------------------
# Solverparameter: Blender-Eigenschaften und vehicle_parameter.json
# -----------------------------------------------------------------------------
# Modellparameter bleiben von den gemessenen Daten in road_input.csv getrennt.

class ROADINPUT_VehicleParameter(bpy.types.PropertyGroup):
    road_input_csv_output_path: bpy.props.StringProperty(
        name="Road Input CSV",
        description="Ausgabedatei fuer die exportierte Strassenanregung",
        default="//road_input.csv",
        subtype="FILE_PATH",
    )

    vehicle_parameter_json_output_path: bpy.props.StringProperty(
        name="Vehicle Parameter JSON",
        description="Ausgabedatei fuer die Solverparameter",
        default="//vehicle_parameter.json",
        subtype="FILE_PATH",
    )

    vehicle_response_csv_output_path: bpy.props.StringProperty(
        name="Vehicle Response CSV",
        description="Solver-Ausgabe und Eingabedatei fuer den Baker",
        default="//vehicle_response.csv",
        subtype="FILE_PATH",
    )

    fahrzeugmasse_kg: bpy.props.FloatProperty(
        name="Fahrzeugmasse [kg]",
        default=1800.0,
        min=100.0,
        max=4000.0,
    )

    gefedertemasse_anteil: bpy.props.FloatProperty(
        name="Gefederte-Masse-Anteil [-]",
        default=0.85,
        min=0.5,
        max=0.98,
    )

    ungefedertemasse_pro_rad_kg: bpy.props.FloatProperty(
        name="Ungefederte Masse pro Rad [kg]",
        default=45.0,
        min=10.0,
        max=120.0,
    )
    federsteifigkeit_n_pro_m: bpy.props.FloatProperty(
        name="Federsteifigkeit [N/m]",
        default=30000.0,
        min=5000.0,
        max=100000.0,
    )

    daempferkonstante_n_s_pro_m: bpy.props.FloatProperty(
        name="Daempferkonstante [N*s/m]",
        default=3000.0,
        min=100.0,
        max=15000.0,
    )

    reifensteifigkeit_n_pro_m: bpy.props.FloatProperty(
        name="Reifensteifigkeit [N/m]",
        default=200000.0,
        min=50000.0,
        max=500000.0,
    )

    max_einfederung_m: bpy.props.FloatProperty(
        name="Max. Einfederung [m]",
        default=0.035,
        min=0.01,
        max=0.50,
    )

    max_ausfederung_m: bpy.props.FloatProperty(
        name="Max. Ausfederung [m]",
        default=0.035,
        min=0.01,
        max=0.50,
    )

    lastverlagerung_aktiv: bpy.props.BoolProperty(
        name="Lastverlagerung aktiv",
        default=True,
    )

    lastverlagerung_laengs_aktiv: bpy.props.BoolProperty(
        name="Laengs-Lastverlagerung aktiv",
        default=True,
    )

    lastverlagerung_quer_aktiv: bpy.props.BoolProperty(
        name="Quer-Lastverlagerung aktiv",
        default=True,
    )

    schwerpunkt_hoehe_m: bpy.props.FloatProperty(
        name="Schwerpunkt-Hoehe [m]",
        default=0.55,
        min=0.10,
        max=1.50,
    )

    lastverlagerung_laengs_vorzeichen: bpy.props.FloatProperty(
        name="Laengs-Vorzeichen",
        default=1.0,
        min=-1.0,
        max=1.0,
    )

    lastverlagerung_quer_vorzeichen: bpy.props.FloatProperty(
        name="Quer-Vorzeichen",
        default=1.0,
        min=-1.0,
        max=1.0,
    )

    rollen_skalierung: bpy.props.FloatProperty(
        name="Rollen-Skalierung",
        description="1.0 = bisherige Seitenneigung, 0.5 = halbierte Seitenneigung",
        default=0.5,
        min=0.0,
        max=2.0,
    )

    laengs_glaettungsfenster_frames: bpy.props.IntProperty(
        name="Laengs-Glaettungsfenster [Frames]",
        description="Ungerade Fensterbreite fuer die Beschleunigungsglaettung",
        default=5,
        min=1,
        max=21,
        step=2,
    )

    laengs_spitzenfilter_fenster_frames: bpy.props.IntProperty(
        name="Laengs-Spitzenfilter [Frames]",
        description="Ungerade Fensterbreite fuer den lokalen Median",
        default=5,
        min=1,
        max=21,
        step=2,
    )

    laengs_spitzenfilter_schwelle_mps2: bpy.props.FloatProperty(
        name="Spitzen-Schwelle [m/s²]",
        description="Groessere Abweichungen vom lokalen Median werden ersetzt",
        default=0.5,
        min=0.0,
        max=20.0,
        precision=3,
    )

    laengs_totzone_mps2: bpy.props.FloatProperty(
        name="Laengs-Totzone [m/s²]",
        description="Kleinere geglaettete Laengsbeschleunigungen werden zu null",
        default=0.10,
        min=0.0,
        max=5.0,
        precision=3,
    )

    berechnung_unterschritte_pro_frame: bpy.props.IntProperty(
        name="Solver-Unterschritte pro Frame",
        default=20,
        min=1,
        max=1000,
    )

def build_vehicle_parameter_dict(context):
    parameter = context.scene.road_input_vehicle_parameter

    vehicle_parameter_data = {
        "modell": "vier_unabhaengige_viertelfahrzeugmodelle",

        "FAHRZEUGMASSE_KG": parameter.fahrzeugmasse_kg,
        "GEFEDERTE_MASSE_ANTEIL": parameter.gefedertemasse_anteil,
        "UNGEFEDERTEMASSE_PRO_RAD_KG": parameter.ungefedertemasse_pro_rad_kg,

        "FEDERSTEIFIGKEIT_N_PRO_M": parameter.federsteifigkeit_n_pro_m,
        "DAEMPFERKONSTANTE_N_S_PRO_M": parameter.daempferkonstante_n_s_pro_m,
        "REIFENSTEIFIGKEIT_N_PRO_M": parameter.reifensteifigkeit_n_pro_m,

        "MAX_EINFEDERUNG_M": parameter.max_einfederung_m,
        "MAX_AUSFEDERUNG_M": parameter.max_ausfederung_m,

        "LASTVERLAGERUNG_AKTIV": parameter.lastverlagerung_aktiv,
        "LASTVERLAGERUNG_LAENGS_AKTIV": parameter.lastverlagerung_laengs_aktiv,
        "LASTVERLAGERUNG_QUER_AKTIV": parameter.lastverlagerung_quer_aktiv,

        "SCHWERPUNKT_HOEHE_M": parameter.schwerpunkt_hoehe_m,
        "LASTVERLAGERUNG_LAENGS_VORZEICHEN": parameter.lastverlagerung_laengs_vorzeichen,
        "LASTVERLAGERUNG_QUER_VORZEICHEN": parameter.lastverlagerung_quer_vorzeichen,
        "ROLLEN_SKALIERUNG": parameter.rollen_skalierung,
        "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_FENSTER_FRAMES": (
            parameter.laengs_spitzenfilter_fenster_frames
        ),
        "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_SCHWELLE_MPS2": (
            parameter.laengs_spitzenfilter_schwelle_mps2
        ),
        "LAENGS_BESCHLEUNIGUNG_GLAETTUNGSFENSTER_FRAMES": (
            parameter.laengs_glaettungsfenster_frames
        ),
        "LAENGS_BESCHLEUNIGUNG_TOTZONE_MPS2": parameter.laengs_totzone_mps2,
        "BERECHNUNG_UNTERSCHRITTE_PRO_FRAME": (
            parameter.berechnung_unterschritte_pro_frame
        ),
    }

    return vehicle_parameter_data

def write_vehicle_parameter_json(context):
    vehicle_parameter_data = build_vehicle_parameter_dict(context)

    json_path = get_configured_output_path(
        context,
        "vehicle_parameter_json_output_path",
        VEHICLE_PARAMETER_FILENAME,
    )

    with open(json_path, mode="w", encoding="utf-8") as json_file:
        json.dump(
            vehicle_parameter_data,
            json_file,
            indent=4,
            ensure_ascii=False,
        )

    print("============================================================")
    print("Road Input Exporter - Vehicle Parameter JSON Export")
    print("============================================================")
    print("JSON geschrieben:")
    print(json_path)
    print("============================================================")

    return json_path

# -----------------------------------------------------------------------------
# Solver: vier unabhaengige Viertelfahrzeugmodelle und Karosserie-Rekonstruktion
# -----------------------------------------------------------------------------

def read_csv_rows(csv_path):
    if not csv_path.exists():
        raise FileNotFoundError("CSV-Datei nicht gefunden: " + str(csv_path))

    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV-Datei hat keine Kopfzeile: " + str(csv_path))
        fieldnames = list(reader.fieldnames)
        rows = list(reader)

    if not rows:
        raise ValueError("CSV-Datei enthaelt keine Datenzeilen: " + str(csv_path))

    return fieldnames, rows


def write_csv_rows(csv_path, rows):
    if not rows:
        raise ValueError("Keine Datenzeilen zum Schreiben vorhanden.")

    fieldnames = list(rows[0].keys())
    temp_path = csv_path.with_suffix(csv_path.suffix + ".tmp")

    try:
        with temp_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        temp_path.replace(csv_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def read_finite_float(row, column_name, source_name):
    raw_value = row.get(column_name)
    if raw_value is None or str(raw_value).strip() == "":
        raise ValueError(
            f"Leerer Zahlenwert in {source_name}: "
            f"Frame {row.get('frame', 'unbekannt')}, Spalte {column_name}"
        )

    try:
        value = float(raw_value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"Ungueltiger Zahlenwert in {source_name}: "
            f"Frame {row.get('frame', 'unbekannt')}, "
            f"Spalte {column_name}, Wert {raw_value}"
        ) from error

    if not math.isfinite(value):
        raise ValueError(
            f"Nicht endlicher Zahlenwert in {source_name}: "
            f"Frame {row.get('frame', 'unbekannt')}, Spalte {column_name}"
        )

    return value


def validate_solver_parameter(parameter):
    positive_keys = [
        "FAHRZEUGMASSE_KG",
        "UNGEFEDERTEMASSE_PRO_RAD_KG",
        "GEFEDERTEMASSE_PRO_RAD_KG",
        "FEDERSTEIFIGKEIT_N_PRO_M",
        "DAEMPFERKONSTANTE_N_S_PRO_M",
        "REIFENSTEIFIGKEIT_N_PRO_M",
    ]
    for key in positive_keys:
        value = float(parameter[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(key + " muss eine positive endliche Zahl sein.")

    sprung_mass_ratio = float(parameter["GEFEDERTE_MASSE_ANTEIL"])
    if not math.isfinite(sprung_mass_ratio) or not 0.0 < sprung_mass_ratio <= 1.0:
        raise ValueError(
            "GEFEDERTE_MASSE_ANTEIL muss groesser als 0 "
            "und kleiner oder gleich 1 sein."
        )

    for key in [
        "MAX_EINFEDERUNG_M",
        "MAX_AUSFEDERUNG_M",
        "SCHWERPUNKT_HOEHE_M",
    ]:
        value = float(parameter[key])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(key + " muss eine nichtnegative endliche Zahl sein.")

    for key in [
        "LASTVERLAGERUNG_AKTIV",
        "LASTVERLAGERUNG_LAENGS_AKTIV",
        "LASTVERLAGERUNG_QUER_AKTIV",
    ]:
        if not isinstance(parameter[key], bool):
            raise ValueError(key + " muss true oder false sein.")

    for key in [
        "LASTVERLAGERUNG_LAENGS_VORZEICHEN",
        "LASTVERLAGERUNG_QUER_VORZEICHEN",
    ]:
        if not math.isfinite(float(parameter[key])):
            raise ValueError(key + " muss eine endliche Zahl sein.")

    roll_scale = float(parameter["ROLLEN_SKALIERUNG"])
    if not math.isfinite(roll_scale) or not 0.0 <= roll_scale <= 2.0:
        raise ValueError("ROLLEN_SKALIERUNG muss zwischen 0 und 2 liegen.")

    spike_window = parameter[
        "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_FENSTER_FRAMES"
    ]
    if (
        isinstance(spike_window, bool)
        or not float(spike_window).is_integer()
        or int(spike_window) < 1
        or int(spike_window) % 2 == 0
    ):
        raise ValueError(
            "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_FENSTER_FRAMES muss "
            "eine ungerade ganze Zahl groesser oder gleich 1 sein."
        )

    spike_threshold_mps2 = float(
        parameter["LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_SCHWELLE_MPS2"]
    )
    if (
        not math.isfinite(spike_threshold_mps2)
        or not 0.0 <= spike_threshold_mps2 <= 20.0
    ):
        raise ValueError(
            "LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_SCHWELLE_MPS2 muss "
            "zwischen 0 und 20 liegen."
        )

    smoothing_window = parameter[
        "LAENGS_BESCHLEUNIGUNG_GLAETTUNGSFENSTER_FRAMES"
    ]
    if (
        isinstance(smoothing_window, bool)
        or not float(smoothing_window).is_integer()
        or int(smoothing_window) < 1
        or int(smoothing_window) % 2 == 0
    ):
        raise ValueError(
            "LAENGS_BESCHLEUNIGUNG_GLAETTUNGSFENSTER_FRAMES muss "
            "eine ungerade ganze Zahl groesser oder gleich 1 sein."
        )

    dead_zone_mps2 = float(parameter["LAENGS_BESCHLEUNIGUNG_TOTZONE_MPS2"])
    if not math.isfinite(dead_zone_mps2) or not 0.0 <= dead_zone_mps2 <= 5.0:
        raise ValueError(
            "LAENGS_BESCHLEUNIGUNG_TOTZONE_MPS2 muss zwischen 0 und 5 liegen."
        )

    substeps = parameter["BERECHNUNG_UNTERSCHRITTE_PRO_FRAME"]
    if (
        isinstance(substeps, bool)
        or not float(substeps).is_integer()
        or int(substeps) < 1
    ):
        raise ValueError(
            "BERECHNUNG_UNTERSCHRITTE_PRO_FRAME muss eine ganze Zahl "
            "groesser oder gleich 1 sein."
        )


def load_solver_parameter(parameter_path):
    parameter = DEFAULT_SOLVER_PARAMETER.copy()

    if parameter_path.exists():
        with parameter_path.open("r", encoding="utf-8") as json_file:
            loaded_parameter = json.load(json_file)
        if not isinstance(loaded_parameter, dict):
            raise ValueError("vehicle_parameter.json muss ein JSON-Objekt enthalten.")
        parameter.update(loaded_parameter)

    parameter["GEFEDERTEMASSE_PRO_RAD_KG"] = (
        parameter["FAHRZEUGMASSE_KG"]
        * parameter["GEFEDERTE_MASSE_ANTEIL"]
        / 4.0
    )
    validate_solver_parameter(parameter)
    return parameter


def validate_road_input_columns(fieldnames, rows, parameter):
    if not rows:
        raise ValueError("road_input.csv enthaelt keine Datenzeilen.")

    required = [
        "frame",
        "t_s",
        "dt_s",
        "car_spurweite_mean_m",
        "car_radstand_mean_m",
    ]
    for rad_key in RAD_OBJECT_NAMES:
        required.extend([
            "rad_offset_x_local_" + rad_key + "_m",
            "rad_offset_y_local_" + rad_key + "_m",
        ])

    if parameter["LASTVERLAGERUNG_AKTIV"]:
        if parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]:
            required.append("car_acc_forward_local_mps2")
        if parameter["LASTVERLAGERUNG_QUER_AKTIV"]:
            required.append("car_acc_lateral_local_mps2")

    missing = [column for column in required if column not in fieldnames]
    if missing:
        raise ValueError(
            "Fehlende Pflichtspalten in road_input.csv: " + ", ".join(missing)
        )

    for rad_key in RAD_OBJECT_NAMES:
        envelope_column = (
            "rad_envelope_required_max_z_rel_local_" + rad_key + "_m"
        )
        center_column = "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m"
        if envelope_column not in fieldnames and center_column not in fieldnames:
            raise ValueError(
                f"Keine Strassenanregung fuer {rad_key}. Erwartet: "
                f"{envelope_column} oder {center_column}"
            )


def validate_road_input_values(rows, parameter):
    numeric_columns = [
        "frame",
        "t_s",
        "dt_s",
        "car_spurweite_mean_m",
        "car_radstand_mean_m",
    ]
    for rad_key in RAD_OBJECT_NAMES:
        numeric_columns.extend([
            "rad_offset_x_local_" + rad_key + "_m",
            "rad_offset_y_local_" + rad_key + "_m",
        ])

    if parameter["LASTVERLAGERUNG_AKTIV"]:
        if parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]:
            numeric_columns.append("car_acc_forward_local_mps2")
        if parameter["LASTVERLAGERUNG_QUER_AKTIV"]:
            numeric_columns.append("car_acc_lateral_local_mps2")

    parsed_rows = []
    for row in rows:
        parsed_rows.append({
            column: read_finite_float(row, column, ROAD_INPUT_FILENAME)
            for column in numeric_columns
        })

    frames = [values["frame"] for values in parsed_rows]
    if any(not frame.is_integer() for frame in frames):
        raise ValueError("Alle frame-Werte muessen ganze Zahlen sein.")
    if len(set(frames)) != len(frames):
        raise ValueError("road_input.csv enthaelt doppelte frame-Werte.")
    if any(current <= previous for previous, current in zip(frames, frames[1:])):
        raise ValueError("Die frame-Werte muessen streng aufsteigend sein.")

    times = [values["t_s"] for values in parsed_rows]
    if any(current <= previous for previous, current in zip(times, times[1:])):
        raise ValueError("Die t_s-Werte muessen streng aufsteigend sein.")

    time_steps = [values["dt_s"] for values in parsed_rows]
    if time_steps[0] != 0.0:
        raise ValueError("dt_s muss im ersten Frame 0.0 sein.")
    if any(value <= 0.0 for value in time_steps[1:]):
        raise ValueError("dt_s muss ab dem zweiten Frame groesser als 0 sein.")

    for values in parsed_rows:
        for column in ["car_spurweite_mean_m", "car_radstand_mean_m"]:
            if values[column] <= 0.0:
                raise ValueError(column + " muss in jedem Frame groesser als 0 sein.")


def read_road_excitation(row, rad_key):
    columns = [
        "rad_envelope_required_max_z_rel_local_" + rad_key + "_m",
        "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m",
    ]
    for column in columns:
        raw_value = row.get(column)
        if raw_value is None or str(raw_value).strip() == "":
            continue
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Ungueltige Strassenanregung fuer {rad_key} in "
                f"Frame {row.get('frame', 'unbekannt')}: {raw_value}"
            ) from error
        if math.isnan(value):
            continue
        if not math.isfinite(value):
            raise ValueError(
                f"Ungueltige Strassenanregung fuer {rad_key} in "
                f"Frame {row.get('frame', 'unbekannt')}"
            )
        return value

    raise ValueError(
        f"Keine gueltige Strassenanregung fuer {rad_key} in "
        f"Frame {row.get('frame', 'unbekannt')}"
    )


def initialize_solver_state():
    return {
        rad_key: {
            "gefedertemasse_z_local_m": 0.0,
            "gefedertemasse_geschwindigkeit_local_mps": 0.0,
            "gefedertemasse_beschleunigung_local_mps2": 0.0,
            "ungefedertemasse_z_local_m": 0.0,
            "ungefedertemasse_geschwindigkeit_local_mps": 0.0,
            "ungefedertemasse_beschleunigung_local_mps2": 0.0,
        }
        for rad_key in RAD_OBJECT_NAMES
    }


def get_centered_window(values, index, window):
    if len(values) <= window:
        return values

    half_window = window // 2
    start = min(
        max(index - half_window, 0),
        len(values) - window,
    )
    return values[start:start + window]


def median(values):
    sorted_values = sorted(values)
    middle = len(sorted_values) // 2
    if len(sorted_values) % 2 == 1:
        return sorted_values[middle]
    return (sorted_values[middle - 1] + sorted_values[middle]) / 2.0


def filter_longitudinal_acceleration(road_input_rows, parameter):
    raw_values = [
        read_finite_float(
            row,
            "car_acc_forward_local_mps2",
            ROAD_INPUT_FILENAME,
        )
        for row in road_input_rows
    ]
    spike_window = int(
        parameter["LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_FENSTER_FRAMES"]
    )
    spike_threshold_mps2 = float(
        parameter["LAENGS_BESCHLEUNIGUNG_SPITZENFILTER_SCHWELLE_MPS2"]
    )
    smoothing_window = int(
        parameter["LAENGS_BESCHLEUNIGUNG_GLAETTUNGSFENSTER_FRAMES"]
    )
    dead_zone_mps2 = float(
        parameter["LAENGS_BESCHLEUNIGUNG_TOTZONE_MPS2"]
    )
    sample_count = len(raw_values)
    despiked_values = []
    filtered_values = []

    for index in range(sample_count):
        local_values = get_centered_window(
            raw_values,
            index,
            spike_window,
        )
        local_median = median(local_values)
        raw_value = raw_values[index]
        despiked_values.append(
            local_median
            if abs(raw_value - local_median) > spike_threshold_mps2
            else raw_value
        )

    for index in range(sample_count):
        window_values = get_centered_window(
            despiked_values,
            index,
            smoothing_window,
        )
        smoothed_value = sum(window_values) / len(window_values)
        solver_value = (
            0.0
            if abs(smoothed_value) <= dead_zone_mps2
            else smoothed_value
        )
        filtered_values.append({
            "raw_mps2": raw_values[index],
            "despiked_mps2": despiked_values[index],
            "smoothed_mps2": smoothed_value,
            "solver_mps2": solver_value,
        })

    return filtered_values


def calculate_load_transfer_forces(
    row,
    parameter,
    track_width_m,
    wheelbase_m,
    longitudinal_acceleration_mps2,
):
    additional_forces = {rad_key: 0.0 for rad_key in RAD_OBJECT_NAMES}
    longitudinal_force = 0.0
    lateral_force = 0.0

    if not parameter["LASTVERLAGERUNG_AKTIV"]:
        return additional_forces, longitudinal_force, lateral_force

    if parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]:
        longitudinal_force = (
            parameter["LASTVERLAGERUNG_LAENGS_VORZEICHEN"]
            * parameter["FAHRZEUGMASSE_KG"]
            * longitudinal_acceleration_mps2
            * parameter["SCHWERPUNKT_HOEHE_M"]
            / wheelbase_m
        )
        force_per_wheel = longitudinal_force / 2.0
        additional_forces["VL"] += force_per_wheel
        additional_forces["VR"] += force_per_wheel
        additional_forces["HL"] -= force_per_wheel
        additional_forces["HR"] -= force_per_wheel

    if parameter["LASTVERLAGERUNG_QUER_AKTIV"]:
        lateral_force = (
            parameter["LASTVERLAGERUNG_QUER_VORZEICHEN"]
            * parameter["FAHRZEUGMASSE_KG"]
            * float(row["car_acc_lateral_local_mps2"])
            * parameter["SCHWERPUNKT_HOEHE_M"]
            / track_width_m
        )
        force_per_wheel = lateral_force / 2.0
        additional_forces["VL"] -= force_per_wheel
        additional_forces["HL"] -= force_per_wheel
        additional_forces["VR"] += force_per_wheel
        additional_forces["HR"] += force_per_wheel

    return additional_forces, longitudinal_force, lateral_force


def calculate_corner_forces(state, road_excitation_m, parameter, load_force_n):
    tire_compression_m = road_excitation_m - state["ungefedertemasse_z_local_m"]
    tire_force_n = parameter["REIFENSTEIFIGKEIT_N_PRO_M"] * tire_compression_m
    spring_force_n = parameter["FEDERSTEIFIGKEIT_N_PRO_M"] * (
        state["ungefedertemasse_z_local_m"]
        - state["gefedertemasse_z_local_m"]
    )
    damper_force_n = parameter["DAEMPFERKONSTANTE_N_S_PRO_M"] * (
        state["ungefedertemasse_geschwindigkeit_local_mps"]
        - state["gefedertemasse_geschwindigkeit_local_mps"]
    )
    spring_damper_force_n = spring_force_n + damper_force_n

    return {
        "reifen_eindrueckung_m": tire_compression_m,
        "reifen_kraft_N": tire_force_n,
        "feder_kraft_N": spring_force_n,
        "daempfer_kraft_N": damper_force_n,
        "feder_daempfer_kraft_N": spring_damper_force_n,
        "gefedertemasse_beschleunigung_mps2": (
            spring_damper_force_n + load_force_n
        ) / parameter["GEFEDERTEMASSE_PRO_RAD_KG"],
        "ungefedertemasse_beschleunigung_mps2": (
            tire_force_n - spring_damper_force_n
        ) / parameter["UNGEFEDERTEMASSE_PRO_RAD_KG"],
    }


def integrate_corner_state(state, forces, dt_s):
    state["gefedertemasse_beschleunigung_local_mps2"] = (
        forces["gefedertemasse_beschleunigung_mps2"]
    )
    state["ungefedertemasse_beschleunigung_local_mps2"] = (
        forces["ungefedertemasse_beschleunigung_mps2"]
    )
    state["gefedertemasse_geschwindigkeit_local_mps"] += (
        state["gefedertemasse_beschleunigung_local_mps2"] * dt_s
    )
    state["ungefedertemasse_geschwindigkeit_local_mps"] += (
        state["ungefedertemasse_beschleunigung_local_mps2"] * dt_s
    )
    state["gefedertemasse_z_local_m"] += (
        state["gefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )
    state["ungefedertemasse_z_local_m"] += (
        state["ungefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def reconstruct_body(state, track_width_m, wheelbase_m):
    sprung_z = {
        rad_key: state[rad_key]["gefedertemasse_z_local_m"]
        for rad_key in RAD_OBJECT_NAMES
    }
    heave_m = sum(sprung_z.values()) / 4.0
    front_z_m = (sprung_z["VL"] + sprung_z["VR"]) / 2.0
    rear_z_m = (sprung_z["HL"] + sprung_z["HR"]) / 2.0
    left_z_m = (sprung_z["VL"] + sprung_z["HL"]) / 2.0
    right_z_m = (sprung_z["VR"] + sprung_z["HR"]) / 2.0

    pitch_rad = math.atan2(front_z_m - rear_z_m, wheelbase_m)
    roll_rad = math.atan2(left_z_m - right_z_m, track_width_m)
    return heave_m, pitch_rad, roll_rad


def calculate_vehicle_response_rows(fieldnames, road_input_rows, parameter):
    validate_road_input_columns(fieldnames, road_input_rows, parameter)
    validate_road_input_values(road_input_rows, parameter)

    state = initialize_solver_state()
    output_rows = []
    substeps = int(parameter["BERECHNUNG_UNTERSCHRITTE_PRO_FRAME"])

    if (
        parameter["LASTVERLAGERUNG_AKTIV"]
        and parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]
    ):
        longitudinal_acceleration_data = filter_longitudinal_acceleration(
            road_input_rows,
            parameter,
        )
    else:
        longitudinal_acceleration_data = [
            {
                "raw_mps2": 0.0,
                "despiked_mps2": 0.0,
                "smoothed_mps2": 0.0,
                "solver_mps2": 0.0,
            }
            for _ in road_input_rows
        ]

    for index, row in enumerate(road_input_rows):
        frame = int(float(row["frame"]))
        t_s = float(row["t_s"])
        dt_s = float(row["dt_s"])
        wheelbase_m = float(row["car_radstand_mean_m"])
        track_width_m = float(row["car_spurweite_mean_m"])
        acceleration_data = longitudinal_acceleration_data[index]

        load_forces, longitudinal_force, lateral_force = (
            calculate_load_transfer_forces(
                row,
                parameter,
                track_width_m,
                wheelbase_m,
                acceleration_data["solver_mps2"],
            )
        )
        road_excitation = {
            rad_key: read_road_excitation(row, rad_key)
            for rad_key in RAD_OBJECT_NAMES
        }
        output_row = {
            "frame": frame,
            "t_s": t_s,
            "dt_s": dt_s,
            "car_acc_forward_raw_mps2": acceleration_data["raw_mps2"],
            "car_acc_forward_despiked_mps2": (
                acceleration_data["despiked_mps2"]
            ),
            "car_acc_forward_smoothed_mps2": (
                acceleration_data["smoothed_mps2"]
            ),
            "car_acc_forward_solver_mps2": acceleration_data["solver_mps2"],
            "lastverlagerung_laengs_kraft_N": longitudinal_force,
            "lastverlagerung_quer_kraft_N": lateral_force,
        }

        last_forces = {}
        sub_dt_s = dt_s / substeps
        for _ in range(substeps):
            for rad_key in RAD_OBJECT_NAMES:
                forces = calculate_corner_forces(
                    state[rad_key],
                    road_excitation[rad_key],
                    parameter,
                    load_forces[rad_key],
                )
                integrate_corner_state(state[rad_key], forces, sub_dt_s)
                last_forces[rad_key] = forces

        for rad_key in RAD_OBJECT_NAMES:
            forces = last_forces[rad_key]
            output_row["strassenanregung_z_rel_local_" + rad_key + "_m"] = (
                road_excitation[rad_key]
            )
            for source_key, output_prefix in [
                ("gefedertemasse_z_local_m", "gefedertemasse_z_local_"),
                (
                    "gefedertemasse_geschwindigkeit_local_mps",
                    "gefedertemasse_geschwindigkeit_local_",
                ),
                (
                    "gefedertemasse_beschleunigung_local_mps2",
                    "gefedertemasse_beschleunigung_local_",
                ),
                ("ungefedertemasse_z_local_m", "ungefedertemasse_z_local_"),
                (
                    "ungefedertemasse_geschwindigkeit_local_mps",
                    "ungefedertemasse_geschwindigkeit_local_",
                ),
                (
                    "ungefedertemasse_beschleunigung_local_mps2",
                    "ungefedertemasse_beschleunigung_local_",
                ),
            ]:
                unit = "m"
                if source_key.endswith("mps"):
                    unit = "mps"
                elif source_key.endswith("mps2"):
                    unit = "mps2"
                output_row[output_prefix + rad_key + "_" + unit] = (
                    state[rad_key][source_key]
                )

            for force_key, output_prefix in [
                ("reifen_eindrueckung_m", "reifen_eindrueckung_"),
                ("reifen_kraft_N", "reifen_kraft_"),
                ("feder_kraft_N", "feder_kraft_"),
                ("daempfer_kraft_N", "daempfer_kraft_"),
                ("feder_daempfer_kraft_N", "feder_daempfer_kraft_"),
            ]:
                unit = "m" if force_key.endswith("_m") else "N"
                output_row[output_prefix + rad_key + "_" + unit] = (
                    forces[force_key]
                )
            output_row["zusatzkraft_lastverlagerung_" + rad_key + "_N"] = (
                load_forces[rad_key]
            )

        heave_m, pitch_rad, roll_rad = reconstruct_body(
            state, track_width_m, wheelbase_m
        )
        roll_rad *= parameter["ROLLEN_SKALIERUNG"]
        output_row["karosserie_hub_local_m"] = heave_m
        output_row["karosserie_nicken_rad"] = pitch_rad
        output_row["karosserie_rollen_rad"] = roll_rad

        for rad_key in RAD_OBJECT_NAMES:
            offset_x_m = float(row["rad_offset_x_local_" + rad_key + "_m"])
            offset_y_m = float(row["rad_offset_y_local_" + rad_key + "_m"])
            body_z_at_wheel_m = (
                heave_m + pitch_rad * offset_y_m - roll_rad * offset_x_m
            )
            suspension_raw_m = (
                state[rad_key]["ungefedertemasse_z_local_m"]
                - body_z_at_wheel_m
            )
            suspension_m = clamp(
                suspension_raw_m,
                -parameter["MAX_AUSFEDERUNG_M"],
                parameter["MAX_EINFEDERUNG_M"],
            )
            output_row["karosserie_z_am_rad_" + rad_key + "_m"] = (
                body_z_at_wheel_m
            )
            output_row["rad_federweg_raw_z_local_" + rad_key + "_m"] = (
                suspension_raw_m
            )
            output_row["rad_federweg_z_local_" + rad_key + "_m"] = suspension_m

        output_rows.append(output_row)

    return output_rows


def solve_vehicle_response(context):
    road_input_path = get_configured_output_path(
        context,
        "road_input_csv_output_path",
        ROAD_INPUT_FILENAME,
    )
    parameter_path = get_configured_output_path(
        context,
        "vehicle_parameter_json_output_path",
        VEHICLE_PARAMETER_FILENAME,
    )
    response_path = get_configured_output_path(
        context,
        "vehicle_response_csv_output_path",
        VEHICLE_RESPONSE_FILENAME,
    )

    fieldnames, road_input_rows = read_csv_rows(road_input_path)
    parameter = load_solver_parameter(parameter_path)
    output_rows = calculate_vehicle_response_rows(
        fieldnames, road_input_rows, parameter
    )
    write_csv_rows(response_path, output_rows)
    print(
        f"Solver abgeschlossen: {response_path} "
        f"({len(output_rows)} Frames)"
    )
    return response_path


# -----------------------------------------------------------------------------
# Baker: vehicle_response.csv validieren und auf Blender-Objekte backen
# -----------------------------------------------------------------------------

def required_response_columns():
    columns = [
        "frame",
        "t_s",
        "dt_s",
        "karosserie_hub_local_m",
        "karosserie_nicken_rad",
        "karosserie_rollen_rad",
    ]
    for rad_key in RAD_OBJECT_NAMES:
        columns.extend([
            "karosserie_z_am_rad_" + rad_key + "_m",
            "rad_federweg_z_local_" + rad_key + "_m",
        ])
    return columns


def get_bake_targets():
    body = bpy.data.objects.get(KAROSSERIE_OBJECT_NAME)
    wheels = {
        rad_key: bpy.data.objects.get(object_name)
        for rad_key, object_name in RAD_FEDERUNG_OBJECT_NAMES.items()
    }
    missing = []
    if body is None:
        missing.append(KAROSSERIE_OBJECT_NAME)
    missing.extend(
        RAD_FEDERUNG_OBJECT_NAMES[rad_key]
        for rad_key, obj in wheels.items()
        if obj is None
    )
    if missing:
        raise ValueError("Bake-Zielobjekte fehlen: " + ", ".join(missing))
    return body, wheels


def parse_response_rows(rows):
    parsed_rows = []
    previous_frame = None

    for row in rows:
        frame_value = read_finite_float(row, "frame", VEHICLE_RESPONSE_FILENAME)
        if not frame_value.is_integer():
            raise ValueError("frame muss in vehicle_response.csv ganzzahlig sein.")
        frame = int(frame_value)
        if previous_frame is not None and frame <= previous_frame:
            raise ValueError(
                "frame muss in vehicle_response.csv streng aufsteigend sein."
            )

        parsed = {
            "frame": frame,
            "karosserie_hub_local_m": read_finite_float(
                row, "karosserie_hub_local_m", VEHICLE_RESPONSE_FILENAME
            ),
            "karosserie_nicken_rad": read_finite_float(
                row, "karosserie_nicken_rad", VEHICLE_RESPONSE_FILENAME
            ),
            "karosserie_rollen_rad": read_finite_float(
                row, "karosserie_rollen_rad", VEHICLE_RESPONSE_FILENAME
            ),
        }
        for rad_key in RAD_OBJECT_NAMES:
            parsed["karosserie_z_am_rad_" + rad_key + "_m"] = read_finite_float(
                row,
                "karosserie_z_am_rad_" + rad_key + "_m",
                VEHICLE_RESPONSE_FILENAME,
            )
            parsed["rad_federweg_z_local_" + rad_key + "_m"] = read_finite_float(
                row,
                "rad_federweg_z_local_" + rad_key + "_m",
                VEHICLE_RESPONSE_FILENAME,
            )

        parsed_rows.append(parsed)
        previous_frame = frame

    return parsed_rows


def bake_vehicle_response(context):
    response_path = get_configured_output_path(
        context,
        "vehicle_response_csv_output_path",
        VEHICLE_RESPONSE_FILENAME,
    )
    fieldnames, rows = read_csv_rows(response_path)
    missing = [
        column for column in required_response_columns()
        if column not in fieldnames
    ]
    if missing:
        raise ValueError(
            "Fehlende Pflichtspalten in vehicle_response.csv: "
            + ", ".join(missing)
        )

    parsed_rows = parse_response_rows(rows)
    body, wheels = get_bake_targets()
    scene = bpy.context.scene
    original_frame = scene.frame_current

    try:
        body.animation_data_clear()
        for wheel in wheels.values():
            wheel.animation_data_clear()

        for row in parsed_rows:
            frame = row["frame"]
            scene.frame_set(frame)

            body.location.z = (
                KAROSSERIE_BASIS_Z_LOCAL_M
                + row["karosserie_hub_local_m"]
            )
            body.rotation_euler.x = row["karosserie_nicken_rad"]
            body.rotation_euler.y = row["karosserie_rollen_rad"]
            body.keyframe_insert(data_path="location", frame=frame)
            body.keyframe_insert(data_path="rotation_euler", frame=frame)

            for rad_key, wheel in wheels.items():
                wheel.location.z = (
                    RAD_FEDERUNG_BASIS_Z_LOCAL_M[rad_key]
                    + row["karosserie_z_am_rad_" + rad_key + "_m"]
                    + row["rad_federweg_z_local_" + rad_key + "_m"]
                )
                wheel.keyframe_insert(data_path="location", frame=frame)
    finally:
        if scene.frame_current != original_frame:
            scene.frame_set(original_frame)

    print(f"Bake abgeschlossen: {len(parsed_rows)} Frames aus {response_path}")
    return response_path


def export_input_files(context, validate=True):
    if validate:
        validate_export_setup(context)
    csv_path = export_road_input_csv(context)
    json_path = write_vehicle_parameter_json(context)
    return csv_path, json_path


def run_complete_pipeline(context):
    validate_export_setup(context)
    csv_path, json_path = export_input_files(context, validate=False)
    response_path = solve_vehicle_response(context)
    bake_vehicle_response(context)
    return csv_path, json_path, response_path


# -----------------------------------------------------------------------------
# Blender-UI: 3D Viewport -> N -> Road Input
# -----------------------------------------------------------------------------
# Beim Registrieren startet keine Berechnung; alle Schritte werden per Button
# einzeln oder ueber "Run Complete Pipeline" ausgefuehrt.

def run_all_diagnostic_tests(context):
    return validate_export_setup(context)


class ROADINPUT_OT_run_diagnostic_tests(bpy.types.Operator):
    bl_idname = "road_input.run_diagnostic_tests"
    bl_label = "Validate Setup"
    bl_description = "Prueft Objekte, Szene, Parameter und Raycasts"

    def execute(self, context):
        try:
            result = run_all_diagnostic_tests(context)
            self.report(
                {"INFO"},
                f"Setup OK ({result['passed_count']} Checks, "
                f"{len(result['warnings'])} Warnungen).",
            )
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER in Diagnostic Tests:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_export_inputs(bpy.types.Operator):
    bl_idname = "road_input.export_inputs"
    bl_label = "1. Validate + Export Inputs"
    bl_description = "Prueft das Setup und exportiert CSV und JSON"

    def execute(self, context):
        try:
            csv_path, json_path = export_input_files(context)
            self.report({"INFO"}, "Inputs exportiert.")
            print("CSV:", csv_path)
            print("JSON:", json_path)
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER beim Input-Export:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_solve(bpy.types.Operator):
    bl_idname = "road_input.solve"
    bl_label = "2. Solve vehicle_response.csv"
    bl_description = "Berechnet die Fahrzeugreaktion aus CSV und JSON"

    def execute(self, context):
        try:
            response_path = solve_vehicle_response(context)
            self.report({"INFO"}, "Solver abgeschlossen.")
            print("Response:", response_path)
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER im Solver:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_bake(bpy.types.Operator):
    bl_idname = "road_input.bake"
    bl_label = "3. Bake Animation"
    bl_description = "Backt vehicle_response.csv auf Karosserie und Federung"

    def execute(self, context):
        try:
            response_path = bake_vehicle_response(context)
            self.report({"INFO"}, "Bake abgeschlossen.")
            print("Response:", response_path)
            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER beim Bake:", error)
            return {"CANCELLED"}


class ROADINPUT_OT_test_and_export(bpy.types.Operator):
    bl_idname = "road_input.test_and_export"
    bl_label = "Run Complete Pipeline"
    bl_description = "Validiert, exportiert, berechnet und backt die Animation"

    def execute(self, context):
        try:
            csv_path, json_path, response_path = run_complete_pipeline(context)

            self.report(
                {"INFO"},
                "Pipeline abgeschlossen: Export, Solver und Bake OK.",
            )

            print("CSV:", csv_path)
            print("JSON:", json_path)
            print("Response:", response_path)

            return {"FINISHED"}

        except Exception as error:
            self.report({"ERROR"}, str(error))
            print("FEHLER in kompletter Pipeline:", error)
            return {"CANCELLED"}


class ROADINPUT_PT_panel(bpy.types.Panel):
    bl_label = "Vehicle Dynamics Toolchain"
    bl_idname = "ROADINPUT_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Road Input"

    def draw(self, context):
        layout = self.layout
        parameter = context.scene.road_input_vehicle_parameter

        layout.label(text="Exporter + Solver + Baker")

        layout.separator()
        layout.label(text="Output Paths")
        layout.prop(parameter, "road_input_csv_output_path")
        layout.prop(parameter, "vehicle_parameter_json_output_path")
        layout.prop(parameter, "vehicle_response_csv_output_path")

        layout.separator()

        layout.operator(
            "road_input.run_diagnostic_tests",
            text="Validate Setup",
            icon="CHECKMARK",
        )

        layout.operator(
            "road_input.export_inputs",
            text="1. Validate + Export Inputs",
            icon="EXPORT",
        )

        layout.operator(
            "road_input.solve",
            text="2. Solve vehicle_response.csv",
            icon="FILE_REFRESH",
        )

        layout.operator(
            "road_input.bake",
            text="3. Bake Animation",
            icon="ACTION",
        )

        layout.separator()

        layout.operator(
            "road_input.test_and_export",
            text="Run Complete Pipeline",
            icon="PLAY",
        )

        layout.separator()
        layout.label(text="Viertelfahrzeugmodell Parameter")

        layout.prop(parameter, "fahrzeugmasse_kg")
        layout.prop(parameter, "gefedertemasse_anteil")
        layout.prop(parameter, "ungefedertemasse_pro_rad_kg")

        layout.separator()
        layout.label(text="Feder / Daempfer / Reifen")

        layout.prop(parameter, "federsteifigkeit_n_pro_m")
        layout.prop(parameter, "daempferkonstante_n_s_pro_m")
        layout.prop(parameter, "reifensteifigkeit_n_pro_m")

        layout.separator()
        layout.label(text="Federweg Grenzen")

        layout.prop(parameter, "max_einfederung_m")
        layout.prop(parameter, "max_ausfederung_m")

        layout.separator()
        layout.label(text="Lastverlagerung")

        layout.prop(parameter, "lastverlagerung_aktiv")
        layout.prop(parameter, "lastverlagerung_laengs_aktiv")
        layout.prop(parameter, "lastverlagerung_quer_aktiv")
        layout.prop(parameter, "schwerpunkt_hoehe_m")
        layout.prop(parameter, "lastverlagerung_laengs_vorzeichen")
        layout.prop(parameter, "lastverlagerung_quer_vorzeichen")
        layout.prop(parameter, "laengs_spitzenfilter_fenster_frames")
        layout.prop(parameter, "laengs_spitzenfilter_schwelle_mps2")
        layout.prop(parameter, "laengs_glaettungsfenster_frames")
        layout.prop(parameter, "laengs_totzone_mps2")
        layout.prop(parameter, "rollen_skalierung")

        layout.separator()
        layout.label(text="Numerik")
        layout.prop(parameter, "berechnung_unterschritte_pro_frame")


classes = [
    ROADINPUT_VehicleParameter,

    ROADINPUT_OT_run_diagnostic_tests,
    ROADINPUT_OT_export_inputs,
    ROADINPUT_OT_solve,
    ROADINPUT_OT_bake,
    ROADINPUT_OT_test_and_export,

    ROADINPUT_PT_panel,
]


def unregister():
    if hasattr(bpy.types.Scene, "road_input_vehicle_parameter"):
        del bpy.types.Scene.road_input_vehicle_parameter

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except RuntimeError:
            pass
        except ValueError:
            pass


def register():
    unregister()

    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.road_input_vehicle_parameter = bpy.props.PointerProperty(
        type=ROADINPUT_VehicleParameter
    )


if __name__ == "__main__":
    register()
