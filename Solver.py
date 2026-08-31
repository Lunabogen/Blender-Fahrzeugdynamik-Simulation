# ============================================================
# STEP 2 Solver
# ============================================================

import json
import math
from pathlib import Path

import pandas as pd

RAD_KEYS = ["VL", "VR", "HL", "HR"]

DEFAULT_PARAMETER = {
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

    "BERECHNUNG_UNTERSCHRITTE_PRO_FRAME": 20,
}

# Parameter pruefen
# =============================================================================
def pruefe_parameter(parameter: dict) -> None:
    positive_parameter = [
        "FAHRZEUGMASSE_KG",
        "UNGEFEDERTEMASSE_PRO_RAD_KG",
        "FEDERSTEIFIGKEIT_N_PRO_M",
        "DAEMPFERKONSTANTE_N_S_PRO_M",
        "REIFENSTEIFIGKEIT_N_PRO_M",
        "GEFEDERTEMASSE_PRO_RAD_KG",
    ]

    for parameter_name in positive_parameter:
        parameter_wert = float(parameter[parameter_name])

        if not math.isfinite(parameter_wert) or parameter_wert <= 0.0:
            raise ValueError(
                parameter_name + " muss eine positive endliche Zahl sein."
            )

    gefederte_masse_anteil = float(
        parameter["GEFEDERTE_MASSE_ANTEIL"]
    )

    if (
        not math.isfinite(gefederte_masse_anteil)
        or gefederte_masse_anteil <= 0.0
        or gefederte_masse_anteil > 1.0
    ):
        raise ValueError(
            "GEFEDERTE_MASSE_ANTEIL muss groesser als 0 "
            "und kleiner oder gleich 1 sein."
        )

    nichtnegative_parameter = [
        "MAX_EINFEDERUNG_M",
        "MAX_AUSFEDERUNG_M",
        "SCHWERPUNKT_HOEHE_M",
    ]

    for parameter_name in nichtnegative_parameter:
        parameter_wert = float(parameter[parameter_name])

        if not math.isfinite(parameter_wert) or parameter_wert < 0.0:
            raise ValueError(
                parameter_name
                + " muss eine nichtnegative endliche Zahl sein."
            )

    for parameter_name in [
        "LASTVERLAGERUNG_AKTIV",
        "LASTVERLAGERUNG_LAENGS_AKTIV",
        "LASTVERLAGERUNG_QUER_AKTIV",
    ]:
        if not isinstance(parameter[parameter_name], bool):
            raise ValueError(parameter_name + " muss true oder false sein.")

    for parameter_name in [
        "LASTVERLAGERUNG_LAENGS_VORZEICHEN",
        "LASTVERLAGERUNG_QUER_VORZEICHEN",
    ]:
        if not math.isfinite(float(parameter[parameter_name])):
            raise ValueError(parameter_name + " muss eine endliche Zahl sein.")

    unterschritte = parameter["BERECHNUNG_UNTERSCHRITTE_PRO_FRAME"]

    if (
        isinstance(unterschritte, bool)
        or not float(unterschritte).is_integer()
        or int(unterschritte) < 1
    ):
        raise ValueError(
            "BERECHNUNG_UNTERSCHRITTE_PRO_FRAME muss "
            "eine ganze Zahl groesser oder gleich 1 sein."
        )


# JSON laden
# =============================================================================
def lade_parameter(parameter_pfad: Path) -> dict:
    parameter = DEFAULT_PARAMETER.copy()

    if parameter_pfad.exists():
        with parameter_pfad.open("r", encoding="utf-8") as json_file:
            geladene_parameter = json.load(json_file)

        parameter.update(geladene_parameter)

    else:
        print(
            "vehicle_parameter.json nicht gefunden. "
            "Standardparameter werden verwendet."
        )

    parameter["GEFEDERTEMASSE_PRO_RAD_KG"] = (
        parameter["FAHRZEUGMASSE_KG"]
        * parameter["GEFEDERTE_MASSE_ANTEIL"]
        / 4.0
    )

    pruefe_parameter(parameter)

    return parameter


# CSV-Spalten pruefen
# =============================================================================
def pruefe_road_input_spalten(
    road_input_df: pd.DataFrame,
    parameter: dict,
) -> None:
    if road_input_df.empty:
        raise ValueError("road_input.csv enthaelt keine Datenzeilen.")

    basis_spalten = ["frame", "t_s", "dt_s"]

    for spalte in basis_spalten:
        if spalte not in road_input_df.columns:
            raise ValueError(
                "Fehlende Pflichtspalte in road_input.csv: " + spalte
            )

    geometrie_spalten = [
        "car_spurweite_mean_m",
        "car_radstand_mean_m",
    ]

    for rad_key in RAD_KEYS:
        geometrie_spalten.append("rad_offset_x_local_" + rad_key + "_m")
        geometrie_spalten.append("rad_offset_y_local_" + rad_key + "_m")

    for spalte in geometrie_spalten:
        if spalte not in road_input_df.columns:
            raise ValueError(
                "Fehlende Geometriespalte in road_input.csv: " + spalte
            )

    if parameter["LASTVERLAGERUNG_AKTIV"]:
        if (
            parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]
            and "car_acc_forward_local_mps2" not in road_input_df.columns
        ):
            raise ValueError(
                "Fehlende Spalte fuer Laengs-Lastverlagerung: "
                "car_acc_forward_local_mps2"
            )

        if (
            parameter["LASTVERLAGERUNG_QUER_AKTIV"]
            and "car_acc_lateral_local_mps2" not in road_input_df.columns
        ):
            raise ValueError(
                "Fehlende Spalte fuer Quer-Lastverlagerung: "
                "car_acc_lateral_local_mps2"
            )

    for rad_key in RAD_KEYS:
        envelope_spalte = (
            "rad_envelope_required_max_z_rel_local_" + rad_key + "_m"
        )
        center_spalte = (
            "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m"
        )

        if (
            envelope_spalte not in road_input_df.columns
            and center_spalte not in road_input_df.columns
        ):
            raise ValueError(
                "Keine Strassenanregung fuer "
                + rad_key
                + " in road_input.csv. Erwartet: "
                + envelope_spalte
                + " oder "
                + center_spalte
            )


# CSV-Werte pruefen
# =============================================================================
def pruefe_road_input_werte(
    road_input_df: pd.DataFrame,
    parameter: dict,
) -> None:
    numerische_spalten = [
        "frame",
        "t_s",
        "dt_s",
        "car_spurweite_mean_m",
        "car_radstand_mean_m",
    ]

    for rad_key in RAD_KEYS:
        numerische_spalten.append(
            "rad_offset_x_local_" + rad_key + "_m"
        )
        numerische_spalten.append(
            "rad_offset_y_local_" + rad_key + "_m"
        )

    if parameter["LASTVERLAGERUNG_AKTIV"]:
        if parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]:
            numerische_spalten.append("car_acc_forward_local_mps2")

        if parameter["LASTVERLAGERUNG_QUER_AKTIV"]:
            numerische_spalten.append("car_acc_lateral_local_mps2")

    numerische_werte = {}

    for spalte in numerische_spalten:
        werte = pd.to_numeric(road_input_df[spalte], errors="coerce")

        if werte.isna().any() or not werte.map(math.isfinite).all():
            raise ValueError(
                "Ungueltiger oder fehlender Zahlenwert in Spalte: " + spalte
            )

        numerische_werte[spalte] = werte

    frame_werte = numerische_werte["frame"]

    if not (frame_werte % 1.0 == 0.0).all():
        raise ValueError("Alle frame-Werte muessen ganze Zahlen sein.")

    if frame_werte.duplicated().any():
        raise ValueError("road_input.csv enthaelt doppelte frame-Werte.")

    if not frame_werte.is_monotonic_increasing:
        raise ValueError("Die frame-Werte muessen aufsteigend sortiert sein.")

    t_werte_s = numerische_werte["t_s"]

    if len(t_werte_s) > 1 and (t_werte_s.diff().iloc[1:] <= 0.0).any():
        raise ValueError("Die t_s-Werte muessen streng aufsteigend sein.")

    dt_werte_s = numerische_werte["dt_s"]

    if dt_werte_s.iloc[0] != 0.0:
        raise ValueError("dt_s muss im ersten Frame 0.0 sein.")

    if len(dt_werte_s) > 1 and (dt_werte_s.iloc[1:] <= 0.0).any():
        raise ValueError("dt_s muss ab dem zweiten Frame groesser als 0 sein.")

    for spalte in ["car_spurweite_mean_m", "car_radstand_mean_m"]:
        if (numerische_werte[spalte] <= 0.0).any():
            raise ValueError(spalte + " muss in jedem Frame groesser als 0 sein.")


# Strassenanregung aus CSV laden
# =============================================================================
def lese_strassenanregung_z_rel_local_m(
    row: pd.Series,
    rad_key: str,
) -> float:
    envelope_spalte = (
        "rad_envelope_required_max_z_rel_local_" + rad_key + "_m"
    )

    center_spalte = (
        "rad_center_fahrbahn_z_rel_local_" + rad_key + "_m"
    )

    if envelope_spalte in row.index:
        envelope_wert = row[envelope_spalte]

        if not pd.isna(envelope_wert):
            envelope_wert = float(envelope_wert)

            if not math.isfinite(envelope_wert):
                raise ValueError(
                    "Ungueltige Envelope-Strassenanregung fuer "
                    + rad_key
                    + " in Frame "
                    + str(row["frame"])
                )

            return envelope_wert

    if center_spalte in row.index:
        center_wert = row[center_spalte]

        if not pd.isna(center_wert):
            center_wert = float(center_wert)

            if not math.isfinite(center_wert):
                raise ValueError(
                    "Ungueltige Center-Strassenanregung fuer "
                    + rad_key
                    + " in Frame "
                    + str(row["frame"])
                )

            return center_wert

    raise ValueError(
        "Keine gueltige Strassenanregung fuer "
        + rad_key
        + " in Frame "
        + str(row["frame"])
    )


# Solver-Zustand initialisieren
# =============================================================================
def initialisiere_zustand() -> dict:
    zustand = {}

    for rad_key in RAD_KEYS:
        zustand[rad_key] = {
            "gefedertemasse_z_local_m": 0.0,
            "gefedertemasse_geschwindigkeit_local_mps": 0.0,
            "gefedertemasse_beschleunigung_local_mps2": 0.0,

            "ungefedertemasse_z_local_m": 0.0,
            "ungefedertemasse_geschwindigkeit_local_mps": 0.0,
            "ungefedertemasse_beschleunigung_local_mps2": 0.0,
        }

    return zustand


# Lastverlagerung auf vier Rad-Ecken verteilen
# =============================================================================
def berechne_zusatzkraefte_lastverlagerung(
    row: pd.Series,
    parameter: dict,
    car_spurweite_mean_m: float,
    car_radstand_mean_m: float,
) -> tuple:
    zusatzkraefte_lastverlagerung_N = {
        rad_key: 0.0 for rad_key in RAD_KEYS
    }

    lastverlagerung_laengs_kraft_N = 0.0
    lastverlagerung_quer_kraft_N = 0.0

    if not parameter["LASTVERLAGERUNG_AKTIV"]:
        return (
            zusatzkraefte_lastverlagerung_N,
            lastverlagerung_laengs_kraft_N,
            lastverlagerung_quer_kraft_N,
        )

    if parameter["LASTVERLAGERUNG_LAENGS_AKTIV"]:
        car_acc_forward_local_mps2 = float(
            row["car_acc_forward_local_mps2"]
        )

        lastverlagerung_laengs_kraft_N = (
            parameter["LASTVERLAGERUNG_LAENGS_VORZEICHEN"]
            * parameter["FAHRZEUGMASSE_KG"]
            * car_acc_forward_local_mps2
            * parameter["SCHWERPUNKT_HOEHE_M"]
            / car_radstand_mean_m
        )

        lastverlagerung_laengs_pro_rad_N = (
            lastverlagerung_laengs_kraft_N / 2.0
        )

        zusatzkraefte_lastverlagerung_N["VL"] += (
            lastverlagerung_laengs_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["VR"] += (
            lastverlagerung_laengs_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["HL"] -= (
            lastverlagerung_laengs_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["HR"] -= (
            lastverlagerung_laengs_pro_rad_N
        )

    if parameter["LASTVERLAGERUNG_QUER_AKTIV"]:
        car_acc_lateral_local_mps2 = float(
            row["car_acc_lateral_local_mps2"]
        )

        lastverlagerung_quer_kraft_N = (
            parameter["LASTVERLAGERUNG_QUER_VORZEICHEN"]
            * parameter["FAHRZEUGMASSE_KG"]
            * car_acc_lateral_local_mps2
            * parameter["SCHWERPUNKT_HOEHE_M"]
            / car_spurweite_mean_m
        )

        lastverlagerung_quer_pro_rad_N = (
            lastverlagerung_quer_kraft_N / 2.0
        )

        zusatzkraefte_lastverlagerung_N["VL"] -= (
            lastverlagerung_quer_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["HL"] -= (
            lastverlagerung_quer_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["VR"] += (
            lastverlagerung_quer_pro_rad_N
        )
        zusatzkraefte_lastverlagerung_N["HR"] += (
            lastverlagerung_quer_pro_rad_N
        )

    return (
        zusatzkraefte_lastverlagerung_N,
        lastverlagerung_laengs_kraft_N,
        lastverlagerung_quer_kraft_N,
    )


# Kraftberechnung einer Rad-Ecke
# =============================================================================
def berechne_kraefte_einer_rad_ecke(
    zustand_rad: dict,
    strassenanregung_z_rel_local_m: float,
    parameter: dict,
    zusatzkraft_lastverlagerung_N: float,
) -> dict:
    reifen_eindruckung_m = (
        strassenanregung_z_rel_local_m
        - zustand_rad["ungefedertemasse_z_local_m"]
    )
    reifen_kraft_N = (
        parameter["REIFENSTEIFIGKEIT_N_PRO_M"]
        * reifen_eindruckung_m
    )

    feder_kraft_N = (
        parameter["FEDERSTEIFIGKEIT_N_PRO_M"]
        * (
            zustand_rad["ungefedertemasse_z_local_m"]
            - zustand_rad["gefedertemasse_z_local_m"]
        )
    )

    daempfer_kraft_N = (
        parameter["DAEMPFERKONSTANTE_N_S_PRO_M"]
        * (
            zustand_rad[
                "ungefedertemasse_geschwindigkeit_local_mps"
            ]
            - zustand_rad[
                "gefedertemasse_geschwindigkeit_local_mps"
            ]
        )
    )
    feder_daempfer_kraft_N = feder_kraft_N + daempfer_kraft_N

    gefedertemasse_beschleunigung_mps2 = (
        feder_daempfer_kraft_N
        + zusatzkraft_lastverlagerung_N
    ) / parameter["GEFEDERTEMASSE_PRO_RAD_KG"]

    ungefedertemasse_beschleunigung_mps2 = (
        reifen_kraft_N
        - feder_daempfer_kraft_N
    ) / parameter["UNGEFEDERTEMASSE_PRO_RAD_KG"]

    return {
        "reifen_eindrueckung_m": reifen_eindruckung_m,
        "reifen_kraft_N": reifen_kraft_N,
        "feder_kraft_N": feder_kraft_N,
        "daempfer_kraft_N": daempfer_kraft_N,
        "feder_daempfer_kraft_N": feder_daempfer_kraft_N,
        "gefedertemasse_beschleunigung_mps2": gefedertemasse_beschleunigung_mps2,
        "ungefedertemasse_beschleunigung_mps2": (
            ungefedertemasse_beschleunigung_mps2
        ),
    }


# Beschleunigung und Zustand integrieren
# =============================================================================
def integriere_eine_rad_ecke(
    zustand_rad: dict,
    kraefte: dict,
    dt_s: float,
) -> None:
    zustand_rad["gefedertemasse_beschleunigung_local_mps2"] = (
        kraefte["gefedertemasse_beschleunigung_mps2"]
    )
    zustand_rad["ungefedertemasse_beschleunigung_local_mps2"] = (
        kraefte["ungefedertemasse_beschleunigung_mps2"]
    )

    zustand_rad["gefedertemasse_geschwindigkeit_local_mps"] += (
        zustand_rad["gefedertemasse_beschleunigung_local_mps2"] * dt_s
    )
    zustand_rad["ungefedertemasse_geschwindigkeit_local_mps"] += (
        zustand_rad["ungefedertemasse_beschleunigung_local_mps2"] * dt_s
    )

    zustand_rad["gefedertemasse_z_local_m"] += (
        zustand_rad["gefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )
    zustand_rad["ungefedertemasse_z_local_m"] += (
        zustand_rad["ungefedertemasse_geschwindigkeit_local_mps"] * dt_s
    )


# Wert begrenzen
# =============================================================================
def clamp(wert: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(wert, maximum))


# Karosserie rekonstruieren
# =============================================================================
def rekonstruiere_karosserie(
    zustand: dict,
    car_spurweite_mean_m: float,
    car_radstand_mean_m: float,
) -> tuple:
    gefedertemasse_z_local_VL_m = zustand["VL"][
        "gefedertemasse_z_local_m"
    ]
    gefedertemasse_z_local_VR_m = zustand["VR"][
        "gefedertemasse_z_local_m"
    ]
    gefedertemasse_z_local_HL_m = zustand["HL"][
        "gefedertemasse_z_local_m"
    ]
    gefedertemasse_z_local_HR_m = zustand["HR"][
        "gefedertemasse_z_local_m"
    ]

    karosserie_hub_local_m = (
        gefedertemasse_z_local_VL_m
        + gefedertemasse_z_local_VR_m
        + gefedertemasse_z_local_HL_m
        + gefedertemasse_z_local_HR_m
    ) / 4.0

    vorderachse_gefedertemasse_z_local_m = (
        gefedertemasse_z_local_VL_m
        + gefedertemasse_z_local_VR_m
    ) / 2.0

    hinterachse_gefedertemasse_z_local_m = (
        gefedertemasse_z_local_HL_m
        + gefedertemasse_z_local_HR_m
    ) / 2.0

    karosserie_nicken_rad = math.atan2(
        vorderachse_gefedertemasse_z_local_m
        - hinterachse_gefedertemasse_z_local_m,
        car_radstand_mean_m,
    )

    linke_seite_gefedertemasse_z_local_m = (
        gefedertemasse_z_local_VL_m
        + gefedertemasse_z_local_HL_m
    ) / 2.0

    rechte_seite_gefedertemasse_z_local_m = (
        gefedertemasse_z_local_VR_m
        + gefedertemasse_z_local_HR_m
    ) / 2.0

    karosserie_rollen_rad = math.atan2(
        linke_seite_gefedertemasse_z_local_m
        - rechte_seite_gefedertemasse_z_local_m,
        car_spurweite_mean_m,
    )

    return (
        karosserie_hub_local_m,
        karosserie_nicken_rad,
        karosserie_rollen_rad,
    )

# CSV schreiben
# =============================================================================
def schreibe_vehicle_response_csv(
    output_pfad: Path,
    output_rows: list,
) -> None:
    vehicle_response_df = pd.DataFrame(output_rows)
    vehicle_response_df.to_csv(output_pfad, index=False, encoding="utf-8")

    print("============================================================")
    print("vehicle_response.csv geschrieben:")
    print(output_pfad)
    print("Anzahl Output-Zeilen:", len(vehicle_response_df))
    print("Anzahl Output-Spalten:", len(vehicle_response_df.columns))
    print("============================================================")


# Alle Frames berechnen
# =============================================================================
def berechne_vehicle_response_rows(
    road_input_df: pd.DataFrame,
    parameter: dict,
) -> list:
    pruefe_road_input_spalten(
        road_input_df=road_input_df,
        parameter=parameter,
    )
    pruefe_road_input_werte(
        road_input_df=road_input_df,
        parameter=parameter,
    )

    zustand = initialisiere_zustand()

    output_rows = []

    berechnung_unterschritte_pro_frame = int(
        parameter["BERECHNUNG_UNTERSCHRITTE_PRO_FRAME"]
    )

    print("============================================================")
    print("Viertelfahrzeugmodell Solver - Berechnung ueber alle Frames")
    print("============================================================")

    for index, row in road_input_df.iterrows():
        frame = int(row["frame"])
        t_s = float(row["t_s"])
        dt_s = float(row["dt_s"])

        car_radstand_mean_m = float(row["car_radstand_mean_m"])
        car_spurweite_mean_m = float(row["car_spurweite_mean_m"])

        (
            zusatzkraefte_lastverlagerung_N,
            lastverlagerung_laengs_kraft_N,
            lastverlagerung_quer_kraft_N,
        ) = berechne_zusatzkraefte_lastverlagerung(
            row=row,
            parameter=parameter,
            car_spurweite_mean_m=car_spurweite_mean_m,
            car_radstand_mean_m=car_radstand_mean_m,
        )

        strassenanregungen_z_rel_local_m = {}

        for rad_key in RAD_KEYS:
            strassenanregungen_z_rel_local_m[rad_key] = (
                lese_strassenanregung_z_rel_local_m(
                    row=row,
                    rad_key=rad_key,
                )
            )

        output_row = {
            "frame": frame,
            "t_s": t_s,
            "dt_s": dt_s,
            "lastverlagerung_laengs_kraft_N": (
                lastverlagerung_laengs_kraft_N
            ),
            "lastverlagerung_quer_kraft_N": (
                lastverlagerung_quer_kraft_N
            ),
        }

        # ------------------------------------------------------------
        # 1. Vier Rad-Ecken berechnen und Zustand integrieren
        #    mit Unterschritten fuer numerische Stabilitaet
        # ------------------------------------------------------------

        sub_dt_s = dt_s / berechnung_unterschritte_pro_frame

        letzte_kraefte = {}

        for _ in range(berechnung_unterschritte_pro_frame):
            for rad_key in RAD_KEYS:
                strassenanregung_z_rel_local_m = (
                    strassenanregungen_z_rel_local_m[rad_key]
                )

                zusatzkraft_lastverlagerung_N = (
                    zusatzkraefte_lastverlagerung_N[rad_key]
                )

                kraefte = berechne_kraefte_einer_rad_ecke(
                    zustand_rad=zustand[rad_key],
                    strassenanregung_z_rel_local_m=strassenanregung_z_rel_local_m,
                    parameter=parameter,
                    zusatzkraft_lastverlagerung_N=(
                        zusatzkraft_lastverlagerung_N
                    ),
                )

                integriere_eine_rad_ecke(
                    zustand_rad=zustand[rad_key],
                    kraefte=kraefte,
                    dt_s=sub_dt_s,
                )

                letzte_kraefte[rad_key] = kraefte

        for rad_key in RAD_KEYS:
            kraefte = letzte_kraefte[rad_key]
            strassenanregung_z_rel_local_m = (
                strassenanregungen_z_rel_local_m[rad_key]
            )

            output_row[
                "strassenanregung_z_rel_local_" + rad_key + "_m"
            ] = strassenanregung_z_rel_local_m

            output_row[
                "gefedertemasse_z_local_" + rad_key + "_m"
            ] = zustand[rad_key]["gefedertemasse_z_local_m"]

            output_row[
                "gefedertemasse_geschwindigkeit_local_" + rad_key + "_mps"
            ] = zustand[rad_key]["gefedertemasse_geschwindigkeit_local_mps"]

            output_row[
                "gefedertemasse_beschleunigung_local_" + rad_key + "_mps2"
            ] = zustand[rad_key]["gefedertemasse_beschleunigung_local_mps2"]

            output_row[
                "ungefedertemasse_z_local_" + rad_key + "_m"
            ] = zustand[rad_key]["ungefedertemasse_z_local_m"]

            output_row[
                "ungefedertemasse_geschwindigkeit_local_" + rad_key + "_mps"
            ] = zustand[rad_key]["ungefedertemasse_geschwindigkeit_local_mps"]

            output_row[
                "ungefedertemasse_beschleunigung_local_" + rad_key + "_mps2"
            ] = zustand[rad_key]["ungefedertemasse_beschleunigung_local_mps2"]

            output_row[
                "reifen_eindrueckung_" + rad_key + "_m"
            ] = kraefte["reifen_eindrueckung_m"]

            output_row[
                "reifen_kraft_" + rad_key + "_N"
            ] = kraefte["reifen_kraft_N"]

            output_row[
                "feder_kraft_" + rad_key + "_N"
            ] = kraefte["feder_kraft_N"]

            output_row[
                "daempfer_kraft_" + rad_key + "_N"
            ] = kraefte["daempfer_kraft_N"]

            output_row[
                "feder_daempfer_kraft_" + rad_key + "_N"
            ] = kraefte["feder_daempfer_kraft_N"]

            output_row[
                "zusatzkraft_lastverlagerung_" + rad_key + "_N"
            ] = zusatzkraefte_lastverlagerung_N[rad_key]

        # ------------------------------------------------------------
        # 2. Karosserie aus vier Zustaenden der gefederten Masse rekonstruieren
        # ------------------------------------------------------------

        (
            karosserie_hub_local_m,
            karosserie_nicken_rad,
            karosserie_rollen_rad,
        ) = rekonstruiere_karosserie(
            zustand=zustand,
            car_radstand_mean_m=car_radstand_mean_m,
            car_spurweite_mean_m=car_spurweite_mean_m,
        )

        output_row["karosserie_hub_local_m"] = karosserie_hub_local_m
        output_row["karosserie_nicken_rad"] = karosserie_nicken_rad
        output_row["karosserie_rollen_rad"] = karosserie_rollen_rad

        # ------------------------------------------------------------
        # 3. Karosserie-Hoehe am Rad und Federweg fuer Baker berechnen
        # ------------------------------------------------------------

        for rad_key in RAD_KEYS:
            rad_offset_x_local_m = float(
                row["rad_offset_x_local_" + rad_key + "_m"]
            )

            rad_offset_y_local_m = float(
                row["rad_offset_y_local_" + rad_key + "_m"]
            )

            karosserie_z_am_rad_m = (
                karosserie_hub_local_m
                + karosserie_nicken_rad * rad_offset_y_local_m
                - karosserie_rollen_rad * rad_offset_x_local_m
            )

            rad_federweg_raw_z_local_m = (
                zustand[rad_key]["ungefedertemasse_z_local_m"]
                - karosserie_z_am_rad_m
            )

            rad_federweg_z_local_m = clamp(
                rad_federweg_raw_z_local_m,
                -parameter["MAX_AUSFEDERUNG_M"],
                parameter["MAX_EINFEDERUNG_M"],
            )

            output_row[
                "karosserie_z_am_rad_" + rad_key + "_m"
            ] = karosserie_z_am_rad_m

            output_row[
                "rad_federweg_raw_z_local_" + rad_key + "_m"
            ] = rad_federweg_raw_z_local_m

            output_row[
                "rad_federweg_z_local_" + rad_key + "_m"
            ] = rad_federweg_z_local_m

        output_rows.append(output_row)

        if index % 50 == 0:
            print("Frame berechnet:", frame)

    print("============================================================")
    print("FERTIG: Berechnung ueber alle Frames abgeschlossen.")
    print("============================================================")

    return output_rows


# Hauptprogramm
# =============================================================================
def main() -> None:
    projekt_ordner = Path(__file__).resolve().parent

    road_input_pfad = projekt_ordner / "road_input.csv"
    parameter_pfad = projekt_ordner / "vehicle_parameter.json"

    print("============================================================")
    print("Viertelfahrzeugmodell Solver - Eingangsdaten Check")
    print("============================================================")

    print("Projektordner:")
    print(projekt_ordner)

    print("road_input.csv:")
    print(road_input_pfad)

    print("vehicle_parameter.json:")
    print(parameter_pfad)

    parameter = lade_parameter(parameter_pfad)

    print("------------------------------------------------------------")
    print("Geladene Parameter:")
    print("FAHRZEUGMASSE_KG:", parameter["FAHRZEUGMASSE_KG"])
    print("GEFEDERTE_MASSE_ANTEIL:", parameter["GEFEDERTE_MASSE_ANTEIL"])
    print("GEFEDERTEMASSE_PRO_RAD_KG:", parameter["GEFEDERTEMASSE_PRO_RAD_KG"])
    print("UNGEFEDERTEMASSE_PRO_RAD_KG:", parameter["UNGEFEDERTEMASSE_PRO_RAD_KG"])
    print("FEDERSTEIFIGKEIT_N_PRO_M:", parameter["FEDERSTEIFIGKEIT_N_PRO_M"])
    print("DAEMPFERKONSTANTE_N_S_PRO_M:", parameter["DAEMPFERKONSTANTE_N_S_PRO_M"])
    print("REIFENSTEIFIGKEIT_N_PRO_M:", parameter["REIFENSTEIFIGKEIT_N_PRO_M"])
    print("MAX_EINFEDERUNG_M:", parameter["MAX_EINFEDERUNG_M"])
    print("MAX_AUSFEDERUNG_M:", parameter["MAX_AUSFEDERUNG_M"])
    print("LASTVERLAGERUNG_AKTIV:", parameter["LASTVERLAGERUNG_AKTIV"])
    print(
        "LASTVERLAGERUNG_LAENGS_AKTIV:",
        parameter["LASTVERLAGERUNG_LAENGS_AKTIV"],
    )
    print(
        "LASTVERLAGERUNG_QUER_AKTIV:",
        parameter["LASTVERLAGERUNG_QUER_AKTIV"],
    )

    if not road_input_pfad.exists():
        raise FileNotFoundError(
            "road_input.csv nicht gefunden: "
            + str(road_input_pfad)
        )

    road_input_df = pd.read_csv(road_input_pfad)

    pruefe_road_input_spalten(
        road_input_df=road_input_df,
        parameter=parameter,
    )
    pruefe_road_input_werte(
        road_input_df=road_input_df,
        parameter=parameter,
    )

    print("------------------------------------------------------------")
    print("road_input.csv gelesen.")
    print("Anzahl Frames:", len(road_input_df))
    print("Anzahl Spalten:", len(road_input_df.columns))

    erste_row = road_input_df.iloc[0]

    print("------------------------------------------------------------")
    print("Kontrolle des ersten Frames:")
    print("frame:", int(erste_row["frame"]))
    print("t_s:", float(erste_row["t_s"]))
    print("dt_s:", float(erste_row["dt_s"]))

    car_spurweite_mean_m = float(erste_row["car_spurweite_mean_m"])
    car_radstand_mean_m = float(erste_row["car_radstand_mean_m"])

    print("car_spurweite_mean_m:", car_spurweite_mean_m)
    print("car_radstand_mean_m:", car_radstand_mean_m)

    (
        zusatzkraefte_lastverlagerung_N,
        lastverlagerung_laengs_kraft_N,
        lastverlagerung_quer_kraft_N,
    ) = berechne_zusatzkraefte_lastverlagerung(
        row=erste_row,
        parameter=parameter,
        car_spurweite_mean_m=car_spurweite_mean_m,
        car_radstand_mean_m=car_radstand_mean_m,
    )

    print(
        "lastverlagerung_laengs_kraft_N:",
        lastverlagerung_laengs_kraft_N,
    )
    print(
        "lastverlagerung_quer_kraft_N:",
        lastverlagerung_quer_kraft_N,
    )

    for rad_key in RAD_KEYS:
        strassenanregung_z_rel_local_m = (
            lese_strassenanregung_z_rel_local_m(
                row=erste_row,
                rad_key=rad_key,
            )
        )

        print(
            "strassenanregung_z_rel_local_"
            + rad_key
            + "_m:",
            strassenanregung_z_rel_local_m,
        )
        print(
            "zusatzkraft_lastverlagerung_"
            + rad_key
            + "_N:",
            zusatzkraefte_lastverlagerung_N[rad_key],
        )

    print("============================================================")
    print("FERTIG: Eingangsdaten sind grundsaetzlich plausibel.")
    print("============================================================")

    output_pfad = projekt_ordner / "vehicle_response.csv"

    output_rows = berechne_vehicle_response_rows(
        road_input_df=road_input_df,
        parameter=parameter,
    )

    schreibe_vehicle_response_csv(
        output_rows=output_rows,
        output_pfad=output_pfad,
    )


if __name__ == "__main__":
    main()
