import bpy
import csv
from pathlib import Path


KAROSSERIE_OBJEKT_NAME = "Karosserie_Steuerung"

RAD_FEDERUNG_OBJEKT_NAMEN = {
    "VL": "Rad_VorneLinks_Federung_Steuerung",
    "VR": "Rad_VorneRechts_Federung_Steuerung",
    "HL": "Rad_HintenLinks_Federung_Steuerung",
    "HR": "Rad_HintenRechts_Federung_Steuerung",
}

RESPONSE_CSV_DATEINAME = "vehicle_response.csv"

KAROSSERIE_BASIS_Z_LOCAL_M = 0.8

RAD_FEDERUNG_BASIS_Z_LOCAL_M = {
    "VL": 0.32,
    "VR": 0.32,
    "HL": 0.32,
    "HR": 0.32,
}

KAROSSERIE_BASIS_NICKEN_RAD = 0.0
KAROSSERIE_BASIS_ROLLEN_RAD = 0.0   

ERFORDERLICHE_RESPONSE_KOPFZEILE = [
    "frame",
    "t_s",
    "dt_s",

    "karosserie_hub_local_m",
    "karosserie_nicken_rad",
    "karosserie_rollen_rad",

    "karosserie_z_am_rad_VL_m",
    "karosserie_z_am_rad_VR_m",
    "karosserie_z_am_rad_HL_m",
    "karosserie_z_am_rad_HR_m",

    "rad_federweg_z_local_VL_m",
    "rad_federweg_z_local_VR_m",
    "rad_federweg_z_local_HL_m",
    "rad_federweg_z_local_HR_m",
]

# Blender-Ordner holen
# ============================================================
def hole_blend_ordner() -> Path:
    if bpy.data.filepath == "":
        raise RuntimeError(
            "Die .blend-Datei ist noch nicht gespeichert. "
            "Bitte zuerst die Blender-Datei speichern."
        )

    blend_ordner = Path(bpy.path.abspath("//"))
    return blend_ordner


# vehicle_response.csv Pfad holen
# ============================================================
def hole_vehicle_response_csv_pfad() -> Path:
    blend_ordner = hole_blend_ordner()

    csv_pfad = blend_ordner / RESPONSE_CSV_DATEINAME

    if not csv_pfad.exists():
        raise FileNotFoundError(
            "vehicle_response.csv wurde nicht gefunden.\n"
            + "Erwarteter Pfad:\n"
            + str(csv_pfad)
        )

    return csv_pfad


# vehicle_response.csv lesen
# ============================================================
def lese_vehicle_response_csv(csv_pfad: Path) -> tuple[list, list]:
    with csv_pfad.open(
        mode="r",
        newline="",
        encoding="utf-8-sig",
    ) as csv_datei:
        csv_reader = csv.DictReader(csv_datei)

        if csv_reader.fieldnames is None:
            raise RuntimeError(
                "vehicle_response.csv konnte nicht gelesen werden. "
                "Die Datei hat keine Kopfzeile."
            )

        response_kopfzeile = list(csv_reader.fieldnames)
        response_datenzeilen = list(csv_reader)

    if len(response_datenzeilen) == 0:
        raise RuntimeError(
            "vehicle_response.csv wurde gelesen, "
            "enthaelt aber keine Datenzeilen."
        )

    return response_datenzeilen, response_kopfzeile


# response_kopfzeile pruefen
# ============================================================
def pruefe_response_kopfzeile(response_kopfzeile: list) -> None:
    fehlende_kopfzeile_eintraege = []

    for kopfzeile_eintrag in ERFORDERLICHE_RESPONSE_KOPFZEILE:
        if kopfzeile_eintrag not in response_kopfzeile:
            fehlende_kopfzeile_eintraege.append(kopfzeile_eintrag)

    if len(fehlende_kopfzeile_eintraege) > 0:
        meldung = (
            "Folgende erforderliche Eintraege fehlen "
            "in der Kopfzeile von vehicle_response.csv:\n"
        )

        for kopfzeile_eintrag in fehlende_kopfzeile_eintraege:
            meldung += "- " + kopfzeile_eintrag + "\n"

        raise RuntimeError(meldung)


# Float-Wert aus response_datenzeile lesen
# ============================================================
def lese_float_aus_response_datenzeile(
    response_datenzeile: dict,
    kopfzeile_eintrag: str,
) -> float:
    wert_text = response_datenzeile.get(kopfzeile_eintrag)

    if wert_text is None or wert_text == "":
        frame_text = response_datenzeile.get("frame", "unbekannt")

        raise RuntimeError(
            "Leerer Zahlenwert in vehicle_response.csv:\n"
            + "Frame: "
            + str(frame_text)
            + "\n"
            + "Kopfzeile-Eintrag: "
            + kopfzeile_eintrag
        )

    try:
        wert_float = float(wert_text)
    except Exception as fehler:
        frame_text = response_datenzeile.get("frame", "unbekannt")

        raise RuntimeError(
            "Zahlenwert konnte nicht gelesen werden:\n"
            + "Frame: "
            + str(frame_text)
            + "\n"
            + "Kopfzeile-Eintrag: "
            + kopfzeile_eintrag
            + "\n"
            + "Wert: "
            + str(wert_text)
        ) from fehler

    return wert_float


# Zielobjekte in Blender pruefen und holen
# =============================================================
def hole_zielobjekte() -> dict:
    zielobjekte = {
        "karosserie": None,
        "rad_federung": {},
    }

    fehlende_objekt_namen = []

    karosserie_objekt = bpy.data.objects.get(KAROSSERIE_OBJEKT_NAME)

    if karosserie_objekt is None:
        fehlende_objekt_namen.append(KAROSSERIE_OBJEKT_NAME)
    else:
        zielobjekte["karosserie"] = karosserie_objekt

    for rad_key, rad_federung_objekt_name in RAD_FEDERUNG_OBJEKT_NAMEN.items():
        rad_federung_objekt = bpy.data.objects.get(
            rad_federung_objekt_name
        )

        if rad_federung_objekt is None:
            fehlende_objekt_namen.append(rad_federung_objekt_name)
        else:
            zielobjekte["rad_federung"][rad_key] = rad_federung_objekt

    if len(fehlende_objekt_namen) > 0:
        meldung = "Folgende Zielobjekte fehlen in Blender:\n"

        for objekt_name in fehlende_objekt_namen:
            meldung += "- " + objekt_name + "\n"

        raise RuntimeError(meldung)

    return zielobjekte


# Karosserie bake
# =============================================================
def bake_karosserie(
    response_datenzeilen: list,
    karosserie_objekt,
) -> None:
    karosserie_objekt.animation_data_clear()

    for response_datenzeile in response_datenzeilen:
        frame = int(
            lese_float_aus_response_datenzeile(
                response_datenzeile=response_datenzeile,
                kopfzeile_eintrag="frame",
            )
        )

        karosserie_hub_local_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_hub_local_m",
        )

        karosserie_nicken_rad = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_nicken_rad",
        )

        karosserie_rollen_rad = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_rollen_rad",
        )

        bpy.context.scene.frame_set(frame)

        karosserie_objekt.location.z = (
            KAROSSERIE_BASIS_Z_LOCAL_M
            + karosserie_hub_local_m
        )

        karosserie_objekt.rotation_euler.x = (
            KAROSSERIE_BASIS_NICKEN_RAD
            + karosserie_nicken_rad
        )

        karosserie_objekt.rotation_euler.y = (
            KAROSSERIE_BASIS_ROLLEN_RAD
            + karosserie_rollen_rad
        )

        karosserie_objekt.keyframe_insert(
            data_path="location",
            frame=frame,
        )

        karosserie_objekt.keyframe_insert(
            data_path="rotation_euler",
            frame=frame,
        )

# Rad-Federung bake
# =============================================================
def bake_rad_federung(
    response_datenzeilen: list,
    rad_federung_objekte: dict,
) -> None:
    for rad_key, rad_federung_objekt in rad_federung_objekte.items():
        rad_federung_objekt.animation_data_clear()

    for response_datenzeile in response_datenzeilen:
        frame = int(
            lese_float_aus_response_datenzeile(
                response_datenzeile=response_datenzeile,
                kopfzeile_eintrag="frame",
            )
        )

        karosserie_z_am_rad_VL_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_z_am_rad_VL_m",
        )

        karosserie_z_am_rad_VR_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_z_am_rad_VR_m",
        )

        karosserie_z_am_rad_HL_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_z_am_rad_HL_m",
        )

        karosserie_z_am_rad_HR_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="karosserie_z_am_rad_HR_m",
        )

        rad_federweg_z_local_VL_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="rad_federweg_z_local_VL_m",
        )

        rad_federweg_z_local_VR_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="rad_federweg_z_local_VR_m",
        )

        rad_federweg_z_local_HL_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="rad_federweg_z_local_HL_m",
        )

        rad_federweg_z_local_HR_m = lese_float_aus_response_datenzeile(
            response_datenzeile=response_datenzeile,
            kopfzeile_eintrag="rad_federweg_z_local_HR_m",
        )

        bpy.context.scene.frame_set(frame)

        rad_federung_objekte["VL"].location.z = (
            RAD_FEDERUNG_BASIS_Z_LOCAL_M["VL"]
            + karosserie_z_am_rad_VL_m
            + rad_federweg_z_local_VL_m
        )

        rad_federung_objekte["VR"].location.z = (
            RAD_FEDERUNG_BASIS_Z_LOCAL_M["VR"]
            + karosserie_z_am_rad_VR_m
            + rad_federweg_z_local_VR_m
        )

        rad_federung_objekte["HL"].location.z = (
            RAD_FEDERUNG_BASIS_Z_LOCAL_M["HL"]
            + karosserie_z_am_rad_HL_m
            + rad_federweg_z_local_HL_m
        )

        rad_federung_objekte["HR"].location.z = (
            RAD_FEDERUNG_BASIS_Z_LOCAL_M["HR"]
            + karosserie_z_am_rad_HR_m
            + rad_federweg_z_local_HR_m
        )

        rad_federung_objekte["VL"].keyframe_insert(
            data_path="location",
            frame=frame,
        )

        rad_federung_objekte["VR"].keyframe_insert(
            data_path="location",
            frame=frame,
        )

        rad_federung_objekte["HL"].keyframe_insert(
            data_path="location",
            frame=frame,
        )

        rad_federung_objekte["HR"].keyframe_insert(
            data_path="location",
            frame=frame,
        )


        
# Main
# =============================================================
def main() -> None:
    csv_pfad = hole_vehicle_response_csv_pfad()

    response_datenzeilen, response_kopfzeile = lese_vehicle_response_csv(
        csv_pfad=csv_pfad,
    )

    pruefe_response_kopfzeile(
        response_kopfzeile=response_kopfzeile,
    )

    zielobjekte = hole_zielobjekte()

    bake_karosserie(
        response_datenzeilen=response_datenzeilen,
        karosserie_objekt=zielobjekte["karosserie"],
    )

    bake_rad_federung(
        response_datenzeilen=response_datenzeilen,
        rad_federung_objekte=zielobjekte["rad_federung"],
    )

    print("============================================================")
    print("Vehicle Response Baker - Bake OK")
    print("============================================================")
    print("CSV:", csv_pfad)
    print("Datenzeilen:", len(response_datenzeilen))
    print("Kopfzeile: OK")
    print("Zielobjekte: OK")
    print("Karosserie gebaked: OK")
    print("Rad-Federung gebaked: OK")
    print("============================================================")


main()