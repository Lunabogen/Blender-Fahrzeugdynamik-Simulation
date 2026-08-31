# Blender-Fahrzeugdynamik-Simulation

Python-Toolchain zur Erfassung der Strassenanregung in Blender, zur Berechnung
einer vereinfachten vertikalen Fahrzeugreaktion und zum Backen der Ergebnisse
als Keyframes.

## Datenfluss

```text
Blender-Animation
    -> road_input.csv
    -> vehicle_parameter.json
    -> Viertelfahrzeug-Solver
    -> vehicle_response.csv
    -> Blender-Keyframes
