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
```

`Vehicle_Dynamics_Toolchain.py` enthaelt Exporter, Solver, Baker und Blender-UI
in einer Datei. Die einzelnen Stufen koennen getrennt ausgefuehrt werden; mit
**Run Complete Pipeline** laufen sie nacheinander.

## Modellidee

Das Modell verwendet vier unabhaengige Viertelfahrzeugmodelle fuer die Ecken
VL, VR, HL und HR. Jede Ecke besitzt eine gefederte und eine ungefederte Masse,
eine Feder, einen Daempfer und einen Reifen. Aus den vier vertikalen Antworten
werden anschliessend Karosserie-Hub, Nicken und Rollen kinematisch rekonstruiert.

Es handelt sich um ein vereinfachtes Visualisierungsmodell, nicht um ein
vollstaendiges Mehrkoerper-Fahrdynamikmodell. Die Vorzeichen der Laengs- und
Querlastverlagerung muessen im konkreten Blender-Achsensystem visuell geprueft
werden.

## Voraussetzungen

- Blender 3.6 oder neuer
- gespeicherte `.blend`-Datei
- keine zusaetzlichen Python-Pakete
- passende Objektnamen in der Blender-Szene

### Eingabeobjekte

| Funktion | Objektname |
| --- | --- |
| Lokale Fahrzeugreferenz und Fahrtrajektorie | `Fahrzeug_Pfadsteuerung` |
| Raycast-Kollider der Fahrbahn | `Fahrbahn_Kollider` |
| Rad vorne links | `Rad_VorneLinks` |
| Rad vorne rechts | `Rad_VorneRechts` |
| Rad hinten links | `Rad_HintenLinks` |
| Rad hinten rechts | `Rad_HintenRechts` |

### Bake-Ziele

| Funktion | Objektname |
| --- | --- |
| Karosserie | `Karosserie_Steuerung` |
| Federung vorne links | `Rad_VorneLinks_Federung_Steuerung` |
| Federung vorne rechts | `Rad_VorneRechts_Federung_Steuerung` |
| Federung hinten links | `Rad_HintenLinks_Federung_Steuerung` |
| Federung hinten rechts | `Rad_HintenRechts_Federung_Steuerung` |

`local` bezeichnet in der gesamten Toolchain ausschliesslich das lokale
Koordinatensystem von `Fahrzeug_Pfadsteuerung`.

## Installation und Start

1. `Vehicle_Dynamics_Toolchain.py` in Blender im Workspace **Scripting**
   oeffnen.
2. **Run Script** ausfuehren.
3. Im 3D Viewport mit `N` die Sidebar oeffnen.
4. Den Tab **Road Input** waehlen.

Das Ausfuehren des Skripts registriert nur die Benutzeroberflaeche. Export,
Solver und Baker starten erst durch einen Button.

## Bedienung

1. **Validate Setup** prueft Objekte, Szene, Parameter und Raycasts, ohne Dateien
   zu exportieren oder Keyframes zu schreiben.
2. **1. Validate + Export Inputs** erzeugt `road_input.csv` und
   `vehicle_parameter.json` im Ordner der `.blend`-Datei.
3. **2. Solve vehicle_response.csv** berechnet die Fahrzeugantwort.
4. **3. Bake Animation** schreibt die Antwort als Keyframes in die Bake-Ziele.
5. **Run Complete Pipeline** fuehrt alle vier Schritte in dieser Reihenfolge aus.

Nach Aenderungen an Fahrbahn, Fahrspur, Objektgeometrie oder Animation muessen
die Eingabedateien erneut exportiert und der Solver erneut ausgefuehrt werden.

## Exporter

Der Exporter wertet jeden Frame des Blender-Framebereichs aus. Er bestimmt:

- Zeit, Fahrzeugposition, Gierwinkel, Geschwindigkeit und Beschleunigung
- Radpositionen, Spurweite und Radstand
- Fahrbahnhoehe unter jedem Rad per Center-Raycast
- erforderliche Reifenhoehe per Fuenfpunkt-Reifenhuellen-Raycast
- Diagnosewerte fuer Treffer und Trefferanzahl

Die Reifenhuellen-Anregung wird vom Solver bevorzugt. Nur wenn diese Spalte
fehlt, wird auf den Center-Raycast zurueckgegriffen. Der Exporter schreibt keine
Keyframes und stellt den urspruenglichen Blender-Frame wieder her.

## Solverparameter

Die Parameter werden bewusst in `vehicle_parameter.json` gespeichert und nicht
mit den gemessenen Frame-Daten in `road_input.csv` vermischt.

Wichtige Gruppen:

- Fahrzeugmasse und Verteilung auf gefederte/ungefederte Masse
- Feder-, Daempfer- und Reifensteifigkeit
- Ein- und Ausfederungsgrenzen
- Laengs- und Querlastverlagerung
- Schwerpunkt-Hoehe und Vorzeichenkonventionen
- Roll-Skalierung
- numerische Unterschritte pro Frame

### Filter fuer Laengsbeschleunigung

Die Laengsbeschleunigung durchlaeuft drei Stufen:

1. **Spitzenfilter:** Vergleich mit dem lokalen Median. Eine Abweichung oberhalb
   der Schwelle wird durch den Median ersetzt.
2. **Glaettung:** gleitender Mittelwert ueber das konfigurierte Framefenster.
3. **Totzone:** kleine geglaettete Werte werden auf null gesetzt.

`vehicle_response.csv` enthaelt Raw-, Despiked-, Smoothed- und Solver-Wert.
Dadurch kann jede Stufe in Excel oder einem anderen CSV-Tool kontrolliert
werden. Ein groesseres Fenster entfernt nicht automatisch nur mehr Spitzen:
Fenster und Schwelle muessen gemeinsam passend zur Framerate gewaehlt werden.

## Solver

Pro Frame wird fuer jede Fahrzeugecke:

1. die Strassenanregung gelesen,
2. Reifen-, Feder- und Daempferkraft berechnet,
3. die Lastverlagerung als zusaetzliche Kraft angesetzt,
4. Beschleunigung, Geschwindigkeit und Weg in Unterschritten integriert,
5. der Federweg auf die vorgegebenen Grenzen begrenzt.

Aus den vier gefederten Massen werden danach Hub, Nicken und Rollen berechnet.
Die Ergebnisse und Diagnosegroessen werden sicher ueber eine temporaere Datei
in `vehicle_response.csv` geschrieben.

## Baker

Der Baker validiert zuerst die Pflichtspalten der `vehicle_response.csv`. Danach
loescht er die vorhandenen Animationsdaten der Bake-Ziele und schreibt:

- Karosserie-Z, Nicken und Rollen auf `Karosserie_Steuerung`
- vertikale Bewegung der vier Federungsobjekte

Der beim Start aktive Frame wird nach dem Bake wiederhergestellt.

## Erzeugte Dateien

| Datei | Erzeuger | Zweck |
| --- | --- | --- |
| `road_input.csv` | Exporter | Geometrie, Kinematik und Strassenanregung pro Frame |
| `vehicle_parameter.json` | Exporter/UI | Solverparameter |
| `vehicle_response.csv` | Solver | Zustaende, Kraefte, Filterwerte und Bake-Groessen |

## Hinweise zur Diagnose

- **Blend-Datei ist nicht gespeichert:** Projekt zuerst speichern, da alle
  Dateien daneben geschrieben werden.
- **Objekt fehlt:** Namen exakt mit der Tabelle oben abgleichen.
- **Kein Raycast-Treffer:** Kollidermesh, Position, Ray-Distanz und Radradius
  pruefen.
- **Falsche Nick- oder Rollrichtung:** Vorzeichenparameter an das tatsaechliche
  Blender-Achsensystem anpassen und visuell validieren.
- **Kurzer Nickimpuls auf gerader Strecke:** In `vehicle_response.csv` die vier
  Laengsbeschleunigungsstufen vergleichen und erst dann Filterfenster oder
  Schwelle anpassen.

