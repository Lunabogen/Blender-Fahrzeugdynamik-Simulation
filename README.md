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

* gespeicherte `.blend`-Datei
* keine zusaetzlichen Python-Pakete
* passende Objektnamen in der Blender-Szene

### Eingabeobjekte

| Funktion                                    | Objektname               |
| ------------------------------------------- | ------------------------ |
| Lokale Fahrzeugreferenz und Fahrtrajektorie | `Fahrzeug_Pfadsteuerung` |
| Raycast-Kollider der Fahrbahn               | `Fahrbahn_Kollider`      |
| Rad vorne links                             | `Rad_VorneLinks`         |
| Rad vorne rechts                            | `Rad_VorneRechts`        |
| Rad hinten links                            | `Rad_HintenLinks`        |
| Rad hinten rechts                           | `Rad_HintenRechts`       |

### Bake-Ziele

| Funktion               | Objektname                            |
| ---------------------- | ------------------------------------- |
| Karosserie             | `Karosserie_Steuerung`                |
| Federung vorne links   | `Rad_VorneLinks_Federung_Steuerung`   |
| Federung vorne rechts  | `Rad_VorneRechts_Federung_Steuerung`  |
| Federung hinten links  | `Rad_HintenLinks_Federung_Steuerung`  |
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

### Ausgabepfade

Im Bereich **Output Paths** koennen die Ziele fuer `road_input.csv`,
`vehicle_parameter.json` und `vehicle_response.csv` frei gewaehlt werden. Die
Standardwerte beginnen mit `//`; in Blender bedeutet das den Ordner der aktuell
gespeicherten `.blend`-Datei. Solver und Baker verwenden automatisch denselben
konfigurierten Pfad fuer `vehicle_response.csv`.

## Bedienung

1. **Validate Setup** prueft Objekte, Szene, Parameter und Raycasts, ohne Dateien
   zu exportieren oder Keyframes zu schreiben.
2. **1. Validate + Export Inputs** erzeugt `road_input.csv` und
   `vehicle_parameter.json`.
3. **2. Solve vehicle_response.csv** berechnet die Fahrzeugantwort.
4. **3. Bake Animation** schreibt die Antwort als Keyframes in die Bake-Ziele.
5. **Run Complete Pipeline** fuehrt alle vier Schritte in dieser Reihenfolge aus.

Nach Aenderungen an Fahrbahn, Fahrspur, Objektgeometrie oder Animation muessen
die Eingabedateien erneut exportiert und der Solver erneut ausgefuehrt werden.

## Exporter

Der Exporter wertet jeden Frame des Blender-Framebereichs aus. Er bestimmt:

* Zeit, Fahrzeugposition, Gierwinkel, Geschwindigkeit und Beschleunigung
* Radpositionen, Spurweite und Radstand
* Fahrbahnhoehe unter jedem Rad per Center-Raycast
* erforderliche Reifenhoehe per Fuenfpunkt-Reifenhuellen-Raycast
* Diagnosewerte fuer Treffer und Trefferanzahl

Die Reifenhuellen-Anregung wird vom Solver bevorzugt. Nur wenn diese Spalte
fehlt, wird auf den Center-Raycast zurueckgegriffen. Der Exporter schreibt keine
Keyframes und stellt den urspruenglichen Blender-Frame wieder her.

## Solverparameter

Die Parameter werden bewusst in `vehicle_parameter.json` gespeichert und nicht
mit den gemessenen Frame-Daten in `road_input.csv` vermischt.

Wichtige Gruppen:

* Fahrzeugmasse und Verteilung auf gefederte/ungefederte Masse
* Feder-, Daempfer- und Reifensteifigkeit
* Ein- und Ausfederungsgrenzen
* Laengs- und Querlastverlagerung
* Schwerpunkt-Hoehe und Vorzeichenkonventionen
* Roll-Skalierung
* numerische Unterschritte pro Frame

### Parameter fuer Lastverlagerung und Beschleunigungsfilter

* **Schwerpunkt-Hoehe [m]:** Dieser Wert beschreibt, wie hoch der Schwerpunkt
  des Fahrzeugs ueber der Fahrbahn liegt. Ein groesserer Wert erzeugt
  staerkeres Nicken und Rollen, ein kleinerer Wert macht die
  Karosseriebewegung schwaecher.

* **Laengs-Vorzeichen:** Dieser Wert bestimmt die Richtung der Nickbewegung beim
  Beschleunigen und Bremsen. Wenn das Fahrzeug in die falsche Richtung nickt,
  den Wert zwischen `1` und `-1` wechseln.

* **Quer-Vorzeichen:** Dieser Wert bestimmt die Richtung der Rollbewegung in
  Kurven. Wenn sich die Karosserie zur falschen Seite neigt, den Wert zwischen
  `1` und `-1` wechseln.

* **Laengs-Spitzenfilter [Frames]:** Dieser Wert bestimmt, wie viele aktuelle
  und vergangene Frames zum Erkennen kurzer unpassender Spruenge in der
  Laengsbeschleunigung verglichen werden. Ein groesserer Wert entfernt eher
  laengere Spruenge, kann aber echte schnelle Aenderungen staerker verzoegern;
  `1` schaltet den Filter praktisch aus.

* **Spitzen-Schwelle [m/s²]:** Dieser Wert bestimmt, wie stark die aktuelle
  Laengsbeschleunigung von den vorherigen Werten abweichen darf, bevor sie
  ersetzt wird. Ein kleinerer Wert filtert strenger, ein groesserer Wert
  behaelt mehr Originaldaten.

* **Laengs-Glaettungsfenster [Frames]:** Dieser Wert bestimmt, ueber wie viele
  aktuelle und vergangene Werte ein Durchschnitt fuer die Nickbewegung gebildet
  wird. Ein groesserer Wert macht die Bewegung ruhiger, aber langsamer; `1`
  schaltet die Glaettung aus.

* **Laengs-Totzone [m/s²]:** Dieser Wert legt fest, welche kleinen
  Laengsbeschleunigungen als null behandelt werden. Ein groesserer Wert
  unterdrueckt mehr leichtes Nicken, kann aber auch sanfte reale
  Beschleunigungen entfernen.

* **Quer-Spitzenfilter [Frames]:** Dieser Wert funktioniert wie der
  Laengs-Spitzenfilter, jedoch fuer seitliche Beschleunigung und damit fuer das
  Rollen. Ein groesserer Wert entfernt eher kurze seitliche Spruenge, kann aber
  den Beginn echter Kurvenbewegungen verzoegern.

* **Quer-Spitzen-Schwelle [m/s²]:** Dieser Wert bestimmt, wie stark die aktuelle
  Querbeschleunigung von den vorherigen Werten abweichen darf. Ein kleinerer
  Wert filtert strenger, ein groesserer Wert laesst mehr originale
  Kurvenbewegung bestehen.

* **Quer-Glaettungsfenster [Frames]:** Dieser Wert bestimmt, ueber wie viele
  aktuelle und vergangene Querwerte ein Durchschnitt gebildet wird. Ein
  groesserer Wert macht das Rollen ruhiger, aber traeger; `1` schaltet die
  Glaettung aus.

* **Quer-Totzone [m/s²]:** Dieser Wert legt fest, welche kleinen
  Querbeschleunigungen als null behandelt werden. Ein groesserer Wert
  verhindert leichtes Rollen auf gerader Strecke, kann aber sanfte
  Kurvenbewegungen entfernen.

* **Rollen-Skalierung:** Dieser Wert skaliert die fertig berechnete sichtbare
  Seitenneigung der Karosserie. `1` zeigt die volle Rollbewegung, `0,5` die
  Haelfte und `0` schaltet sie aus.

Die Fenster werden weiterhin in Frames angegeben. Bei einer Aenderung der
Framerate aendert sich deshalb auch ihre zeitliche Breite. Beispielsweise
entsprechen fuenf Frames bei 25 FPS einer Zeitspanne von 0,20 s und bei 50 FPS
einer Zeitspanne von 0,10 s.

### Filter fuer Laengs- und Querbeschleunigung

Laengs- und Querbeschleunigung durchlaufen jeweils drei Stufen:

1. **Spitzenfilter:** Vergleich mit dem lokalen Median. Eine Abweichung oberhalb
   der Schwelle wird durch den Median ersetzt.
2. **Glaettung:** gleitender Mittelwert ueber das konfigurierte Framefenster.
3. **Totzone:** kleine geglaettete Werte werden auf null gesetzt.

Beide Fenster sind kausal. Fuer Frame `i` verwendet ein Fenster der Breite `5`
nur die Frames `i-4` bis `i`; zukuenftige Frames werden nicht gelesen. Am Anfang
der Animation wird das Fenster automatisch auf die bereits vorhandenen Frames
verkuerzt. Dadurch beginnt die Filterwirkung nicht vor dem tatsaechlichen
Eingangssignal, groessere Glaettungsfenster koennen die Reaktion jedoch
verzoegern.

`vehicle_response.csv` enthaelt fuer beide Beschleunigungsrichtungen Raw-,
Despiked-, Smoothed- und Solver-Wert. Dadurch kann jede Stufe in Excel oder
einem anderen CSV-Tool kontrolliert werden. Ein groesseres Fenster entfernt
nicht automatisch nur mehr Spitzen: Fenster und Schwelle muessen gemeinsam
passend zur Framerate gewaehlt werden.

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

Der Baker validiert zuerst die Pflichtspalten der `vehicle_response.csv`.
Anschliessend entfernt er nur die zuvor vom Baker geschriebenen Keyframes der
kontrollierten Achsen und schreibt:

* Karosserie-Z, Nicken und Rollen auf `Karosserie_Steuerung`
* vertikale Bewegung der vier Federungsobjekte

Andere Achsen, vorhandene Driver und Animationen der untergeordneten
Lenkungs-, Sturz- und Drehungsobjekte bleiben erhalten. Der beim Start aktive
Frame wird nach dem Bake wiederhergestellt.

## Erzeugte Dateien

| Datei                    | Erzeuger    | Zweck                                                                        |
| ------------------------ | ----------- | ---------------------------------------------------------------------------- |
| `road_input.csv`         | Exporter    | Geometrie, Kinematik und Strassenanregung pro Frame; Pfad in der UI waehlbar |
| `vehicle_parameter.json` | Exporter/UI | Solverparameter; Pfad in der UI waehlbar                                     |
| `vehicle_response.csv`   | Solver      | Zustaende, Kraefte, Filterwerte und Bake-Groessen; Pfad in der UI waehlbar   |

## Hinweise zur Diagnose

* **Blend-Datei ist nicht gespeichert:** Projekt zuerst speichern.
* **Objekt fehlt:** Namen exakt mit der Tabelle oben abgleichen.
* **Kein Raycast-Treffer:** Kollidermesh, Position, Ray-Distanz und Radradius
  pruefen.
* **Falsche Nick- oder Rollrichtung:** Vorzeichenparameter an das tatsaechliche
  Blender-Achsensystem anpassen und visuell validieren.
* **Kurzer Nick- oder Rollimpuls:** In `vehicle_response.csv` Raw-, Despiked-,
  Smoothed- und Solver-Werte vergleichen und erst danach Filterfenster oder
  Schwelle anpassen.

## Lizenz

Vor einer externen Veroeffentlichung eine passende Lizenzdatei ergaenzen.
