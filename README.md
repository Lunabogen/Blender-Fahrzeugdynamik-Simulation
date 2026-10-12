# Blender-Fahrzeugdynamik
## Kurzanleitung

**Vehicle Dynamics Toolchain v1.6.5** · README · 12. Oktober 2026

Modelle einsetzen, Fahrwege bearbeiten, Parameter einstellen und die Animation erzeugen. Die Anleitung folgt dem aktuellen Skript und enthält alle 30 Panel-Einstellungen.

## 1 Grundidee des Codes

PfadFolgen steuert Fahrposition und Ausrichtung. Das Skript liest diese Bewegung und die Fahrbahn je Frame, filtert die Beschleunigung, berechnet die vier Radreaktionen und erzeugt die Karosseriebewegung.

| **Schritt** | **Ergebnis** |
| ----------- | ----------------------------------------------------------------------------------------------- |
| Export | Fahrbewegung und Straßenhöhen als CSV; Panelparameter als JSON. |
| Solver | Federbewegung, Karosseriehub, Nicken und Rollen als Response-CSV. |
| Baker | Karosserie_Steuerung: Position Z und Rotation X/Y. Vier Federung_Steuerungen: nur Position Z. |

Lokale X/Y-Positionen und Skalierungen werden nicht gebacken. Lenkung und Raddrehung bleiben Aufgabe der vorhandenen Steuerung. Der erste Animationsframe dient als Höhenbasis; anfangs versunkene Räder werden nicht automatisch angehoben.

## 2 Modell ersetzen und binden
1. Modell importieren und in Karosserie sowie vier Räder aufteilen. Räder nicht mit der Karosserie verbinden.
2. Am Startframe Größe, Richtung und Radpositionen anpassen. Im Fahrzeugbezug zeigt +Y nach vorne, +X nach rechts und +Z nach oben.
3. Neue Meshes an die vorhandenen Zweige hängen. Erst das Mesh, zuletzt den Elternteil wählen; Strg + P und Objekt mit beibehaltenem Transform verwenden.

| **Neues Modellteil** | **Elternobjekt im vorhandenen Fahrzeug** |
| ----------------------- | ---------------------------------------------------------- |
| Karosserie | Karosserie als vorhandenes Kind von Karosserie_Steuerung |
| Vorne links / rechts | Rad_VorneLinks / Rad_VorneRechts |
| Hinten links / rechts | Rad_HintenLinks / Rad_HintenRechts |

Die Kette Federung → Sturz/Lenkung → Drehung → Rad bleibt erhalten. Rad…Mate/KaroMate und die vom Skript gesuchten Objekte nicht ungezielt löschen. Alte Radgeometrie darf die automatische Radiusmessung nicht verfälschen; auch versteckte Meshes werden berücksichtigt.

Radmitte und Drehung-Steuerung müssen dieselbe Achse haben. Bei Unwucht den übergeordneten Drehpunkt prüfen. Bei zu niedriger Gesamtposition die gemeinsame Fahrzeug_Hauptsteuerung anheben. Spurweite, Radstand und Radius werden beim Export gelesen. Steuerungen der Kopien tragen _02, _03 usw.; Namen der visuellen Meshes sind frei.

## 3 Modell kopieren und das Fahrzeug bearbeiten

### Ein neues Fahrzeug erstellen
1. In Objektmodus wechseln. Im Panel Road Input einen neuen Fahrzeug-Index eintragen, z. B. 02. Kopiervorlage ist immer 01.
2. Gemeinsamen Pfad verwenden: Aus = eigener Fahrweg, Ein = gemeinsamer Fahrweg mit 01. Diese Option gilt nur bei Neuerstellung.
3. Fahrzeug erstellen / bestätigen drücken. Die neue Kopie erhält _02 und liegt zunächst an derselben Stelle wie 01.

### Was wird kopiert

| **Bestandteil** | **Verhalten beim Erstellen** |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Steuerung und Hierarchie | Fahrzeug_Pfadsteuerung samt Kindern: Hauptsteuerung, Karosserie, Radsteuerungen, Lenkung und gebundene Objekte. Kameras nur, wenn sie in diesem Zweig liegen. |
| Vorausschau | Vorausschau_Steuerung und ihre Kinder werden auch als separater Zweig kopiert. |
| Modelle und Geometrie | Gebundene Meshes mit eigenen Mesh-Daten; ebenso Kurvendaten. Die Geometrie von 02 ist getrennt bearbeitbar. |
| Beschränkungen und Modifier | Einstellungen werden übernommen, interne Ziele auf 02 umgestellt. Shrinkwrap behält Fahrbahn_Kollider als Ziel. |
| Animation und Driver | Eigene Animationsaktionen, umgestellte Driver-Ziele. Alte Bake-Schlüssel auf Karosserie und Federung werden in der Kopie entfernt. |
| Fahrweg | Standard: eigener TestPfad_02 mit Einstellungen. Bei gemeinsamer Pfadoption wird der Pfad von 01 verwendet. |
| Gemeinsame Ressourcen | Stadt und Fahrbahn_Kollider werden nicht kopiert. Materialien, Texturen und Node-Gruppen können gemeinsam bleiben. |

### Fahrzeug auswählen und einzeln ändern
1. Für Export, Solver und Bake Fahrzeug-Index 02 einstellen und Fahrzeug erstellen / bestätigen drücken. Ein vorhandenes Fahrzeug wird nur bestätigt, nicht erneut komplett kopiert.
2. Im Outliner den Zweig mit _02 öffnen. Meshes: Karosserie-/Radzweig auswählen. Route: TestPfad_02. Timing: Versatzfaktor an Fahrzeug_Pfadsteuerung_02.
3. Nur den eigenen Pfad verschieben oder drehen.
4. Parameter für 02 im Panel einstellen und Run Complete Pipeline ausführen. Für 03 denselben Ablauf mit Index 03 wiederholen.

## 4 Panelparameter für Masse und Federbewegung

Panel öffnen: Skript im Texteditor ausführen, Maus in die 3D-Ansicht, N → Road Input. Die Werte unten sind Skriptstandards; gespeicherte Szenen können andere Werte enthalten. Wirkungen gelten bei sonst gleichen Einstellungen.

| **Parameter und Standard** | **Verwendung** | **Größer** | **Kleiner** |
| -------------------------------------------------- | ----------------------------------------------------- | ---------------------------------------------------------------- | ----------------------------------------------------------------- |
| Fahrzeugmasse [kg]<br> Standard 1800 | Fahrzeuggewicht für Berechnung und Lastverlagerung. | Mehr Trägheit und stärkere Lasttransferkräfte. | Weniger Trägheit und schwächere Lasttransferkräfte. |
| Gefederte-Masse-Anteil [-]<br> Standard 0,85 | Anteil der Masse, die auf der Federung liegt. | Karosserie reagiert meist langsamer. | Karosserie reagiert meist schneller. |
| Ungefederte Masse pro Rad [kg]<br> Standard 45 | Masse jedes Radbereichs. | Räder reagieren träger auf schnelle Unebenheiten. | Räder folgen Unebenheiten meist schneller. |
| Federsteifigkeit [N/m]<br> Standard 30 000 | Stärke der Federung. | Härter; bei langsamem Lastwechsel weniger Federbewegung. | Weicher; mehr Federbewegung und größere Karosserieneigung. |
| Daempferkonstante [N*s/m]<br> Standard 3000 | Unterdrückt Nachschwingen. | Meist weniger Nachschwingen; zu hoch macht die Bewegung träge. | Mehr und länger anhaltendes Nachschwingen. |
| Reifensteifigkeit [N/m]<br> Standard 200 000 | Vertikale Reifenreaktion. | Weniger Reifenkompression; schnelle Radbewegungen möglich. | Mehr Reifenkompression; starre Reifenmesh können tiefer wirken. |
| Max. Einfederung [m]<br> Standard 0,035 | Begrenzt den positiven gebackenen Federweg. | Mehr Radbewegung nach oben erlaubt. | Radbewegung nach oben wird früher abgeschnitten. |
| Max. Ausfederung [m]<br> Standard 0,035 | Begrenzt den negativen gebackenen Federweg. | Mehr Radbewegung nach unten erlaubt. | Radbewegung nach unten wird früher abgeschnitten. |
| Solver-Unterschritte pro Frame<br> Standard 20 | Genauigkeit und Stabilität der Berechnung. | Kleinere Rechenschritte; mehr Rechenzeit. | Schneller; höheres Risiko numerischer Schwingungen. |

Diese Werte ändern die berechnete Bewegung, nicht Meshgröße oder Pfadgeschwindigkeit. Die vier Räder verwenden dieselben Feder- und Reifenparameter. Federweggrenzen begrenzen die Ausgabe; sie simulieren keinen mechanischen Anschlag.

### Schnelle Anpassung

Zu starkes Nachschwingen: Daempferkonstante vorsichtig erhöhen. Zu große Federbewegung: Federsteifigkeit erhöhen. Versunkene Räder zuerst am Startframe ausrichten und Fahrbahn_Kollider prüfen; nicht nur die Federsteifigkeit verändern.

## 5 Panelparameter für Lastverlagerung und Filter

| **Parameter und Standard** | **Verwendung** | **Größer oder ein** | **Kleiner oder aus** |
| ---------------------------------------------------------------- | ------------------------------------------------------------------ | ------------------------------------------------------ | -------------------------------------------------------- |
| Lastverlagerung aktiv<br> Standard Ein | Gesamtschalter für Lasttransfer. | Ein: Reaktion auf Bremsen, Beschleunigen und Kurven. | Aus: nur Reaktion auf Fahrbahneingaben. |
| Laengs-Lastverlagerung aktiv<br> Standard Ein | Zusätzliche Reaktion auf Bremsen und Beschleunigen. | Ein: zusätzliches Nicken. | Aus: kein Nicken aus Längslasttransfer. |
| Quer-Lastverlagerung aktiv<br> Standard Ein | Zusätzliche Reaktion auf Kurvenfahrt. | Ein: zusätzliche Seitenneigung. | Aus: keine Seitenneigung aus Querlasttransfer. |
| Schwerpunkt-Hoehe [m]<br> Standard 0,55 | Stärke der Lastverlagerung. | Stärkeres Nicken und Rollen aus Lasttransfer. | Schwächeres Nicken und Rollen aus Lasttransfer. |
| Laengs-Vorzeichen<br> Quer-Vorzeichen<br> Jeweils Standard 1 | Richtung und Stärke je Achse; negatives Vorzeichen kehrt sie um. | Größerer Betrag: stärkere Reaktion. | Kleinerer Betrag: schwächer; 0 schaltet die Achse aus. |
| Rollen-Skalierung<br> Standard 0,5 | Skaliert die gesamte ausgegebene Seitenneigung. | Mehr sichtbare Seitenneigung. | Weniger Seitenneigung; 0 unterdrückt Rollen. |

### Beschleunigungsfilter

Laengs wirkt auf die Brems-/Beschleunigungsreaktion, Quer auf die Kurvenreaktion. Die folgenden Paare werden getrennt eingestellt. Sie ändern den Lasttransfer, nicht den Fahrweg oder die tatsächliche Fahrgeschwindigkeit.

| **Parameter und Standard** | **Verwendung** | **Größer** | **Kleiner** |
| ------------------------------------------------------------------------------- | -------------------------------------------- | ------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Laengs-Spitzenfilter<br> Quer-Spitzenfilter<br> [Frames], jeweils 5 | Ersetzt auffällige Spitzen. | Längere Vergleichshistorie; kann echte schnelle Wechsel stärker unterdrücken. | Kürzere Historie; 1 deaktiviert diesen Filter. |
| Spitzen-Schwelle<br> Quer-Spitzen-Schwelle<br> [m/s²], jeweils 0,5 | Bestimmt, welche Spitzen ersetzt werden. | Weniger Spitzen werden ersetzt; Reaktion bleibt direkter. | Mehr Spitzen werden ersetzt; echte Bremswechsel können schwächer werden. |
| Laengs-Glaettungsfenster<br> Quer-Glaettungsfenster<br> [Frames], jeweils 5 | Glättet die Beschleunigung für den Solver. | Ruhigere, verzögerte und weichere Lasttransferreaktion. | Direkter, aber unruhiger; 1 deaktiviert die Glättung. |
| Laengs-Totzone<br> Quer-Totzone<br> [m/s²], jeweils 0,10 | Ignoriert kleine Beschleunigungen. | Weniger kleine Wank-/Nickbewegungen; schwache Manöver können verschwinden. | Mehr kleine Reaktionen und mögliches Zittern. |

Fenster nur ungerade einstellen: 1, 3, 5 … 21. Schwelle 0 deaktiviert den Spitzenfilter nicht. Bei zu starkem Bremsnicken zuerst Schwerpunkt-Hoehe oder den Betrag von Laengs-Vorzeichen senken; bei zu starkem Rollen Rollen-Skalierung senken.

## 6 Fahrweg Fahrprofil und Kamera

### TestPfad bearbeiten
1. Nur den TestPfad des gewünschten Fahrzeugs wählen. Objekt-Skalierung 1/1/1 beibehalten. In Objektmodus mit G verschieben oder um Global Z drehen; die Fahrzeughierarchie nicht gleichzeitig mittransformieren.
2. Tab → Bearbeitungsmodus. G verschiebt Punkte, E verlängert den gewählten Endpunkt. Für zusätzliche Punkte ein Segment auswählen und Unterteilen verwenden.
3. Für runde Kurven die betreffenden Bézier-Punkte wählen und Griffart auf Automatisch stellen. Bei engen Kreuzungen die Griffe kontrollieren, damit der Pfad auf der Straße bleibt.

Fahrzeug_Pfadsteuerung_XX → PfadFolgen: Ziel = eigener TestPfad, Vorne-Achse Y, Oben-Achse Z, Kurve folgen und Feste Position ein, Kurvenradius aus, Einfluss 1. Vorausschau_Steuerung_XX muss denselben eigenen Pfad verwenden. Die Beschränkung kann beim Bearbeiten eingeschaltet bleiben.

### Dauerhaft an die Fahrbahn anlegen

Am TestPfad unter dem Schraubenschlüssel Modifikator hinzufügen → Deformieren → Schrumpfen/Umhüllen wählen. Folgende Einstellungen bleiben aktiv:

| **Option** | **Einstellung** |
| ------------------------- | --------------------------------------------------------------------------------------------------------- |
| Methode / Einrastmodus | Projekt / Auf der Oberfläche |
| Achse und Richtung | Nur Z; Negativ und Positiv aktivieren |
| Ziel / Grenze / Versatz | Fahrbahn_Kollider / 0 m / 0 m |
| Apply On Spline | Schaltfläche in der Titelleiste einschalten; Tooltip prüfen. Viewport und Bearbeitungsmodus anzeigen. |

Der Modifier bleibt erhalten; nicht Anwenden wählen. Er hält Kontrollpunkte auf der Kollisionsfläche. Bei starken Unebenheiten Punkte mit Unterteilen ergänzen: Bézier-Segmente können zwischen den Punkten trotzdem abweichen. Außerhalb der Kollisionsfläche ist keine Projektion möglich. Neue Fahrzeugkopien übernehmen diese Einstellungen; bestehende Kopien werden nicht nachträglich synchronisiert.

### Gleichmäßig fahren und bis zum Stillstand bremsen

Versatzfaktor am PfadFolgen animieren. Gleichmäßige Fortschritte: Linear. Bremsen: zwei Schlüssel bei Bremsbeginn und Stopp setzen, dazwischen alte Schlüssel entfernen; den ersten im Graph Editor wählen, T → Quadratic, Strg + E → Ease Out. Am Stopp T → Konstant; spätere Schlüssel auf denselben Wert setzen.

Beispiel bei 24 fps: Frame 347 = 0,440; Frame 376 = 0,511. Die vorangehende lineare Fahrt muss etwa 0,004897 Fortschritt je Frame haben, damit am Bremsbeginn kein Geschwindigkeitssprung entsteht. Diese Interpolation bremst den Pfadfortschritt; den tatsächlichen Geschwindigkeitsverlauf bei Bedarf im CSV prüfen.

### Kamera folgt ohne mitzudrehen

Kamera wählen. Position kopieren hinzufügen: Ziel = Fahrzeug_Pfadsteuerung_XX, X/Y/Z ein, beide Räume World Space, Offset ein. Danach Position und feste Blickrichtung einstellen; Rotationsschlüssel entfernen und drehende/verfolgende Beschränkungen deaktivieren. Für feste Höhe Z ausschalten.

Aktuelle Ansicht übernehmen: Maus in die 3D-Ansicht, Strg + Alt + Ziffernblock 0. Das richtet die aktive Kamera aus. Bei mehreren Kameras zuerst die gewünschte Kamera mit Strg + Ziffernblock 0 aktiv setzen.

## 7 Fahrzeugauswahl Ausgabe und kompletter Ablauf

| **Panel-Einstellung** | **Verwendung und Wirkung** |
| ----------------------------------------------------- | --------------------------------------------------------------------------------------- |
| Fahrzeug-Index<br> Standard 01 | Wählt das Fahrzeug und den Dateisuffix, z. B. 02. Kein Einfluss auf Bewegungsstärke. |
| Gemeinsamen Pfad verwenden<br> Standard Aus | Nur bei Neuerstellung: Aus kopiert einen eigenen Pfad; Ein verwendet den Pfad von 01. |
| Road Input CSV<br> //road_input.csv | Speicherort der Fahrbewegung und Straßenanregung. |
| Vehicle Parameter JSON<br> //vehicle_parameter.json | Speicherort der exportierten Panelparameter. |
| Vehicle Response CSV<br> //vehicle_response.csv | Speicherort der berechneten Bewegungen für den Baker. |
| Ausgabedateien ueberschreiben<br> Standard Ein | Ein ersetzt vorhandene Ausgabedateien; Aus bricht bei vorhandenem Dateinamen ab. |

Die drei Pfade bleiben ohne eigenen Index: Das Skript hängt _01, _02 usw. automatisch an. // verwendet den Ordner der gespeicherten .blend-Datei. Der Ausgabeordner muss existieren.

### Bedienung in der richtigen Reihenfolge
1. Datei speichern, Start-/Endframe und fps festlegen. Skript ausführen und das Panel Road Input öffnen.
2. Fahrzeug-Index 01 → Fahrzeug erstellen / bestätigen. Modell binden, Radachsen und Startpose kontrollieren.
3. Für ein neues Fahrzeug Index 02/03 eingeben, Pfadoption wählen und Fahrzeug erstellen / bestätigen drücken. Die Vorlage ist immer 01; Steuerung, Mesh-Daten, Aktionen und eigener Pfad werden kopiert.
4. Eigenes Modell einsetzen, nur dessen Pfad bearbeiten und Versatzfaktor animieren. Indexzuordnung von Lenkung und Vorausschau prüfen.
5. Panelparameter für genau dieses Fahrzeug einstellen. Beim Wechsel des Index bleiben die bisherigen Panelwerte stehen; fahrzeugeigene Werte werden nicht automatisch geladen.
6. Validate Setup ausführen, danach Run Complete Pipeline. Alternativ nacheinander 1. Validate + Export Inputs → 2. Solve vehicle_response.csv → 3. Bake Animation.
7. Animation prüfen, .blend speichern. Für jedes weitere Fahrzeug Index und Parameter erneut einstellen und die Pipeline ausführen.
8. Aktive Kamera und Render-Ausgabe wählen. Für Video FFmpeg Video mit MPEG-4/H.264 einstellen, dann Rendern → Animation rendern. PNG-Ausgabe erzeugt stattdessen Einzelbilder.

### Nach Änderungen erneut ausführen

Pfad, Timing, Modellgeometrie oder Panelparameter geändert: gesamte Pipeline erneut ausführen. Solve allein liest die gespeicherte JSON; Bake allein liest die gespeicherte Response-CSV. Beim erneuten Bake ersetzt der Code seine bisherigen Z- und Karosserie-Rotationsschlüssel.

Manuelle Basisänderungen an bereits gebackenen Kanälen: die betreffenden alten Kanäle zuerst löschen, dann den Startframe ausrichten. Vorhandene Schlüssel können die manuelle Pose beim Framewechsel sonst wieder überschreiben. Reines Kameraausrichten braucht keinen neuen Solverlauf.
