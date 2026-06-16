#!/usr/bin/env python3
# =============================================================================
# Drone-Camera-Kalibrierungs-Node
# -----------------------------------------------------------------------------
# Zweck dieses Nodes:
#   Berechnet die statische Transformation von der Drohnenmitte (drone_center)
#   zu jeder montierten Kamera. Dafuer wird die Drohne unter einer
#   vorab vermessenen AprilTag-Map platziert. Ein zentraler Tag (z.B. id=100)
#   ist starr auf der Drohne montiert und definiert den drone_center-Frame.
#   Die anderen Tags sind in der Welt verteilt (map-Frame).
#
# Datenfluss pro Sample:
#   1) Ein anderer Node (Live-Position pro Kamera) liefert
#      T_map_camera als PoseStamped:
#         "Wo sitzt die Kamera, ausgedrueckt im map-Frame?"
#   2) Aus der Kalibrierungs-Map lesen wir T_map_drone:
#         "Wo sitzt der drone_center-Tag im map-Frame?"
#   3) Wir wollen T_drone_camera (statische Befestigung der Kamera am Rumpf):
#         T_drone_camera = inv(T_map_drone) @ T_map_camera
#   4) Pro Kamera werden viele solche Messungen gesammelt und am Ende
#      gemittelt (Translation arithmetisch, Quaternion vorzeichenrobust).
#
# Ergebnis:
#   Ein YAML mit Translation, Quaternion, Rotationsmatrix und 4x4-Matrix
#   pro aktiver Kamera. Dieses YAML kann spaeter von anderen Nodes als
#   feste Extrinsik (drone -> camera) geladen werden.
# =============================================================================

import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# -----------------------------------------------------------------------------
# Hilfsfunktionen rund um Rotation und Pose.
# Sie sind bewusst frei (kein Klassenkontext), damit sie auch einzeln
# wiederverwendbar und leicht testbar sind.
# -----------------------------------------------------------------------------

def quat_to_rot(q):
    """Quaternion [x, y, z, w] in 3x3-Rotationsmatrix umrechnen.

    Schritt fuer Schritt:
      1) Quaternion normalisieren (sonst fehlerhaftes R).
      2) Sonderfall abfangen: extrem kleine Norm -> Einheitsrotation
         zurueckgeben (vermeidet Division durch 0).
      3) Standard-Formel fuer R aus (x, y, z, w) anwenden.
    """
    x, y, z, w = q
    n = np.linalg.norm(q)

    # Sicherheitsnetz: bei einem (fast) Null-Quaternion gibt es keine
    # sinnvolle Rotation. Wir geben die Identitaet zurueck, damit der
    # Aufrufer keinen NaN bekommt.
    if n < 1e-12:
        return np.eye(3)

    # Normalisierte Komponenten als Floats; ohne diese Zeile bauen wir die
    # Matrix mit dem unnormalisierten Quaternion und das R waere keine
    # echte Rotationsmatrix (det != 1).
    x, y, z, w = q / n

    # Klassische Direktformel fuer R aus dem Quaternion.
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def rot_to_quat(R):
    """3x3-Rotationsmatrix in Quaternion [x, y, z, w] umrechnen.

    Verwendet die numerisch stabile Variante (Shepperd's Methode):
      - Je nachdem, welches Diagonalelement bzw. die Spur am groessten ist,
        wird ein anderer Zweig benutzt.
      - Damit vermeiden wir Wurzeln aus kleinen oder negativen Zahlen.
    """
    tr = np.trace(R)

    if tr > 0:
        # Spur positiv -> w ist die "groesste" Komponente, stabilster Zweig.
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        # Spur <= 0 -> waehle den Zweig nach groesstem Diagonalelement.
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            # x dominiert.
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            # y dominiert.
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            # z dominiert.
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

    # Rueckgabe normalisieren: kleine numerische Drift wegbuegeln.
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def pose_to_matrix(position, quaternion):
    """Position + Quaternion zu 4x4-Homogentransformation kombinieren.

    Aufbau:
      [ R  t ]
      [ 0  1 ]
    R kommt aus dem Quaternion, t aus der Position.
    """
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(np.array(quaternion, dtype=float))
    T[:3, 3] = np.array(position, dtype=float)
    return T


def msg_to_matrix(msg):
    """PoseStamped-Nachricht in 4x4-Homogentransformation umwandeln.

    Bequemer Adapter: extrahiert Position + Orientierung aus der
    ROS-Nachricht und nutzt pose_to_matrix.
    """
    p = msg.pose.position
    q = msg.pose.orientation

    return pose_to_matrix(
        [p.x, p.y, p.z],
        [q.x, q.y, q.z, q.w]
    )


def average_quaternions(quats):
    """Robuste Mittelung mehrerer Quaternionen.

    Hintergrund:
      q und -q beschreiben dieselbe Rotation (Doppeldeckung der Quaternionen).
      Naive Mittelung kann sich daher gegenseitig ausloeschen.

    Vorgehen:
      1) Referenz waehlen (erstes Quaternion).
      2) Jedes weitere Quaternion ggf. mit -1 multiplizieren, sodass sein
         Skalarprodukt mit der Referenz >= 0 ist. Damit liegen alle
         Quaternionen in derselben Halbsphaere.
      3) Komponentenweise mitteln und das Ergebnis normalisieren.

    Hinweis: Fuer kleine Streuungen (statische Kalibrierung) ist das
    eine vollkommen ausreichende Naeherung an die SLERP-Mittelung.
    """
    if len(quats) == 0:
        # Leere Liste -> Identitaetsrotation als sicherer Fallback.
        return np.array([0.0, 0.0, 0.0, 1.0])

    ref = quats[0]
    aligned = []

    for q in quats:
        q = np.array(q, dtype=float)
        q = q / np.linalg.norm(q)

        # Halbsphaeren-Korrektur: gleiches Vorzeichen wie Referenz.
        if np.dot(ref, q) < 0:
            q = -q

        aligned.append(q)

    # Arithmetisches Mittel + Normalisierung.
    q_avg = np.mean(aligned, axis=0)
    return q_avg / np.linalg.norm(q_avg)


# -----------------------------------------------------------------------------
# Eigentlicher ROS2-Node.
# -----------------------------------------------------------------------------

class DroneCameraCalibrationNode(Node):

    def __init__(self):
        """Node initialisieren und konfigurierte Kameras abonnieren.

        Reihenfolge der Initialisierung:
          1) Parameter deklarieren (Defaults setzen).
          2) Parameter auslesen, Pflichtfelder pruefen.
          3) Map-YAML einlesen und drone_center-Pose extrahieren.
          4) Kamera-Konfigurationen bauen, deaktivierte ausfiltern.
          5) Plausibilitaetschecks (mind. 1 Kamera, eindeutige Namen).
          6) Sample-Container vorbereiten und Subscriptions anlegen.
          7) Status loggen, damit man im Terminal sieht, was aktiv ist.
        """
        super().__init__('drone_camera_calibration_node')

        # --- (1) Parameter deklarieren -------------------------------------
        # Pfade und Tag-ID werden ueblicherweise im Launch-File gesetzt.
        self.declare_parameter('map_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('drone_center_tag_id', 100)

        # Pro Kamera-Slot: enabled (an/aus), Topic, semantischer Name
        # ("front" oder "bottom"). Der Name landet als Schluessel im YAML
        # und als child_frame und ist damit fuer alle Folge-Nodes wichtig.
        self.declare_parameter('camera_1_enabled', True)
        self.declare_parameter('camera_1_topic', '/liveposition_camera_1')
        self.declare_parameter('camera_1_name', 'front')

        self.declare_parameter('camera_2_enabled', True)
        self.declare_parameter('camera_2_topic', '/liveposition_camera_2')
        self.declare_parameter('camera_2_name', 'bottom')

        # samples_required: wie viele Messungen pro Kamera, bevor wir
        # die Kalibrierung als "fertig" werten.
        # write_every_sample: True = laufend rausschreiben (gut zum Mitlesen
        # waehrend der Aufnahme); False = nur einmal am Ende speichern.
        self.declare_parameter('samples_required', 100)
        self.declare_parameter('write_every_sample', True)

        # --- (2) Parameter auslesen ----------------------------------------
        self.map_file = self.get_parameter('map_file').value
        self.output_file = self.get_parameter('output_file').value
        self.drone_center_tag_id = int(self.get_parameter('drone_center_tag_id').value)

        self.samples_required = int(self.get_parameter('samples_required').value)
        self.write_every_sample = bool(self.get_parameter('write_every_sample').value)

        # Pflichtfelder hart abbrechen, statt spaeter mit einem kryptischen
        # IOError bei open() abzustuerzen.
        if not self.map_file:
            raise RuntimeError('Parameter map_file ist leer.')

        if not self.output_file:
            raise RuntimeError('Parameter output_file ist leer.')

        # --- (3) drone_center aus der Map-YAML laden -----------------------
        # Wir loesen den Tag direkt jetzt auf und cachen die 4x4-Matrix,
        # damit wir in jedem Callback nur eine Matrix-Multiplikation
        # ausfuehren und nicht das YAML erneut parsen muessen.
        self.T_map_drone = self.load_drone_center_pose(self.map_file)

        # --- (4) Kamera-Konfigurationen aufbauen ---------------------------
        # Liste statt zwei Hardcoded-Bloecke: macht Erweiterung auf weitere
        # Kameras spaeter trivial und vermeidet duplizierten Code.
        camera_configs = [
            {
                'slot': 'camera_1',
                'enabled': bool(self.get_parameter('camera_1_enabled').value),
                'topic': self.get_parameter('camera_1_topic').value,
                'name': self.get_parameter('camera_1_name').value,
            },
            {
                'slot': 'camera_2',
                'enabled': bool(self.get_parameter('camera_2_enabled').value),
                'topic': self.get_parameter('camera_2_topic').value,
                'name': self.get_parameter('camera_2_name').value,
            },
        ]

        # Nur enabled=True wird abonniert und gewertet.
        self.active_cameras = [c for c in camera_configs if c['enabled']]

        # --- (5) Plausibilitaetschecks -------------------------------------
        # Ohne aktive Kamera ist der Node sinnlos -> sofort abbrechen.
        if len(self.active_cameras) == 0:
            raise RuntimeError(
                'Keine Kamera aktiviert. Mindestens eine der Optionen '
                'camera_1_enabled oder camera_2_enabled muss True sein.'
            )

        # Doppelter Name (z.B. beide "front") wuerde sich im YAML
        # gegenseitig ueberschreiben -> hart abbrechen.
        names = [c['name'] for c in self.active_cameras]
        if len(set(names)) != len(names):
            raise RuntimeError(
                f'Doppelter camera_*_name in aktivierten Kameras: {names}. '
                'Bitte eindeutige Namen vergeben (z.B. "front", "bottom").'
            )

        # --- (6) Sample-Container + Subscriptions --------------------------
        # Indexieren ueber den semantischen Namen (nicht ueber den Slot),
        # damit die Datenstruktur direkt zum Ausgabe-YAML passt.
        self.samples = {c['name']: [] for c in self.active_cameras}

        # Subscriptions getrennt halten (Garbage-Collection-Schutz);
        # ohne Referenz wuerde rclpy sie unter Umstaenden verwerfen.
        self.subscriptions_list = []
        for cfg in self.active_cameras:
            name = cfg['name']
            topic = cfg['topic']

            # Wichtig: n=name als Default-Argument im Lambda. Ohne diesen
            # Trick wuerde Python die Variable spaet binden und am Ende
            # ALLE Callbacks denselben (letzten) Namen verwenden.
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, n=name: self.pose_callback(msg, n),
                10  # Queue-Tiefe; 10 ist fuer langsame Kalibrierung mehr als genug.
            )
            self.subscriptions_list.append(sub)

        # --- (7) Status loggen ---------------------------------------------
        # Hilft, beim Start im Terminal direkt zu sehen, welche Map,
        # welche Tag-ID und welche Kameras tatsaechlich verwendet werden.
        self.get_logger().info('Drone-Camera-Kalibrierung gestartet.')
        self.get_logger().info(f'Map-Datei: {self.map_file}')
        self.get_logger().info(f'Ausgabe-Datei: {self.output_file}')
        self.get_logger().info(
            f'Drohnenmittelpunkt Tag-ID: {self.drone_center_tag_id}'
        )

        for cfg in self.active_cameras:
            self.get_logger().info(
                f"Aktive Kamera: name='{cfg['name']}' "
                f"(slot={cfg['slot']}, topic={cfg['topic']})"
            )

        # Deaktivierte Kameras ebenfalls loggen -> hilfreich, wenn man sich
        # spaeter wundert, warum nur eine Kamera Samples liefert.
        disabled = [c for c in camera_configs if not c['enabled']]
        for cfg in disabled:
            self.get_logger().info(
                f"Deaktivierte Kamera: slot={cfg['slot']} "
                f"(name='{cfg['name']}', topic={cfg['topic']})"
            )

    def load_drone_center_pose(self, path):
        """Pose des Drohnenmittelpunkt-Tags aus der Map-YAML lesen.

        Erwartetes Format (vereinfacht):
          tags:
            - id: 100
              pose:
                position: {x, y, z}
                rotation: {x, y, z, w}

        Vorgehen:
          1) Datei einlesen.
          2) Liste 'tags' durchgehen.
          3) Beim Treffer (id == drone_center_tag_id) die Pose in eine
             4x4-Matrix wandeln und zurueckgeben.
          4) Kein Treffer -> hart abbrechen, denn ohne diese Pose koennen
             wir die Transformation drone -> camera nicht bilden.
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        tags = data.get('tags', [])

        for tag in tags:
            if int(tag.get('id')) == self.drone_center_tag_id:
                pose = tag['pose']
                pos = pose['position']
                rot = pose['rotation']

                T = pose_to_matrix(
                    [pos['x'], pos['y'], pos['z']],
                    [rot['x'], rot['y'], rot['z'], rot['w']]
                )

                self.get_logger().info(
                    f'Drohnenmittelpunkt aus YAML geladen: '
                    f'Tag-ID {self.drone_center_tag_id}'
                )

                return T

        # Wenn wir hier landen, gibt es schlicht keine Datenbasis fuer
        # die Kalibrierung -> RuntimeError, kein stilles Weitermachen.
        raise RuntimeError(
            f'Drohnenmittelpunkt mit id={self.drone_center_tag_id} '
            f'nicht in {path} gefunden.'
        )

    def pose_callback(self, msg, camera_name):
        """Eingehende Pose in drone_center-Frame umrechnen und sammeln.

        Wird fuer JEDE eingehende PoseStamped genau einer Kamera aufgerufen.
        Schritte:
          1) Abbrechen, wenn diese Kamera schon "voll" ist (Sample-Limit).
          2) Eingangs-Pose T_map_camera in 4x4 umwandeln.
          3) Mit cached T_map_drone in drone_center-Frame transformieren:
                T_drone_camera = inv(T_map_drone) @ T_map_camera
          4) Sample anhaengen, Zaehler loggen.
          5) Optional: YAML sofort schreiben (write_every_sample=True)
             oder erst dann, wenn alle aktiven Kameras fertig sind.
        """
        # (1) Sobald das Sample-Limit erreicht ist, ignorieren wir weitere
        # Nachrichten dieser Kamera. So bleibt der Mittelwert stabil und
        # die Datei wird nicht endlos weitergeschrieben.
        if len(self.samples[camera_name]) >= self.samples_required:
            return

        # (2) Eingangs-Pose: wo ist die Kamera im map-Frame?
        T_map_camera = msg_to_matrix(msg)

        # (3) Umrechnen in den drone_center-Frame. inv(T_map_drone) ist
        # rechnerisch billig (4x4) und numerisch hier voellig unkritisch.
        # Mathematisch:  drone <- map  *  map <- camera  =  drone <- camera
        T_drone_camera = np.linalg.inv(self.T_map_drone) @ T_map_camera

        # (4) Anhaengen und Fortschritt loggen.
        self.samples[camera_name].append(T_drone_camera)

        n = len(self.samples[camera_name])

        self.get_logger().info(
            f'{camera_name}: Sample {n}/{self.samples_required}'
        )

        # (5) Speicher-Strategie umsetzen.
        if self.write_every_sample or self.all_done():
            self.write_result()

    def average_transform(self, transforms):
        """Translationen arithmetisch, Quaternionen vorzeichenrobust mitteln.

        Schritt fuer Schritt:
          - Aus jeder 4x4-Matrix Translation und Quaternion extrahieren.
          - Translation: einfache komponentenweise Mittelung (np.mean).
          - Quaternion:  average_quaternions (Halbsphaeren-aware).
          - R aus dem gemittelten q neu bauen, damit die Matrix wirklich
            zum Quaternion passt (statt eine Mittelung von R-Eintraegen
            zu machen, die i.A. KEINE gueltige Rotationsmatrix mehr waere).
          - 4x4-Endmatrix zusammensetzen.
        """
        translations = []
        quaternions = []

        for T in transforms:
            translations.append(T[:3, 3])
            quaternions.append(rot_to_quat(T[:3, :3]))

        t_avg = np.mean(translations, axis=0)
        q_avg = average_quaternions(quaternions)
        R_avg = quat_to_rot(q_avg)

        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3] = t_avg

        return T_avg, t_avg, q_avg, R_avg

    def all_done(self):
        """True, wenn alle AKTIVEN Kameras die geforderten Samples haben.

        Wichtig: nur aktive Kameras werden betrachtet. Falls
        camera_2 deaktiviert ist, ist die Kalibrierung fertig, sobald
        camera_1 ihre Samples voll hat.
        """
        for cfg in self.active_cameras:
            if len(self.samples[cfg['name']]) < self.samples_required:
                return False
        return True

    def matrix_to_list(self, M):
        """Numpy-Matrix in einfache Python-Listen fuer YAML-Ausgabe wandeln.

        Hintergrund: yaml.safe_dump kann numpy-Skalare/-Arrays nicht
        zuverlaessig serialisieren. Wir wandeln alles explizit in float
        und verschachtelte Python-Listen.
        """
        return [[float(v) for v in row] for row in M]

    def write_result(self):
        """Aktuelle Ergebnisse in die Ausgabe-YAML schreiben.

        Wird je nach Konfiguration nach jedem Sample oder nur am Ende
        aufgerufen. Schreibt IMMER den vollstaendigen aktuellen Stand,
        d.h. die Datei ist nach jedem Aufruf in sich konsistent.

        Aufbau:
          - parent_frame: drone_center
          - cameras: dict mit semantischem Namen ("front", "bottom") als Key
              -> translation, rotation_quaternion, rotation_matrix,
                 transform_matrix_4x4 (alles redundant, fuer maximale
                 Bequemlichkeit beim Konsumieren).
        """
        # Liste der aktiven Kameras separat ausweisen, damit Folge-Nodes
        # ohne zusaetzlichen Kontext sehen, welche Kameras erfasst wurden.
        active_names = [c['name'] for c in self.active_cameras]

        output = {
            'parent_frame': 'drone_center',
            'description': (
                'Gemittelte Transformationen vom Drohnenmittelpunkt '
                'zu den Kameras'
            ),
            'samples_required': self.samples_required,
            'active_cameras': active_names,
            'cameras': {}
        }

        # Pro Kamera mitteln und Block bauen.
        for camera_name, transforms in self.samples.items():
            # Kameras ohne Samples ueberspringen (z.B. wenn der Topic noch
            # nichts geliefert hat). So enthaelt die Datei nur "echte" Eintraege.
            if len(transforms) == 0:
                continue

            T_avg, t_avg, q_avg, R_avg = self.average_transform(transforms)

            output['cameras'][camera_name] = {
                'samples_used': len(transforms),
                # child_frame: ueblicher TF-Name; passt zu Konventionen wie
                # 'front_optical_frame' / 'bottom_optical_frame'.
                'child_frame': f'{camera_name}_optical_frame',
                'translation': {
                    'x': float(t_avg[0]),
                    'y': float(t_avg[1]),
                    'z': float(t_avg[2]),
                },
                'rotation_quaternion': {
                    'x': float(q_avg[0]),
                    'y': float(q_avg[1]),
                    'z': float(q_avg[2]),
                    'w': float(q_avg[3]),
                },
                # Sowohl 3x3-Rotation als auch 4x4-Transformation mit ablegen,
                # damit nachgelagerte Nodes sich aussuchen koennen, was sie
                # brauchen (z.B. fuer numpy-Matrix-Anwendung direkt).
                'rotation_matrix': self.matrix_to_list(R_avg),
                'transform_matrix_4x4': self.matrix_to_list(T_avg),
            }

        # Datei atomar genug fuer Kalibrierung: ein einzelner Open+Dump.
        # Fuer wirklich atomares Schreiben muesste man in Temp + os.replace,
        # aber bei statischer Kalibrierung ist das uebertrieben.
        with open(self.output_file, 'w') as f:
            yaml.safe_dump(output, f, sort_keys=False)

        # Eindeutiges "Fertig"-Signal, damit der Benutzer im Terminal sieht,
        # dass er den Node jetzt beenden kann.
        if self.all_done():
            self.get_logger().info(
                'Kalibrierung abgeschlossen. YAML wurde final geschrieben.'
            )


def main(args=None):
    """Einstiegspunkt: rclpy initialisieren und Node spinnen.

    Lebenszyklus:
      1) rclpy.init: ROS2-Client-Library hochfahren.
      2) Node instanziieren (Parameter, Subscriptions, Map-Load).
      3) spin: blockiert, bis Ctrl+C / Shutdown -> verarbeitet Callbacks.
      4) Aufraeumen: Node destroyen und rclpy herunterfahren.
    """
    rclpy.init(args=args)
    node = DroneCameraCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()