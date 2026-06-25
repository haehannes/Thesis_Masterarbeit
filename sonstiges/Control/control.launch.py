# control_console_uxrce.launch.py
#
# Startet die uXRCE-DDS-Bedienkonsole und macht die wichtigsten Werte
# direkt hier (bzw. per Kommandozeile) einstellbar. In Klammern jeweils
# der aktuelle Default-Wert:
#   - demo_edge_length_m : Kantenlaenge des Demo-Quadrats [m]        (2.0)
#   - demo_speed_mps     : Fluggeschwindigkeit im Demo-Modus [m/s]   (0.3)
#   - descent_speed_mps  : Landegeschwindigkeit (Sinkflug) [m/s]     (0.3)
#   - takeoff_offset_m   : Start-/Flughoehe ueber aktueller Pos. [m] (1.5)
#   - demo_center_x      : Zentrum X (ENU) des Quadrats [m]          (0.0)
#   - demo_center_y      : Zentrum Y (ENU) des Quadrats [m]          (0.0)
#
# Hinweis: Das Landen per Leertaste (SPACE) ist davon unberuehrt und
# jederzeit moeglich; es nutzt genau den hier gesetzten descent_speed_mps.
#
# WICHTIG (curses + ros2 launch):
#   Die Konsole ist ein curses-UI und braucht ein echtes Terminal (TTY).
#   Unter "ros2 launch" wird die Ausgabe normalerweise umgeleitet, dann
#   startet curses NICHT. Deshalb wird der Node hier in einem eigenen
#   xterm-Fenster gestartet (prefix="xterm -e"). Voraussetzung: xterm ist
#   installiert  ->  sudo apt install xterm
#
#   Wer kein xterm will/hat, startet stattdessen direkt im aktuellen
#   Terminal (gleiche Parameter, ein Beispiel):
#
#     ros2 run <DEIN_PAKET> control_console_uxrce --ros-args \
#         -p demo_edge_length_m:=2.0 \
#         -p demo_speed_mps:=0.3 \
#         -p descent_speed_mps:=0.3 \
#         -p takeoff_offset_m:=1.5
#
# ANPASSEN: PACKAGE und EXECUTABLE auf deinen Workspace setzen.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# >>> Hier ggf. an deinen Workspace anpassen <<<
PACKAGE = "drone_vision_pose_publisher"
EXECUTABLE = "control_console_uxrce"


def generate_launch_description():
    """Launch-Beschreibung mit einstellbaren Demo-/Hoehen-/Lande-Parametern."""
    declared = []

    def arg(name, default, description):
        """Ein LaunchArgument deklarieren und als Substitution zurueckgeben."""
        declared.append(
            DeclareLaunchArgument(
                name, default_value=default, description=description
            )
        )
        return LaunchConfiguration(name)

    # Default-Werte stehen jeweils als zweites Argument von arg(...).
    edge = arg("demo_edge_length_m", "2.0", "Kantenlaenge des Demo-Quadrats [m]")
    speed = arg("demo_speed_mps", "0.3", "Fluggeschwindigkeit im Demo-Modus [m/s]")
    descent = arg(
        "descent_speed_mps", "0.3", "Landegeschwindigkeit / Sinkflug [m/s]"
    )
    takeoff = arg(
        "takeoff_offset_m", "1.5", "Start-/Flughoehe ueber aktueller Position [m]"
    )
    center_x = arg("demo_center_x", "0.0", "Zentrum X (ENU) des Quadrats [m]")
    center_y = arg("demo_center_y", "0.0", "Zentrum Y (ENU) des Quadrats [m]")

    console = Node(
        package=PACKAGE,
        executable=EXECUTABLE,
        name="control_console",
        output="screen",
        emulate_tty=True,
        # curses braucht ein echtes TTY -> eigenes Terminalfenster.
        # Auskommentieren, falls du den Node direkt per "ros2 run" startest.
        prefix="xterm -e",
        parameters=[
            {
                "demo_edge_length_m": edge,
                "demo_speed_mps": speed,
                "descent_speed_mps": descent,
                "takeoff_offset_m": takeoff,
                "demo_center_x": center_x,
                "demo_center_y": center_y,
            }
        ],
    )

    return LaunchDescription(declared + [console])