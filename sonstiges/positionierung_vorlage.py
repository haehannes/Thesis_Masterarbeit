### Launch File auf dem Jetson (host) -- uXRCE-DDS-Variante
#
# Schlankes Setup mit einem einzigen Vision-Pose-Node, der:
#   - Tag-Detections von der Kamera direkt liest
#   - Drohnen-Pose direkt berechnet (inkl. Kamera->Drohnenmitte)
#   - VehicleOdometry direkt an PX4 publiziert (uXRCE-DDS)
#
# Unterschiede zur MAVROS-Variante:
#   - KEIN mavros_node mehr.
#   - Stattdessen wird der Micro-XRCE-DDS-Agent mitgestartet (ExecuteProcess).
#   - Der Vision-Node ist vision_pose_uxrce_node (statt vision_pose_direct_node)
#     und publiziert auf /fmu/in/vehicle_visual_odometry statt auf
#     /mavros/vision_pose/pose_cov.
#
# Behalten:
#   - Beide gscam-Nodes (Kamera 2 weiterhin deaktiviert)
#   - Isaac-TF-Verkabelung (tag_map -> isaac -> camera_*_optical_frame)
#   - tag_map -> Karte (fuer Visualisierung in RViz)
#
# Transport: UDP ueber Ethernet (PX4-Parameter UXRCE_DDS_CFG = Ethernet).
# Der Agent lauscht auf UDP-Port 8888; PX4 verbindet sich aktiv zur in
# UXRCE_DDS_AG_IP konfigurierten Jetson-IP (192.168.0.10).
# Voraussetzung: Jetson-Ethernet hat eine IP im PX4-Subnetz (192.168.0.x)
# und PX4 ist per Ping erreichbar.

import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    map_file = (
        "/home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/"
        #"Einstellungen/maps/Aufgenommen/test_501_bis_509_a4.yaml"
        #"Einstellungen/maps/Aufgenommen/tagslam_lauf3_Zimmer_24_05.yaml"
        #"Einstellungen/maps/Aufgenommen/bearbeitet_Kleiner.yaml"
        #"Einstellungen/maps/Aufgenommen/OIC_0_bis_12_Lauf2.yaml"
        "Einstellungen/maps/Aufgenommen/OIC_gesamt_teilw_Lauf2.yaml"
        #"Einstellungen/maps/Aufgenommen/OIC_2_groessen_Lauf2.yaml"
    )

    calibration_file = (
        "/home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/"
        "Einstellungen/maps/Kalibrierung/map_Kalibrierung_result.yaml"
    )

    # Kamera 1 (Boden): camera_info-URL fuer 1280x720 Profil
    camera_info_url = (
        'file:///home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/'
        'Einstellungen/Kamera/C922/logitech_c920_1280_720.yaml'
    )

    # ====================================================================
    # Micro-XRCE-DDS-Agent -- UDP ueber Ethernet
    # ====================================================================
    # PX4 (UXRCE_DDS_CFG = Ethernet) verbindet sich aktiv zur Agent-IP
    # (UXRCE_DDS_AG_IP = 192.168.0.10) auf dem UDP-Port unten. Der Agent
    # muss daher nur auf diesem Port lauschen.
    #
    # Pruefe nach dem Start mit:  ros2 topic list | grep /fmu
    uxrce_agent_port = '8888'

    return LaunchDescription([

        #############################################################
        # Micro-XRCE-DDS-Agent (ersetzt die MAVROS-Verbindung)
        # UDP ueber Ethernet.
        #############################################################
        ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'udp4',
                '-p', uxrce_agent_port,
            ],
            output='screen',
        ),

        ############################################################
        # Kamera 1: Blickrichtung Boden
        # 1280x720 @ 60 FPS, hardware-beschleunigte MJPEG-Pipeline.
        ############################################################
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_camera_1',
            namespace='camera_1',
            output='screen',
            parameters=[{
                'gscam_config': (
                    'v4l2src device=/dev/video0 do-timestamp=true ! '
                    'image/jpeg,width=1280,height=720,framerate=60/1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvv4l2decoder mjpeg=1 ! '
                    'queue leaky=downstream max-size-buffers=1 ! '
                    'nvvidconv ! '
                    'video/x-raw,format=BGRx ! '
                    'videoconvert ! '
                    'video/x-raw,format=RGB,framerate=60/1'
                ),
                'camera_name': 'camera_1',
                'frame_id': 'camera_1_optical_frame',
                'camera_info_url': camera_info_url,
                'use_gst_timestamps': True,
                'sync_sink': False,
                'preroll': False,
                'image_encoding': 'rgb8',
                'use_sensor_data_qos': False,
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
                ('camera/image_raw/compressed', 'image_raw/compressed'),
                ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
                ('camera/image_raw/theora', 'image_raw/theora'),
            ],
        ),

        ############################################################
        # Kamera 2: Blickrichtung nach vorne
        # --- DEAKTIVIERT ---
        ############################################################
        # Node(
        #     package='gscam',
        #     executable='gscam_node',
        #     name='gscam_camera_2',
        #     namespace='camera_2',
        #     output='screen',
        #     parameters=[{
        #         'gscam_config': (
        #             'v4l2src device=/dev/video2 do-timestamp=true ! '
        #             'image/jpeg,width=640,height=480,framerate=30/1 ! '
        #             'queue leaky=downstream max-size-buffers=1 ! '
        #             'nvv4l2decoder mjpeg=1 ! '
        #             'queue leaky=downstream max-size-buffers=1 ! '
        #             'nvvidconv ! '
        #             'video/x-raw,format=BGRx ! '
        #             'videoconvert ! '
        #             'video/x-raw,format=RGB,framerate=30/1'
        #         ),
        #         'camera_name': 'camera_2',
        #         'frame_id': 'camera_2_optical_frame',
        #         'camera_info_url':
        #             'file:///home/hannes/Masterarbeit/workspaces/tagslam_ws_humble/'
        #             'Einstellungen/Kamera/C920/front_640_480_C920.yaml',
        #         'use_gst_timestamps': True,
        #         'sync_sink': False,
        #         'preroll': False,
        #         'image_encoding': 'rgb8',
        #         'use_sensor_data_qos': False,
        #     }],
        #     remappings=[
        #         ('camera/image_raw', 'image_raw'),
        #         ('camera/camera_info', 'camera_info'),
        #         ('camera/image_raw/compressed', 'image_raw/compressed'),
        #         ('camera/image_raw/compressedDepth', 'image_raw/compressedDepth'),
        #         ('camera/image_raw/theora', 'image_raw/theora'),
        #     ],
        # ),

        ############################################################
        # Statische TFs fuer Isaac AprilTag.
        # Isaac braucht die TF-Kette tag_map -> isaac -> camera_*_optical_frame.
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag_map_to_isaac_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'tag_map',
                'isaac',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='isaac_to_camera_1_optical_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'isaac',
                'camera_1_optical_frame',
            ],
        ),

        # --- DEAKTIVIERT (Kamera 2) ---
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='isaac_to_camera_2_optical_tf',
        #     arguments=[
        #         '0', '0', '0',
        #         '0', '0', '0', '1',
        #         'isaac',
        #         'camera_2_optical_frame',
        #     ],
        # ),

        ############################################################
        # Karte-Frame (optional, fuer RViz-Visualisierung).
        # tag_map -> Karte ist Identitaet, weil die Karte direkt
        # in tag_map-Koordinaten vorliegt.
        ############################################################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag_map_to_karte_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'tag_map',
                'Karte',
            ],
        ),

        ############################################################
        # Der EINE Vision-Pose-Node (uXRCE-DDS).
        #
        # Liest direkt:
        #   /camera_1/tag_detections  (Kamera Boden)
        #
        # Publiziert direkt:
        #   /fmu/in/vehicle_visual_odometry  (VehicleOdometry, NED/FRD)
        ############################################################
        Node(
            package='drone_vision_pose_publisher',
            executable='vision_pose_uxrce_node',
            name='vision_pose_uxrce_node',
            output='screen',
            parameters=[{
                # Karten und Kalibrierung
                'map_file': map_file,
                'calibration_file': calibration_file,

                # Kamera-1 (Boden)
                'cam1_topic': '/camera_1/tag_detections',
                'cam1_name': 'camera_1',
                'cam1_weight': 1.0,

                # Kamera-2 (vorne) -- DEAKTIVIERT
                'cam2_weight': 0.0,

                # ----- uXRCE-Ausgabe -----
                # VehicleOdometry an PX4. pose_frame=1 -> NED (passend zu
                # einer ENU-Karte nach der internen ENU->NED-Konvertierung).
                'publish_topic': '/fmu/in/vehicle_visual_odometry',
                'pose_frame': 1,
                'timesync_topic': '/fmu/out/timesync_status',
                'publish_rate_hz': 60.0,

                # map_frame ist bei VehicleOdometry ohne Bedeutung (kein
                # frame_id-String), bleibt nur fuer interne Logs.
                'map_frame': 'tag_map',

                # Wie alt darf eine Kamera-Schaetzung sein?
                'max_estimate_age_s': 0.3,

                # Tag-Mindestanzahl pro Kamera-Frame.
                'min_tags_required': 1,

                # RANSAC-Parameter
                'ransac_base_threshold_m': 0.015,
                'ransac_reference_distance_m': 0.30,
                'ransac_min_inliers_ratio': 0.7,
                'ransac_max_iterations': 20,
                'ransac_refinement_iterations': 2,
                'ransac_max_distance_factor': 50.0,

                # Kovarianz: VehicleOdometry traegt die Varianzfelder.
                'use_covariance': True,
                'covariance_pos_base_sigma_m': 0.012,
                'covariance_rot_base_sigma_rad': 0.0122,
                'covariance_reference_distance_m': 1.5,
                'covariance_reference_tag_size_m': 0.17,

                # Debug: Tag-Karte als static TF publizieren (false = null CPU)
                'publish_tag_map_tf': False,
                'tag_frame_prefix': 'tag_',

                # Isaac-Referenzgroesse nur setzen, wenn isaac_ros_apriltag
                # eine andere size hat als die Karte:
                #'isaac_reference_size': 0.162,
            }],
        ),

    ])