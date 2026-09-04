"""
Bringup unificato per la simulazione drone-rover AgroRisk.

I componenti vengono avviati in sequenza:

    1. Webots e controller del drone e del rover
    2. mission_logger
    3. mission_manager
    4. agro_alert_publisher

Prima di procedere allo stadio successivo, il launch verifica nel grafo
ROS2 la presenza delle subscription necessarie. Ogni controllo prevede
un timeout: se la condizione non viene soddisfatta, il bringup viene
interrotto per evitare di avviare la simulazione con componenti non
correttamente inizializzati.
"""

import os

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


_logger = launch.logging.get_logger('agro_bringup')


def _has_subscriber(topic, node_name):
    """Restituisce un comando shell che verifica una subscription ROS2."""
    return (
        f"ros2 topic info {topic} --verbose 2>/dev/null | "
        f"awk -v node='{node_name}' "
        "'/^Node name: /{n=$0; sub(/^Node name: /, \"\", n)} "
        "/Endpoint type: SUBSCRIPTION/{if (n == node) f=1} "
        "END{exit !f}'"
    )


def _wait_step(name, condition_cmd, description, timeout_s):
    """Crea un gate di attesa con polling e timeout."""
    script = (
        'SECONDS=0; '
        f'until {condition_cmd}; do '
        f'if [ "$SECONDS" -ge {timeout_s} ]; then '
        f'echo "[agro_bringup] timeout ({timeout_s}s) '
        f'in attesa di: {description}." >&2; '
        'exit 1; '
        'fi; '
        'sleep 0.5; '
        'done; '
        f'echo "[agro_bringup] pronto: {description}"'
    )

    return ExecuteProcess(
        cmd=['bash', '-c', script],
        name=name,
        output='screen',
    )


def generate_launch_description():
    # Riutilizza il launch Webots già definito nel package agro_webots.
    agro_webots_share = get_package_share_directory('agro_webots')

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agro_webots_share,
                'launch',
                'agro_webots_launch.py',
            )
        )
    )

    # --- Stadio 1: Webots e controller ---
    wait_webots_ready = _wait_step(
        name='wait_webots_ready',
        condition_cmd=(
            _has_subscriber('/agro/mission_cmd', 'drone_controller_webots')
            + ' && '
            + _has_subscriber('/agro/risk_confirmed', 'rover_controller_webots')
            + ' && '
            + _has_subscriber('/agro/treatment_done', 'drone_controller_webots')
        ),
        description=(
            'controller Webots associati alle interfacce ROS2 richieste'
        ),
        timeout_s=180,
    )

    # --- Stadio 2: mission_logger ---
    mission_logger = Node(
        package='agro_mission',
        executable='mission_logger',
        name='mission_logger',
        output='screen',
    )

    wait_logger_ready = _wait_step(
        name='wait_mission_logger_ready',
        condition_cmd=_has_subscriber(
            '/agro/events',
            'mission_logger',
        ),
        description='mission_logger sottoscritto a /agro/events',
        timeout_s=20,
    )

    # --- Stadio 3: mission_manager ---
    mission_manager = Node(
        package='agro_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )

    wait_manager_ready = _wait_step(
        name='wait_mission_manager_ready',
        condition_cmd=(
            _has_subscriber('/agro/alert', 'mission_manager')
            + ' && '
            + _has_subscriber('/agro/events', 'mission_manager')
            + ' && '
            + _has_subscriber('/agro/risk_confirmed', 'mission_manager')
        ),
        description=(
            'mission_manager sottoscritto a /agro/alert, /agro/events e '
            '/agro/risk_confirmed'
        ),
        timeout_s=20,
    )

    # --- Stadio 4: generazione delle allerte ---
    agro_alert_publisher = Node(
        package='agro_mission',
        executable='agro_alert_publisher',
        name='agro_alert_publisher',
        output='screen',
    )

    def _on_gate_failure(stage_description, event):
        _logger.error(
            f"'{stage_description}' non ha superato il controllo di "
            f"readiness (returncode={event.returncode}). "
            "Interruzione del bringup."
        )
        return [
            Shutdown(
                reason=f'readiness fallita: {stage_description}'
            )
        ]

    def on_webots_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_logger_ready,
                        on_exit=on_logger_gate_exit,
                    )
                ),
                mission_logger,
                wait_logger_ready,
            ]

        return _on_gate_failure('avvio Webots/controller', event)

    def on_logger_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_manager_ready,
                        on_exit=on_manager_gate_exit,
                    )
                ),
                mission_manager,
                wait_manager_ready,
            ]

        return _on_gate_failure('avvio mission_logger', event)

    def on_manager_gate_exit(event, _context):
        if event.returncode == 0:
            return [agro_alert_publisher]

        return _on_gate_failure('avvio mission_manager', event)

    # Registra l'handler del primo gate prima di avviarne il processo.
    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_webots_ready,
                on_exit=on_webots_gate_exit,
            )
        ),
        webots_launch,
        wait_webots_ready,
    ])