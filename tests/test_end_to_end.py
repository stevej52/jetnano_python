"""The whole stack: a transmitter and a receiver in threads, talking over a real
UDP socket and a loopback serial port, with the Wi-Fi link cut and restored."""
import socket
import threading

import serial

from conftest import wait_until
from jetnano_control.config import Config, ReceiverConfig, TelemetryConfig, TransmitterConfig
from jetnano_control.links import SerialLink, UdpLink
from jetnano_control.receiver import Receiver
from jetnano_control.servo import Drive, DriveCalibration, MockServoOutput
from jetnano_control.transmitter import Transmitter
from test_transmitter import ScriptedJoystick


def free_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_failover_recovery_and_failsafe():
    port = free_port()
    loop = serial.serial_for_url("loop://", timeout=0.1)
    cfg = Config(
        drive=DriveCalibration(esc_arm_seconds=0.0, throttle_slew_per_s=0.0),
        receiver=ReceiverConfig(control_rate_hz=100.0, fresh_for_s=0.3, status_interval_s=0.1),
        transmitter=TransmitterConfig(rate_hz=50.0, status_print_interval_s=100.0, shutdown_frames=3),
        telemetry=TelemetryConfig(enabled=False),
    )
    rx_links = [UdpLink("wifi", 1, local_port=port, bind_host="127.0.0.1"),
                SerialLink("radio", 3, port="loop", serial_factory=lambda: loop, max_send_hz=30)]
    tx_links = [UdpLink("wifi", 1, remote_host="127.0.0.1", remote_port=port),
                SerialLink("radio", 3, port="loop", serial_factory=lambda: loop, max_send_hz=30)]
    output = MockServoOutput()
    receiver = Receiver(cfg, rx_links, Drive(cfg.drive, output))
    joystick = ScriptedJoystick()
    transmitter = Transmitter(cfg, tx_links, joystick)
    tx_udp = tx_links[0]

    rx_thread = threading.Thread(target=receiver.run, name="rx", daemon=True)
    tx_thread = threading.Thread(target=transmitter.run, name="tx", daemon=True)
    rx_thread.start()
    tx_thread.start()
    try:
        # 1. Control arrives over Wi-Fi and moves the servos: steer +0.5, throttle +0.2.
        assert wait_until(lambda: receiver.active_link == "wifi" and receiver.drive.current.armed)
        assert wait_until(lambda: output.angles.get(1) == 110.0 and output.angles.get(2) == 57.5
                          and output.angles.get(0) == 97.0)
        assert wait_until(lambda: "active=wifi" in transmitter.robot_status)   # status came back over UDP

        # 2. Wi-Fi dies: the transmitter's datagrams go to a port nobody listens on.
        tx_udp.remote = ("127.0.0.1", free_port())
        assert wait_until(lambda: receiver.active_link == "radio", timeout=3.0)
        assert receiver.switches == 1 and not receiver.failsafe and receiver.drive.current.armed
        joystick.set(axes=(0.0, 0.0, -1.0, 0.2))
        assert wait_until(lambda: output.angles.get(1) == 30.0 and output.angles.get(2) == 135.0)
        assert wait_until(lambda: "active=radio" in transmitter.robot_status)

        # 3. Wi-Fi comes back and takes over again.
        tx_udp.remote = ("127.0.0.1", port)
        assert wait_until(lambda: receiver.active_link == "wifi", timeout=3.0)
        assert receiver.switches == 2 and receiver.failsafe_entries == 0

        # 4. The operator's joystick drops out: the robot is told to hold neutral at once.
        joystick.attached = False
        assert wait_until(lambda: not receiver.drive.current.armed)
        assert not receiver.failsafe and output.angles[0] == 90.0 and output.angles[1] == 85.0
        joystick.attached = True
        assert wait_until(lambda: receiver.drive.current.armed)

        # 5. The transmitter quits: e-stop frames stop the robot immediately, then the watchdog trips.
        transmitter.request_stop()
        tx_thread.join(timeout=5)
        assert not tx_thread.is_alive()
        assert wait_until(lambda: not receiver.drive.current.armed, timeout=1.0)
        assert wait_until(lambda: receiver.failsafe, timeout=2.0)
        assert receiver.failsafe_entries == 1 and receiver.active_link is None
    finally:
        receiver.request_stop()
        rx_thread.join(timeout=5)
        if tx_thread.is_alive():
            transmitter.request_stop()
            tx_thread.join(timeout=5)
    assert not rx_thread.is_alive()
    assert output.closed and output.angles == {0: 90.0, 1: 85.0, 2: 85.0}
    assert receiver.loop_errors == 0
    assert all(not link.is_open for link in rx_links + tx_links)
