"""Integration tests: OdriDriver with MockBackend."""
import time
import pytest

from odri_can.can.mock_backend import MockBackend
from odri_can.protocol.odri_driver import OdriDriver


@pytest.fixture
def driver():
    backend = MockBackend()
    backend.open()
    drv = OdriDriver(backend)
    drv.set_iq_limit(10.0)
    yield drv
    drv.shutdown()
    backend.close()


def _wait_for(condition, timeout=3.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(interval)
    return False


def test_status_received(driver):
    """Driver receives status frames from mock."""
    driver.enable_telemetry(True)
    assert _wait_for(lambda: driver.state.is_connected, timeout=1.0)


def test_enable_sequence(driver):
    """Standard enable sequence works."""
    driver.enable_telemetry(True)
    driver.set_iq_ref(0.0, 0.0)
    driver.enable_system()
    driver.enable_motors()
    # Mock simulates ~1.5s alignment
    assert _wait_for(lambda: driver.state.both_motors_ready, timeout=3.0)
    assert driver.state.sys_enabled
    assert driver.state.motor1.enabled
    assert driver.state.motor2.enabled


def test_iq_clamping(driver):
    """IqRef is clamped to limit."""
    driver.set_iq_limit(1.0)
    driver.set_iq_ref(5.0, -5.0)
    # Internal stored value is clamped
    assert driver._iq_ref_1 == 1.0
    assert driver._iq_ref_2 == -1.0


def test_motor_response(driver):
    """Sending IqRef causes motor to accelerate (in mock model)."""
    driver.enable_telemetry(True)
    driver.set_iq_ref(0.0, 0.0)
    driver.enable_system()
    driver.enable_motors()
    assert _wait_for(lambda: driver.state.both_motors_ready, timeout=3.0)

    driver.set_iq_ref(0.5, -0.5)
    # Wait for motor velocity to build up
    assert _wait_for(lambda: abs(driver.state.motor1.velocity) > 0.1, timeout=2.0)
    # Motor 1 should spin positive, motor 2 negative
    assert driver.state.motor1.velocity > 0
    assert driver.state.motor2.velocity < 0


def test_estop(driver):
    """E-stop disables motors immediately."""
    driver.enable_telemetry(True)
    driver.enable_system()
    driver.enable_motors()
    assert _wait_for(lambda: driver.state.both_motors_ready, timeout=3.0)

    driver.e_stop()
    time.sleep(0.1)
    assert not driver.state.motor1.enabled
    assert not driver.state.motor2.enabled
    assert not driver.state.sys_enabled
