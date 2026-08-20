import json
import urllib.request
import pytest
import rclpy
from wifi_receiver.wifi_receiver_node import WifiReceiverNode, run_cmd


def test_run_cmd_list_execution():
    """Verifies that run_cmd safely executes commands passed as a list."""
    ok, out, err = run_cmd(["echo", "hello_wifi"], timeout=5)
    assert ok is True
    assert out == "hello_wifi"


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def wifi_node(ros_context):
    # Use port 5099 for testing to avoid conflicts
    node = WifiReceiverNode()
    yield node
    node.destroy_node()


def test_wifi_receiver_node_http_api(wifi_node):
    """Verifies HTTP CORS preflight OPTIONS and POST payload handling."""
    port = wifi_node.port

    # Test OPTIONS preflight
    req_options = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/connect-wifi",
        method="OPTIONS"
    )
    with urllib.request.urlopen(req_options) as resp:
        assert resp.status == 200
        assert resp.headers.get("Access-Control-Allow-Origin") == "*"

    # Test POST with missing SSID (returns 400 Bad Request)
    req_post_empty = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/connect-wifi",
        data=json.dumps({}).encode('utf-8'),
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    with pytest.raises(urllib.error.HTTPError) as exc_info:
        urllib.request.urlopen(req_post_empty)
    assert exc_info.value.code == 400
