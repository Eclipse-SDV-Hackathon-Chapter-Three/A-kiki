#!/usr/bin/env python3
"""
Camera Pipeline Tester
======================
Debug tool to test each stage of the camera streaming pipeline:
1. CARLA -> ROS2 topics
2. ROS2 -> zenoh
3. zenoh -> WebSocket
4. WebSocket -> Browser
"""

import subprocess
import time
import sys
import os

def check_carla():
    """Check if CARLA is running"""
    print("🔍 Checking CARLA connection...")
    try:
        import carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        print(f"✅ CARLA connected: {world.get_map().name}")
        return True
    except Exception as e:
        print(f"❌ CARLA not running: {e}")
        return False

def check_ros2_topics():
    """Check if ROS2 camera topics exist"""
    print("\n🔍 Checking ROS2 camera topics...")
    try:
        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 topic list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        topics = result.stdout.strip().split('\n')
        camera_topics = [t for t in topics if 'camera' in t and 'image' in t]

        if camera_topics:
            print(f"✅ Found {len(camera_topics)} camera topics:")
            for topic in camera_topics:
                print(f"   - {topic}")
            return True
        else:
            print("❌ No camera topics found")
            print("   Available topics:")
            for topic in topics[:10]:
                print(f"   - {topic}")
            return False
    except Exception as e:
        print(f"❌ Error checking ROS2 topics: {e}")
        return False

def check_ros2_camera_data():
    """Check if camera data is being published"""
    print("\n🔍 Checking ROS2 camera data...")
    try:
        result = subprocess.run(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'timeout 3 ros2 topic echo /carla/hero/camera/rgb/front/image_color --once'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if 'height:' in result.stdout and 'width:' in result.stdout:
            # Extract dimensions
            for line in result.stdout.split('\n'):
                if 'height:' in line or 'width:' in line:
                    print(f"✅ Camera data: {line.strip()}")
            return True
        else:
            print("❌ No camera data received")
            return False
    except subprocess.TimeoutExpired:
        print("❌ Timeout waiting for camera data")
        return False
    except Exception as e:
        print(f"❌ Error checking camera data: {e}")
        return False

def test_standalone_bridge():
    """Test the camera bridge standalone"""
    print("\n🔍 Testing camera zenoh bridge (standalone)...")
    print("   Starting bridge for 10 seconds...")

    try:
        process = subprocess.Popen(
            ['python3', 'ros2_camera_zenoh_bridge.py', '--quality', '85'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # Wait and capture output
        time.sleep(10)
        process.terminate()
        stdout, stderr = process.communicate(timeout=5)

        # Check output
        if 'Stats:' in stdout or 'Stats:' in stderr:
            print("✅ Bridge is publishing frames!")
            print("   Sample output:")
            for line in (stdout + stderr).split('\n')[-5:]:
                if line.strip():
                    print(f"   {line}")
            return True
        else:
            print("❌ Bridge not publishing frames")
            print("   Output:")
            print(stdout[-500:] if stdout else "No stdout")
            print(stderr[-500:] if stderr else "No stderr")
            return False

    except Exception as e:
        print(f"❌ Error testing bridge: {e}")
        return False

def test_zenoh_flow():
    """Test if zenoh is receiving data"""
    print("\n🔍 Testing zenoh data flow...")
    print("   Checking zenoh router...")

    result = subprocess.run(['pgrep', '-f', 'zenohd'], capture_output=True)
    if result.returncode == 0:
        print("✅ zenoh router is running")
    else:
        print("⚠️  zenoh router not running, starting it...")
        subprocess.Popen(['zenohd'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)

    # Try to subscribe to camera data
    print("   Subscribing to camera/front_rgb/compressed for 5 seconds...")
    try:
        process = subprocess.Popen(
            ['python3', '-c', '''
import zenoh
import time
import sys

session = zenoh.open(zenoh.Config())
count = [0]

def listener(sample):
    count[0] += 1
    print(f"Received frame {count[0]}, size: {len(sample.payload)} bytes", flush=True)

subscriber = session.declare_subscriber("camera/**/compressed", listener)
time.sleep(5)
subscriber.undeclare()
session.close()
print(f"Total frames received: {count[0]}", flush=True)
'''],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        stdout, stderr = process.communicate(timeout=10)

        if 'Received frame' in stdout:
            print("✅ zenoh is receiving camera data!")
            print(f"   {stdout.strip()}")
            return True
        else:
            print("❌ No data received from zenoh")
            print(f"   Output: {stdout}")
            print(f"   Errors: {stderr}")
            return False

    except Exception as e:
        print(f"❌ Error testing zenoh: {e}")
        return False

def main():
    print("=" * 60)
    print("Camera Pipeline Diagnostic Tool")
    print("=" * 60)

    # Run all checks
    checks = [
        ("CARLA Connection", check_carla),
        ("ROS2 Topics", check_ros2_topics),
        ("ROS2 Camera Data", check_ros2_camera_data),
        ("Camera Bridge", test_standalone_bridge),
        ("Zenoh Data Flow", test_zenoh_flow),
    ]

    results = {}
    for name, check_func in checks:
        try:
            results[name] = check_func()
        except Exception as e:
            print(f"❌ {name} check failed with exception: {e}")
            results[name] = False
        time.sleep(1)

    # Summary
    print("\n" + "=" * 60)
    print("Summary:")
    print("=" * 60)
    for name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status} - {name}")

    print("\n" + "=" * 60)
    if all(results.values()):
        print("🎉 All checks passed! Camera pipeline should work.")
    else:
        print("⚠️  Some checks failed. Please fix the issues above.")

        # Provide suggestions
        if not results.get("CARLA Connection"):
            print("\n💡 Suggestion: Start CARLA simulator first")
            print("   cd /path/to/carla && ./CarlaUE4.sh")

        if not results.get("ROS2 Topics"):
            print("\n💡 Suggestion: Start CARLA vehicle spawner")
            print("   python3 carla_vehicle_spawner.py")

    print("=" * 60)

if __name__ == '__main__':
    main()
