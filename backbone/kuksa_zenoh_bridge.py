#!/usr/bin/env python3
"""
KUKSA-zenoh Bridge
==================

Bridge for VSS-compliant police vehicle data between ROS2 and zenoh.
Receives rich police vehicle data from ROS2 and publishes it via zenoh
for the police station monitoring system.

Features:
- VSS data validation
- Real-time emergency alerts
- Data aggregation and filtering
- Performance monitoring
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zenoh
import json
import time
from datetime import datetime, timezone
import jsonschema
from typing import Dict, Any, Optional
import logging
import asyncio
import threading

class KuksaZenohBridge(Node):
    def __init__(self):
        super().__init__('kuksa_zenoh_bridge')

        # Parameters
        self.declare_parameter('zenoh_config', '')
        self.declare_parameter('police_district', 'central')
        self.declare_parameter('enable_data_validation', True)
        self.declare_parameter('emergency_alert_topics', True)

        # Initialize zenoh session
        self._init_zenoh_session()

        # Load VSS schema for validation
        self.vss_schema = self._load_vss_schema()

        # ROS2 subscribers
        self.vss_subscriber = self.create_subscription(
            String,
            'police/vss_data',
            self.handle_vss_data,
            10
        )

        # Statistics and monitoring
        self.stats = {
            'messages_received': 0,
            'messages_published': 0,
            'validation_errors': 0,
            'emergency_alerts': 0,
            'start_time': time.time(),
            'last_emergency': None,
            'active_units': set()
        }

        # Emergency detection
        self.emergency_states = {}  # Track emergency states per unit
        self.alert_cooldown = {}    # Prevent spam alerts

        # Performance monitoring
        self.performance_timer = self.create_timer(30.0, self.log_performance_stats)

        self.get_logger().info("KUKSA-zenoh Bridge initialized")

    def _init_zenoh_session(self):
        """Initialize zenoh session with configuration"""
        try:
            zenoh_config = self.get_parameter('zenoh_config').get_parameter_value().string_value
            if zenoh_config:
                config = zenoh.Config.from_file(zenoh_config)
            else:
                config = zenoh.Config()

            self.zenoh_session = zenoh.open(config)
            self.get_logger().info("zenoh session opened successfully")

            # Publisher key expressions
            self.district = self.get_parameter('police_district').get_parameter_value().string_value
            self.zenoh_keys = {
                'vss_data': f'police/{self.district}/vehicle/{{unit_id}}/vss',
                'status': f'police/{self.district}/vehicle/{{unit_id}}/status',
                'emergency': f'police/{self.district}/alerts/emergency/{{unit_id}}',
                'fleet_status': f'police/{self.district}/fleet/status',
                'heartbeat': f'police/{self.district}/bridge/heartbeat'
            }

            # Start heartbeat
            self.heartbeat_timer = self.create_timer(10.0, self.send_heartbeat)

        except Exception as e:
            self.get_logger().error(f"Failed to initialize zenoh session: {e}")
            raise

    def _load_vss_schema(self) -> Optional[Dict]:
        """Load VSS schema for data validation"""
        try:
            schema_path = 'vss_police_schema.json'
            with open(schema_path, 'r') as f:
                schema = json.load(f)
            self.get_logger().info("VSS schema loaded for validation")
            return schema
        except Exception as e:
            self.get_logger().warning(f"Could not load VSS schema: {e}")
            return None

    def _validate_vss_data(self, data: Dict[str, Any]) -> bool:
        """Validate VSS data against schema"""
        if not self.vss_schema or not self.get_parameter('enable_data_validation').get_parameter_value().bool_value:
            return True

        try:
            jsonschema.validate(data, self.vss_schema)
            return True
        except jsonschema.ValidationError as e:
            self.stats['validation_errors'] += 1
            self.get_logger().warning(f"VSS data validation failed: {e.message}")
            return False
        except Exception as e:
            self.get_logger().error(f"Validation error: {e}")
            return False

    def _extract_unit_id(self, vss_data: Dict[str, Any]) -> Optional[str]:
        """Extract unit ID from VSS data"""
        try:
            return vss_data.get('metadata', {}).get('unit_id') or \
                   vss_data.get('Vehicle', {}).get('VehicleIdentification', {}).get('Police', {}).get('UnitNumber')
        except Exception:
            return None

    def _detect_emergency_changes(self, unit_id: str, vss_data: Dict[str, Any]):
        """Detect and alert on emergency status changes"""
        try:
            current_time = time.time()
            vehicle = vss_data.get('Vehicle', {})
            police = vehicle.get('Police', {})
            emergency = police.get('Emergency', {})

            # Extract emergency states
            current_emergency = {
                'lightbar_active': emergency.get('Lightbar', {}).get('IsActive', False),
                'siren_active': emergency.get('Siren', {}).get('IsActive', False),
                'panic_button': emergency.get('PanicButton', {}).get('IsPressed', False),
                'availability': police.get('Status', {}).get('Availability', 'UNKNOWN'),
                'active_call': police.get('Equipment', {}).get('MDT', {}).get('ActiveCall')
            }

            # Check for emergency state changes
            previous_emergency = self.emergency_states.get(unit_id, {})
            emergency_alerts = []

            # Panic button alert (immediate, high priority)
            if current_emergency['panic_button'] and not previous_emergency.get('panic_button', False):
                emergency_alerts.append({
                    'type': 'PANIC_BUTTON',
                    'priority': 'CRITICAL',
                    'message': f'🚨 PANIC BUTTON ACTIVATED - {unit_id}',
                    'location': vehicle.get('CurrentLocation', {}),
                    'officers': police.get('Officer', {})
                })

            # Emergency equipment activation
            if (current_emergency['lightbar_active'] and not previous_emergency.get('lightbar_active', False)) or \
               (current_emergency['siren_active'] and not previous_emergency.get('siren_active', False)):
                emergency_alerts.append({
                    'type': 'EMERGENCY_ACTIVATION',
                    'priority': 'HIGH',
                    'message': f'🚨 Emergency equipment activated - {unit_id}',
                    'lightbar': emergency.get('Lightbar', {}),
                    'siren': emergency.get('Siren', {}),
                    'location': vehicle.get('CurrentLocation', {})
                })

            # High priority call assignment
            if (current_emergency['availability'] in ['EMERGENCY', 'CRITICAL'] and
                previous_emergency.get('availability') not in ['EMERGENCY', 'CRITICAL']):
                call_info = current_emergency.get('active_call', {})
                if call_info:
                    emergency_alerts.append({
                        'type': 'HIGH_PRIORITY_CALL',
                        'priority': 'HIGH',
                        'message': f'📻 High priority call assigned - {unit_id}',
                        'call': call_info,
                        'location': vehicle.get('CurrentLocation', {})
                    })

            # Send emergency alerts
            for alert in emergency_alerts:
                self._send_emergency_alert(unit_id, alert)

            # Update emergency state tracking
            self.emergency_states[unit_id] = current_emergency

        except Exception as e:
            self.get_logger().error(f"Error detecting emergency changes: {e}")

    def _send_emergency_alert(self, unit_id: str, alert: Dict[str, Any]):
        """Send emergency alert via zenoh"""
        try:
            current_time = time.time()
            cooldown_key = f"{unit_id}_{alert['type']}"

            # Check alert cooldown to prevent spam
            if cooldown_key in self.alert_cooldown:
                if current_time - self.alert_cooldown[cooldown_key] < 30:  # 30 second cooldown
                    return

            self.alert_cooldown[cooldown_key] = current_time

            # Prepare alert message
            alert_message = {
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'unit_id': unit_id,
                'district': self.district,
                'alert': alert,
                'bridge_id': self.get_namespace()
            }

            # Publish emergency alert
            key = self.zenoh_keys['emergency'].format(unit_id=unit_id)
            self.zenoh_session.put(key, json.dumps(alert_message))

            self.stats['emergency_alerts'] += 1
            self.stats['last_emergency'] = current_time

            self.get_logger().warning(f"Emergency alert sent: {alert['message']}")

        except Exception as e:
            self.get_logger().error(f"Failed to send emergency alert: {e}")

    def handle_vss_data(self, msg):
        """Handle incoming VSS data from ROS2"""
        try:
            self.stats['messages_received'] += 1
            self.get_logger().warning(f"★★★★★ ROS2 VSS DATA RECEIVED ★★★★★, Length: {len(msg.data)}")

            # Parse VSS data
            vss_data = json.loads(msg.data)

            # Validate data
            if not self._validate_vss_data(vss_data):
                return

            # Extract unit ID
            unit_id = self._extract_unit_id(vss_data)
            if not unit_id:
                self.get_logger().warning("Could not extract unit ID from VSS data")
                return

            self.stats['active_units'].add(unit_id)

            # Detect emergency changes
            self._detect_emergency_changes(unit_id, vss_data)

            # Enrich data with bridge metadata
            enriched_data = {
                **vss_data,
                'bridge_metadata': {
                    'received_timestamp': datetime.now(timezone.utc).isoformat(),
                    'bridge_id': self.get_namespace(),
                    'district': self.district,
                    'processing_latency_ms': 0  # Could add actual latency measurement
                }
            }

            # Publish to zenoh
            key = self.zenoh_keys['vss_data'].format(unit_id=unit_id)
            self.get_logger().info(f"🚀 DEBUG: Publishing to zenoh key: {key}")
            self.zenoh_session.put(key, json.dumps(enriched_data))
            self.get_logger().info(f"📊 DEBUG: Published VSS data for {unit_id}")

            self.stats['messages_published'] += 1

        except Exception as e:
            self.get_logger().error(f"Error handling VSS data: {e}")

    def send_heartbeat(self):
        """Send bridge heartbeat and fleet status"""
        try:
            current_time = datetime.now(timezone.utc).isoformat()

            # Bridge heartbeat
            heartbeat = {
                'timestamp': current_time,
                'bridge_id': self.get_namespace(),
                'district': self.district,
                'status': 'ACTIVE',
                'uptime_seconds': time.time() - self.stats['start_time'],
                'statistics': {
                    'messages_received': self.stats['messages_received'],
                    'messages_published': self.stats['messages_published'],
                    'validation_errors': self.stats['validation_errors'],
                    'emergency_alerts': self.stats['emergency_alerts']
                }
            }

            self.zenoh_session.put(self.zenoh_keys['heartbeat'], json.dumps(heartbeat))

            # Fleet status summary
            fleet_status = {
                'timestamp': current_time,
                'district': self.district,
                'active_units_count': len(self.stats['active_units']),
                'active_units': list(self.stats['active_units']),
                'last_emergency': self.stats['last_emergency'],
                'bridge_status': 'ACTIVE'
            }

            self.zenoh_session.put(self.zenoh_keys['fleet_status'], json.dumps(fleet_status))

        except Exception as e:
            self.get_logger().error(f"Error sending heartbeat: {e}")

    def log_performance_stats(self):
        """Log performance statistics"""
        uptime = time.time() - self.stats['start_time']
        msg_rate = self.stats['messages_received'] / uptime if uptime > 0 else 0

        self.get_logger().info(
            f"Performance Stats - "
            f"Uptime: {uptime:.1f}s, "
            f"Msg Rate: {msg_rate:.2f}/s, "
            f"Active Units: {len(self.stats['active_units'])}, "
            f"Errors: {self.stats['validation_errors']}, "
            f"Alerts: {self.stats['emergency_alerts']}"
        )

def main(args=None):
    rclpy.init(args=args)

    try:
        bridge = KuksaZenohBridge()

        # Start bridge
        bridge.get_logger().info("Starting KUKSA-zenoh Bridge...")
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Bridge error: {e}")
    finally:
        if 'bridge' in locals():
            # Close zenoh session
            if hasattr(bridge, 'zenoh_session'):
                bridge.zenoh_session.close()
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()