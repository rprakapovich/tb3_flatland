#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class ShapeOverlayNode(Node):
    """
    A node for generating and publishing geometric primitives in Rviz2.
    Uses MarkerArray for efficient batch transfer.
    """
    def __init__(self):
        super().__init__('shape_overlay_node')
        
        self.publisher = self.create_publisher(MarkerArray, '/map_shapes', 10)
        
        self.timer = self.create_timer(1.0, self.publish_shapes)
        self.get_logger().info('Shape Overlay Node is ready. Check /map_shapes in Rviz2.')

    def create_base_marker(self, marker_id, m_type, x, y, r, g, b, a):
        """A helper function for creating a marker skeleton"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'custom_shapes'
        m.id = marker_id
        m.type = m_type
        m.action = Marker.ADD
        
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = -0.1 + marker_id/100
        
        m.pose.orientation.w = 1.0

        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = float(a)
        
        return m

    def create_hexagon(self, marker_id, x, y, inradius, r, g, b, a=1.0):
        """
        Creates a regular hexagon with the vertex facing up.
        :param inradius: Radius of the inscribed circle (from the center to the midpoint of the edge)
        """
        m = self.create_base_marker(marker_id, Marker.TRIANGLE_LIST, x, y, r, g, b, a)
        
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        
        circumradius = inradius / math.cos(math.pi / 6.0)
        
        for i in range(6):
            angle1 = i * (math.pi / 3.0)
            angle2 = (i + 1) * (math.pi / 3.0)
            
            p0 = Point(x=0.0, y=0.0, z=0.0) 
            p1 = Point(x=circumradius * math.cos(angle1), y=circumradius * math.sin(angle1), z=0.0)
            p2 = Point(x=circumradius * math.cos(angle2), y=circumradius * math.sin(angle2), z=0.0)
            
            m.points.extend([p0, p1, p2])
            
        return m

    def create_circle(self, marker_id, x, y, radius, r, g, b, a=1.0):
        """Creates a circle using the CYLINDER primitive"""
        m = self.create_base_marker(marker_id, Marker.CYLINDER, x, y, r, g, b, a)
        
        m.scale.x = radius * 2.0
        m.scale.y = radius * 2.0
        m.scale.z = 0.01 
        
        return m

    def create_square(self, marker_id, x, y, side_length, r, g, b, a=0.8):
        """Creates a square using the CUBE primitive"""
        m = self.create_base_marker(marker_id, Marker.CUBE, x, y, r, g, b, a)
        
        m.scale.x = float(side_length)
        m.scale.y = float(side_length)
        m.scale.z = 0.01 
        
        return m

    def publish_shapes(self):
        msg = MarkerArray()
        
        # 1. Black hexagon in the center (0, 0), inscribed radius = 2.875 meters
        msg.markers.append(
            self.create_hexagon(marker_id=0, x=0.0, y=0.0, inradius=2.875, r=0.0, g=0.0, b=0.0)
        )

        #2. Gray hexagon at center (0, 0), inscribed radius = 2.55 meters
        msg.markers.append(
            self.create_hexagon(marker_id=1, x=0.0, y=0.0, inradius=2.55, r=0.5, g=0.5, b=0.5)
        )
        
        # 3. Head (3.5, 0), inscribed radius = 2.5 meters
        msg.markers.append(
            self.create_hexagon(marker_id=2, x=3.5, y=0.0, inradius=1.25, r=0.0, g=1.0, b=0.0)
        )

        num = 2        
        for i in range(2):
            for j in range(2):
                # 4+. Green paws
                num += 1
                msg.markers.append(
                    self.create_hexagon(marker_id=num, x=1.8-i*3.6, y=2.7-j*5.4, inradius=0.6875, r=0.0, g=1.0, b=0.0)
                )                
        for i in range(-1,2):
            for j in range(-1,2):
                # 8+. White circles with coordinates X=0.0+i*1.1, Y=0.0+j*1.1, radius = 0.15 meters
                num += 1
                msg.markers.append(
                    self.create_circle(marker_id=num, x=0.0+i*1.1, y=0.0+j*1.1, radius=0.15, r=1.0, g=1.0, b=1.0)
                )
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShapeOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()