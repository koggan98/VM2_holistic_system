import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import trimesh  # Library for working with STL files
import os

class AddSTLObject(Node):
    def __init__(self):
        super().__init__('add_stl_object')

        # Publisher for CollisionObject
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)

        # Path to the STL file

        stl_file_path2 = os.path.join( os.path.dirname(os.path.abspath(__file__)), "collisionObjects", "stls", "base_link.STL")
        print(stl_file_path2)

        #stl_file_path = "/home/alessandra/ros2_humble/src/urdf_ros2_rviz2/meshes/table/base_link.STL"
        
        #print('Get current working directory : ', os.getcwd())
        #print('Absolute path of file:     ', 
        #    os.path.abspath(__file__))
        #print('Absolute directoryname: ', 
        #    os.path.dirname(os.path.abspath(__file__)))


        # Load STL file and convert it to ROS Mesh
        table_mesh = self.load_stl_as_ros_mesh(stl_file_path2)

        # Pose of the table
        table_pose = Pose()
        table_pose.position.x = 0.0  # X position of the table
        table_pose.position.y = 0.0  # Y position of the table
        table_pose.position.z = -1.03/2 + 0.06  # Z position of the table (height)
        table_pose.orientation.w = 1.0  # Orientation of the table

        # Create a CollisionObject
        collision_object = CollisionObject()
        collision_object.id = "table"
        collision_object.header = Header()
        collision_object.header.frame_id = "world"
        collision_object.meshes.append(table_mesh)
        collision_object.mesh_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD

        # Publish the CollisionObject to the planning scene
        self.publisher.publish(collision_object)
        self.get_logger().info("STL table successfully added!")

    def load_stl_as_ros_mesh(self, stl_path):
        """Load STL file and convert it into ROS-compatible Mesh."""
        mesh = trimesh.load(stl_path)
        ros_mesh = Mesh()

        # Convert STL faces into MeshTriangles
        for face in mesh.faces:
            # Ensure face indices are integers within the acceptable range
            triangle = MeshTriangle()
            triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
            ros_mesh.triangles.append(triangle)

        # Convert STL vertices into ROS-friendly Points
        for vertex in mesh.vertices:
            point = Point()
            point.x, point.y, point.z = float(vertex[0]), float(vertex[1]), float(vertex[2])
            ros_mesh.vertices.append(point)

        return ros_mesh

def main(args=None):
    rclpy.init(args=args)
    node = AddSTLObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
