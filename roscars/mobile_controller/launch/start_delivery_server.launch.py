from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

class StartDeliveryLauncher:
    def __init__(self):
        self.roscar_ns_arg = DeclareLaunchArgument(
            'roscar_namespaces',
            default_value='roscar_01,roscar_02',
            description='Comma-separated list of roscar namespaces'
        )
        self.roscar_domain_arg = DeclareLaunchArgument(
            'roscar_domains',
            default_value='11,12',
            description='Comma-separated list of domain IDs corresponding to namespaces'
        )

        self.roscar_ns_config = LaunchConfiguration('roscar_namespaces')
        self.roscar_domain_config = LaunchConfiguration('roscar_domains')

    def generate_nodes(self, context):
        ns_list = self.roscar_ns_config.perform(context).split(',')
        domain_list = self.roscar_domain_config.perform(context).split(',')

        if len(ns_list) != len(domain_list):
            raise RuntimeError("[launch] roscar_namespaces와 roscar_domains의 길이가 다릅니다.")

        actions = []

        for ns, domain in zip(ns_list, domain_list):
            ns = ns.strip().lstrip('/')
            domain = domain.strip()

            server_node = Node(
                package='mobile_controller',
                executable='start_delivery_server',
                namespace=ns,
                name='start_delivery_server',
                output='screen',
                env={'ROS_DOMAIN_ID': domain}
            )

            actions.append(server_node)

        return actions

    def generate_launch_description(self):
        return LaunchDescription([
            self.roscar_ns_arg,
            self.roscar_domain_arg,
            OpaqueFunction(function=self.generate_nodes)
        ])

def generate_launch_description():
    return StartDeliveryLauncher().generate_launch_description()
