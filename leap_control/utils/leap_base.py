from rclpy.node import Node

from rclpy.qos import qos_profile_rosout_default

class LeapBase(Node):
    def __init__(
        self,
        node_name,
        *,
        context=None,
        cli_args=None,
        namespace=None,
        use_global_arguments=True,
        enable_rosout=True,
        rosout_qos_profile=qos_profile_rosout_default,
        start_parameter_services=True,
        parameter_overrides=None,
        allow_undeclared_parameters=False,
        automatically_declare_parameters_from_overrides=False,
        enable_logger_service=False,
    ):
        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            rosout_qos_profile=rosout_qos_profile,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides,
            enable_logger_service=enable_logger_service,
        )

    def get_rosparam(self, parameter_name, default_value):
        self.declare_parameter(parameter_name, default_value)
        parameter = self.get_parameter(parameter_name).value
        self.get_logger().info(f"Starting finger manager for: {parameter}")

        return parameter
