#!/usr/bin/env python
import os
import platform
import time
import math
from typing import TYPE_CHECKING, Optional

import rclpy
import rclpy.node
from rclpy.validate_namespace import validate_namespace

if TYPE_CHECKING:
    import rclpy.client

from controller_manager_msgs.srv import SwitchController
from mirte_msgs.srv import (
    GetColorHSL,
    GetColorRGBW,
    GetEncoder,
    GetIntensity,
    GetIntensityDigital,
    GetKeypad,
    GetPinValue,
    GetRange,
    SetMotorSpeed,
    SetOLEDFile,
    SetOLEDText,
    SetPinValue,
    SetServoAngle,
)
from rcl_interfaces.srv import ListParameters

mirte = {}

# No QoS Profiles are set, but this might not be required, since they might already behave like ROS 1 persistant.


class Robot:
    """Robot API

    This class allows you to control the robot from Python. The getters and setters
    are just wrappers calling ROS topics or services.
    """

    # Implementation Notes:
    # This class creates a hidden ROS Node for the communication.
    # This should only be run once, however this can not be prevented from the web interface.
    # Therefore the node is also anonymized with the current time.

    def __init__(
        self, machine_namespace: Optional[str] = None, hardware_namespace: str = "io"
    ):
        """Intialize the Mirte Robot API

        Parameters:
            machine_namespace (Optional[str], optional): The Namespace from '/' to the ROS namespace for the specific Mirte. Defaults to "/{HOSTNAME}". (This only has to be changed when running the Robot API from a different machine directly. It is configured correctly for the Web interface)
            hardware_namespace (str, optional): The namespace for the hardware peripherals. Defaults to "io".
        """
        self._machine_namespace = (
            machine_namespace
            if machine_namespace and validate_namespace(machine_namespace)
            else "/" + platform.node().replace("-", "_").lower()
        )
        self._hardware_namespace = (
            hardware_namespace
            if validate_namespace(
                hardware_namespace
                if hardware_namespace.startswith("/")
                else (self._machine_namespace + "/" + hardware_namespace)
            )
            else "io"
        )

        ROS_DISTRO = os.getenv("ROS_DISTRO")

        rclpy.init()

        # This node should be only ran once.
        # No 'anonymous' flag available, so use unix epoch nano seconds to pad name
        self._node = rclpy.node.Node(
            "_mirte_python_api_" + str(time.time_ns()),
            namespace=self._machine_namespace,
            start_parameter_services=True,
        )

        # Stop robot when exited
        rclpy.get_default_context().on_shutdown(self._at_exit)

        self.CONTROLLER = "diffbot_base_controller"

        self.PWM = 3  # PrivateConstants.PWM when moving to Python3
        self.INPUT = 0
        self.OUTPUT = 1
        self.PULLUP = 11
        self.ANALOG = 2

        # Start timing
        self.begin_time = time.time()
        self.last_call = 0

        # Call controller_manager/switch_controller service to disable/enable the ROS diff_drive_controller
        # By default this class will control the robot though PWM (controller stopped). Only in case
        # the controller is needed, it will be enabled.
        self.switch_controller_service = self._node.create_client(
            SwitchController, "controller_manager/switch_controller"
        )

        if ROS_DISTRO[0] >= "i":  # Check if the ROS Distro is IRON or newer
            # Not available untill ROS Iron
            if not self._node.wait_for_node(
                self._hardware_namespace + "telemetrix", 10
            ):
                self._node.get_logger().fatal(
                    f"Telemetrix node at '{self._node.get_namespace() + self._hardware_namespace  + 'telemetrix'}' was not found! Aborting"
                )
                exit(-1)
        list_parameters: rclpy.client.Client = self._node.create_client(
            ListParameters, self._hardware_namespace + "/telemetrix/list_parameters"
        )

        # Service for motor speed
        self.motors = {}  # FIXME: Is self.motors used?
        motors_future: rclpy.Future = list_parameters.call_async(
            ListParameters.Request(prefixes=["motor"], depth=3)
        )
        rclpy.spin_until_future_complete(self._node, motors_future)

        motor_prefixes: list[str] = motors_future.result().result.prefixes
        if len(motor_prefixes) > 0:
            self.motors = [
                motor_prefix.split(".")[-1] for motor_prefix in motor_prefixes
            ]
            self.motor_services = {}
            for motor in self.motors:
                self._node.get_logger().info(
                    f"Created service client for motor [{motor}]"
                )
                self.motor_services[motor] = self._node.create_client(
                    SetMotorSpeed,
                    self._hardware_namespace + "/motor/" + motor + "/set_speed",
                )

            # Service for motor speed

        servo_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["servo"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, servo_future)

        servo_prefixes = servo_future.result().result.prefixes
        if len(servo_prefixes) > 0:
            servos = [servo_prefix.split(".")[-1] for servo_prefix in servo_prefixes]
            self.servo_services: dict[str, rclpy.client.Client] = {}
            for servo in servos:
                self._node.get_logger().info(
                    f"Created service client for servo [{servo}]"
                )
                self.servo_services[servo] = self._node.create_client(
                    SetServoAngle,
                    self._hardware_namespace + "/servo/" + servo + "/set_angle",
                )

        ## Sensors
        ## The sensors are now just using a blocking service call. This is intentionally
        ## since one first needs to learn to program in a synchronous way without events.
        ## Event based programming is already possible using the ROS topics for the
        ## same sensors. At a later stage we will expose this as well to this API and
        ## maybe even to blockly.

        # Services for distance sensors
        distance_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["distance"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, distance_future)

        distance_prefixes = distance_future.result().result.prefixes
        if len(distance_prefixes) > 0:
            distance_sensors = [sensor.split(".")[-1] for sensor in distance_prefixes]
            self.distance_services = {}
            for sensor in distance_sensors:
                self._node.get_logger().info(
                    f"Created service client for distance sensor [{sensor}]"
                )
                self.distance_services[sensor] = self._node.create_client(
                    GetRange,
                    self._hardware_namespace + "/distance/" + sensor + "/get_range",
                )

        # Services for oled
        oled_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["oled"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, oled_future)

        oled_prefixes = oled_future.result().result.prefixes
        if len(oled_prefixes) > 0:
            oleds = [oled.split(".")[-1] for oled in oled_prefixes]
            self.oled_services = {}
            for oled in oleds:
                self._node.get_logger().info(
                    f"Created service client for oled [{oled}]"
                )
                self.oled_services[oled] = {
                    "text": self._node.create_client(
                        SetOLEDText,
                        self._hardware_namespace + "/oled/" + oled + "/set_text",
                    ),
                    "file": self._node.create_client(
                        SetOLEDFile,
                        self._hardware_namespace + "/oled/" + oled + "/set_file",
                    ),
                }

        # Services for intensity sensors (TODO: how to expose the digital version?)
        intensity_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["intensity"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, intensity_future)

        intensity_prefixes = intensity_future.result().result.prefixes
        if len(intensity_prefixes) > 0:
            intensity_sensors = [sensor.split(".")[-1] for sensor in intensity_prefixes]
            self.intensity_services = {}

            # We can not get the types (analog and/or digital) of the intensity sensor
            # straight from the parameter server (it might be just set as the PCB without
            # explicit values. We can however deduct what is there by checking the
            # services.
            service_list = set(
                [
                    service
                    for service, service_type in self._node.get_service_names_and_types()
                ]
            )
            for sensor in intensity_sensors:
                if (
                    self._hardware_namespace + "/intensity/" + sensor + "/get_analog"
                    in service_list
                ):
                    self._node.get_logger().info(
                        f"Created service client for intensity [{sensor}]"
                    )
                    self.intensity_services[sensor] = self._node.create_client(
                        GetIntensity,
                        self._hardware_namespace
                        + "/intensity/"
                        + sensor
                        + "/get_analog",
                    )
                if (
                    self._hardware_namespace + "/intensity/" + sensor + "/get_digital"
                    in service_list
                ):
                    self._node.get_logger().info(
                        f"Created service client for digital intensity [{sensor}]"
                    )
                    self.intensity_services[sensor + "_digital"] = (
                        self._node.create_client(
                            GetIntensityDigital,
                            self._hardware_namespace
                            + "/intensity/"
                            + sensor
                            + "/get_digital",
                        )
                    )

        # Services for encoder sensors
        encoder_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["encoder"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, encoder_future)

        encoder_prefixes = encoder_future.result().result.prefixes
        if len(encoder_prefixes) > 0:
            encoder_sensors = [sensor.split(".")[-1] for sensor in encoder_prefixes]
            self.encoder_services = {}
            for sensor in encoder_sensors:
                self._node.get_logger().info(
                    f"Created service client for encoder [{sensor}]"
                )
                self.encoder_services[sensor] = self._node.create_client(
                    GetEncoder,
                    self._hardware_namespace + "/encoder/" + sensor + "/get_encoder",
                )

        # Services for keypad sensors
        keypad_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["keypad"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, keypad_future)

        keypad_prefixes = keypad_future.result().result.prefixes
        if len(keypad_prefixes) > 0:
            keypad_sensors = [sensor.split(".")[-1] for sensor in keypad_prefixes]
            self.keypad_services = {}
            for sensor in keypad_sensors:
                self._node.get_logger().info(
                    f"Created service client for keypad [{sensor}]"
                )
                self.keypad_services[sensor] = self._node.create_client(
                    GetKeypad,
                    self._hardware_namespace + "/keypad/" + sensor + "/get_key",
                )

        # Services for color sensors
        color_future = list_parameters.call_async(
            ListParameters.Request(prefixes=["color"], depth=3)
        )

        rclpy.spin_until_future_complete(self._node, color_future)

        color_prefixes = color_future.result().result.prefixes
        if len(color_prefixes) > 0:
            color_sensors = [sensor.split(".")[-1] for sensor in color_prefixes]
            self.color_services: dict[str, dict[str, "rclpy.client.Client"]] = {}
            for sensor in color_sensors:
                self._node.get_logger().info(
                    f"Created service client for color sensor [{sensor}]"
                )
                self.color_services[sensor] = {
                    "RGBW": self._node.create_client(
                        GetColorRGBW,
                        self._hardware_namespace + "/color/" + sensor + "/get_rgbw",
                    ),
                    "HSL": self._node.create_client(
                        GetColorHSL,
                        self._hardware_namespace + "/color/" + sensor + "/get_hsl",
                    ),
                }

        self.get_pin_value_service = self._node.create_client(
            GetPinValue, self._hardware_namespace + "/get_pin_value"
        )
        self.set_pin_value_service = self._node.create_client(
            SetPinValue, self._hardware_namespace + "/set_pin_value"
        )

    def _call_service(
        self, client: rclpy.client.Client, request: rclpy.client.SrvTypeRequest
    ) -> rclpy.client.SrvTypeResponse:
        client.wait_for_service()

        future_response = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future_response)
        return future_response.result()

    # FIXME: Check if services are available, if not don't hard error on:
    # AttributeError: 'Robot' object has no attribute 'oled_services'. Did you mean: '_services'?
    def _check_available(self, services: dict[str] | None, id: str) -> bool:
        if services is None:
            return False
        return id in services

    def getTimestamp(self):
        """Gets the elapsed time in seconds since the initialization fo the Robot.

        Returns:
            float: Time in seconds since the initialization of the Robot. Fractions of a second may be present if the system clock provides them.
        """

        return time.time() - self.begin_time

    def getTimeSinceLastCall(self):
        """Gets the elapsed time in seconds since the last call to this function.

        Returns:
            float: Time in seconds since last call to this function. Fractions of a second may be present if the system clock provides them.
        """

        last_call = self.last_call
        self.last_call = time.time()
        if last_call == 0:
            return 0
        else:
            return time.time() - last_call

    def getDistance(self, sensor):
        """Gets data from a HC-SR04 distance sensor: calculated distance in meters.

        Parameters:
            sensor (str): The name of the sensor as defined in the configuration.

        Returns:
            int: Range in meters measured by the HC-SR04 sensor. (The distance gets clamped to minimum and maximum range of the HC-SR04 sensor)

        Warning:
            A maximum of 6 distance sensors is supported.
        """

        value = self._call_service(self.distance_services[sensor], GetRange.Request())
        range = value.range

        distance: float = range.range

        # FIXME: What to do about nan?
        if distance == math.inf:
            distance = range.max_range
        elif distance == -math.inf:
            distance = range.min_range

        return distance

    def getIntensity(self, sensor, type="analog"):
        """Gets data from an intensity sensor.

        Parameters:
            sensor (str): The name of the sensor as defined in the configuration.
            type (str): The type of the sensor (either 'analog' or 'digital').

        Returns:
            int: Value of the sensor (0-255 when analog, 0-1 when digital).
        """
        if type == "analog":
            value = self._call_service(
                self.intensity_services[sensor], GetIntensity.Request()
            )
        if type == "digital":
            value = self._call_service(
                self.intensity_services[sensor + "_digital"],
                GetIntensityDigital.Request(),
            )
        return value.data

    def getEncoder(self, sensor):
        """Gets data from an encoder: every encoder pulse increments the counter.

        Parameters:
            sensor (str): The name of the sensor as defined in the configuration.

        Returns:
            int: Number of encoder pulses since boot of the robot.
        """

        value = self._call_service(self.encoder_services[sensor], GetEncoder.Request())
        return value.data

    def getKeypad(self, keypad):
        """Gets the value of the keypad: the button that is pressed.

        Parameters:
            keypad (str): The name of the sensor as defined in the configuration.

        Returns:
            str: The name of the button ('up', 'down', 'left', 'right', 'enter').
        """

        value = self._call_service(self.keypad_services[keypad], GetKeypad.Request())
        return value.data

    def getColorRGBW(self, sensor):
        """Gets the value of the color sensor.

        Parameters:
            sensor (str): The name of the sensor as defined in the configuration.

        Returns:
            {r, g, b, w}: Scaled (0-1) values per R(ed), G(reen), B(lue), and W(hite).
        """

        value = self._call_service(
            self.color_services[sensor]["RGBW"], GetColorRGBW.Request()
        )
        return {
            "r": value.color.r,
            "g": value.color.g,
            "b": value.color.b,
            "w": value.color.w,
        }

    def getColor(self, sensor):
        """Gets the value of the color sensor.

        Parameters:
            sensor (str): The name of the sensor as defined in the configuration.

        Returns:
            {h, s, l}: Hue (0-360), Saturation (0-1), Lightness.
        """

        value = self._call_service(
            self.color_services[sensor]["HSL"], GetColorHSL.Request()
        )
        return {
            "h": value.color.h,
            "s": value.color.s,
            "l": value.color.l,
        }

    def getAnalogPinValue(self, pin):
        """Gets the input value of an analog pin.

        Parameters:
            pin (str): The pin number of an analog pin as printed on the microcontroller.

        Returns:
            int: Value between 0-255.
        """

        value = self._call_service(
            self.get_pin_value_service, GetPinValue.Request(pin=str(pin), type="analog")
        )
        return value.data

    def setAnalogPinValue(self, pin, value):
        """Sets the output value of an analog pin (PWM).

        Parameters:
            pin (str): The pin number of an analog pin as printed on the microcontroller.
            value (int): Value between 0-255.
        """

        value = self._call_service(
            self.set_pin_value_service,
            SetPinValue.Request(pin=str(pin), type="analog", value=value),
        )
        return value.status

    def setOLEDText(self, oled, text):
        """Shows text on the OLED.

        Parameters:
            oled (str): The name of the sensor as defined in the configuration.
            text (str): String to be shown on the 128x64 OLED.
        """
        value = self._call_service(
            self.oled_services[oled]["text"], SetOLEDText.Request(text=str(text))
        )
        return value.status

    def setOLEDImage(self, oled, image):
        """Shows image on the OLED.

        Parameters:
            oled (str): The name of the sensor as defined in the configuration.
            image (str): Image name/path either an absolute path, a path
                            relative to the folder of the mirte-oled-images
                            repository or and package relative path (pkg://PACKAGE_NAME/REST/OF/PATH).
                            The image extension can be omitted if its png.
        """

        value = self._call_service(
            self.oled_services[oled]["file"],
            SetOLEDFile.Request(
                path=(image if ("." in image.split("/")[-1]) else (image + ".png"))
            ),
        )
        return value.status

    def setOLEDAnimation(self, oled, animation):
        """Shows animation on the OLED.

        Parameters:
            oled (str): The name of the sensor as defined in the configuration.
            animation (str): Animation (directory) name/path either an absolute
                                path, a path relative to the folder of the
                                mirte-oled-images repository or and package
                                relative path (pkg://PACKAGE_NAME/REST/OF/PATH).
        """

        value = self._call_service(
            self.oled_services[oled]["file"], SetOLEDFile.Request(path=animation)
        )
        return value.status

    def getDigitalPinValue(self, pin):
        """Gets the input value of a digital pin.

        Parameters:
            pin (str): The pin number of an analog pin as printed on the microcontroller.

        Returns:
            bool: The input value.
        """

        value = self._call_service(
            self.get_pin_value_service,
            GetPinValue.Request(pin=str(pin), type="digital"),
        )
        return value.data

    def setServoAngle(self, servo, angle):
        """Sets the angle of a servo.

        Parameters:
            servo (str): The name of the sensor as defined in the configuration.
            angle (int): The angle of the servo (range [0-360], but some servos
                            might be physically limited to [0-180].

        Returns:
            bool: True if set successfully.

        Warning:
            The servo uses the Servo library from Arduino (through Telemetrix). This also
            means that, when a servo is used and the library is enabled, the last timer on
            the MCU will be used for timing of the servos. This timer therefore can not be
            used for PWM anymore. For Arduino Nano/Uno this means pins D9 and D10 will not
            have PWM anymore. For the SMT32 this means pins A1, A2, A3, A15, B3, B10, and B11
            will not have PWM anymore.

        Warning:
            A maximum of 12 servos is supported.
        """
        value = self._call_service(
            self.servo_services[servo],
            SetServoAngle.Request(
                angle=float(angle), degrees=SetServoAngle.Request.DEGREES
            ),
        )
        return value.status

    def setDigitalPinValue(self, pin, value):
        """Sets the output value of a digital pin.

        Parameters:
            pin (str): The pin number of an analog pin as printed on the microcontroller.
            value (bool): Value to set.
        """

        value = self._call_service(
            self.set_pin_value_service,
            SetPinValue.Request(pin=str(pin), type="digital", value=value),
        )
        return value.status

    def setMotorSpeed(self, motor, value):
        """Sets the speed of the motor.

        Parameters:
            motor (str): The name of the sensor as defined in the configuration.
            value (int): The 'directional duty cycle' (range [-100, 100]) of the PWM
                         signal (-100: full backward, 0: stand still, 100: full forward).

        Returns:
            bool: True if set successfully.
        """

        motor = self._call_service(
            self.motor_services[motor], SetMotorSpeed.Request(speed=int(value))
        )
        return motor.status

    def setMotorControl(self, status):
        """Enables/disables the motor controller. This is enabled on boot, but can
        be disabled/enabled at runtime. This makes the ROS control node pause,
        so it will not respond to Twist messages anymore when disabled.

        Parameters:
            status (bool): To which status the motor controller should be set.

        Returns:
            none
        """
        if status:
            self._call_service(
                self.switch_controller_service,
                SwitchController.Request(
                    activate_controllers=[self.CONTROLLER], activate_asap=True
                ),
            )
        else:
            self._call_service(
                self.switch_controller_service,
                SwitchController.Request(deactivate_controllers=[self.CONTROLLER]),
            )
        return

    def stop(self):
        """Stops all DC motors defined in the configuration

        Note:
            This function is always called when a script exits (either by the user
            or when it finished.

        """
        for motor in self.motors:
            self.setMotorSpeed(motor, 0)

    def _at_exit(self):
        self.stop()
        rclpy.try_shutdown()


# We need a special function to initiate the Robot() because the main.py need to call the
# init_node() (see: https://answers.ros.org/question/266612/rospy-init_node-inside-imported-file/)
def createRobot(
    machine_namespace: Optional[str] = None, hardware_namespace: str = "io"
):
    """Creates and return instance of the robot class.

    Parameters:
            machine_namespace (Optional[str], optional): The Namespace from '/' to the ROS namespace for the specific Mirte. Defaults to "/{HOSTNAME}". (This only has to be changed when running the Robot API from a different machine directly. It is configured correctly for the Web interface)
            hardware_namespace (str, optional): The namespace for the hardware peripherals. Defaults to "io".

    Returns:
       Robot: The initialize Robot class.
    """

    global mirte
    mirte = Robot(machine_namespace, hardware_namespace)
    return mirte
