#include "sobit_light_library/sobit_light_library.h"
#include "sobit_light_library/sobit_light_joint_controller.hpp"
#include "sobit_light_library/sobit_light_wheel_controller.hpp"
#include <pybind11/pybind11.h>

using namespace sobit_light;


PYBIND11_MODULE(sobit_light_module, m) {
    pybind11::enum_<Joint>(m, "Joint")
        .value("kArmShoulderRollJoint", Joint::kArmShoulderRollJoint)
        .value("kArmShoulderPitchJoint", Joint::kArmShoulderPitchJoint)
        .value("kArmElbowPitchJoint", Joint::kArmElbowPitchJoint)
        .value("kArmForearmRollJoint", Joint::kArmForearmRollJoint)
        .value("kArmWristPitchJoint", Joint::kArmWristPitchJoint)
        .value("kArmWristRollJointt", Joint::kArmWristRollJoint)
        .value("kHandJoint", Joint::kHandJoint)
        .value("kHeadYawJoint", Joint::kHeadYawJoint)
        .value("kHeadPitchJoint", Joint::kHeadPitchJoint)
        .value("kJointNum", Joint::kJointNum)
        .export_values();

    pybind11::class_<JointController>(m, "JointController")
        .def(pybind11::init<const std::string&>())
        .def("moveToPose", &JointController::moveToPose, "move Pose",
             pybind11::arg("pose_name"),
             pybind11::arg("sec") = 5.0)
        .def("moveAllJointsRad", &JointController::moveAllJointsRad, "move all Joint",
             pybind11::arg("arm_shoulder_pan"),
             pybind11::arg("arm_shoulder_tilt"),
             pybind11::arg("arm_elbow_tilt"),
             pybind11::arg("arm_wrist_tilt"),
             pybind11::arg("hand"),
             pybind11::arg("head_camera_pan"),
             pybind11::arg("head_camera_tilt"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveAllJointsDeg", &JointController::moveAllJointsDeg, "move all Joint",
             pybind11::arg("arm_shoulder_pan"),
             pybind11::arg("arm_shoulder_tilt"),
             pybind11::arg("arm_elbow_tilt"),
             pybind11::arg("arm_wrist_tilt"),
             pybind11::arg("hand"),
             pybind11::arg("head_camera_pan"),
             pybind11::arg("head_camera_tilt"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveJointRad", &JointController::moveJointRad, "moveJoint",
             pybind11::arg("joint_num"),
             pybind11::arg("rad"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveJointDeg", &JointController::moveJointDeg, "moveJoint",
             pybind11::arg("joint_num"),
             pybind11::arg("rad"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHeadRad", &JointController::moveHeadRad, "move Head PanTilt",
             pybind11::arg("pan_rad"),
             pybind11::arg("tilt_rad"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHeadDeg", &JointController::moveHeadDeg, "move Head PanTilt",
             pybind11::arg("pan_rad"),
             pybind11::arg("tilt_rad"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveArmRad", &JointController::moveArmRad, "move Arm",
             pybind11::arg("arm_shoulder_pan"),
             pybind11::arg("arm_shoulder_tilt"),
             pybind11::arg("arm_elbow_tilt"),
             pybind11::arg("arm_wrist_tilt"),
             pybind11::arg("hand"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveArmDeg", &JointController::moveArmDeg, "move Arm",
             pybind11::arg("arm_shoulder_pan"),
             pybind11::arg("arm_shoulder_tilt"),
             pybind11::arg("arm_elbow_tilt"),
             pybind11::arg("arm_wrist_tilt"),
             pybind11::arg("hand"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHandToTargetCoord", &JointController::moveHandToTargetCoord, "moveHandToTargetCoord",
             pybind11::arg("target_pos_x"), pybind11::arg("target_pos_y"), pybind11::arg("target_pos_z"),
             pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHandToTargetTF", &JointController::moveHandToTargetTF, "moveHandToTargetTF",
             pybind11::arg("target_name"),
             pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHandToPlaceCoord", &JointController::moveHandToPlaceCoord, "move Hand To Placeable Position Coordinate",
             pybind11::arg("target_pos_x"), pybind11::arg("target_pos_y"), pybind11::arg("target_pos_z"),
             pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("moveHandToPlaceTF", &JointController::moveHandToPlaceTF, "move Hand To Placeable Position TF",
             pybind11::arg("target_name"),
             pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
             pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
        .def("graspDecision", &JointController::graspDecision, "grasp Decision",
             pybind11::arg("min_curr") = 300,
             pybind11::arg("max_curr") = 1000)
        .def("placeDecision", &JointController::placeDecision, "place Decision",
             pybind11::arg("min_curr") = 500,
             pybind11::arg("max_curr") = 1000);

    pybind11::class_<WheelController>(m, "WheelController")
        .def(pybind11::init<const std::string&>())
        .def("controlWheelLinear", &WheelController::controlWheelLinear, "control Wheel Linear",
             pybind11::arg("distance"))
        .def("controlWheelRotateRad", &WheelController::controlWheelRotateRad, "control Wheel Rotate Rad",
             pybind11::arg("angle_rad"))
        .def("controlWheelRotateDeg", &WheelController::controlWheelRotateDeg, "control Wheel Rotate Deg",
             pybind11::arg("angle_deg"))
        .def("rad2Deg", &WheelController::rad2Deg, "rad 2 Deg",
             pybind11::arg("rad"))
        .def("deg2Rad", &WheelController::deg2Rad, "deg 2 Rad",
             pybind11::arg("deg"));
}
