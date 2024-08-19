#include "sobit_light_library/sobit_light_library.hpp"
#include "sobit_light_library/sobit_light_joint_controller.hpp"
#include "sobit_light_library/sobit_light_wheel_controller.hpp"

#include <pybind11/pybind11.h>

// using namespace sobit_light;
namespace sobit_light {

PYBIND11_MODULE(sobit_light_module, m) {
  pybind11::enum_<Joint>(m, "Joint")
    .value("kArmShoulderRollJoint" , Joint::kArmShoulderRollJoint)
    .value("kArmShoulderPitchJoint", Joint::kArmShoulderPitchJoint)
    .value("kArmElbowPitchJoint"   , Joint::kArmElbowPitchJoint)
    .value("kArmForearmRollJoint"  , Joint::kArmForearmRollJoint)
    .value("kArmWristPitchJoint"   , Joint::kArmWristPitchJoint)
    .value("kArmWristRollJointt"   , Joint::kArmWristRollJoint)
    .value("kHandJoint"            , Joint::kHandJoint)
    .value("kHeadYawJoint"         , Joint::kHeadYawJoint)
    .value("kHeadPitchJoint"       , Joint::kHeadPitchJoint)
    .value("kJointNum"             , Joint::kJointNum)
    .export_values();

  pybind11::class_<sobit_light::JointController, std::shared_ptr<sobit_light::JointController>>(m, "JointController")
    .def(pybind11::init<const std::string&>())
    .def("moveToPose", &JointController::moveToPose, "moveToPose",
        pybind11::arg("pose_name"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveAllJointsDeg", &JointController::moveAllJointsDeg, "moveAllJointsDeg",
    //     pybind11::arg("arm_shoulder_roll"),
    //     pybind11::arg("arm_shoulder_pitch"),
    //     pybind11::arg("arm_elbow_pitch"),
    //     pybind11::arg("arm_forearm_roll"),
    //     pybind11::arg("arm_wrist_pitch"),
    //     pybind11::arg("arm_wrist_roll"),
    //     pybind11::arg("hand"),
    //     pybind11::arg("head_yaw"),
    //     pybind11::arg("head_pitch"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveAllJointsRad", &JointController::moveAllJointsRad, "moveAllJointsRad",
        pybind11::arg("arm_shoulder_roll"),
        pybind11::arg("arm_shoulder_pitch"),
        pybind11::arg("arm_elbow_pitch"),
        pybind11::arg("arm_forearm_roll"),
        pybind11::arg("arm_wrist_pitch"),
        pybind11::arg("arm_wrist_roll"),
        pybind11::arg("hand"),
        pybind11::arg("head_yaw"),
        pybind11::arg("head_pitch"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveJointDeg", &JointController::moveJointDeg, "moveJointDeg",
    //     pybind11::arg("joint_num"),
    //     pybind11::arg("rad"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveJointRad", &JointController::moveJointRad, "moveJointRad",
        pybind11::arg("joint_num"),
        pybind11::arg("rad"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveArmDeg", &JointController::moveArmDeg, "moveArmDeg",
    //     pybind11::arg("arm_shoulder_roll"),
    //     pybind11::arg("arm_shoulder_pitch"),
    //     pybind11::arg("arm_elbow_pitch"),
    //     pybind11::arg("arm_forearm_roll"),
    //     pybind11::arg("arm_wrist_pitch"),
    //     pybind11::arg("arm_wrist_roll"),
    //     pybind11::arg("hand"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveArmRad", &JointController::moveArmRad, "moveArmRad",
        pybind11::arg("arm_shoulder_roll"),
        pybind11::arg("arm_shoulder_pitch"),
        pybind11::arg("arm_elbow_pitch"),
        pybind11::arg("arm_forearm_roll"),
        pybind11::arg("arm_wrist_pitch"),
        pybind11::arg("arm_wrist_roll"),
        pybind11::arg("hand"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadDeg", &JointController::moveHeadDeg, "moveHeadDeg",
    //     pybind11::arg("head_yaw"),
    //     pybind11::arg("head_pitch"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHeadRad", &JointController::moveHeadRad, "moveHeadRad",
        pybind11::arg("head_yaw"),
        pybind11::arg("head_pitch"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToTargetCoord", &JointController::moveHandToTargetCoord, "moveHandToTargetCoord",
        pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
        pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToTargetTF", &JointController::moveHandToTargetTF, "moveHandToTargetTF",
        pybind11::arg("target_name"),
        pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToPlaceCoord", &JointController::moveHandToPlaceCoord, "moveHandToPlaceCoord",
        pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
        pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToPlaceTF", &JointController::moveHandToPlaceTF, "moveHandToPlaceTF",
        pybind11::arg("target_name"),
        pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadToTargetCoord", &JointController::moveHeadToTargetCoord, "moveHeadToTargetCoord",
    //     pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
    //     pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadToTargetTF", &JointController::moveHeadToTargetTF, "moveHeadToTargetTF",
    //     pybind11::arg("target_name"),
    //     pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("graspDecision", &JointController::graspDecision, "graspDecision",
        pybind11::arg("min_curr") = 300,
        pybind11::arg("max_curr") = 1000)
    .def("placeDecision", &JointController::placeDecision, "placeDecision",
        pybind11::arg("min_curr") = 500,
        pybind11::arg("max_curr") = 1000);

//   pybind11::class_<WheelController>(m, "WheelController")
  pybind11::class_<WheelController, std::shared_ptr<WheelController>>(m, "WheelController")
    // .def(pybind11::init<const std::string&>())
    .def(pybind11::init<>())
    .def("controlWheelLinear", &WheelController::controlWheelLinear, "controlWheelLinear",
        pybind11::arg("distance"))
    .def("controlWheelRotateRad", &WheelController::controlWheelRotateRad, "controlWheelRotateRad",
        pybind11::arg("angle_rad"))
    .def("controlWheelRotateDeg", &WheelController::controlWheelRotateDeg, "controlWheelRotateDeg",
        pybind11::arg("angle_deg"))
    .def("rad2Deg", &WheelController::rad2Deg, "rad2Deg",
        pybind11::arg("rad"))
    .def("deg2Rad", &WheelController::deg2Rad, "deg2Rad",
        pybind11::arg("deg"));

}

}  // namespace sobit_light
