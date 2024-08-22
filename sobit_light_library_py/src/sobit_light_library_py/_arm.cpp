#include "sobit_light_library/sobit_light_joint_controller.hpp"

#include "sobit_light_library_py/pybind11.hpp"


PYBIND11_MODULE(_arm, m) {
  m.doc() = "Python wrapper for the arm controller in the sobit_light_library";

  pybind11::enum_<sobit_light::Joint>(m, "Joint")
    .value("kArmShoulderRollJoint" , sobit_light::Joint::kArmShoulderRollJoint)
    .value("kArmShoulderPitchJoint", sobit_light::Joint::kArmShoulderPitchJoint)
    .value("kArmElbowPitchJoint"   , sobit_light::Joint::kArmElbowPitchJoint)
    .value("kArmForearmRollJoint"  , sobit_light::Joint::kArmForearmRollJoint)
    .value("kArmWristPitchJoint"   , sobit_light::Joint::kArmWristPitchJoint)
    .value("kArmWristRollJointt"   , sobit_light::Joint::kArmWristRollJoint)
    .value("kHandJoint"            , sobit_light::Joint::kHandJoint)
    .value("kHeadYawJoint"         , sobit_light::Joint::kHeadYawJoint)
    .value("kHeadPitchJoint"       , sobit_light::Joint::kHeadPitchJoint)
    .value("kJointNum"             , sobit_light::Joint::kJointNum)
    .export_values()
    ;

  pybind11::class_<sobit_light::JointController, std::shared_ptr<sobit_light::JointController>>(m, "ArmController")
    .def(pybind11::init<const std::string&>())
    .def("moveToPose", &sobit_light::JointController::moveToPose, "moveToPose",
        pybind11::arg("pose_name"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveAllJointsDeg", &sobit_light::JointController::moveAllJointsDeg, "moveAllJointsDeg",
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
    .def("moveAllJointsRad", &sobit_light::JointController::moveAllJointsRad, "moveAllJointsRad",
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
    // .def("moveJointDeg", &sobit_light::JointController::moveJointDeg, "moveJointDeg",
    //     pybind11::arg("joint_num"),
    //     pybind11::arg("rad"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveJointRad", &sobit_light::JointController::moveJointRad, "moveJointRad",
        pybind11::arg("joint_num"),
        pybind11::arg("rad"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveArmDeg", &sobit_light::JointController::moveArmDeg, "moveArmDeg",
    //     pybind11::arg("arm_shoulder_roll"),
    //     pybind11::arg("arm_shoulder_pitch"),
    //     pybind11::arg("arm_elbow_pitch"),
    //     pybind11::arg("arm_forearm_roll"),
    //     pybind11::arg("arm_wrist_pitch"),
    //     pybind11::arg("arm_wrist_roll"),
    //     pybind11::arg("hand"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveArmRad", &sobit_light::JointController::moveArmRad, "moveArmRad",
        pybind11::arg("arm_shoulder_roll"),
        pybind11::arg("arm_shoulder_pitch"),
        pybind11::arg("arm_elbow_pitch"),
        pybind11::arg("arm_forearm_roll"),
        pybind11::arg("arm_wrist_pitch"),
        pybind11::arg("arm_wrist_roll"),
        pybind11::arg("hand"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadDeg", &sobit_light::JointController::moveHeadDeg, "moveHeadDeg",
    //     pybind11::arg("head_yaw"),
    //     pybind11::arg("head_pitch"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHeadRad", &sobit_light::JointController::moveHeadRad, "moveHeadRad",
        pybind11::arg("head_yaw"),
        pybind11::arg("head_pitch"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToTargetCoord", &sobit_light::JointController::moveHandToTargetCoord, "moveHandToTargetCoord",
        pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
        pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToTargetTF", &sobit_light::JointController::moveHandToTargetTF, "moveHandToTargetTF",
        pybind11::arg("target_name"),
        pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToPlaceCoord", &sobit_light::JointController::moveHandToPlaceCoord, "moveHandToPlaceCoord",
        pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
        pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("moveHandToPlaceTF", &sobit_light::JointController::moveHandToPlaceTF, "moveHandToPlaceTF",
        pybind11::arg("target_name"),
        pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
        pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadToTargetCoord", &sobit_light::JointController::moveHeadToTargetCoord, "moveHeadToTargetCoord",
    //     pybind11::arg("target_x"), pybind11::arg("target_y"), pybind11::arg("target_z"),
    //     pybind11::arg("shift_x") , pybind11::arg("shift_y") , pybind11::arg("shift_z"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    // .def("moveHeadToTargetTF", &sobit_light::JointController::moveHeadToTargetTF, "moveHeadToTargetTF",
    //     pybind11::arg("target_name"),
    //     pybind11::arg("shift_x"), pybind11::arg("shift_y"), pybind11::arg("shift_z"),
    //     pybind11::arg("sec") = 5.0, pybind11::arg("is_sleep") = true)
    .def("graspDecision", &sobit_light::JointController::graspDecision, "graspDecision",
        pybind11::arg("min_curr") = 300,
        pybind11::arg("max_curr") = 1000)
    .def("placeDecision", &sobit_light::JointController::placeDecision, "placeDecision",
        pybind11::arg("min_curr") = 500,
        pybind11::arg("max_curr") = 1000)
    ;
}
