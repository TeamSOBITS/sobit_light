#include "sobit_light_library/sobit_light_wheel_controller.hpp"

#include "sobit_light_library_py/pybind11.hpp"


PYBIND11_MODULE(_wheel, m) {
  m.doc() = "Python wrapper for the wheel controller in the sobit_light_library";

  pybind11::class_<sobit_light::WheelController, std::shared_ptr<sobit_light::WheelController>>(m, "WheelController")
//   pybind11::class_<sobit_light::WheelController>(m, "WheelController")
    .def(pybind11::init<const std::string&>())
    .def("controlWheelLinear", &sobit_light::WheelController::controlWheelLinear, "controlWheelLinear",
        pybind11::arg("distance"))
    .def("controlWheelRotateRad", &sobit_light::WheelController::controlWheelRotateRad, "controlWheelRotateRad",
        pybind11::arg("angle_rad"))
    .def("controlWheelRotateDeg", &sobit_light::WheelController::controlWheelRotateDeg, "controlWheelRotateDeg",
        pybind11::arg("angle_deg"))
    .def("rad2Deg", &sobit_light::WheelController::rad2Deg, "rad2Deg",
        pybind11::arg("rad"))
    .def("deg2Rad", &sobit_light::WheelController::deg2Rad, "deg2Rad",
        pybind11::arg("deg"))
    ;
}
