#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../vendor/qbsofthand_industry_api_1.0.3/include/qbsofthand_industry_api/qbsofthand_industry_api.h"

namespace py = pybind11;
using namespace qbsofthand_industry_api;

PYBIND11_MODULE(qbsofthand_industry_api_pybind, m) {
    m.doc() = "Python bindings for qbSoftHand Industry API";

    py::class_<qbSoftHandIndustryAPI>(m, "qbSoftHandIndustryAPI")
        .def(py::init<>())
        .def(py::init<const std::string &, const int &>(),
             py::arg("device_ip"),
             py::arg("max_timeout"))
        .def("getCurrent", &qbSoftHandIndustryAPI::getCurrent,
             "Get the actual qb SoftHand Industry motor torque in percent value w.r.t. the maximum value.")
        .def("getPosition", &qbSoftHandIndustryAPI::getPosition,
             "Get the actual qb SoftHand Industry motor position in percent value w.r.t. the maximum value.")
        .def("getVelocity", &qbSoftHandIndustryAPI::getVelocity,
             "Get the actual qb SoftHand Industry motor velocity in percent value w.r.t. the maximum value.")
        .def("getStatistics", &qbSoftHandIndustryAPI::getStatistics,
             "Get the qb SoftHand Industry device information.")
        .def("isInitialized", &qbSoftHandIndustryAPI::isInitialized,
             "Return true if the initialization procedure has succeeded.")
        .def("setClosure",
             py::overload_cast<const float &>(&qbSoftHandIndustryAPI::setClosure),
             py::arg("position"),
             "Send the given percent-position closure command reference to the qb SoftHand Industry.\n"
             "Args:\n"
             "    position: The percent-position command reference, in range [0, 100]%\n"
             "Returns:\n"
             "    0 on success; -1 if the position is out of range; -3 if communication is lost.")
        .def("setClosure",
             py::overload_cast<const float &, const float &, const float &>(&qbSoftHandIndustryAPI::setClosure),
             py::arg("position"),
             py::arg("velocity"),
             py::arg("current"),
             "Send the given percent-position closure command reference to the qb SoftHand Industry,\n"
             "together with the speed reference command and the maximum force.\n"
             "Args:\n"
             "    position: The percent-position command reference, in range [0, 100]%\n"
             "    velocity: The speed command reference, in range [12.5, 100]%\n"
             "    current: The current threshold for the motor, in range [62.5, 100]%\n"
             "Returns:\n"
             "    0 on success; -1 if at least one of the percent values is out of range; -3 if communication is lost.")
        .def("setIP", &qbSoftHandIndustryAPI::setIP,
             py::arg("net_ip"),
             py::arg("net_mask"),
             py::arg("net_gateway"),
             "Change the qb SoftHand Industry IPv4 address, network mask and gateway of the device.\n"
             "Args:\n"
             "    net_ip: The device network IP address, e.g. 192.168.1.110\n"
             "    net_mask: The device network mask, e.g. 255.255.255.0\n"
             "    net_gateway: The device network gateway, e.g. 192.168.1.1\n"
             "Returns:\n"
             "    0 on success; -1 if at least one of the given IPv4-format addresses is wrong.")
        .def("waitForTargetReached", &qbSoftHandIndustryAPI::waitForTargetReached,
             "Wait until the qb SoftHand Industry has reached the commanded position.\n"
             "This method should be called after a setClosure one, to wait for the action to be completed.\n"
             "Returns:\n"
             "    0 on success; -3 if communication is lost.");
}
