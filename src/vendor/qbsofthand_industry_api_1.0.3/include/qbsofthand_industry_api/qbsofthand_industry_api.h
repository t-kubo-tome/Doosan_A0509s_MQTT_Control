/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2019, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QBSOFTHAND_INDUSTRY_API_H
#define QBSOFTHAND_INDUSTRY_API_H

// standard libraries
#include <memory>
#include <string>

namespace qbsofthand_industry_api {
class qbSoftHandIndustryAPI {
 public:
  /**
   * Initialize the qb SoftHand Industry API handler by opening the proper UDP socket and by testing its communication.
   * The constructor try to connect to the qb SoftHand Industry for at most 30 seconds and then returns.
   * Through the method 'isInitialized()' is possible to test whether the initialization succeeded or not.
   * The default IPv4 address used for the UDP socket connection is 192.168.1.110.
   */
  qbSoftHandIndustryAPI();

  /**
   * Initialize the qb SoftHand Industry API handler by opening the proper UDP socket and by testing its communication.
   * The constructor try to connect to the qb SoftHand Industry for at most 30 seconds and then returns.
   * Through the method 'isInitialized()' is possible to test whether the initialization succeeded or not.
   * \param device_ip The IPv4 address used for the UDP socket connection. It must be a valid IPv4 address.
   */
  explicit qbSoftHandIndustryAPI(const std::string &device_ip, const int &max_timeout);

  /**
   * Close the UDP socket and perform the proper shutdown procedures.
   */
  virtual ~qbSoftHandIndustryAPI();

  /**
   * Get the actual qb SoftHand Industry motor torque in percent value w.r.t. the maximum value.
   */
  float getCurrent();

  /**
   * Get the actual qb SoftHand Industry motor position in percent value w.r.t. the maximum value.
   */
  float getPosition();

  /**
   * Get the actual qb SoftHand Industry motor velocity in percent value w.r.t. the maximum value.
   */
  float getVelocity();

  /**
   * Get the qb SoftHand Industry device information.
   */
  std::string getStatistics();

  /**
   * Return true if the initialization procedure has succeeded.
   * \return \p true on success.
   */
  bool isInitialized();

  /**
   * Send the given percent-position closure command reference to the qb SoftHand Industry.
   * \param position The percent-position command reference, in range [\p 0, \p 100]% where \p 0 is the fully open
   * configuration, and \p 100 is the fully closed.
   * \return \p 0 on success; \p -1 if the position is out of range; \p -3 if communication is lost.
   */
  int setClosure(const float &position);

  /**
   * Send the given percent-position closure command reference to the qb SoftHand Industry, together with the speed
   * reference command and the maximum force that should be applied during the grasp.
   * This is the most complete command to send a reference to the qb SoftHand and should be used for special cases. In
   * normal usage, the simpler version above should be preferred.
   * All the parameters are expressed in percent w.r.t. the maximum possible value.
   * \param position The percent-position command reference, in range [\p 0, \p 100]% where \p 0 is the fully open
   * configuration, and \p 100 is the fully closed.
   * \param velocity The speed command reference, in range [\p 12.5, \p 100]% where \p 12.5 is the minimum velocity of
   * the closure, and \p 100 is full speed.
   * \param current The current threshold for the motor, in range [\p 62.5, \p 100]% where \p 62.5 is the minimum force
   * that the hand can apply, and \p 100 is the maximum.
   * \return \p 0 on success; \p -1 if at least one of the percent values is out of range; \p -3 if communication is lost.
   */
  int setClosure(const float &position, const float &velocity, const float &current);

  /**
   * Change the qb SoftHand Industry IPv4 address, network mask and gateway of the device.
   * It is worth noticing that the device should be connected first to use this command.
   * \param net_ip The device network IP address, e.g. 192.168.1.110.
   * \param net_mask The device network mask, e.g. 255.255.255.0.
   * \param net_gateway The device network gateway, e.g. 192.168.1.1.
   * \return \p 0 on success; \p -1 if at least one of the given IPv4-format addresses is wrong.
   */
  int setIP(const std::string &net_ip, const std::string &net_mask, const std::string &net_gateway);

  /**
   * Wait until the qb SoftHand Industry has reached the commanded position.
   * This method should be called after a \p setClosure one, to wait for the action to be completed.
   * \return \p 0 on success; \p -3 if communication is lost.
   */
  int waitForTargetReached();

 private:
  class Implementation;
  std::unique_ptr<Implementation> pimpl_;
};
}  // namespace qbsofthand_industry_api

#endif // QBSOFTHAND_INDUSTRY_API_H