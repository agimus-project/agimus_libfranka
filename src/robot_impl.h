// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <memory>
#include <sstream>
#include <type_traits>

#include <franka/model.h>
#include <franka/robot.h>
#include <agimus_research_interface/robot/rbk_types.h>
#include <agimus_research_interface/robot/service_traits.h>
#include <agimus_research_interface/robot/service_types.h>

#include "logger.h"
#include "network.h"
#include "robot_control.h"

namespace franka {

RobotState convertRobotState(const agimus_research_interface::robot::RobotState& robot_state) noexcept;

class Robot::Impl : public RobotControl {
 public:
  explicit Impl(std::unique_ptr<Network> network,
                size_t log_size,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  RobotState update(const agimus_research_interface::robot::MotionGeneratorCommand* motion_command,
                    const agimus_research_interface::robot::ControllerCommand* control_command) override;

  void throwOnMotionError(const RobotState& robot_state, uint32_t motion_id) override;

  virtual RobotState readOnce();

  /**
   * Updates the joint-level based torque commands of an active joint effort control
   *
   * @param control_input the new joint-level based torques
   *
   * @throw ControlException if an error related to torque control or motion generation occurred.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   * @throw std::invalid_argument if joint-level torque commands are NaN or infinity.
   */
  virtual void writeOnce(const Torques& control_input);

  virtual void writeOnce(const JointPositions& motion_generator_input);
  virtual void writeOnce(const JointVelocities& motion_generator_input);
  virtual void writeOnce(const CartesianPose& motion_generator_input);
  virtual void writeOnce(const CartesianVelocities& motion_generator_input);

  virtual void writeOnce(const JointPositions& motion_generator_input,
                         const Torques& control_input);

  virtual void writeOnce(const JointVelocities& motion_generator_input,
                         const Torques& control_input);

  virtual void writeOnce(const CartesianPose& motion_generator_input, const Torques& control_input);

  virtual void writeOnce(const CartesianVelocities& motion_generator_input,
                         const Torques& control_input);

  ServerVersion serverVersion() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  uint32_t startMotion(
      agimus_research_interface::robot::Move::ControllerMode controller_mode,
      agimus_research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const agimus_research_interface::robot::Move::Deviation& maximum_path_deviation,
      const agimus_research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) override;
  void cancelMotion(uint32_t motion_id) override;
  void finishMotion(uint32_t motion_id,
                    const agimus_research_interface::robot::MotionGeneratorCommand* motion_command,
                    const agimus_research_interface::robot::ControllerCommand* control_command) override;

  /**
   * Finishes a running torque-control
   *
   * @param motion_id the id of the running control process
   * @param control_input the final control-input
   */
  void finishMotion(uint32_t motion_id, const Torques& control_input);

  template <typename T, typename... TArgs>
  uint32_t executeCommand(TArgs... /* args */);

  Model loadModel() const;

  agimus_research_interface::robot::ControllerCommand createControllerCommand(
      const Torques& control_input);

  agimus_research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const JointPositions& motion_input);

  agimus_research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const JointVelocities& motion_input);

  agimus_research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const CartesianPose& motion_input);

  agimus_research_interface::robot::MotionGeneratorCommand createMotionCommand(
      const CartesianVelocities& motion_input);

 protected:
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;

 private:
  RobotState current_state_;

  template <typename MotionGeneratorType>
  void writeOnce(const MotionGeneratorType& motion_generator_input);

  template <typename MotionGeneratorType>
  void writeOnce(const MotionGeneratorType& motion_generator_input, const Torques& control_input);

  std::string commandNotPossibleMsg() const {
    std::stringstream ss;
    ss << " command rejected: command not possible in the current mode ("
       << static_cast<franka::RobotMode>(robot_mode_) << ")!";
    if (robot_mode_ == agimus_research_interface::robot::RobotMode::kOther) {
      ss << " Did you open the brakes?";
    }
    return ss.str();
  }

  template <typename T>
  using IsBaseOfGetterSetter =
      std::is_base_of<agimus_research_interface::robot::GetterSetterCommandBase<T, T::kCommand>, T>;

  template <typename T>
  std::enable_if_t<IsBaseOfGetterSetter<T>::value> handleCommandResponse(
      const typename T::Response& response) const {
    using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

    switch (response.status) {
      case T::Status::kSuccess:
        break;
      case T::Status::kCommandNotPossibleRejected:
        throw CommandException("libfranka: "s + agimus_research_interface::robot::CommandTraits<T>::kName +
                               commandNotPossibleMsg());
      case T::Status::kInvalidArgumentRejected:
        throw CommandException("libfranka: "s + agimus_research_interface::robot::CommandTraits<T>::kName +
                               " command rejected: invalid argument!");
      // case T::Status::kCommandRejectedDueToActivatedSafetyFunctions:
      //   throw CommandException("libfranka: "s + agimus_research_interface::robot::CommandTraits<T>::kName +
      //                          " command rejected due to activated safety function! Please disable "
      //                          "all safety functions. ");
      default:
        throw ProtocolException("libfranka: Unexpected response while handling "s +
                                agimus_research_interface::robot::CommandTraits<T>::kName + " command!");
    }
  }

  template <typename T>
  std::enable_if_t<!IsBaseOfGetterSetter<T>::value> handleCommandResponse(
      const typename T::Response& response) const {
    using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

    switch (response.status) {
      case T::Status::kSuccess:
        break;
      case T::Status::kCommandNotPossibleRejected:
        throw CommandException("libfranka: "s + agimus_research_interface::robot::CommandTraits<T>::kName +
                               commandNotPossibleMsg());
      // case T::Status::kCommandRejectedDueToActivatedSafetyFunctions:
      //   throw CommandException("libfranka: "s + agimus_research_interface::robot::CommandTraits<T>::kName +
      //                          " command rejected due to activated safety function! Please disable "
      //                          "all safety functions.");
      default:
        throw ProtocolException("libfranka: Unexpected response while handling "s +
                                agimus_research_interface::robot::CommandTraits<T>::kName + " command!");
    }
  }

  agimus_research_interface::robot::RobotCommand sendRobotCommand(
      const agimus_research_interface::robot::MotionGeneratorCommand* motion_command,
      const agimus_research_interface::robot::ControllerCommand* control_command) const;
  agimus_research_interface::robot::RobotState receiveRobotState();
  void updateState(const agimus_research_interface::robot::RobotState& robot_state);

  std::unique_ptr<Network> network_;

  Logger logger_;

  const RealtimeConfig realtime_config_;  // NOLINT(readability-identifier-naming)
  uint16_t ri_version_;

  agimus_research_interface::robot::RobotMode robot_mode_ = agimus_research_interface::robot::RobotMode::kOther;
  agimus_research_interface::robot::MotionGeneratorMode motion_generator_mode_;
  agimus_research_interface::robot::MotionGeneratorMode current_move_motion_generator_mode_ =
      agimus_research_interface::robot::MotionGeneratorMode::kIdle;
  agimus_research_interface::robot::ControllerMode controller_mode_ =
      agimus_research_interface::robot::ControllerMode::kOther;
  agimus_research_interface::robot::ControllerMode current_move_controller_mode_;
  uint64_t message_id_;
};

template <>
inline void Robot::Impl::handleCommandResponse<agimus_research_interface::robot::Move>(
    const agimus_research_interface::robot::Move::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case agimus_research_interface::robot::Move::Status::kSuccess:
      break;
    case agimus_research_interface::robot::Move::Status::kMotionStarted:
      if (motionGeneratorRunning()) {
        throw ProtocolException(
            "libfranka: "s +
            agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
            " received unexpected motion started message.");
      }
      break;
    case agimus_research_interface::robot::Move::Status::kEmergencyAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command aborted: User Stop pressed!");
    case agimus_research_interface::robot::Move::Status::kReflexAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command aborted: motion aborted by reflex!");
    case agimus_research_interface::robot::Move::Status::kInputErrorAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command aborted: invalid input provided!");
    case agimus_research_interface::robot::Move::Status::kCommandNotPossibleRejected:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          commandNotPossibleMsg());
    case agimus_research_interface::robot::Move::Status::kStartAtSingularPoseRejected:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command rejected: cannot start at singular pose!");
    case agimus_research_interface::robot::Move::Status::kInvalidArgumentRejected:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command rejected: maximum path deviation out of range!");
    case agimus_research_interface::robot::Move::Status::kPreempted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command preempted!");
    case agimus_research_interface::robot::Move::Status::kAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command aborted!");
    // case agimus_research_interface::robot::Move::Status::kPreemptedDueToActivatedSafetyFunctions:
    //  throw CommandException(
    //      "libfranka: "s +
    //      agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
    //      " command preempted due to activated safety function! Please disable all safety "
    //      "functions.");
    // case agimus_research_interface::robot::Move::Status::kCommandRejectedDueToActivatedSafetyFunctions:
    //   throw CommandException(
    //       "libfranka: "s +
    //       agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
    //       " command rejected due to activated safety function! Please disable all safety "
    //       "functions.");
    default:
      throw ProtocolException(
          "libfranka: Unexpected response while handling "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
          " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<agimus_research_interface::robot::StopMove>(
    const agimus_research_interface::robot::StopMove::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case agimus_research_interface::robot::StopMove::Status::kSuccess:
      break;
    case agimus_research_interface::robot::StopMove::Status::kCommandNotPossibleRejected:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::StopMove>::kName +
          commandNotPossibleMsg());
    case agimus_research_interface::robot::StopMove::Status::kAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::StopMove>::kName +
          commandNotPossibleMsg());
    case agimus_research_interface::robot::StopMove::Status::kEmergencyAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::StopMove>::kName +
          " command aborted: User Stop pressed!");
    case agimus_research_interface::robot::StopMove::Status::kReflexAborted:
      throw CommandException(
          "libfranka: "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::StopMove>::kName +
          " command aborted: motion aborted by reflex!");
    // case agimus_research_interface::robot::StopMove::Status::kCommandRejectedDueToActivatedSafetyFunctions:
    //   throw CommandException(
    //       "libfranka: "s +
    //       agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
    //       " command rejected due to activated safety function! Please disable all safety "
    //       "functions.");
    default:
      throw ProtocolException(
          "libfranka: Unexpected response while handling "s +
          agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::StopMove>::kName +
          " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<agimus_research_interface::robot::AutomaticErrorRecovery>(
    const agimus_research_interface::robot::AutomaticErrorRecovery::Response& response) const {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

  switch (response.status) {
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::kSuccess:
      break;
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::kEmergencyAborted:
      throw CommandException("libfranka: "s +
                             agimus_research_interface::robot::CommandTraits<
                                 agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted: User Stop pressed!");
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::kReflexAborted:
      throw CommandException("libfranka: "s +
                             agimus_research_interface::robot::CommandTraits<
                                 agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted: motion aborted by reflex!");
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::kCommandNotPossibleRejected:
      throw CommandException("libfranka: "s +
                             agimus_research_interface::robot::CommandTraits<
                                 agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                             commandNotPossibleMsg());
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::
        kManualErrorRecoveryRequiredRejected:
      throw CommandException("libfranka: "s +
                             agimus_research_interface::robot::CommandTraits<
                                 agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command rejected: manual error recovery required!");
    case agimus_research_interface::robot::AutomaticErrorRecovery::Status::kAborted:
      throw CommandException("libfranka: "s +
                             agimus_research_interface::robot::CommandTraits<
                                 agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                             " command aborted!");
    // case agimus_research_interface::robot::AutomaticErrorRecovery::Status::
    //     kCommandRejectedDueToActivatedSafetyFunctions:
    //   throw CommandException(
    //       "libfranka: "s +
    //       agimus_research_interface::robot::CommandTraits<agimus_research_interface::robot::Move>::kName +
    //       " command rejected due to activated safety function! Please disable all safety "
    //       "functions.");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s +
                              agimus_research_interface::robot::CommandTraits<
                                  agimus_research_interface::robot::AutomaticErrorRecovery>::kName +
                              " command!");
  }
}

template <typename T, typename... TArgs>
uint32_t Robot::Impl::executeCommand(TArgs... args) {
  uint32_t command_id = network_->tcpSendRequest<T>(args...);
  typename T::Response response = network_->tcpBlockingReceiveResponse<T>(command_id);
  handleCommandResponse<T>(response);
  return command_id;
}

}  // namespace franka
