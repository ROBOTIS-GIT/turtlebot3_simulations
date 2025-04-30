#ifndef TURTLEBOT3_GAZEBO_OBSTACLE2_PLUGIN_HPP
#define TURTLEBOT3_GAZEBO_OBSTACLE2_PLUGIN_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <chrono>

namespace turtlebot3_gazebo
{

class Obstacle2Plugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  Obstacle1Plugin() = default;
  ~Obstacle1Plugin() override = default;

  void Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override;

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  std::chrono::steady_clock::time_point startTime;
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO_OBSTACLE2_PLUGIN_HPP
