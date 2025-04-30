#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <chrono>

using namespace gz;
using namespace sim;

namespace turtlebot3_gazebo
{

class ObstaclesPlugin
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(
    const Entity &entity,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &,
    EventManager &) override
  {
    this->model = Model(entity);
    this->startTime = std::chrono::steady_clock::now();
  }

  void PreUpdate(
    const UpdateInfo &info,
    EntityComponentManager &ecm) override
  {
    if (!this->model.Valid(ecm))
      return;

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - this->startTime;
    double t = fmod(elapsed.count(), 40.0);
    double angle = 2 * M_PI * t / 40.0;

    gz::math::Pose3d pose(
      gz::math::Vector3d(0, 0, 0),
      gz::math::Quaterniond(0, 0, angle));

    this->model.SetWorldPoseCmd(ecm, pose);
  }

private:
  Model model{kNullEntity};
  std::chrono::steady_clock::time_point startTime;
};

}  // namespace turtlebot3_gazebo

GZ_ADD_PLUGIN(
  turtlebot3_gazebo::ObstaclesPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  turtlebot3_gazebo::ObstaclesPlugin,
  "turtlebot3_gazebo::ObstaclesPlugin")
