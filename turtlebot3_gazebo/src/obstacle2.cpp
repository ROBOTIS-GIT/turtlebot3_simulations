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

class Obstacle2Plugin
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
    double cycle = 150.0;
    double t = std::fmod(elapsed.count(), cycle);

    gz::math::Vector3d start, end;
    double localT = 0.0;
    double duration = 1.0;

    if (t <= 10.0)
    {
      start = gz::math::Vector3d(-2.0, -2.0, 0.25);
      end   = gz::math::Vector3d(-1.3, -1.8, 0.25);
      duration = 10.0;
      localT = t - 0.0;
    }
    else if (t <= 40.0)
    {
      start = gz::math::Vector3d(-1.3, -1.8, 0.25);
      end   = gz::math::Vector3d(0.5, 2.0, 0.25);
      duration = 30.0;
      localT = t - 10.0;
    }
    else if (t <= 55.0)
    {
      start = gz::math::Vector3d(0.5, 2.0, 0.25);
      end   = gz::math::Vector3d(-2.0, 1.5, 0.25);
      duration = 15.0;
      localT = t - 40.0;
    }
    else if (t <= 85.0)
    {
      start = gz::math::Vector3d(-2.0, 1.5, 0.25);
      end   = gz::math::Vector3d(1.5, -0.2, 0.25);
      duration = 30.0;
      localT = t - 55.0;
    }
    else if (t <= 100.0)
    {
      start = gz::math::Vector3d(1.5, -0.2, 0.25);
      end   = gz::math::Vector3d(1.5, -2.0, 0.25);
      duration = 15.0;
      localT = t - 85.0;
    }
    else if (t <= 110.0)
    {
      start = gz::math::Vector3d(1.5, -2.0, 0.25);
      end   = gz::math::Vector3d(0.0, -1.5, 0.25);
      duration = 10.0;
      localT = t - 100.0;
    }
    else if (t <= 115.0)
    {
      start = gz::math::Vector3d(0.0, -1.5, 0.25);
      end   = gz::math::Vector3d(-0.5, -1.0, 0.25);
      duration = 5.0;
      localT = t - 110.0;
    }
    else if (t <= 120.0)
    {
      start = gz::math::Vector3d(-0.5, -1.0, 0.25);
      end   = gz::math::Vector3d(-1.0, -1.5, 0.25);
      duration = 5.0;
      localT = t - 115.0;
    }
    else if (t <= 125.0)
    {
      start = gz::math::Vector3d(-1.0, -1.5, 0.25);
      end   = gz::math::Vector3d(-1.5, -1.9, 0.25);
      duration = 5.0;
      localT = t - 120.0;
    }
    else if (t <= 130.0)
    {
      start = gz::math::Vector3d(-1.5, -1.9, 0.25);
      end   = gz::math::Vector3d(-2.0, -2.0, 0.25);
      duration = 5.0;
      localT = t - 125.0;
    }
    else
    {
      start = gz::math::Vector3d(-2.0, -2.0, 0.25);
      end   = start;
      localT = 0.0;
    }

    double alpha = std::min(localT / duration, 1.0);
    gz::math::Vector3d currentPos = start + (end - start) * alpha;
    gz::math::Pose3d pose(currentPos, gz::math::Quaterniond::Identity);

    this->model.SetWorldPoseCmd(ecm, pose);
  }

private:
  Model model{kNullEntity};
  std::chrono::steady_clock::time_point startTime;
};

}  // namespace turtlebot3_gazebo

GZ_ADD_PLUGIN(
  turtlebot3_gazebo::Obstacle2Plugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  turtlebot3_gazebo::Obstacle2Plugin,
  "turtlebot3_gazebo::Obstacle2Plugin")
