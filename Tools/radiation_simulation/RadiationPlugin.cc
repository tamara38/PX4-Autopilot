#include <random>
#include <string>
#include <cmath>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>

constexpr double GAMMA_CONSTANT_CS137 = 0.098; // µSv·m²/(h·MBq) JEFF-3.1 standard for Cs-137
constexpr double SOURCE_ACTIVITY_MBQ = 2000.0; //(MBq)
constexpr double DETECTOR_SENSITIVITY_CPS_PER_USVH = 1300.0; // sensitivity, "2x2" NaI(Tl), Cs-137

double poisson_apply(double trueDoseRate, double dt){

    if(trueDoseRate < 0.0){
      return 0.0;
    }

    static std::random_device rd;
    static std::mt19937 gen(rd());

    double lambda = trueDoseRate * DETECTOR_SENSITIVITY_CPS_PER_USVH * dt;
    std::poisson_distribution<int> poisson_dist(lambda);
    int noise_impulses = poisson_dist(gen);

    double simulated_cps = (double)noise_impulses / dt;

    double noise_dose = simulated_cps / DETECTOR_SENSITIVITY_CPS_PER_USVH;
    return noise_dose;
}

using namespace gz;
using namespace sim;

class RadiationPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->worldEntity = _entity;

    this->vehicleModelName = _sdf->Get<std::string>("vehicle_model", "x500_0").first;
    this->sourceModelName  = _sdf->Get<std::string>("source_model", "radiation_source").first;
    this->sourceStrength   = _sdf->Get<double>("source_strength", SOURCE_ACTIVITY_MBQ).first;
    this->publishHz        = _sdf->Get<double>("publish_rate", 10.0).first;

    std::string topic = _sdf->Get<std::string>("topic", "/radiation/reading").first;
    this->pub = this->node.Advertise<msgs::Float>(topic);

    gzmsg << "[RadiationPlugin] watching model '" << this->vehicleModelName
          << "', source '" << this->sourceModelName
          << "', publishing on '" << topic << "'\n";
  }

  void PostUpdate(const UpdateInfo &_info,
                   const EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    double periodNs = 1e9 / this->publishHz;
    this->accumNs += std::chrono::duration_cast<std::chrono::nanoseconds>(
                          _info.dt).count();
    if (this->accumNs < periodNs)
      return;
    this->accumNs = 0;

    World world(this->worldEntity);

    if (this->vehicleEntity == kNullEntity)
      this->vehicleEntity = world.ModelByName(_ecm, this->vehicleModelName);
    if (this->sourceEntity == kNullEntity)
      this->sourceEntity = world.ModelByName(_ecm, this->sourceModelName);

    if (this->vehicleEntity == kNullEntity || this->sourceEntity == kNullEntity)
      return;

    math::Pose3d dronePose  = worldPose(this->vehicleEntity, _ecm);
    math::Pose3d sourcePose = worldPose(this->sourceEntity, _ecm);

    double distance = dronePose.Pos().Distance(sourcePose.Pos());
    if (distance < 0.1)
      distance = 0.1;

    // (G * A) / d^2
    double trueDoseRateUSvH = (this->sourceStrength * GAMMA_CONSTANT_CS137) / (distance * distance);

    double dt = 1 / this->publishHz;
    double reading = poisson_apply(trueDoseRateUSvH, dt);

    msgs::Float msg;
    msg.set_data(static_cast<float>(reading));
    this->pub.Publish(msg);
  }

private:
  Entity worldEntity{kNullEntity};
  Entity vehicleEntity{kNullEntity};
  Entity sourceEntity{kNullEntity};

  std::string vehicleModelName;
  std::string sourceModelName;
  double sourceStrength{SOURCE_ACTIVITY_MBQ};
  double publishHz{10.0};

  std::int64_t accumNs{0};

  transport::Node node;
  transport::Node::Publisher pub;
  std::mt19937 rng{std::random_device{}()};
};

GZ_ADD_PLUGIN(RadiationPlugin,
              System,
              RadiationPlugin::ISystemConfigure,
              RadiationPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(RadiationPlugin, "radiation_system::RadiationPlugin")
