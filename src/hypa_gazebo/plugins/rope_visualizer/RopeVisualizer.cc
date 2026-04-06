

#include <gz/msgs/marker.pb.h>

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

using namespace gz;
using namespace sim;

namespace hypa_gazebo
{

/// \brief Private implementation class
class RopeVisualizerPrivate
{
  /// \brief Gazebo transport node
  public:
  transport::Node node;

  /// \brief Cached entity ID for start link
  public:
  Entity startLinkEntity{kNullEntity};

  /// \brief Cached entity ID for end link
  public:
  Entity endLinkEntity{kNullEntity};

  /// \brief Name of the model this plugin is attached to
  public:
  std::string modelName;

  /// \brief Name of the start link
  public:
  std::string startLinkName{"start_link"};

  /// \brief Name of the end link
  public:
  std::string endLinkName{"end_link"};

  /// \brief ID of the start point marker
  public:
  const int kStartPointId = 1;

  /// \brief ID of the end point marker
  public:
  const int kEndPointId = 2;

  /// \brief ID of the line marker
  public:
  const int kLineId = 3;

  /// \brief Position of the start point
  public:
  math::Vector3d startPoint = math::Vector3d::Zero;

  /// \brief Position of the end point
  public:
  math::Vector3d endPoint = math::Vector3d::Zero;

  /// \brief Color of the markers
  public:
  math::Color markerColor{1.0f, 0.0f, 0.0f, 1.0f};

  /// \brief Set of currently placed markers
  public:
  std::unordered_set<int> placedMarkers;

  /// \brief Namespace for marker placement
  public:
  std::string ns{"rope_visualizer"};

  /// \brief Width of the line
  public:
  double lineWidth{0.02};

  /// \brief Frame counter for throttling updates
  public:
  int frameCounter{0};

  /// \brief Update every N frames (configurable)
  public:
  int updateInterval{5};

  /// \brief True if initialized
  public:
  bool initialized{false};

  /// \brief Whether to show endpoints
  public:
  bool showEndpoints{true};
};

/////////////////////////////////////////////////
/// \brief Rope visualizer class
class RopeVisualizer : public System,
                       public ISystemConfigure,
                       public ISystemPreUpdate
{
  public:
  /// \brief Constructor
  RopeVisualizer() : dataPtr(std::make_unique<RopeVisualizerPrivate>())
  {
  }

  /// \brief Destructor
  ~RopeVisualizer() override
  {
    this->Reset();
  }

  /// \brief Configure the plugin
  /// \param[in] _entity The entity this plugin is attached to
  /// \param[in] _sdf The SDF element of this plugin
  /// \param[in] _ecm The entity component manager
  /// \param[in] _eventMgr The event manager
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager & /*_eventMgr*/) override
  {
    // Verify this is a model
    auto modelComp = _ecm.Component<components::Model>(_entity);
    if (!modelComp)
    {
      gzerr << "RopeVisualizer plugin must be attached to a model\n";
      return;
    }
    this->dataPtr->modelName =
        _ecm.Component<components::Name>(_entity)->Data();

    // Load SDF parameters
    if (_sdf->HasElement("start_link"))
    {
      this->dataPtr->startLinkName = _sdf->Get<std::string>("start_link");
    }

    if (_sdf->HasElement("end_link"))
    {
      this->dataPtr->endLinkName = _sdf->Get<std::string>("end_link");
    }

    if (_sdf->HasElement("line_width"))
    {
      this->dataPtr->lineWidth = _sdf->Get<double>("line_width");
    }

    if (_sdf->HasElement("color"))
    {
      auto colorElem = _sdf->FindElement("color");
      if (colorElem)
      {
        double r = colorElem->Get<double>("r", 0.2).first;
        double g = colorElem->Get<double>("g", 0.8).first;
        double b = colorElem->Get<double>("b", 0.2).first;
        double a = colorElem->Get<double>("a", 1.0).first;
        this->dataPtr->markerColor.Set(r, g, b, a);
      }
    }

    if (_sdf->HasElement("show_endpoints"))
    {
      this->dataPtr->showEndpoints = _sdf->Get<bool>("show_endpoints");
    }

    if (_sdf->HasElement("update_interval"))
    {
      this->dataPtr->updateInterval = _sdf->Get<int>("update_interval");
      if (this->dataPtr->updateInterval < 1)
        this->dataPtr->updateInterval = 1;
    }

    this->dataPtr->initialized = true;

    this->dataPtr->ns = "rope_" + this->dataPtr->startLinkName + "_" +
                        this->dataPtr->endLinkName;

    // Cache entity IDs for faster lookup in PreUpdate
    this->CacheEntities(_ecm);

    gzmsg << "RopeVisualizer configured for model: " << this->dataPtr->modelName
          << ", start_link: " << this->dataPtr->startLinkName
          << ", end_link: " << this->dataPtr->endLinkName << "\n";
  }

  /// \brief Pre-update callback - dynamically update visual line segment
  /// \param[in] _info Update info
  /// \param[in] _ecm Entity component manager
  void PreUpdate(const UpdateInfo & /*_info*/,
                 EntityComponentManager &_ecm) override
  {
    if (!this->dataPtr->initialized)
    {
      gzwarn << "RopeVisualizer: Not initialized\n";
      return;
    }

    // Throttle updates - only update every N frames
    if (++this->dataPtr->frameCounter % this->dataPtr->updateInterval != 0)
    {
      return;
    }

    // Use cached entity IDs instead of searching every update
    if (this->dataPtr->startLinkEntity == kNullEntity ||
        this->dataPtr->endLinkEntity == kNullEntity)
    {
      static bool warned = false;
      if (!warned)
      {
        gzwarn << "RopeVisualizer: Cached entities not valid, attempting to "
                  "re-cache\n";
        warned = true;
      }
      return;
    }

    // Get current poses using cached entities
    auto startPoseComp =
        _ecm.Component<components::Pose>(this->dataPtr->startLinkEntity);
    auto endPoseComp =
        _ecm.Component<components::Pose>(this->dataPtr->endLinkEntity);

    if (!startPoseComp || !endPoseComp)
    {
      return;
    }

    math::Vector3d newStart =
        gz::sim::worldPose(this->dataPtr->startLinkEntity, _ecm).Pos();
    math::Vector3d newEnd =
        gz::sim::worldPose(this->dataPtr->endLinkEntity, _ecm).Pos();

    // Check if positions have changed
    // const double threshold = 1e-6;//
    // 四根绳子时太卡了，由于仿真时高频微小振动，导致四个绳子都在频繁更新marker
    const double threshold = 1e-3;
    bool changed =
        (newStart - this->dataPtr->startPoint).Length() > threshold ||
        (newEnd - this->dataPtr->endPoint).Length() > threshold;

    if (changed)
    {
      this->dataPtr->startPoint = newStart;
      this->dataPtr->endPoint = newEnd;
      // gzdbg << "RopeVisualizer: Drawing line from (" << newStart.X() << ", "
      //       << newStart.Y() << ", " << newStart.Z() << ") to (" << newEnd.X()
      //       << ", " << newEnd.Y() << ", " << newEnd.Z() << ")\n";
      this->UpdateVisualization();
    }
  }

  private:
  /// \brief Setup common marker message fields
  /// \param[in] _markerMsg The marker message to setup
  /// \param[in] _id The id of the marker
  /// \param[in] _color The color of the marker
  void SetupMarkerCommon(gz::msgs::Marker &_markerMsg, int _id,
                         const gz::math::Color &_color)
  {
    _markerMsg.set_ns(this->dataPtr->ns);
    _markerMsg.set_id(_id);
    _markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
    gz::msgs::Set(_markerMsg.mutable_material()->mutable_ambient(), _color);
    gz::msgs::Set(_markerMsg.mutable_material()->mutable_diffuse(), _color);
  }

  /// \brief Cache entity IDs for start and end links
  /// \param[in] _ecm Entity component manager
  void CacheEntities(EntityComponentManager &_ecm)
  {
    // Find and cache start link entity
    std::vector<Entity> startLinks = _ecm.EntitiesByComponents(
        components::Name(this->dataPtr->startLinkName), components::Link());
    if (!startLinks.empty())
    {
      this->dataPtr->startLinkEntity = startLinks[0];
    }
    else
    {
      gzerr << "RopeVisualizer: Start link '" << this->dataPtr->startLinkName
            << "' not found in model '" << this->dataPtr->modelName << "'\n";
    }

    // Find and cache end link entity
    std::vector<Entity> endLinks = _ecm.EntitiesByComponents(
        components::Name(this->dataPtr->endLinkName), components::Link());
    if (!endLinks.empty())
    {
      this->dataPtr->endLinkEntity = endLinks[0];
    }
    else
    {
      gzerr << "RopeVisualizer: End link '" << this->dataPtr->endLinkName
            << "' not found in model '" << this->dataPtr->modelName << "'\n";
    }
  }

  /// \brief Update the visualization
  void UpdateVisualization()
  {
    // Draw endpoint markers (if enabled)
    if (this->dataPtr->showEndpoints)
    {
      this->DrawPoint(this->dataPtr->kStartPointId, this->dataPtr->startPoint,
                      this->dataPtr->markerColor);
      this->DrawPoint(this->dataPtr->kEndPointId, this->dataPtr->endPoint,
                      this->dataPtr->markerColor);
    }

    // Draw connecting line
    this->DrawLine(this->dataPtr->kLineId, this->dataPtr->startPoint,
                   this->dataPtr->endPoint, this->dataPtr->markerColor);
  }

  /// \brief Draw a point marker
  /// \param[in] _id The id of the marker
  /// \param[in] _point The position of the point
  /// \param[in] _color The color of the marker
  void DrawPoint(int _id, const gz::math::Vector3d &_point,
                 const gz::math::Color &_color)
  {
    gz::msgs::Marker markerMsg;
    this->SetupMarkerCommon(markerMsg, _id, _color);
    markerMsg.set_type(gz::msgs::Marker::SPHERE);
    gz::msgs::Set(markerMsg.mutable_scale(), gz::math::Vector3d(0.1, 0.1, 0.1));
    gz::msgs::Set(
        markerMsg.mutable_pose(),
        gz::math::Pose3d(_point.X(), _point.Y(), _point.Z(), 0, 0, 0));

    this->dataPtr->node.Request("/marker", markerMsg);
    this->dataPtr->placedMarkers.insert(_id);
  }

  /// \brief Draw a line between two points as a cylinder (rope)
  /// \param[in] _id The id of the marker
  /// \param[in] _startPoint The start point
  /// \param[in] _endPoint The end point
  /// \param[in] _color The color
  void DrawLine(int _id, const gz::math::Vector3d &_startPoint,
                const gz::math::Vector3d &_endPoint,
                const gz::math::Color &_color)
  {
    gz::msgs::Marker markerMsg;
    this->SetupMarkerCommon(markerMsg, _id, _color);
    markerMsg.set_type(gz::msgs::Marker::CYLINDER);

    // Calculate midpoint position
    gz::math::Vector3d midPoint = (_startPoint + _endPoint) / 2.0;

    // Calculate cylinder height (distance between points)
    double height = (_endPoint - _startPoint).Length();

    // Calculate direction (rotation from start to end)
    gz::math::Vector3d direction = _endPoint - _startPoint;
    direction.Normalize();

    // Default cylinder direction is along Z axis, need to calculate rotation
    // Use quaternion to represent rotation from Z axis to target direction
    gz::math::Vector3d zAxis(0, 0, 1);
    gz::math::Quaterniond rotation;
    if (direction == zAxis)
    {
      rotation = gz::math::Quaterniond::Identity;
    }
    else if (direction == -zAxis)
    {
      rotation = gz::math::Quaterniond(M_PI, 0, 0);
    }
    else
    {
      gz::math::Vector3d axis = zAxis.Cross(direction);
      axis.Normalize();
      double angle = acos(zAxis.Dot(direction));
      rotation = gz::math::Quaterniond(axis, angle);
    }

    // Set scale: cylinder radius is line_width/2, height is distance between
    // points
    gz::msgs::Set(markerMsg.mutable_scale(),
                  gz::math::Vector3d(this->dataPtr->lineWidth,
                                     this->dataPtr->lineWidth, height));

    // Set position and pose
    gz::math::Pose3d pose(midPoint, rotation);
    gz::msgs::Set(markerMsg.mutable_pose(), pose);

    this->dataPtr->node.Request("/marker", markerMsg);
    this->dataPtr->placedMarkers.insert(_id);
  }

  /// \brief Delete a marker
  /// \param[in] _id The id of the marker to delete
  void DeleteMarker(int _id)
  {
    if (this->dataPtr->placedMarkers.find(_id) ==
        this->dataPtr->placedMarkers.end())
    {
      return;
    }

    gz::msgs::Marker markerMsg;
    markerMsg.set_ns(this->dataPtr->ns);
    markerMsg.set_id(_id);
    markerMsg.set_action(gz::msgs::Marker::DELETE_MARKER);
    this->dataPtr->node.Request("/marker", markerMsg);
    this->dataPtr->placedMarkers.erase(_id);
  }

  /// \brief Reset and delete all markers
  void Reset()
  {
    this->DeleteMarker(this->dataPtr->kStartPointId);
    this->DeleteMarker(this->dataPtr->kEndPointId);
    this->DeleteMarker(this->dataPtr->kLineId);
  }

  private:
  /// \brief Private data pointer
  std::unique_ptr<RopeVisualizerPrivate> dataPtr;
};
}  // namespace hypa_gazebo

/**
 * Except for the first parameter, the other parameters are optional, telling
 * the engine the plugin implementation (interface hooks) that can be loaded and
 * called. For example, gz::sim::System is the base class interface, registering
 * it tells the engine that this plugin is a simulation system, and can load and
 * call its required methods (such as basic system functions). Additional
 * interfaces (such as ISystemConfigure) further specify specific lifecycle
 * hooks (such as configuration and update), and the engine decides when to call
 * based on that.
 */
GZ_ADD_PLUGIN(hypa_gazebo::RopeVisualizer, gz::sim::System,
              hypa_gazebo::RopeVisualizer::ISystemConfigure,
              hypa_gazebo::RopeVisualizer::ISystemPreUpdate)
/**
 * Can add other aliases
 *  GZ_ADD_PLUGIN_ALIAS(hypa_gazebo::RopeVisualizer,
 *  "hypa_gazebo::AliasNameRopeVisualizer")
 */