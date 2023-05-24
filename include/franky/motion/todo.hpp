class StopMotion : public WaypointMotion {
public:
  explicit StopMotion() : WaypointMotion{
      {{.reference_type=Waypoint::ReferenceType::Relative, .max_dynamics = true}}} {}
};


class PositionHold : public WaypointMotion {
public:
  explicit PositionHold(double duration) : WaypointMotion{
      {{.reference_type=Waypoint::ReferenceType::Relative, .minimum_time = duration}}} {}
};