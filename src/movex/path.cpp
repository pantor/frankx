#include <movex/path/path.hpp>


namespace movex {

size_t Path::get_index(double s) const {
    auto ptr = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), s);
    size_t index = std::distance(cumulative_lengths.begin(), ptr);
    return std::min(index, segments.size() - 1);
}

std::tuple<std::shared_ptr<Segment>, double> Path::get_local(double s) const {
    size_t index = get_index(s);
    auto segment = segments.at(index);
    double s_local = (index == 0) ? s : s - cumulative_lengths[index - 1];
    return {segment, s_local};
}

void Path::init_path_points(const std::vector<Waypoint>& waypoints) {
    if (waypoints.size() < 2) {
        throw std::runtime_error("Path needs at least 2 waypoints as input, but has only " + std::to_string(waypoints.size()) + ".");
    }

    std::vector<std::shared_ptr<LineSegment>> line_segments;

    double elbow_current = waypoints[0].elbow.value_or(0.0);
    Affine affine_current = waypoints[0].affine;

    Vector7d vector_current = waypoints[0].getTargetVector(affine_current, elbow_current);
    Vector7d vector_next;

    for (size_t i = 1; i < waypoints.size(); i += 1) {
        vector_next = waypoints[i].getTargetVector(affine_current, elbow_current);
        affine_current = Affine(vector_next);
        elbow_current = vector_next(6);

        auto segment = std::make_shared<LineSegment>(vector_current, vector_next);
        line_segments.emplace_back(segment);
        std::swap(vector_current, vector_next);
    }

    double cumulative_length {0.0};
    for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
        if (waypoints[i].blend_max_distance > 0.0) {
            auto& left = line_segments[i - 1];
            auto& right = line_segments[i];

            Vector7d lm = (left->end - left->start) / left->get_length();
            Vector7d rm = (right->end - right->start) / right->get_length();

            double s_abs_max = std::min<double>({ left->get_length() / 2, right->get_length() / 2 });

            auto blend = std::make_shared<QuarticBlendSegment>(left->start, lm, right->start, rm, left->get_length(), waypoints[i].blend_max_distance, s_abs_max);
            double s_abs = blend->get_length() / 2;

            auto new_left = std::make_shared<LineSegment>(left->start, left->q(left->get_length() - s_abs));
            auto new_right = std::make_shared<LineSegment>(right->q(s_abs), right->end);

            cumulative_length += new_left->get_length();
            segments.emplace_back(new_left);
            cumulative_lengths.emplace_back(cumulative_length);

            cumulative_length += blend->get_length();
            segments.emplace_back(blend);
            cumulative_lengths.emplace_back(cumulative_length);

            right = new_right;

        } else {
            cumulative_length += line_segments[i - 1]->get_length();
            segments.emplace_back(line_segments[i - 1]);
            cumulative_lengths.emplace_back(cumulative_length);
        }
    }

    cumulative_length += line_segments.back()->get_length();
    segments.emplace_back(line_segments.back());
    cumulative_lengths.emplace_back(cumulative_length);
    length = cumulative_length;
}

Path::Path(const std::vector<Waypoint>& waypoints) {
    init_path_points(waypoints);
}

Path::Path(const std::vector<Affine>& waypoints, double blend_max_distance) {
    std::vector<Waypoint> converted(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i += 1) {
        converted[i] = Waypoint(waypoints[i], std::nullopt, blend_max_distance);
    }
    init_path_points(converted);
}

double Path::get_length() const {
    return length;
}

Vector7d Path::q(double s) const {
    auto [segment, s_local] = get_local(s);
    return segment->q(s_local);
}

Vector7d Path::q(double s, const Affine& frame) const {
    Vector7d init {q(s)};
    return (Affine(init) * frame.inverse()).vector_with_elbow(init(6));
}

Vector7d Path::pdq(double s) const {
    auto [segment, s_local] = get_local(s);
    return segment->pdq(s_local);
}

Vector7d Path::pddq(double s) const {
    auto [segment, s_local] = get_local(s);
    return segment->pddq(s_local);
}

Vector7d Path::pdddq(double s) const {
    auto [segment, s_local] = get_local(s);
    return segment->pddq(s_local);
}

Vector7d Path::dq(double s, double ds) const {
    auto [segment, s_local] = get_local(s);
    return segment->dq(s_local, ds);
}

Vector7d Path::ddq(double s, double ds, double dds) const {
    auto [segment, s_local] = get_local(s);
    return segment->ddq(s_local, ds, dds);
}

Vector7d Path::dddq(double s, double ds, double dds, double ddds) const {
    auto [segment, s_local] = get_local(s);
    return segment->dddq(s_local, ds, dds, ddds);
}

Vector7d Path::max_pddq() const {
    Vector7d result;
    for (size_t i = 0; i < 7; i += 1) {
        auto max_ptr = *std::max_element(segments.begin(), segments.end(), [&](std::shared_ptr<Segment> l, std::shared_ptr<Segment> r){
            return std::abs(l->max_pddq()(i)) < std::abs(r->max_pddq()(i));
        });
        result(i) = std::abs(max_ptr->max_pddq()(i));
    }
    return result;
}

Vector7d Path::max_pdddq() const {
    Vector7d result;
    for (size_t i = 0; i < 7; i += 1) {
        auto max_ptr = *std::max_element(segments.begin(), segments.end(), [&](std::shared_ptr<Segment> l, std::shared_ptr<Segment> r){
            return std::abs(l->max_pdddq()(i)) < std::abs(r->max_pdddq()(i));
        });
        result(i) = std::abs(max_ptr->max_pdddq()(i));
    }
    return result;
}

} // namespace movex
