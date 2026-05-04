#include "mapping.hpp"

// Point implementations
Point::Point(double x_, double y_) : x(x_), y(y_) {}

double Point::distance_to(const Point& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Line implementations
double Line::length() const {
    return start.distance_to(end);
}

double Line::distance_to(const Point& p) const {
    return std::abs(a * p.x + b * p.y + c) / std::sqrt(a * a + b * b);
}

double Line::angle() const {
    return std::atan2(end.y - start.y, end.x - start.x);
}

// SplitAndMerge implementations
SplitAndMerge::SplitAndMerge(double st, double ma, 
                             double md, int min_p)
    : split_threshold(st),
      merge_angle(ma * M_PI / 180.0),
      merge_distance(md),
      min_points(min_p) {}

void SplitAndMerge::fitLine(const std::vector<Point>& points, Line& line) {
    if (points.size() < 2) return;
    
    line.points = points;
    
    // Calculate centroid
    double mx = 0, my = 0;
    for (const auto& p : points) {
        mx += p.x;
        my += p.y;
    }
    mx /= points.size();
    my /= points.size();
    
    // Calculate covariance
    double xx = 0, yy = 0, xy = 0;
    for (const auto& p : points) {
        double dx = p.x - mx;
        double dy = p.y - my;
        xx += dx * dx;
        yy += dy * dy;
        xy += dx * dy;
    }
    
    // Principal-axis direction of the best-fit line (total least squares).
    const double theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
    const double dir_x = std::cos(theta);
    const double dir_y = std::sin(theta);

    // Convert direction to line normal ax + by + c = 0.
    line.a = -dir_y;
    line.b = dir_x;
    
    line.c = -(line.a * mx + line.b * my);
    
    // Find endpoints by projecting points onto the line direction around centroid.
    double tmin = 1e9, tmax = -1e9;
    
    for (const auto& p : points) {
        double t = (p.x - mx) * dir_x + (p.y - my) * dir_y;
        tmin = std::min(tmin, t);
        tmax = std::max(tmax, t);
    }
    
    line.start.x = mx + tmin * dir_x;
    line.start.y = my + tmin * dir_y;
    line.end.x = mx + tmax * dir_x;
    line.end.y = my + tmax * dir_y;
}

int SplitAndMerge::findFarthestPoint(const std::vector<Point>& points, const Line& line) {
    double maxDist = -1;
    int maxIdx = -1;
    
    for (size_t i = 0; i < points.size(); i++) {
        double dist = line.distance_to(points[i]);
        if (dist > maxDist) {
            maxDist = dist;
            maxIdx = i;
        }
    }
    
    return maxIdx;
}

void SplitAndMerge::split(std::vector<Point>& points, std::vector<Line>& segments) {
    if (points.size() < static_cast<size_t>(min_points)) {
        // Too few points, skip
        return;
    }
    
    // Fit line to these points
    Line line;
    fitLine(points, line);
    
    // Find farthest point
    int splitIdx = findFarthestPoint(points, line);
    double maxError = line.distance_to(points[splitIdx]);
    
    // Check if we need to split
    if (maxError > split_threshold && splitIdx > 0 && splitIdx < static_cast<int>(points.size()) - 1) {
        // Split at the farthest point
        std::vector<Point> left(points.begin(), points.begin() + splitIdx + 1);
        std::vector<Point> right(points.begin() + splitIdx, points.end());
        
        // Recursively split both halves
        split(left, segments);
        split(right, segments);
    } else {
        // Good enough - keep as a segment
        segments.push_back(line);
    }
}

bool SplitAndMerge::canMerge(const Line& a, const Line& b) {
    // Check angle difference
    double angleDiff = std::fmod(std::abs(a.angle() - b.angle()), 2.0 * M_PI);
    if (angleDiff > M_PI) {
        angleDiff = 2.0 * M_PI - angleDiff;
    }
    // Line directions are undirected (theta == theta + pi).
    angleDiff = std::min(angleDiff, M_PI - angleDiff);
    
    if (angleDiff > merge_angle)
        return false;
    
    // Check smallest endpoint gap, independent of segment direction.
    double gap = std::min(
        std::min(a.start.distance_to(b.start), a.start.distance_to(b.end)),
        std::min(a.end.distance_to(b.start), a.end.distance_to(b.end))
    );
    
    if (gap > merge_distance)
        return false;
    
    return true;
}

void SplitAndMerge::merge(std::vector<Line>& segments) {
    if (segments.size() < 2) return;
    
    std::vector<Line> merged;
    Line current = segments[0];
    
    for (size_t i = 1; i < segments.size(); i++) {
        if (canMerge(current, segments[i])) {
            // Merge points
            std::vector<Point> allPoints = current.points;
            allPoints.insert(allPoints.end(), 
                           segments[i].points.begin(), 
                           segments[i].points.end());
            
            // Refit line
            fitLine(allPoints, current);
        } else {
            merged.push_back(current);
            current = segments[i];
        }
    }
    merged.push_back(current);
    segments = merged;
}

std::vector<Line> SplitAndMerge::extractWalls(std::vector<Point>& scan) {
    if (scan.size() < static_cast<size_t>(min_points)) return {};
    
    // Step 1: Split
    std::vector<Line> segments;
    split(scan, segments);
    
    // Step 2: Merge
    merge(segments);
    
    // Filter out short segments
    std::vector<Line> walls;
    for (auto& seg : segments) {
        if (seg.length() > 150.0 && seg.points.size() >= static_cast<size_t>(min_points)) {
            walls.push_back(seg);
        }
    }
    return walls;
}
