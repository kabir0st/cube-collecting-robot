#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

struct Point {
    double x, y;

    Point(double x_ = 0, double y_ = 0);
    double distance_to(const Point& other) const;
};

struct Line {
    Point start, end;
    // line: a*x + b*y + c = 0
    double a, b, c;
    // points belonging to this line
    std::vector<Point> points;

    double length() const;
    double distance_to(const Point& p) const;
    double angle() const;
};

class SplitAndMerge {
private:
    double split_threshold;   // Max error before splitting (mm)
    double merge_angle;       // Max angle difference to merge (radians)
    double merge_distance;    // Max gap between segments to merge (mm)
    int min_points;           // Minimum points per segment
    
    // Simple line fitting using least squares
    void fitLine(const std::vector<Point>& points, Line& line);
    
    // Find index of point farthest from line
    int findFarthestPoint(const std::vector<Point>& points, const Line& line);
    
    // Recursive splitting
    void split(std::vector<Point>& points, std::vector<Line>& segments);
    
    // Check if two segments can be merged
    bool canMerge(const Line& a, const Line& b);
    
    // Merge segments that belong together
    void merge(std::vector<Line>& segments);
    
public:
    SplitAndMerge(double st = 15.0,   // 15 mm split threshold
                  double ma = 2.0,  // 2 degrees
                  double md = 50.0,     // 50 mm gap
                  int min_p = 4);             // minimum points per wall
    
    // Main function: extract walls from LiDAR scan
    std::vector<Line> extractWalls(std::vector<Point>& scan);
};
