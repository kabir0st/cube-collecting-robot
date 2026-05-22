
#include <numeric>
#include <array>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <random>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <chrono>
#include <vector>

#include <calibrations.hpp>
#include <loggers.hpp>
#include <utils.hpp>
#include <types.hpp>
#include <global.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>


// σ = sqrt( Σᵢ Σⱼ (dᵢⱼ - μᵢ)² / Σᵢ (nᵢ - 1) )
// Wel-ford's one-pass form keeps the mean numerically stable even
// when distances are large and tightly clustered.
double get_lidar_sensor_noise(LidarData& lidar_data) {
    double sum_sq_dev = 0.0;
    long   total_dof  = 0;
    std::map<double, std::vector<double>> grouped = get_angle_mapped_lidar_data(lidar_data);
    for (const auto& kv: grouped) {
        const std::vector<double>& samples = kv.second;
        if (samples.size() < 2) continue;
        double mean = 0.0, m2 = 0.0;
        std::size_t n = 0;
        for (double d : samples) {
            ++n;
            const double delta = d - mean;
            mean += delta / static_cast<double>(n);
            m2   += delta * (d - mean);
        }
        sum_sq_dev += m2;
        total_dof  += static_cast<long>(n) - 1;
    }
    return (total_dof > 0)
        ? std::sqrt(sum_sq_dev / static_cast<double>(total_dof))
        : 0.0;
}

CalibrateCamera::CalibrateCamera(const CameraConfig& cfg) : config(cfg) {}

cv::Scalar CalibrateCamera::hsvToRgb(float h, float s, float v) {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
    cv::Mat rgb;
    cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
    return cv::Scalar(rgb.at<cv::Vec3b>(0,0)[0], 
                     rgb.at<cv::Vec3b>(0,0)[1], 
                     rgb.at<cv::Vec3b>(0,0)[2]);
}


bool CalibrateCamera::create_filter_mask(FilterResult& result){
    float h_min = 180.f, s_min = 255.f, v_min = 255.f;
    float h_max = 0.f, s_max = 0.f, v_max = 0.f;
    std::vector<float> h_vals;
    h_vals.reserve(d_index_pixel_count);
    for (int i = 0; i < labels.rows; i++) {
        if (labels.at<int>(i) == dominant_idx) {
            float h = pixels.at<float>(i, 0);
            float s = pixels.at<float>(i, 1);
            float v = pixels.at<float>(i, 2);
            h_vals.push_back(h);
            h_min = std::min(h_min, h);
            s_min = std::min(s_min, s);
            v_min = std::min(v_min, v);
            h_max = std::max(h_max, h);
            s_max = std::max(s_max, s);
            v_max = std::max(v_max, v);
        }
    }

    const float sv_floor_ratio = 0.35f;
    const float min_non_black_sv = 10.f;
    const float min_hue_confidence = 40.f;
    const float red_edge_window = 25.f;

    const float dominant_s = centers.at<float>(dominant_idx, 1);
    const float dominant_v = centers.at<float>(dominant_idx, 2);

    auto compute_lower_sv = [&](float min_channel, float dominant_channel) -> float {
        const float raw_lower = std::max(0.f, min_channel);
        if (dominant_channel <= min_non_black_sv) {
            return raw_lower;
        }
        const float dynamic_floor = std::max(min_non_black_sv, dominant_channel * sv_floor_ratio);
        return std::max(raw_lower, dynamic_floor);
    };

    const float s_lower = compute_lower_sv(s_min, dominant_s);
    const float v_lower = compute_lower_sv(v_min, dominant_v);
    const float s_upper = std::min(255.f, s_max);
    const float v_upper = std::min(255.f, v_max);

    const bool hue_is_reliable =
        dominant_s >= min_hue_confidence && dominant_v >= min_hue_confidence;
    const bool spans_red_edges =
        h_min <= red_edge_window && h_max >= (180.f - red_edge_window);

    const float hue_margin = 10.f;
    const float s_margin = 20.f;
    const float v_margin = 40.f;  

    if ((h_max - h_min) > 90.f && hue_is_reliable && spans_red_edges){
        sort(h_vals.begin(), h_vals.end());
        float max_gap = 0.f;
        int gap_idx = 1;
        for (int i = 1; i < (int)h_vals.size(); i++) {
            float gap = h_vals[i] - h_vals[i - 1];
            if (gap > max_gap) {
                max_gap = gap;
                gap_idx = i;
            }
        }

        float h_low_max = h_vals[gap_idx - 1];
        float h_high_min = h_vals[gap_idx];


        // High side: ~170 → 180
        result.lower1 = cv::Scalar (std::max(0.f, h_high_min - hue_margin), std::max(0.f, s_lower - s_margin), std::max(0.f, v_lower - v_margin));
        result.upper1 = cv::Scalar (180.f, std::min(255.f, s_upper + s_margin), std::min(255.f, v_upper + v_margin));

        // Low side: 0 → ~8
        result.lower2 = cv::Scalar (0.f, std::max(0.f, s_lower - s_margin), std::max(0.f, v_lower - v_margin));
        result.upper2 = cv::Scalar (std::min(180.f, h_low_max + hue_margin), std::min(255.f, s_upper + s_margin), std::min(255.f, v_upper + v_margin));
        result.has_wrap_around = true;
    }
    else
    {
        result.lower1 = cv::Scalar (std::max(0.f, h_min - hue_margin), std::max(0.f, s_lower - s_margin), std::max(0.f, v_lower - v_margin));
        result.upper1 = cv::Scalar (std::min(180.f, h_max + hue_margin), std::min(255.f, s_upper + s_margin), std::min(255.f, v_upper + v_margin));
        result.has_wrap_around = false;
    }
    return true;
}

cv::Scalar CalibrateCamera::get_dominant_color(const cv::Mat& frame, const cv::Rect& roi) {
    cv::Mat roi_frame = frame(roi);
    cvtColor(roi_frame, hsv_frame, cv::COLOR_BGR2HSV);

    pixels = hsv_frame.reshape(1, hsv_frame.rows * hsv_frame.cols);
    pixels.convertTo(pixels, CV_32F);
    // Mat labels, centers; -> moved definition to class
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0);
    kmeans(pixels, k_neighbors, labels, criteria, 10, cv::KMEANS_RANDOM_CENTERS, centers);

    std::vector<int> counts(k_neighbors, 0);
    for (int i = 0; i < labels.rows; i++) {
        counts[labels.at<int>(i)]++;
    }

    dominant_idx = max_element(counts.begin(), counts.end()) - counts.begin();
    d_index_pixel_count = counts[dominant_idx];
    return cv::Scalar(
        centers.at<float>(dominant_idx, 0),
        centers.at<float>(dominant_idx, 1),
        centers.at<float>(dominant_idx, 2));
}
FilterResult CalibrateCamera::start_filter_calibration() {
    FilterResult result;
    cv::VideoCapture cap(0);
    // VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened())
    {
        throw std::runtime_error("Cannot Open camera.");
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_FPS, config.fps_target);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    int w = config.width / 2;
    int h = config.height / 2;
    int x = (config.width - w) / 2;
    int y = (config.height - h);
    const cv::Rect roi = cv::Rect(x, y, w, h);
    cv::Mat frame;
    // Initialize: on the 'q' / empty-frame paths we still write
    // `result.dominant_color = dominant_color`, and reading an uninitialized
    // cv::Scalar back propagates garbage through the FilterResult.
    cv::Scalar dominant_color(0, 0, 0);
    LOG("Cal") << "Press 'c' to capture, 'q' to quit.";
    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        rectangle(frame,
                    roi,
                    cv::Scalar(0, 255, 0), 2);
        imshow("Calibration Setup", frame);
        char key = cv::waitKey(1);
        if (key == 'q') {
            LOG("Cal") << "Quitting without selecting color";
            break;
        }
        else if (key == 'c') {
            dominant_color = get_dominant_color(frame, roi);
            LOG("Cal") << "Color captured. Selected HSV: ("
                       << (int)dominant_color[0] << ", "
                       << (int)dominant_color[1] << ", "
                       << (int)dominant_color[2] << ")";
            result.ready = create_filter_mask(result);
            break;
        }
    }
    cv::destroyAllWindows();
    cap.release();
    result.dominant_color = dominant_color;
    return result;
}

// helper function to load config file
namespace {
void rtrim(std::string& s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
}
void ltrim(std::string& s) {
    size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    s.erase(0, i);
}
// Strip a trailing `# comment` from a line
void strip_comment(std::string& s) {
    bool in_quote = false;
    for (size_t i = 0; i < s.size(); ++i) {
        const char c = s[i];
        if (c == '"') in_quote = !in_quote;
        else if (c == '#' && !in_quote) { s.erase(i); return; }
    }
}
// Strip surrounding double-quotes if present. `"hello"` -> `hello`.
void unquote(std::string& s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
        s = s.substr(1, s.size() - 2);
    }
}
}  // namespace
bool Config::has(const std::string& key) const {
    return _kv.find(key) != _kv.end();
}

std::string Config::get_str(const std::string& key, const std::string& def) const {
    auto it = _kv.find(key);
    return it == _kv.end() ? def : it->second;
}

int Config::get_int(const std::string& key, int def) const {
    auto it = _kv.find(key);
    return it == _kv.end() ? def : std::atoi(it->second.c_str());
}

double Config::get_dbl(const std::string& key, double def) const {
    auto it = _kv.find(key);
    return it == _kv.end() ? def : std::atof(it->second.c_str());
}

bool Config::get_bool(const std::string& key, bool def) const {
    auto it = _kv.find(key);
    if (it == _kv.end()) return def;
    const std::string& v = it->second;
    if (v == "true"  || v == "True"  || v == "yes" || v == "Yes" ||
        v == "on"    || v == "On"    || v == "1") return true;
    if (v == "false" || v == "False" || v == "no"  || v == "No"  ||
        v == "off"   || v == "Off"   || v == "0") return false;
    return def;
}


bool parse_scalar3(const std::string& s, cv::Scalar& out) {
    double a, b, c;
    if (std::sscanf(s.c_str(), "%lf,%lf,%lf", &a, &b, &c) != 3) return false;
    out = cv::Scalar(a, b, c);
    return true;
}

std::string fmt_scalar3(const cv::Scalar& s) {
    std::ostringstream os;
    os << s[0] << ',' << s[1] << ',' << s[2];
    return os.str();
}

bool load_filter_result(const Config& cfg, FilterResult& out) {
    if (!cfg.has("filter.ready") ||
        !cfg.has("filter.has_wrap_around") ||
        !cfg.has("filter.dominant_color") ||
        !cfg.has("filter.lower1") ||
        !cfg.has("filter.upper1")) {
        return false;
    }
    if (!parse_scalar3(cfg.get_str("filter.dominant_color"), out.dominant_color)) return false;
    if (!parse_scalar3(cfg.get_str("filter.lower1"),         out.lower1))         return false;
    if (!parse_scalar3(cfg.get_str("filter.upper1"),         out.upper1))         return false;
    out.has_wrap_around = cfg.get_bool("filter.has_wrap_around", false);
    if (out.has_wrap_around) {
        if (!cfg.has("filter.lower2") || !cfg.has("filter.upper2")) return false;
        if (!parse_scalar3(cfg.get_str("filter.lower2"), out.lower2)) return false;
        if (!parse_scalar3(cfg.get_str("filter.upper2"), out.upper2)) return false;
    }
    out.ready = cfg.get_bool("filter.ready", false);
    return out.ready;
}

bool save_filter_result(const std::string& path, const FilterResult& result) {
    // Read existing file (if any) so we can preserve comments and other
    // sections. Missing file is fine — we'll create it fresh.
    std::vector<std::string> lines;
    {
        std::ifstream in(path);
        std::string line;
        while (std::getline(in, line)) lines.push_back(line);
    }

    // Drop any prior top-level `filter:` block. A block starts at a line whose
    // first non-space char sits at column 0 and matches `filter:` (with
    // optional trailing comment). It ends at the next non-blank top-level
    // line (indent == 0) or EOF. Blank/indented lines in between belong to
    // the block.
    std::vector<std::string> kept;
    kept.reserve(lines.size());
    bool in_filter_block = false;
    for (const std::string& raw : lines) {
        std::string stripped = raw;
        strip_comment(stripped);
        rtrim(stripped);
        const bool blank = stripped.empty();
        size_t indent = 0;
        while (indent < raw.size() && raw[indent] == ' ') ++indent;
        if (in_filter_block) {
            if (!blank && indent == 0) {
                in_filter_block = false;
                kept.push_back(raw);
            }
            // else: still inside the filter block — drop it.
        } else {
            if (!blank && indent == 0 && stripped.rfind("filter:", 0) == 0) {
                in_filter_block = true;
                // drop the `filter:` header itself
            } else {
                kept.push_back(raw);
            }
        }
    }

    // Trim trailing blank lines so the appended block sits flush.
    while (!kept.empty() && kept.back().find_first_not_of(" \t") == std::string::npos) {
        kept.pop_back();
    }

    // Atomic write: write to a sibling temp file, then rename() it over the
    // original. POSIX rename(2) is atomic within a filesystem, so a crash
    // before the rename leaves the old config untouched, and a crash after
    // leaves the new config fully in place. No torn writes.
    const std::string tmp_path = path + ".tmp";
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out) return false;
    for (const std::string& l : kept) out << l << '\n';
    if (!kept.empty()) out << '\n';
    out << "filter:\n";
    out << "  ready: " << (result.ready ? "true" : "false") << '\n';
    out << "  has_wrap_around: " << (result.has_wrap_around ? "true" : "false") << '\n';
    out << "  dominant_color: \"" << fmt_scalar3(result.dominant_color) << "\"\n";
    out << "  lower1: \"" << fmt_scalar3(result.lower1) << "\"\n";
    out << "  upper1: \"" << fmt_scalar3(result.upper1) << "\"\n";
    out << "  lower2: \"" << fmt_scalar3(result.lower2) << "\"\n";
    out << "  upper2: \"" << fmt_scalar3(result.upper2) << "\"\n";
    if (!out.good()) return false;
    out.close();
    // rename(2): atomic on the same filesystem. After this call the new
    // config is fully in place; before it, the old one was untouched.
    if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
        std::remove(tmp_path.c_str());
        return false;
    }
    return true;
}

bool load_config(const std::string& path, Config& out) {
    std::ifstream f(path);
    if (!f) return false;

    // We track section nesting by indent. `prefix_stack[i]` is the section
    // name at indent `indent_stack[i]`. When a new line's indent is <= an
    // entry on the stack, that entry (and any deeper ones) is popped.
    std::vector<std::string> prefix_stack;
    std::vector<int>         indent_stack;

    std::string line;
    while (std::getline(f, line)) {
        strip_comment(line);
        // Count leading spaces — indent for this line.
        size_t i = 0;
        while (i < line.size() && line[i] == ' ') ++i;
        std::string content = line.substr(i);
        rtrim(content);
        if (content.empty()) continue;
        const int indent = static_cast<int>(i);
        // Pop sections at this indent or deeper before we add this line.
        while (!indent_stack.empty() && indent_stack.back() >= indent) {
            indent_stack.pop_back();
            prefix_stack.pop_back();
        }
        const auto col = content.find(':');
        if (col == std::string::npos) continue;  // malformed; skip
        std::string key   = content.substr(0, col);
        std::string value = content.substr(col + 1);
        rtrim(key);
        ltrim(value);
        rtrim(value);
        unquote(value);
        if (value.empty()) {
            // Section header — push its local name; full dotted keys for
            // children are built by joining prefix_stack.
            prefix_stack.push_back(key);
            indent_stack.push_back(indent);
        } else {
            std::string full_key;
            for (const auto& p : prefix_stack) { full_key += p; full_key += '.'; }
            full_key += key;
            out.set(full_key, value);
        }
    }
    return true;
}



// Inliers of a candidate line (n, c) within `thresh` perpendicular distance.
std::vector<int> get_inliers(const std::vector<Point>& pts,
                            double nx, double ny, double c,
                            double thresh) {
    std::vector<int> out;
    for (size_t k = 0; k < pts.size(); ++k) {
        if (std::fabs(nx * pts[k].x + ny * pts[k].y - c) <= thresh)
            out.push_back(k);
    }
    return out;
}


LineFit refit_tls(const std::vector<Point>& pts,
                const std::vector<int>& idx) {
    const size_t n = idx.size();
    double sx = 0.0, sy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sx += pts[idx[i]].x;
        sy += pts[idx[i]].y;
    }
    const double cx = sx / static_cast<double>(n);
    const double cy = sy / static_cast<double>(n);
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = pts[idx[i]].x - cx;
        const double dy = pts[idx[i]].y - cy;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }
    Eigen::Matrix2d C;
    C << sxx, sxy,
        sxy, syy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
    // SelfAdjointEigenSolver returns eigenvalues in ascending order:
    //   col(0) = smaller -> normal
    //   col(1) = larger  -> direction
    LineFit f;
    f.cx = cx; f.cy = cy;
    f.nx = es.eigenvectors()(0, 0);
    f.ny = es.eigenvectors()(1, 0);
    f.dx = es.eigenvectors()(0, 1);
    f.dy = es.eigenvectors()(1, 1);
    return f;
}

// Replace pts with everything that is NOT in inliers.
void erase_inliers(std::vector<Point>& pts,
                const std::vector<int>& inliers) {
    std::vector<bool> mark(pts.size(), false);
    for (int i : inliers) mark[i] = true;
    std::vector<Point> next;
    next.reserve(pts.size() - inliers.size());
    for (size_t k = 0; k < pts.size(); ++k)
        if (!mark[k]) next.push_back(pts[k]);
    pts.swap(next);
}

// using RANSAC to filter out outliers
// and Total-Least-Square model to find the best fitting line
// Method: form the 2x2 sample covariance about the centroid; the
// eigenvector of the smaller eigenvalue is the line normal, the
// eigenvector of the larger eigenvalue is the line direction.
std::vector<LineSeg> find_walls(LidarData& lidar_data,
                                const RansacParams& params_in) {
    std::vector<LineSeg> walls;
    std::vector<Point> points;
    RansacParams params = params_in;

    std::map<double, std::vector<double>> pts_map = get_angle_mapped_lidar_data(
        lidar_data);
    // converting to Cartesian coordinates
    for (auto& pts: pts_map) {
        if (pts.second.empty()) continue;

        const double avg = std::accumulate(
                               pts.second.begin(), pts.second.end(), 0.0) /
                           static_cast<double>(pts.second.size());
        // Closer than ~8 cm is typically the LIDAR seeing its own chassis/mount.
        if (avg < 80) continue;
        // LiDAR CW → math CCW + LIDAR_MOUNT_OFFSET_RAD. Centralized in
        // utils.hpp::lidar_polar_to_body so map-building (here) and the
        // per-scan transform in cox.cpp can never drift out of sync.
        double bx, by;
        lidar_polar_to_body(pts.first, avg, bx, by);
        points.emplace_back(bx, by);
    }
    LOG("MAP") << "Usable Points for MAP : " << points.size();
    
    // floor threshold that scales with sensor noise;
    params.inlier_thresh_mm = std::max(15.0, lidar_data.sensor_noise.load() + 10);
    std::mt19937 rng(params.seed != 0 ? params.seed : std::random_device{}());

    // starting RANSAC algorithm
    const size_t inline_floor = std::ceil(
        params.min_inlier_fraction * points.size());
    
        while(points.size() >= inline_floor && walls.size() < static_cast<size_t>(params.max_lines)) {
            if (points.size() < 2) break;
            std::uniform_int_distribution<int> pick(0, points.size() - 1);
            // best 2 points hypothesis
            std::vector<int> best;
            for (int t = 0; t < params.max_iters_per_line; ++t){
                int i = pick(rng);
                int j = pick(rng);
                if (i == j) continue;

                const double dx = points[j].x - points[i].x;

                const double dy = points[j].y - points[i].y;
                const double len = std::hypot(dx, dy);
                if (len < 0.005) continue;

                const double nx = -dy / len;
                const double ny =  dx / len;
                const double c = nx * points[i].x + ny * points[i].y;

                // getting all the points that falls into this line
                std::vector<int> inl = get_inliers(
                    points, nx, ny, c, params.inlier_thresh_mm);
                if ( inl.size() > best.size())
                    best.swap(inl);
            }
            if (best.size() < static_cast<size_t>(params.inlier_points_req))
                break;
 
        // -- Refit on inliers, then re-collect inliers around the refit -
        const LineFit fit = refit_tls(points, best);
        const double r_refit = fit.nx * fit.cx + fit.ny * fit.cy;
        const std::vector<int> refined = get_inliers(points,
                                                    fit.nx, fit.ny, r_refit,
                                                    params.inlier_thresh_mm);
        if (static_cast<int>(refined.size()) < params.inlier_points_req) {
            // Refit drifted too far; drop original inliers and try again.
            erase_inliers(points, best);
            continue;
        }

        // -- Endpoints from min/max projection along the line direction -
        double t_min =  std::numeric_limits<double>::infinity();
        double t_max = -std::numeric_limits<double>::infinity();
        for (int k : refined) {
            const double t = fit.dx * (points[k].x - fit.cx)
                           + fit.dy * (points[k].y - fit.cy);
            if (t < t_min) t_min = t;
            if (t > t_max) t_max = t;
        }
        // Pad the segment a little so a downstream endpoint test (Cox
        // matcher) covers points hitting just past the last inlier.
        t_min -= params.endpoint_padding_mm;
        t_max += params.endpoint_padding_mm;

        // Build the LineSeg with normal pointing away from origin
        double nx = fit.nx, ny = fit.ny;
        double r  = nx * fit.cx + ny * fit.cy;
        if (r < 0.0) { nx = -nx; ny = -ny; r = -r; }

        LineSeg seg;
        seg.nx = static_cast<float>(nx);
        seg.ny = static_cast<float>(ny);
        seg.r  = static_cast<float>(r);
        seg.ax = static_cast<float>(fit.cx + t_min * fit.dx);
        seg.ay = static_cast<float>(fit.cy + t_min * fit.dy);
        seg.bx = static_cast<float>(fit.cx + t_max * fit.dx);
        seg.by = static_cast<float>(fit.cy + t_max * fit.dy);

        // Reject too-short segments (chair/desk edges in room mode).
        // Erase inliers either way so the next RANSAC round doesn't re-find
        // the same short line.
        const double seg_len = std::hypot(
            static_cast<double>(seg.bx - seg.ax),
            static_cast<double>(seg.by - seg.ay));
        if (params.min_segment_len_mm > 0.0
            && seg_len < params.min_segment_len_mm) {
            erase_inliers(points, refined);
            continue;
        }
        walls.push_back(seg);

        erase_inliers(points, refined);
    }
    LOG("MAP") << "Found " << walls.size() << " Walls.";
    return walls;
}

namespace calib {
namespace {
std::atomic<bool> g_bounds_ready{false};
std::mutex        g_bounds_m;
MapBounds         g_bounds;
}  // namespace

void compute_map_bounds(const std::vector<LineSeg>& walls) {
    if (walls.empty()) return;
    MapBounds b;
    b.min_x =  std::numeric_limits<double>::max();
    b.max_x =  std::numeric_limits<double>::lowest();
    b.min_y =  std::numeric_limits<double>::max();
    b.max_y =  std::numeric_limits<double>::lowest();
    for (const auto& seg : walls) {
        b.min_x = std::min({b.min_x, (double)seg.ax, (double)seg.bx});
        b.max_x = std::max({b.max_x, (double)seg.ax, (double)seg.bx});
        b.min_y = std::min({b.min_y, (double)seg.ay, (double)seg.by});
        b.max_y = std::max({b.max_y, (double)seg.ay, (double)seg.by});
    }
    {
        std::lock_guard<std::mutex> lk(g_bounds_m);
        g_bounds = b;
    }
    g_bounds_ready.store(true, std::memory_order_release);
    LOG("Calib") << "Map bbox: (" << b.min_x << "," << b.min_y
                 << ") to (" << b.max_x << "," << b.max_y << ")";
}

bool get_map_bounds(MapBounds& out) {
    if (!g_bounds_ready.load(std::memory_order_acquire)) return false;
    std::lock_guard<std::mutex> lk(g_bounds_m);
    out = g_bounds;
    return true;
}

}  // namespace calib
