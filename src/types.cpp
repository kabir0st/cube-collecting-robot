#include <types.hpp>

LidarData::LidarData(double noise) {
    sensor_noise.store(noise);
    buffer_id.store(0);
    int i = 0;
    for (Frame &f : frames)
    {
        f.id = i;
        i++;
    }
}

void LidarData::update_buffer_id() {
    std::lock_guard<std::mutex> guard(lock);
    int next = (buffer_id.load() + 1) % static_cast<int>(frames.size());
    frames[next].ang.clear();
    frames[next].dist.clear();
    buffer_id.store(next);
}

std::array<Frame, int(FRAME_COUNT-1)> LidarData::get_latest_snapshot() const {
    std::lock_guard<std::mutex> guard(lock);
    std::array<Frame, FRAME_COUNT-1> result;
    const size_t start = (buffer_id.load() + 1) % frames.size();
    for (size_t k = 0; k < result.size(); ++k) {
        result[k] = frames[(start + k) % frames.size()];
    }
    return result;
}

void TrackerResult::update_frame(const cv::Mat& f, int frame_rate) {
    std::lock_guard<std::mutex> guard(mutex_frame);
    frame = f.clone();
    fps = frame_rate;
}


cv::Mat TrackerResult::get_frame() const {
    std::lock_guard<std::mutex> guard(mutex_frame);
    return frame.clone();
}