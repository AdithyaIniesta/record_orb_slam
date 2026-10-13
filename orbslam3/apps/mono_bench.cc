// mono_bench - how fast is ORB-SLAM3 monocular on this machine?
//
//   mono_bench VOCABULARY SETTINGS.yaml RECORDING_DIR [max_frames] [trajectory_name]
//
// Reads a RigCapture recording (times.txt + mav0/cam0/data/<ns>.png), feeds the frames to ORB-SLAM3 as fast as it can go
// (no sleeping to the recording's timestamps, unlike the upstream example) and times every TrackMonocular() call.
// Image loading is not timed. Real time at N fps needs: 95th percentile (and max) well below 1000/N ms.
#include <System.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static double pct(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, (size_t)(p * (v.size() - 1) + 0.5))];
}

int main(int argc, char **argv)
{
    if (argc < 4) { std::cerr << "usage: mono_bench VOCABULARY SETTINGS.yaml RECORDING_DIR [max_frames] [trajectory_name]\n"; return 1; }
    const std::string rec = argv[3];
    const size_t maxArg = argc > 4 ? (size_t)std::stoul(argv[4]) : 0;
    const size_t maxFrames = maxArg > 0 ? maxArg : (size_t)-1;                // 0 or missing = all frames
    const std::string trajName = argc > 5 ? argv[5] : "";

    std::vector<long long> stamps;
    {
        std::ifstream f(rec + "/times.txt");
        long long t;
        while (f >> t) stamps.push_back(t);
    }
    if (stamps.empty()) { std::cerr << "no frames in " << rec << "/times.txt\n"; return 1; }
    const size_t n = std::min(stamps.size(), maxFrames);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);   // no viewer

    std::vector<double> ms; ms.reserve(n);
    int states[8] = {0};
    size_t lost = 0;
    for (size_t i = 0; i < n; ++i)
    {
        char name[64];
        std::snprintf(name, sizeof(name), "/mav0/cam0/data/%lld.png", stamps[i]);
        cv::Mat im = cv::imread(rec + name, cv::IMREAD_UNCHANGED);
        if (im.empty()) { std::cerr << "cannot read " << rec << name << "\n"; return 1; }
        if (im.channels() == 3) cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
        const double t = (double)(stamps[i] - stamps[0]) * 1e-9;

        const auto a = std::chrono::steady_clock::now();
        const float sc = SLAM.GetImageScale();                   // Camera.imageScale of the settings file: the example resizes, not the library
        if (sc != 1.f) cv::resize(im, im, cv::Size((int)(im.cols * sc), (int)(im.rows * sc)));
        SLAM.TrackMonocular(im, t);
        const auto b = std::chrono::steady_clock::now();
        ms.push_back(std::chrono::duration<double, std::milli>(b - a).count());

        const int s = SLAM.GetTrackingState();                  // -1 not ready, 0 no images, 1 not initialised, 2 OK, 3 recently lost, 4 lost
        states[std::min(std::max(s + 1, 0), 7)]++;
        if (s == 4) ++lost;
    }
    double sum = 0.0; for (double v : ms) sum += v;
    const double dur = (double)(stamps[n - 1] - stamps[0]) * 1e-9;
    std::printf("\n=== mono_bench: %zu frames of %.1f s (recording rate %.1f fps) ===\n", n, dur, dur > 0 ? (n - 1) / dur : 0.0);
    std::printf("per frame: mean %.1f ms | median %.1f | 95th %.1f | 99th %.1f | max %.1f\n", sum / n, pct(ms, 0.5), pct(ms, 0.95), pct(ms, 0.99), pct(ms, 1.0));
    std::printf("throughput: %.1f fps (frames / total tracking time)\n", 1000.0 * n / sum);
    for (double fps : {15.0, 20.0, 30.0, 60.0})
    {
        size_t over = 0; for (double v : ms) if (v > 1000.0 / fps) ++over;
        std::printf("  frames slower than the %2.0f fps budget (%.1f ms): %zu (%.1f %%)\n", fps, 1000.0 / fps, over, 100.0 * over / n);
    }
    std::printf("tracking state per frame: not-initialised %d | OK %d | recently-lost %d | lost %d\n", states[2], states[3], states[4], states[5]);

    SLAM.Shutdown();
    if (!trajName.empty()) SLAM.SaveTrajectoryEuRoC("f_" + trajName + ".txt");
    return 0;
}
