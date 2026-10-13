// mono_live - real-time ORB-SLAM3 monocular on the live camera of the rig.
//
//   mono_live VOCABULARY SETTINGS.yaml [--shm /dev/shm/rig_slam_cam0] [--udp HOST:PORT] [--name NAME] [--seconds N] [--viewer]
//
// The camera stays owned by stereo_capture.py (preview, recording, IMU); start it with  --slam-feed /dev/shm/rig_slam_cam0 .
// This program reads the NEWEST frame from that shared-memory file (see SlamFeed in stereo_capture.py for the layout), runs
// ORB-SLAM3 on it and, when it is slower than the camera, simply skips the frames in between (that is what keeps it real time).
//   - every second one status line: processed fps, mean/worst tracking time, skipped frames, tracking state
//   - --udp HOST:PORT : text datagrams: "P t_ns x y z qx qy qz qw" per tracked frame (camera pose in the map), "S state fps mean_ms skipped" once
//     per second, "N" when a new map starts (the old one was lost)
//   - on Ctrl-C / --seconds: trajectories f_NAME.txt and kf_NAME.txt (EuRoC format, time stamps = the recording clock, ns)
// Monocular: arbitrary scale; it needs real sideways motion to initialise; state 2 = OK, 3 = recently lost, 4 = lost.
#include <System.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static std::atomic<bool> g_stop(false);
static void onSignal(int) { g_stop = true; }

struct FeedHeader                       // must match SlamFeed in stereo_capture.py (little endian, 64 bytes)
{
    uint64_t magic;
    uint64_t seq;
    int64_t t_ns;
    uint32_t frame_id;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t pub_id;                    // counts PUBLISHED frames (frame_id counts camera frames, so it has gaps by design)
};
static uint64_t magicValue()                // the 8 bytes "RIGSLAM1" as the writer stores them
{
    uint64_t m;
    std::memcpy(&m, "RIGSLAM1", 8);
    return m;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "usage: mono_live VOCABULARY SETTINGS.yaml [--shm PATH] [--udp HOST:PORT] [--name NAME] [--seconds N] [--viewer]\n";
        return 1;
    }
    std::string shmPath = "/dev/shm/rig_slam_cam0", udpArg, name = "live";
    double seconds = 0.0;
    bool viewer = false;
    for (int i = 3; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--shm" && i + 1 < argc) shmPath = argv[++i];
        else if (a == "--udp" && i + 1 < argc) udpArg = argv[++i];
        else if (a == "--name" && i + 1 < argc) name = argv[++i];
        else if (a == "--seconds" && i + 1 < argc) seconds = std::atof(argv[++i]);
        else if (a == "--viewer") viewer = true;
        else { std::cerr << "unknown argument " << a << "\n"; return 1; }
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- UDP pose stream (optional)
    int sock = -1;
    sockaddr_in dest;
    std::memset(&dest, 0, sizeof(dest));
    if (!udpArg.empty())
    {
        const size_t c = udpArg.rfind(':');
        if (c == std::string::npos) { std::cerr << "--udp needs HOST:PORT\n"; return 1; }
        dest.sin_family = AF_INET;
        dest.sin_port = htons((uint16_t)std::atoi(udpArg.substr(c + 1).c_str()));
        if (inet_pton(AF_INET, udpArg.substr(0, c).c_str(), &dest.sin_addr) != 1) { std::cerr << "bad IP in --udp\n"; return 1; }
        sock = socket(AF_INET, SOCK_DGRAM, 0);
    }

    // ---- shared-memory feed: wait for the capture program
    int fd = -1;
    std::fprintf(stderr, "waiting for %s (start stereo_capture.py with --slam-feed %s) ...\n", shmPath.c_str(), shmPath.c_str());
    while (!g_stop)
    {
        fd = open(shmPath.c_str(), O_RDONLY);
        struct stat st;
        if (fd >= 0 && fstat(fd, &st) == 0 && st.st_size > (off_t)sizeof(FeedHeader)) break;
        if (fd >= 0) { close(fd); fd = -1; }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (g_stop) return 0;
    struct stat st;
    fstat(fd, &st);
    const unsigned char *base = (const unsigned char *)mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) { std::perror("mmap"); return 1; }
    const FeedHeader *hdr = (const FeedHeader *)base;
    const uint64_t magic = magicValue();
    if (hdr->magic != magic) { std::cerr << "not a RIGSLAM1 feed: " << shmPath << "\n"; return 1; }
    const int W = (int)hdr->width, H = (int)hdr->height, S = (int)hdr->stride;
    const unsigned char *pix = base + 64;
    std::fprintf(stderr, "feed: %dx%d gray\n", W, H);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, viewer);
    const float scale = SLAM.GetImageScale();

    cv::Mat frame(H, W, CV_8UC1);
    uint32_t lastId = 0;
    uint64_t seen = 0, processed = 0, skipped = 0, torn = 0;
    std::vector<double> ms; ms.reserve(4096);
    auto tStart = std::chrono::steady_clock::now(), tReport = tStart;
    double sumMs = 0.0, maxMs = 0.0; int nWin = 0, lastState = -2;
    bool everTracked = false;
    uint64_t winSkipped = 0;

    while (!g_stop)
    {
        if (seconds > 0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count() > seconds) break;
        // seqlock read of the newest frame
        const uint64_t s1 = __atomic_load_n(&hdr->seq, __ATOMIC_ACQUIRE);
        if ((s1 & 1) || s1 == 0 || hdr->pub_id == lastId) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        const int64_t tNs = hdr->t_ns;
        const uint32_t id = hdr->pub_id;
        for (int y = 0; y < H; ++y) std::memcpy(frame.ptr(y), pix + (size_t)y * S, W);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const uint64_t s2 = __atomic_load_n(&hdr->seq, __ATOMIC_ACQUIRE);
        if (s1 != s2) { ++torn; continue; }                      // overwritten while copying: take the next one

        if (lastId != 0 && id > lastId + 1) { skipped += id - lastId - 1; winSkipped += id - lastId - 1; }
        lastId = id; ++seen;

        cv::Mat im = frame;
        const auto a = std::chrono::steady_clock::now();
        if (scale != 1.f) cv::resize(frame, im, cv::Size((int)(W * scale), (int)(H * scale)));
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, (double)tNs * 1e-9);
        const auto b = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double, std::milli>(b - a).count();
        ms.push_back(t); sumMs += t; maxMs = std::max(maxMs, t); ++nWin; ++processed;
        const int state = SLAM.GetTrackingState();
        // a new map starts when initialisation begins again after it had tracked (lost the old map)
        if (sock >= 0 && state == 1 && lastState != 1 && everTracked)
            sendto(sock, "N", 1, 0, (const sockaddr *)&dest, sizeof(dest));
        lastState = state;
        if (state == 2) everTracked = true;

        if (state == 2 && sock >= 0)
        {
            const Sophus::SE3f Twc = Tcw.inverse();
            const Eigen::Vector3f p = Twc.translation();
            const Eigen::Quaternionf q = Twc.unit_quaternion();
            char msg[256];
            const int n = std::snprintf(msg, sizeof(msg), "P %lld %.6f %.6f %.6f %.6f %.6f %.6f %.6f", (long long)tNs,
                                        p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
            sendto(sock, msg, n, 0, (const sockaddr *)&dest, sizeof(dest));
        }

        const double sinceReport = std::chrono::duration<double>(b - tReport).count();
        if (sinceReport >= 1.0)
        {
            static const char *names[] = {"no images", "initialising", "TRACKING", "recently lost", "LOST", "tracking (KLT)"};
            std::fprintf(stderr, "[live] %5.1f fps processed | track mean %5.1f ms worst %5.1f | skipped %llu | %s\n", nWin / sinceReport,
                         sumMs / std::max(nWin, 1), maxMs, (unsigned long long)winSkipped, (state >= 0 && state <= 5) ? names[state] : "?");
            if (sock >= 0)
            {
                char msg[128];
                const int n = std::snprintf(msg, sizeof(msg), "S %d %.1f %.1f %llu", state, nWin / sinceReport, sumMs / std::max(nWin, 1),
                                            (unsigned long long)winSkipped);
                sendto(sock, msg, n, 0, (const sockaddr *)&dest, sizeof(dest));
            }
            tReport = b; sumMs = 0.0; maxMs = 0.0; nWin = 0; winSkipped = 0;
        }
    }

    std::sort(ms.begin(), ms.end());
    if (!ms.empty())
        std::fprintf(stderr, "\n[live] %llu frames processed, %llu skipped (camera was faster), %llu torn reads | mean %.1f ms, median %.1f, 95th %.1f, max %.1f | last state %d\n",
                     (unsigned long long)processed, (unsigned long long)skipped, (unsigned long long)torn,
                     [&] { double s = 0; for (double v : ms) s += v; return s / ms.size(); }(), ms[ms.size() / 2],
                     ms[std::min(ms.size() - 1, (size_t)(0.95 * (ms.size() - 1) + 0.5))], ms.back(), lastState);
    SLAM.Shutdown();
    // ORB-SLAM3's trajectory writers read the first keyframe without checking that there is one, so a run that never
    // initialised (rig kept still, no parallax) crashed here. Only save when it tracked at least once.
    if (everTracked)
    {
        SLAM.SaveTrajectoryEuRoC("f_" + name + ".txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("kf_" + name + ".txt");
    }
    else
        std::fprintf(stderr, "[live] never initialised (monocular needs sideways motion with parallax): no trajectory saved\n");
    return 0;
}
