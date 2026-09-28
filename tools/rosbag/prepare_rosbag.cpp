// Offline ROS 1 bag merging and VLP-16 decoding; no ROS master or replay needed.
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <velodyne_pointcloud/rawdata.h>

#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }

struct Options {
  fs::path output;
  std::string calibration;
  bool decode = false;
  std::vector<fs::path> inputs;
};

void usage() {
  std::cout << "Usage: prepare.sh [--decode-vlp16] [--calibration VLP16.yaml] "
               "--output NEW.bag INPUT.bag [INPUT.bag ...]\n"
               "Merges all messages by recorded timestamp. Existing outputs are refused.\n"
               "--decode-vlp16 replaces /velodyne_packets with /velodyne_points,\n"
               "using the ROS VLP-16 calibration, 0.4-130 m range and per-point time.\n"
               "All other topics (including IMU and cameras) are copied unchanged.\n";
}

Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--decode-vlp16") o.decode = true;
    else if (arg == "--output" || arg == "--calibration") {
      if (++i == argc) throw std::runtime_error("Missing value after " + arg);
      if (arg == "--output") {
        if (!o.output.empty()) throw std::runtime_error("Specify --output once");
        o.output = argv[i];
      } else o.calibration = argv[i];
    } else if (arg == "--") {
      while (++i < argc) o.inputs.emplace_back(argv[i]);
    } else if (arg.rfind("-", 0) == 0) throw std::runtime_error("Unknown option: " + arg);
    else o.inputs.emplace_back(arg);
  }
  if (o.output.empty() || o.inputs.empty()) throw std::runtime_error("Provide --output and input bag(s)");
  if (o.output.extension() != ".bag") throw std::runtime_error("Output must end in .bag");
  if (!o.decode && !o.calibration.empty()) throw std::runtime_error("--calibration requires --decode-vlp16");
  if (fs::symlink_status(o.output).type() != fs::file_type::not_found)
    throw std::runtime_error("Output already exists: " + o.output.string());
  std::set<fs::path> seen;
  for (const auto& input : o.inputs) {
    if (!fs::is_regular_file(input)) throw std::runtime_error("Input is not a file: " + input.string());
    if (!seen.insert(fs::canonical(input)).second) throw std::runtime_error("Duplicate input: " + input.string());
  }
  return o;
}

// Write alongside the destination and publish only after a successful close.
// Hard-link creation refuses an output created by another process in the meantime.
class PendingOutput {
 public:
  explicit PendingOutput(const fs::path& destination) : destination_(fs::absolute(destination)) {
    fs::create_directories(destination_.parent_path());
    std::string pattern = destination_.string() + ".partial.XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    const int fd = ::mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("Cannot create temporary output: " + std::string(std::strerror(errno)));
    ::close(fd);
    temporary_ = name.data();
  }
  ~PendingOutput() {
    std::error_code ignored;
    fs::remove(temporary_, ignored);
  }
  const fs::path& path() const { return temporary_; }
  void commit() { fs::create_hard_link(temporary_, destination_); }
 private:
  fs::path destination_, temporary_;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    usage();
    return 0;
  }
  try {
    const auto options = parse(argc, argv);
    ros::Time::init();
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::vector<std::unique_ptr<rosbag::Bag>> inputs;
    rosbag::View view;
    for (const auto& path : options.inputs) {
      inputs.emplace_back(std::make_unique<rosbag::Bag>(path.string(), rosbag::bagmode::Read));
      view.addQuery(*inputs.back());
    }
    if (view.size() == 0) throw std::runtime_error("Input bags contain no messages");
    std::unique_ptr<velodyne_rawdata::RawData> decoder;
    std::unique_ptr<velodyne_pointcloud::PointcloudXYZIRT> cloud;
    if (options.decode) {
      bool found = false;
      for (const auto* connection : view.getConnections()) {
        if (connection->topic == "/velodyne_points")
          throw std::runtime_error("Input already contains /velodyne_points; use merge mode or separate raw inputs");
        if (connection->topic == "/velodyne_packets") {
          if (connection->datatype != "velodyne_msgs/VelodyneScan")
            throw std::runtime_error("/velodyne_packets must be velodyne_msgs/VelodyneScan");
          found = true;
        }
      }
      if (!found) throw std::runtime_error("No /velodyne_packets topic found");
      const std::string calibration = options.calibration.empty()
          ? ros::package::getPath("velodyne_pointcloud") + "/params/VLP16db.yaml"
          : options.calibration;
      if (!fs::is_regular_file(calibration)) throw std::runtime_error("Missing VLP-16 calibration: " + calibration);
      decoder = std::make_unique<velodyne_rawdata::RawData>();
      if (decoder->setupOffline(calibration, "VLP16", 130., .4))
        throw std::runtime_error("Cannot initialize the VLP-16 decoder");
      decoder->setParameters(.4, 130., 0., 2. * std::acos(-1.));
      cloud = std::make_unique<velodyne_pointcloud::PointcloudXYZIRT>(130., .4, "", "", decoder->scansPerPacket());
      std::cerr << "Decoding VLP-16 using " << calibration << " (0.4-130 m)\n";
    }
    PendingOutput pending(options.output);
    rosbag::Bag output(pending.path().string(), rosbag::bagmode::Write);
    std::size_t messages = 0, scans = 0;
    for (const auto& message : view) {
      if (interrupted) throw std::runtime_error("Interrupted; incomplete output discarded");
      if (decoder && message.getTopic() == "/velodyne_packets") {
        const auto scan = message.instantiate<velodyne_msgs::VelodyneScan>();
        if (!scan || scan->packets.empty()) throw std::runtime_error("Invalid or empty VelodyneScan");
        cloud->setup(scan);
        for (const auto& packet : scan->packets) decoder->unpack(packet, *cloud, scan->header.stamp);
        auto converted = cloud->finishCloud();
        converted.header = scan->header;
        output.write("/velodyne_points", message.getTime(), converted);
        ++scans;
      } else {
        output.write(message.getTopic(), message.getTime(), message, message.getConnectionHeader());
      }
      if (++messages % 10000 == 0)
        std::cerr << "Processed " << messages << "/" << view.size() << " messages\n";
    }
    if (interrupted) throw std::runtime_error("Interrupted; incomplete output discarded");
    output.close();
    pending.commit();
    std::cout << "Saved " << options.output << ": messages=" << messages << ", decoded_scans=" << scans << '\n';
  } catch (const std::exception& error) {
    std::cerr << "prepare_rosbag: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
