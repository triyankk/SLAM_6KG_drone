#include "hesai_lidar_sdk.hpp"
#include "logger.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

using hesai::lidar::DATA_FROM_SERIAL;
using hesai::lidar::DriverParam;
using hesai::lidar::HesaiLidarSdk;
using hesai::lidar::LidarDecodedFrame;
using hesai::lidar::LidarPointXYZICRT;
using hesai::lidar::UdpPacket;

namespace {

constexpr char kFrameMagic[8] = {'O', 'F', 'J', 'T', '1', '6', 'P', '1'};
constexpr std::uint32_t kProtocolVersion = 2;
constexpr std::uint32_t kMaximumPointCount = 1'000'000;

std::atomic<bool> running{true};

#pragma pack(push, 1)
struct PointFrameHeader {
  char magic[8];
  std::uint32_t version;
  std::uint32_t point_count;
  std::uint64_t monotonic_ns;
  std::uint64_t frame_index;
};

struct PackedPoint {
  float x;
  float y;
  float z;
  double timestamp;
  std::uint16_t ring;
  std::uint8_t intensity;
  std::uint8_t confidence;
};
#pragma pack(pop)

static_assert(sizeof(PointFrameHeader) == 32);
static_assert(sizeof(PackedPoint) == 24);

struct Options {
  std::string device;
  std::string correction;
  std::string raw_output;
  int baud = 3'000'000;
  float startup_timeout_s = 5.0F;
};

void handle_signal(int) {
  running.store(false);
}

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool write_all(int fd, const void* source, std::size_t length) {
  const auto* bytes = static_cast<const std::uint8_t*>(source);
  while (length > 0) {
    const ssize_t written = ::write(fd, bytes, length);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    bytes += written;
    length -= static_cast<std::size_t>(written);
  }
  return true;
}

void print_usage(const char* program) {
  std::cerr
      << "Usage: " << program
      << " --device /dev/jt16_usb --correction FILE"
         " [--baud 3000000] [--raw-output FILE]"
         " [--startup-timeout 5]\n";
}

bool parse_options(int argc, char** argv, Options* options) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      return false;
    }
    if (index + 1 >= argc) {
      std::cerr << "Missing value for " << argument << '\n';
      return false;
    }
    const std::string value = argv[++index];
    try {
      if (argument == "--device") {
        options->device = value;
      } else if (argument == "--correction") {
        options->correction = value;
      } else if (argument == "--raw-output") {
        options->raw_output = value;
      } else if (argument == "--baud") {
        options->baud = std::stoi(value);
      } else if (argument == "--startup-timeout") {
        options->startup_timeout_s = std::stof(value);
      } else {
        std::cerr << "Unknown argument: " << argument << '\n';
        return false;
      }
    } catch (const std::exception&) {
      std::cerr << "Invalid value for " << argument << ": " << value << '\n';
      return false;
    }
  }
  if (options->device.empty() || options->correction.empty()) {
    print_usage(argv[0]);
    return false;
  }
  if (options->baud <= 0 || options->startup_timeout_s <= 0.0F) {
    std::cerr << "Baud and startup timeout must be positive\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse_options(argc, argv, &options)) {
    return 2;
  }

  std::ifstream correction_probe(options.correction);
  if (!correction_probe.good()) {
    std::cerr << "JT16 correction file is unreadable: "
              << options.correction << '\n';
    return 2;
  }

  const int frame_output_fd = ::dup(STDOUT_FILENO);
  if (frame_output_fd < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
    std::cerr << "Unable to isolate the binary point stream\n";
    return 2;
  }

  std::ofstream raw_output;
  if (!options.raw_output.empty()) {
    raw_output.open(options.raw_output, std::ios::binary | std::ios::trunc);
    if (!raw_output.good()) {
      std::cerr << "Unable to create raw JT16 capture: "
                << options.raw_output << '\n';
      return 2;
    }
  }

  Logger::GetInstance().setLogTargetRule(HESAI_LOG_TARGET_NONE);
  Logger::GetInstance().setLogLevelRule(
      HESAI_LOG_WARNING | HESAI_LOG_ERROR | HESAI_LOG_FATAL);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  HesaiLidarSdk<LidarPointXYZICRT> sdk;
  DriverParam parameters;
  parameters.input_param.source_type = DATA_FROM_SERIAL;
  parameters.input_param.rs485_com = options.device;
  parameters.input_param.rs232_com = "/dev/null";
  parameters.input_param.point_cloud_baudrate = options.baud;
  parameters.input_param.correction_file_path = options.correction;
  parameters.input_param.correction_save_path = "";
  parameters.input_param.use_ptc_connected = false;
  parameters.input_param.recv_point_cloud_timeout =
      options.startup_timeout_s;
  parameters.decoder_param.enable_packet_loss_tool = true;

  std::mutex output_mutex;
  std::mutex raw_mutex;
  std::atomic<std::uint64_t> frames_emitted{0};
  std::atomic<std::uint64_t> packets_recorded{0};

  sdk.RegRecvCallback(
      std::function<void(const LidarDecodedFrame<LidarPointXYZICRT>&)>(
          [&](const LidarDecodedFrame<LidarPointXYZICRT>& frame) {
            const std::uint64_t callback_monotonic_ns = monotonic_ns();
            const std::uint32_t count = frame.points_num;
            if (!running.load() || count == 0 ||
                count > kMaximumPointCount) {
              return;
            }
            std::vector<PackedPoint> points;
            points.resize(count);
            for (std::uint32_t index = 0; index < count; ++index) {
              points[index] = PackedPoint{
                  frame.points[index].x,
                  frame.points[index].y,
                  frame.points[index].z,
                  frame.points[index].timestamp,
                  frame.points[index].ring,
                  frame.points[index].intensity,
                  frame.points[index].confidence,
              };
            }
            PointFrameHeader header{};
            std::memcpy(header.magic, kFrameMagic, sizeof(kFrameMagic));
            header.version = kProtocolVersion;
            header.point_count = count;
            header.monotonic_ns = callback_monotonic_ns;
            header.frame_index =
                frames_emitted.fetch_add(1, std::memory_order_relaxed) + 1;

            std::lock_guard<std::mutex> lock(output_mutex);
            if (!write_all(frame_output_fd, &header, sizeof(header)) ||
                !write_all(
                    frame_output_fd,
                    points.data(),
                    points.size() * sizeof(PackedPoint))) {
              running.store(false);
            }
          }));

  sdk.RegRecvCallback(std::function<void(const UdpPacket&, double)>(
      [&](const UdpPacket& packet, double) {
        if (!raw_output.good() || packet.packet_len == 0) {
          return;
        }
        std::lock_guard<std::mutex> lock(raw_mutex);
        raw_output.write(
            reinterpret_cast<const char*>(packet.buffer),
            packet.packet_len);
        const auto count =
            packets_recorded.fetch_add(1, std::memory_order_relaxed) + 1;
        if (count % 1000 == 0) {
          raw_output.flush();
        }
      }));

  if (!sdk.Init(parameters)) {
    std::cerr << "Hesai SDK initialization could not start\n";
    return 3;
  }
  sdk.Start();
  if (sdk.lidar_ptr_->GetInitFinish(hesai::lidar::FailInit)) {
    sdk.Stop();
    std::cerr << "JT16 did not produce a decodable point stream\n";
    return 3;
  }

  while (running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  sdk.Stop();
  if (raw_output.good()) {
    raw_output.flush();
  }
  ::close(frame_output_fd);
  std::cerr << "JT16 bridge stopped after "
            << frames_emitted.load(std::memory_order_relaxed)
            << " frames and "
            << packets_recorded.load(std::memory_order_relaxed)
            << " packets\n";
  return 0;
}
