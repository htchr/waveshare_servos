// stop_wheels: HIL safety and readback helper for the waveshare_servos bench.
//
// Default: open the port, WriteSpe(id, 0, acc) on every wheel id, then read back the present
// speed and position a few times and print them.
// --read-only: never command anything, only sample present speed/position (used to find out
// whether wheels are still spinning after a lifecycle transition).
// --registers: before anything else, read what was last written to each servo: mode (33), torque
// enable (40), acceleration (41), goal position (42-43) and goal speed (46-47), both sign-magnitude
// (bit 15). Read-only; used to compare the driver's last goals between builds on the bus itself.
// Every sample is time-stamped, so a spinning wheel's speed also comes out as a least-squares slope
// of its unwrapped position over the sampling window (pos_slope_ls_rad_s).
//
// Built against the vendored SCServo sources of the package (unmodified); see
// build_stop_wheels.sh. Refuses to run while another process holds the port, because two
// processes on the tty corrupt the bus.
//
// Output: human readable sample lines, then one line "RESULT {json}".
// Exit codes: 0 ok, 1 port busy, 2 cannot open port, 3 a servo did not answer,
//             4 (stop mode) a wheel still moving after the stop command, 64 usage.

#include <dirent.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "SCServo.h"

namespace
{

std::vector<int> port_holders(const std::string & port)
{
  std::vector<int> pids;
  char real_port[PATH_MAX];
  if (realpath(port.c_str(), real_port) == nullptr) {
    return pids;
  }
  DIR * proc = opendir("/proc");
  if (proc == nullptr) {
    return pids;
  }
  const int self = getpid();
  while (dirent * e = readdir(proc)) {
    char * end = nullptr;
    const int64_t pid = std::strtol(e->d_name, &end, 10);
    if (*end != '\0' || pid <= 0 || pid == self) {
      continue;
    }
    std::string fd_dir = std::string("/proc/") + e->d_name + "/fd";
    DIR * fds = opendir(fd_dir.c_str());
    if (fds == nullptr) {
      continue;
    }
    while (dirent * f = readdir(fds)) {
      if (f->d_name[0] == '.') {
        continue;
      }
      std::string link = fd_dir + "/" + f->d_name;
      char target[PATH_MAX];
      ssize_t n = readlink(link.c_str(), target, sizeof(target) - 1);
      if (n <= 0) {
        continue;
      }
      target[n] = '\0';
      if (std::strcmp(target, real_port) == 0) {
        pids.push_back(static_cast<int>(pid));
        break;
      }
    }
    closedir(fds);
  }
  closedir(proc);
  return pids;
}

void usage()
{
  std::fprintf(stderr,
    "usage: stop_wheels [--port /dev/ttyACM0] [--baud 1000000] [--ids 3,4] [--acc 150]\n"
    "                   [--read-only] [--registers] [--samples 5] [--interval-ms 100] [--force]\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyACM0";
  int baud = 1000000;
  std::vector<int> ids = {3, 4};
  int acc = 150;
  bool read_only = false;
  bool registers = false;
  bool force = false;
  int samples = 5;
  int interval_ms = 100;

  for (int a = 1; a < argc; a++) {
    std::string arg = argv[a];
    auto next = [&](void) -> std::string {
        if (a + 1 >= argc) {usage(); std::exit(64);}
        return argv[++a];
      };
    if (arg == "--port") {
      port = next();
    } else if (arg == "--baud") {
      baud = std::atoi(next().c_str());
    } else if (arg == "--ids") {
      ids.clear();
      std::stringstream ss(next());
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        ids.push_back(std::atoi(tok.c_str()));
      }
    } else if (arg == "--acc") {
      acc = std::atoi(next().c_str());
    } else if (arg == "--read-only") {
      read_only = true;
    } else if (arg == "--registers") {
      registers = true;
    } else if (arg == "--samples") {
      samples = std::atoi(next().c_str());
    } else if (arg == "--interval-ms") {
      interval_ms = std::atoi(next().c_str());
    } else if (arg == "--force") {
      force = true;
    } else {
      usage();
      return 64;
    }
  }
  if (samples < 2) {
    samples = 2;
  }

  const std::vector<int> holders = port_holders(port);
  if (!holders.empty() && !force) {
    std::printf("port %s is held by pid", port.c_str());
    for (int p : holders) {
      std::printf(" %d", p);
    }
    std::printf("; refusing to touch the bus\n");
    std::printf("RESULT {\"ok\": false, \"error\": \"port_busy\", \"port\": \"%s\"}\n",
      port.c_str());
    return 1;
  }

  SMS_STS bus;
  if (!bus.begin(baud, port.c_str())) {
    std::fflush(stdout);
    std::printf("cannot open %s\n", port.c_str());
    std::printf("RESULT {\"ok\": false, \"error\": \"open_failed\", \"port\": \"%s\"}\n",
      port.c_str());
    return 2;
  }
  std::fflush(stdout);
  bus.IOTimeOut = 20;

  bool answered = true;
  std::vector<int> modes(ids.size(), -1);
  std::vector<int> write_ok(ids.size(), -1);
  // registers as the last writer left them (read before this tool writes anything)
  const int NOREAD = -100000;
  std::vector<int> torque_enable(ids.size(), NOREAD), acc_reg(ids.size(), NOREAD);
  std::vector<int> goal_pos(ids.size(), NOREAD), goal_speed(ids.size(), NOREAD);
  auto sign_magnitude = [NOREAD](int w) {
      if (w < 0) {return NOREAD;}
      return (w & 0x8000) ? -(w & 0x7fff) : w;
    };
  if (registers) {
    for (size_t k = 0; k < ids.size(); k++) {
      const u8 id = static_cast<u8>(ids[k]);
      const int te = bus.readByte(id, SMS_STS_TORQUE_ENABLE);
      const int ac = bus.readByte(id, SMS_STS_ACC);
      torque_enable[k] = te < 0 ? NOREAD : te;
      acc_reg[k] = ac < 0 ? NOREAD : ac;
      goal_pos[k] = sign_magnitude(bus.readWord(id, SMS_STS_GOAL_POSITION_L));
      goal_speed[k] = sign_magnitude(bus.readWord(id, SMS_STS_GOAL_SPEED_L));
      std::printf(
        "registers id=%d mode_reg=%d torque_enable=%d acc=%d goal_position_raw=%d "
        "goal_speed_raw=%d\n",
        ids[k], bus.readByte(id, SMS_STS_MODE), torque_enable[k], acc_reg[k], goal_pos[k],
        goal_speed[k]);
    }
  }
  for (size_t k = 0; k < ids.size(); k++) {
    modes[k] = bus.readByte(static_cast<u8>(ids[k]), SMS_STS_MODE);
    if (!read_only) {
      write_ok[k] = bus.WriteSpe(static_cast<u8>(ids[k]), 0, static_cast<u8>(acc));
    }
  }
  int settle_ms = 0;
  if (!read_only) {
    // the servo ramps down (about 0.6 s from 1 rad/s on the bench); wait until every wheel
    // reports zero speed three reads in a row, at most 4 s
    const auto t_write = std::chrono::steady_clock::now();
    int zero_streak = 0;
    while (zero_streak < 3 &&
      std::chrono::steady_clock::now() - t_write < std::chrono::milliseconds(4000))
    {
      bool all_zero = true;
      for (size_t k = 0; k < ids.size(); k++) {
        const int sp = bus.ReadSpeed(static_cast<u8>(ids[k]));
        if (sp != 0) {
          all_zero = false;
        }
      }
      zero_streak = all_zero ? zero_streak + 1 : 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    settle_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t_write).count());
  }

  const double rad_per_step = 2.0 * M_PI / 4096.0;
  std::vector<std::vector<int>> speed(ids.size()), pos(ids.size());
  std::vector<std::vector<double>> stamp(ids.size());
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < samples; s++) {
    if (s > 0) {
      std::this_thread::sleep_until(t0 + std::chrono::milliseconds(interval_ms * s));
    }
    for (size_t k = 0; k < ids.size(); k++) {
      if (bus.FeedBack(ids[k]) == -1) {
        answered = false;
        std::printf("sample=%d id=%d no_reply\n", s, ids[k]);
        continue;
      }
      const double ts = std::chrono::duration<double>(std::chrono::steady_clock::now() -
        t0).count();
      const int sp = bus.ReadSpeed(-1);
      const int ps = bus.ReadPos(-1);
      speed[k].push_back(sp);
      pos[k].push_back(ps);
      stamp[k].push_back(ts);
      std::printf(
        "sample=%d id=%d present_speed_raw=%d present_speed_rad_s=%.6f present_position_raw=%d\n",
        s, ids[k], sp, sp * rad_per_step, ps);
    }
  }
  bus.end();

  bool still_moving = false;
  std::ostringstream js;
  js << "{\"ok\": " << (answered ? "true" : "false") << ", \"mode\": \"" <<
    (read_only ? "read" : "stop")
     << "\", \"port\": \"" << port << "\", \"interval_ms\": " << interval_ms
     << ", \"settle_ms\": " << settle_ms << ", \"ids\": {";
  for (size_t k = 0; k < ids.size(); k++) {
    int max_abs = 0;
    double sum = 0.0;
    for (int v : speed[k]) {
      max_abs = std::max(max_abs, std::abs(v));
      sum += v;
    }
    // position change over the sampling window, modulo one revolution
    int dpos = 0;
    if (pos[k].size() >= 2) {
      dpos = pos[k].back() - pos[k].front();
      while (dpos > 2048) {dpos -= 4096;}
      while (dpos < -2048) {dpos += 4096;}
    }
    const double span_s = (pos[k].size() >= 2) ? interval_ms * (pos[k].size() - 1) / 1000.0 : 0.0;
    const double slope = span_s > 0 ? dpos * rad_per_step / span_s : 0.0;
    // the speed register jitters by one 50 steps/s quantum at rest, so require more than that
    // or a real position change (8 ticks over the default 0.5 s window = 0.025 rad/s)
    const bool moving = (max_abs > 100) || (std::abs(dpos) > 8);
    still_moving = still_moving || moving;
    // least-squares slope of the unwrapped position against the sample time stamps
    double slope_ls = 0.0;
    if (pos[k].size() >= 3) {
      std::vector<double> u(pos[k].size());
      u[0] = pos[k][0];
      for (size_t s = 1; s < pos[k].size(); s++) {
        int d = pos[k][s] - pos[k][s - 1];
        while (d > 2048) {d -= 4096;}
        while (d < -2048) {d += 4096;}
        u[s] = u[s - 1] + d;
      }
      double mt = 0.0, mu = 0.0;
      for (size_t s = 0; s < u.size(); s++) {
        mt += stamp[k][s]; mu += u[s];
      }
      mt /= u.size();
      mu /= u.size();
      double num = 0.0, den = 0.0;
      for (size_t s = 0; s < u.size(); s++) {
        num += (stamp[k][s] - mt) * (u[s] - mu);
        den += (stamp[k][s] - mt) * (stamp[k][s] - mt);
      }
      slope_ls = den > 0 ? num / den * rad_per_step : 0.0;
    }
    js << (k ? ", " : "") << "\"" << ids[k] << "\": {\"mode\": " << modes[k]
       << ", \"write_ok\": " << write_ok[k]
       << ", \"n\": " << speed[k].size()
       << ", \"max_abs_speed_raw\": " << max_abs
       << ", \"mean_speed_rad_s\": " <<
      (speed[k].empty() ? 0.0 : sum / speed[k].size() * rad_per_step)
       << ", \"dpos_ticks\": " << dpos
       << ", \"pos_slope_rad_s\": " << slope
       << ", \"pos_slope_ls_rad_s\": " << slope_ls
       << ", \"mean_speed_raw\": " << (speed[k].empty() ? 0.0 : sum / speed[k].size())
       << ", \"pos_first_raw\": " << (pos[k].empty() ? -1 : pos[k].front())
       << ", \"pos_last_raw\": " << (pos[k].empty() ? -1 : pos[k].back())
       << ", \"span_s\": " << (stamp[k].size() >= 2 ? stamp[k].back() - stamp[k].front() : 0.0);
    if (registers) {
      js << ", \"registers\": {\"torque_enable\": " << torque_enable[k] << ", \"acc\": " <<
        acc_reg[k]
         << ", \"goal_position_raw\": " << goal_pos[k] << ", \"goal_speed_raw\": " <<
        goal_speed[k] << "}";
    }
    js << ", \"moving\": " << (moving ? "true" : "false") << "}";
  }
  js << "}, \"any_moving\": " << (still_moving ? "true" : "false") << "}";
  std::printf("RESULT %s\n", js.str().c_str());
  if (!answered) {
    return 3;
  }
  if (!read_only && still_moving) {
    return 4;
  }
  return 0;
}
