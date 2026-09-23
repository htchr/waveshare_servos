// hil_eeprom core (PHASE6_SPEC E.1). See eeprom_core.hpp for what it is and why it stands apart
// from the tools.
//
// Every bus access below is one of the six vendored calls R12 allows -- Ping, Read, readByte,
// readWord, writeByte, writeWord -- behind an is_open() guard: on a closed bus the vendored readSCS
// would FD_SET(-1), which aborts under _FORTIFY_SOURCE (see ServoBus::write_acc in
// src/servo_bus.cpp).

#include "eeprom_core.hpp"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace waveshare_servos
{
namespace hil_eeprom
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr const char * kSnapHeader = "hil_eeprom snapshot 1";

bool stopped(const volatile std::sig_atomic_t * stop)
{
  return stop != nullptr && *stop != 0;
}

std::string value_text(int value)
{
  return value == kUnreadable ? "x" : std::to_string(value);
}

// JSON: a register value is a number, or the string "x" when it could not be read -- the same `x`
// as the .snap, so a gate that looks for one finds it in either (E.8).
std::string json_value(int value)
{
  return value == kUnreadable ? "\"x\"" : std::to_string(value);
}

std::string json_string(const std::string & text)
{
  std::string quoted = "\"";
  for (const char c : text) {
    switch (c) {
      case '"':
        quoted += "\\\"";
        break;
      case '\\':
        quoted += "\\\\";
        break;
      case '\n':
        quoted += "\\n";
        break;
      case '\t':
        quoted += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          quoted += ' ';
        } else {
          quoted += c;
        }
    }
  }
  return quoted + "\"";
}

std::string json_ints(const std::vector<int> & values)
{
  std::string text = "[";
  for (std::size_t i = 0; i < values.size(); i++) {
    text += (i == 0 ? "" : ", ") + std::to_string(values[i]);
  }
  return text + "]";
}

std::string json_strings(const std::vector<std::string> & values)
{
  std::string text = "[";
  for (std::size_t i = 0; i < values.size(); i++) {
    text += (i == 0 ? "" : ", ") + json_string(values[i]);
  }
  return text + "]";
}

std::string spaced(const std::vector<int> & values)
{
  std::string text;
  for (std::size_t i = 0; i < values.size(); i++) {
    text += (i == 0 ? "" : " ") + std::to_string(values[i]);
  }
  return text;
}

// A whole-string integer, or false. No std::stoi: it accepts "12abc" and throws on the rest.
bool parse_int(const std::string & text, int64_t min, int64_t max, int * value)
{
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const int64_t parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' || parsed < min || parsed > max) {
    return false;
  }
  *value = static_cast<int>(parsed);
  return true;
}

bool parse_int_list(
  const std::string & text, int64_t min, int64_t max, std::vector<int> * values)
{
  values->clear();
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ',')) {
    int value = 0;
    if (!parse_int(item, min, max, &value)) {
      return false;
    }
    values->push_back(value);
  }
  return !values->empty() && text.back() != ',';
}

// ---- the six vendored calls, guarded ----

int read_byte(ServoBus & bus, int id, int reg)
{
  for (int attempt = 0; attempt < kReadAttempts && bus.is_open(); attempt++) {
    const int value = bus.readByte(static_cast<uint8_t>(id), static_cast<uint8_t>(reg));
    if (value >= 0) {
      return value;
    }
  }
  return kUnreadable;
}

int read_word(ServoBus & bus, int id, int reg)
{
  for (int attempt = 0; attempt < kReadAttempts && bus.is_open(); attempt++) {
    const int value = bus.readWord(static_cast<uint8_t>(id), static_cast<uint8_t>(reg));
    if (value >= 0) {
      return value;
    }
  }
  return kUnreadable;
}

bool pings(ServoBus & bus, int id, int attempts)
{
  for (int attempt = 0; attempt < attempts && bus.is_open(); attempt++) {
    if (bus.Ping(static_cast<uint8_t>(id)) != -1) {
      return true;
    }
  }
  return false;
}

}  // namespace

// ---- tables ----

const std::vector<int> & eeprom_registers()
{
  static const std::vector<int> regs = [] {
      std::vector<int> list = {0, 1};
      for (int reg = 3; reg <= kEepromLast; reg++) {
        list.push_back(reg);
      }
      return list;
    }();
  return regs;
}

const std::vector<int> & compared_registers(bool eeprom_only)
{
  static const std::vector<int> all = [] {
      std::vector<int> list = eeprom_registers();
      list.push_back(kRegTorque);
      list.push_back(kRegLock);
      return list;
    }();
  return eeprom_only ? eeprom_registers() : all;
}

// The memory table's two-byte EEPROM fields: min and max angle (9, 11), max torque (16), the two
// protection words (24, 28) and the offset (31). Written whole, so a failure between the two
// halves cannot leave a word that is neither the old value nor the new one.
const std::vector<int> & word_fields()
{
  static const std::vector<int> fields = {9, 11, 16, 24, 28, 31};
  return fields;
}

const std::vector<std::pair<int, bool>> & volatile_registers()
{
  // goal and present position as words, then voltage, temperature and status
  static const std::vector<std::pair<int, bool>> regs = {
    {kRegGoalPosition, true}, {kRegPresentPosition, true}, {62, false}, {63, false}, {65, false}};
  return regs;
}

int ServoRecord::reg(int address) const
{
  const auto it = regs.find(address);
  return it == regs.end() ? kUnreadable : it->second;
}

int ServoRecord::readable_eeprom() const
{
  int readable = 0;
  for (const int address : eeprom_registers()) {
    readable += reg(address) == kUnreadable ? 0 : 1;
  }
  return readable;
}

bool ServoRecord::complete() const
{
  for (const int address : compared_registers(false)) {
    if (reg(address) == kUnreadable) {
      return false;
    }
  }
  for (const auto & entry : volatile_registers()) {
    const auto it = volatiles.find(entry.first);
    if (it == volatiles.end() || it->second == kUnreadable) {
      return false;
    }
  }
  return true;
}

// ---- the .snap text ----
//
//   hil_eeprom snapshot 1
//   ids 1 2 3 4
//   ok true
//   census 1 2 3 4                                   (only when the census was taken)
//   servo 1 eeprom 3 6 - 9 3 1 0 ...                 (40 values: '-' for 2, 'x' unreadable)
//   servo 1 sram 40=1 55=0
//   servo 1 volatile 42=1026 56=1026 62=122 63=33 65=0

std::string format_snap(const Snapshot & snap)
{
  std::ostringstream text;
  text << kSnapHeader << "\n";
  text << "ids" << (snap.ids.empty() ? "" : " ") << spaced(snap.ids) << "\n";
  text << "ok " << (snap.ok ? "true" : "false") << "\n";
  if (snap.has_census) {
    text << "census" << (snap.census.empty() ? "" : " ") << spaced(snap.census) << "\n";
  }
  for (const auto & entry : snap.servos) {
    const int id = entry.first;
    const ServoRecord & servo = entry.second;
    text << "servo " << id << " eeprom";
    for (int reg = 0; reg <= kEepromLast; reg++) {
      text << " " << (reg == 2 ? "-" : value_text(servo.reg(reg)));
    }
    text << "\n";
    text << "servo " << id << " sram " << kRegTorque << "=" << value_text(servo.reg(kRegTorque)) <<
      " " << kRegLock << "=" << value_text(servo.reg(kRegLock)) << "\n";
    text << "servo " << id << " volatile";
    for (const auto & reg : volatile_registers()) {
      const auto it = servo.volatiles.find(reg.first);
      text << " " << reg.first << "=" <<
        value_text(it == servo.volatiles.end() ? kUnreadable : it->second);
    }
    text << "\n";
  }
  return text.str();
}

namespace
{

// "x" or 0..max
bool parse_value(const std::string & token, int max, int * value)
{
  if (token == "x") {
    *value = kUnreadable;
    return true;
  }
  return parse_int(token, 0, max, value);
}

// "40=1" into {40, 1}
bool parse_pair(const std::string & token, int max, int * reg, int * value)
{
  const std::size_t equals = token.find('=');
  return equals != std::string::npos &&
         parse_int(token.substr(0, equals), 0, 255, reg) &&
         parse_value(token.substr(equals + 1), max, value);
}

}  // namespace

bool parse_snap(const std::string & text, Snapshot * snap, std::string * error)
{
  Snapshot parsed;
  bool header = false;
  bool ids = false;
  bool ok = false;
  std::set<std::pair<int, std::string>> seen;   // {id, kind} lines, each at most once
  std::istringstream lines(text);
  std::string line;
  int number = 0;
  auto fail = [&](const std::string & why) {
      if (error != nullptr) {
        *error = "line " + std::to_string(number) + ": " + why;
      }
      return false;
    };
  while (std::getline(lines, line)) {
    number++;
    std::istringstream words(line);
    std::vector<std::string> tokens;
    for (std::string word; words >> word; ) {
      tokens.push_back(word);
    }
    if (tokens.empty()) {
      continue;
    }
    if (!header) {
      if (line != kSnapHeader) {
        return fail("not a hil_eeprom snapshot (expected '" + std::string(kSnapHeader) + "')");
      }
      header = true;
      continue;
    }
    const std::string & kind = tokens[0];
    if (kind == "ids" || kind == "census") {
      std::vector<int> values;
      for (std::size_t i = 1; i < tokens.size(); i++) {
        int value = 0;
        if (!parse_int(tokens[i], kCensusFirstId, kCensusLastId, &value)) {
          return fail("bad id '" + tokens[i] + "'");
        }
        values.push_back(value);
      }
      if (kind == "ids") {
        if (ids) {
          return fail("a second ids line");
        }
        ids = true;
        parsed.ids = values;
      } else {
        if (parsed.has_census) {
          return fail("a second census line");
        }
        parsed.has_census = true;
        parsed.census = values;
      }
      continue;
    }
    if (kind == "ok") {
      if (ok || tokens.size() != 2 || (tokens[1] != "true" && tokens[1] != "false")) {
        return fail("expected one 'ok true' or 'ok false'");
      }
      ok = true;
      parsed.ok = tokens[1] == "true";
      continue;
    }
    if (kind != "servo" || tokens.size() < 3) {
      return fail("unexpected line '" + line + "'");
    }
    int id = 0;
    if (!parse_int(tokens[1], kCensusFirstId, kCensusLastId, &id)) {
      return fail("bad servo id '" + tokens[1] + "'");
    }
    if (!seen.insert({id, tokens[2]}).second) {
      return fail("a second 'servo " + tokens[1] + " " + tokens[2] + "' line");
    }
    ServoRecord & servo = parsed.servos[id];
    if (tokens[2] == "eeprom") {
      if (tokens.size() != 3 + kEepromLast + 1) {
        return fail("an eeprom line needs " + std::to_string(kEepromLast + 1) + " values");
      }
      for (int reg = 0; reg <= kEepromLast; reg++) {
        const std::string & token = tokens[3 + reg];
        if (reg == 2) {
          if (token != "-") {
            return fail("address 2 must be '-'");
          }
          continue;
        }
        int value = 0;
        if (!parse_value(token, 255, &value)) {
          return fail("bad value '" + token + "' at address " + std::to_string(reg));
        }
        servo.regs[reg] = value;
      }
    } else if (tokens[2] == "sram" || tokens[2] == "volatile") {
      for (std::size_t i = 3; i < tokens.size(); i++) {
        int reg = 0;
        int value = 0;
        if (!parse_pair(tokens[i], 65535, &reg, &value)) {
          return fail("bad register=value '" + tokens[i] + "'");
        }
        if (tokens[2] == "sram") {
          if (reg != kRegTorque && reg != kRegLock) {
            return fail("an sram line holds only 40 and 55");
          }
          servo.regs[reg] = value;
        } else {
          servo.volatiles[reg] = value;
        }
      }
    } else {
      return fail("unexpected line '" + line + "'");
    }
  }
  if (!header) {
    number = 0;
    return fail("empty file");
  }
  if (!ids || !ok) {
    return fail(std::string("missing the ") + (!ids ? "ids" : "ok") + " line");
  }
  *snap = parsed;
  return true;
}

std::string snapshot_json(const Snapshot & snap)
{
  std::ostringstream json;
  json << "{\"ok\": " << (snap.ok ? "true" : "false") << ", \"census\": " <<
    (snap.has_census ? json_ints(snap.census) : "null") << ", \"ids\": {";
  bool first = true;
  for (const auto & entry : snap.servos) {
    const ServoRecord & servo = entry.second;
    json << (first ? "" : ", ") << "\"" << entry.first << "\": {\"n\": " <<
      servo.readable_eeprom() << ", \"eeprom\": {";
    first = false;
    bool first_reg = true;
    for (const int reg : eeprom_registers()) {
      json << (first_reg ? "" : ", ") << "\"" << reg << "\": " << json_value(servo.reg(reg));
      first_reg = false;
    }
    json << "}, \"sram\": {\"" << kRegTorque << "\": " << json_value(servo.reg(kRegTorque)) <<
      ", \"" << kRegLock << "\": " << json_value(servo.reg(kRegLock)) << "}, \"volatile\": {";
    first_reg = true;
    for (const auto & reg : volatile_registers()) {
      const auto it = servo.volatiles.find(reg.first);
      json << (first_reg ? "" : ", ") << "\"" << reg.first << "\": " <<
        json_value(it == servo.volatiles.end() ? kUnreadable : it->second);
      first_reg = false;
    }
    json << "}}";
  }
  json << "}}";
  return json.str();
}

// ---- bus reads ----

std::vector<int> census(
  ServoBus & bus, const volatile std::sig_atomic_t * stop, bool * interrupted)
{
  std::vector<int> found;
  bool stop_seen = false;
  if (bus.is_open()) {
    const uint32_t saved = bus.io_timeout_ms();
    bus.set_io_timeout_ms(kCensusTimeoutMs);
    for (int id = kCensusFirstId; id <= kCensusLastId; id++) {
      if (stopped(stop)) {
        stop_seen = true;
        break;
      }
      if (pings(bus, id, kCensusAttempts)) {
        found.push_back(id);
      }
    }
    bus.set_io_timeout_ms(saved);
  }
  if (interrupted != nullptr) {
    *interrupted = stop_seen;
  }
  return found;
}

ServoRecord read_servo(ServoBus & bus, int id)
{
  ServoRecord servo;
  for (const int reg : compared_registers(false)) {
    servo.regs[reg] = read_byte(bus, id, reg);
  }
  for (const auto & reg : volatile_registers()) {
    servo.volatiles[reg.first] =
      reg.second ? read_word(bus, id, reg.first) : read_byte(bus, id, reg.first);
  }
  return servo;
}

Snapshot take_snapshot(
  ServoBus & bus, const std::vector<int> & ids, bool with_census,
  const volatile std::sig_atomic_t * stop, bool * interrupted)
{
  Snapshot snap;
  snap.ids = ids;
  bool stop_seen = false;
  if (with_census) {
    snap.has_census = true;
    snap.census = census(bus, stop, &stop_seen);
  }
  for (const int id : ids) {
    if (stop_seen || stopped(stop)) {
      stop_seen = true;
      break;
    }
    snap.servos[id] = read_servo(bus, id);
  }
  snap.ok = !stop_seen && bus.is_open();
  for (const int id : ids) {
    const auto it = snap.servos.find(id);
    snap.ok = snap.ok && it != snap.servos.end() && it->second.complete();
  }
  if (interrupted != nullptr) {
    *interrupted = stop_seen;
  }
  return snap;
}

// ---- compare ----

std::vector<Diff> compare(const Snapshot & a, const Snapshot & b, bool eeprom_only)
{
  std::vector<Diff> diffs;
  if ((a.has_census || b.has_census) &&
    (a.has_census != b.has_census || a.census != b.census))
  {
    Diff diff;
    diff.census = true;
    diff.a = a.has_census ? spaced(a.census) : "not taken";
    diff.b = b.has_census ? spaced(b.census) : "not taken";
    diffs.push_back(diff);
  }
  std::set<int> ids;
  for (const auto & entry : a.servos) {
    ids.insert(entry.first);
  }
  for (const auto & entry : b.servos) {
    ids.insert(entry.first);
  }
  for (const int id : ids) {
    const auto in_a = a.servos.find(id);
    const auto in_b = b.servos.find(id);
    if (in_a == a.servos.end() || in_b == b.servos.end()) {
      Diff diff;
      diff.id = id;
      diff.missing_in = in_a == a.servos.end() ? "A" : "B";
      diffs.push_back(diff);
      continue;
    }
    for (const int reg : compared_registers(eeprom_only)) {
      const int va = in_a->second.reg(reg);
      const int vb = in_b->second.reg(reg);
      // an unreadable byte is unequal to everything, another unreadable byte included
      if (va == kUnreadable || vb == kUnreadable || va != vb) {
        Diff diff;
        diff.id = id;
        diff.reg = reg;
        diff.a = value_text(va);
        diff.b = value_text(vb);
        diffs.push_back(diff);
      }
    }
  }
  return diffs;
}

std::string diff_text(const Diff & diff)
{
  if (diff.census) {
    return "census: " + diff.a + " -> " + diff.b;
  }
  if (!diff.missing_in.empty()) {
    return "id " + std::to_string(diff.id) + ": missing in " + diff.missing_in;
  }
  return "id " + std::to_string(diff.id) + " reg " + std::to_string(diff.reg) + ": " + diff.a +
         " -> " + diff.b;
}

namespace
{

std::string diff_json(const Diff & diff)
{
  std::string json = "{\"text\": " + json_string(diff_text(diff));
  if (diff.census) {
    json += ", \"census\": true, \"a\": " + json_string(diff.a) + ", \"b\": " + json_string(diff.b);
  } else if (!diff.missing_in.empty()) {
    json += ", \"id\": " + std::to_string(diff.id) + ", \"missing_in\": " +
      json_string(diff.missing_in);
  } else {
    json += ", \"id\": " + std::to_string(diff.id) + ", \"reg\": " + std::to_string(diff.reg) +
      ", \"a\": " + (diff.a == "x" ? "\"x\"" : diff.a) + ", \"b\": " +
      (diff.b == "x" ? "\"x\"" : diff.b);
  }
  return json + "}";
}

std::string diffs_json(const std::vector<Diff> & diffs)
{
  std::string json = "[";
  for (std::size_t i = 0; i < diffs.size(); i++) {
    json += (i == 0 ? "" : ", ") + diff_json(diffs[i]);
  }
  return json + "]";
}

}  // namespace

// ---- restore ----

std::vector<std::string> source_problems(const Snapshot & source)
{
  std::vector<std::string> problems;
  if (!source.ok) {
    problems.push_back(
      "the source says ok false: it was taken with an unreadable byte or a silent id");
  }
  std::vector<int> lines;
  for (const auto & entry : source.servos) {
    lines.push_back(entry.first);
    for (const int reg : compared_registers(false)) {
      if (entry.second.reg(reg) == kUnreadable) {
        problems.push_back(
          "id " + std::to_string(entry.first) + " reg " + std::to_string(reg) +
          " is x in the source; restoring it would mean guessing");
      }
    }
  }
  for (const int id : source.ids) {
    if (source.servos.count(id) == 0) {
      problems.push_back("id " + std::to_string(id) + " is listed but has no servo line");
    }
  }
  for (const int id : lines) {
    if (std::find(source.ids.begin(), source.ids.end(), id) == source.ids.end()) {
      problems.push_back("id " + std::to_string(id) + " has a servo line but is not listed");
    }
  }
  if (!source.has_census) {
    problems.push_back(
      "the source has no census line, so it cannot say which ids should answer "
      "(take it with --census)");
  } else if (source.census != lines) {
    problems.push_back(
      "the source's census (" + spaced(source.census) + ") is not its servo lines (" +
      spaced(lines) + ")");
  }
  return problems;
}

namespace
{

// Blocks SIGINT, SIGTERM, SIGHUP and SIGQUIT on this thread from the first write of a sequence to
// its end (E.1 "Signals"). The handlers stay installed, so a signal that arrives meanwhile is only
// held; when the mask is restored it runs the handler, which sets the flag -- after the sequence,
// never between an unlock and its lock.
class DeferredSignals
{
public:
  DeferredSignals() = default;
  ~DeferredSignals() {end();}
  DeferredSignals(const DeferredSignals &) = delete;
  DeferredSignals & operator=(const DeferredSignals &) = delete;
  DeferredSignals(DeferredSignals &&) = delete;
  DeferredSignals & operator=(DeferredSignals &&) = delete;

  void begin()
  {
    if (active_) {
      return;
    }
    sigset_t set;
    sigemptyset(&set);
    for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) {
      sigaddset(&set, signal_number);
    }
    active_ = ::pthread_sigmask(SIG_BLOCK, &set, &saved_) == 0;
  }

  void end()
  {
    if (active_) {
      ::pthread_sigmask(SIG_SETMASK, &saved_, nullptr);
      active_ = false;
    }
  }

private:
  bool active_ = false;
  sigset_t saved_{};
};

// Every write goes through here, so a report lists exactly what went on the wire, and the first
// one starts the deferral. The ack is never consulted: SCS::Ack rejects an ack from a new id and
// returns 1 whatever the status byte says (src/SCS.cpp:265-295), so the read-back after each write
// is the only verdict.
class Writer
{
public:
  Writer(ServoBus & bus, std::vector<Written> * log, DeferredSignals * signals)
  : bus_(bus), log_(log), signals_(signals) {}

  void byte(int id, int reg, int from, int value)
  {
    signals_->begin();
    if (bus_.is_open()) {
      bus_.writeByte(
        static_cast<uint8_t>(id), static_cast<uint8_t>(reg), static_cast<uint8_t>(value));
    }
    log_->push_back(Written{id, reg, from, value, 1, reg <= kEepromLast});
  }

  // one frame for both bytes, so a failure cannot leave half of a word written
  void word(int id, int reg, int from, int value)
  {
    signals_->begin();
    if (bus_.is_open()) {
      bus_.writeWord(
        static_cast<uint8_t>(id), static_cast<uint8_t>(reg), static_cast<uint16_t>(value));
    }
    log_->push_back(Written{id, reg, from, value, 2, reg <= kEepromLast});
  }

private:
  ServoBus & bus_;
  std::vector<Written> * log_;
  DeferredSignals * signals_;
};

// Poll until the register reads `want` or kVerifyWindowMs has passed. An EEPROM commit can keep a
// servo off the bus for longer than one read, so a single read-back could fail a write that took.
bool reads_back(ServoBus & bus, int id, int reg, bool word, int want)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(kVerifyWindowMs);
  while (true) {
    const int value = word ? read_word(bus, id, reg) : read_byte(bus, id, reg);
    if (value == want) {
      return true;
    }
    if (Clock::now() >= deadline || !bus.is_open()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kVerifyPollMs));
  }
}

bool answers_within(ServoBus & bus, int id, uint32_t window_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(window_ms);
  while (true) {
    if (pings(bus, id, 1)) {
      return true;
    }
    if (Clock::now() >= deadline || !bus.is_open()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kVerifyPollMs));
  }
}

std::vector<int> unreadable_registers(const ServoRecord & servo)
{
  std::vector<int> regs;
  for (const int reg : compared_registers(false)) {
    if (servo.reg(reg) == kUnreadable) {
      regs.push_back(reg);
    }
  }
  return regs;
}

std::string id_text(int id)
{
  return "id " + std::to_string(id);
}

// The first register of the field `reg` belongs to: a word field's second byte maps to its first.
int field_of(int reg)
{
  const std::vector<int> & words = word_fields();
  return std::find(words.begin(), words.end(), reg - 1) != words.end() ? reg - 1 : reg;
}

bool is_word_field(int reg)
{
  const std::vector<int> & words = word_fields();
  return std::find(words.begin(), words.end(), reg) != words.end();
}

int word_of(const ServoRecord & servo, int reg)
{
  const int low = servo.reg(reg);
  const int high = servo.reg(reg + 1);
  return (low == kUnreadable || high == kUnreadable) ? kUnreadable : (low | (high << 8));
}

}  // namespace

RestoreReport restore(
  ServoBus & bus, const Snapshot & source, const RestoreOptions & options,
  const volatile std::sig_atomic_t * stop)
{
  RestoreReport report;
  auto refuse = [&report](int code, const std::string & why) {
      report.problems.push_back(why);
      report.exit = code;
      return report;
    };

  // 0. the source, before a byte goes out
  report.problems = source_problems(source);
  if (!report.problems.empty()) {
    report.exit = kExitCannotResolve;
    return report;
  }
  if (!bus.is_open()) {
    return refuse(kExitCannotOpen, "the bus is not open");
  }
  const std::vector<int> & expected = source.census;   // == the servo lines (source_problems)

  // 1. the census, and at most one stray id to move back
  bool interrupted = false;
  const std::vector<int> live = census(bus, stop, &interrupted);
  if (interrupted) {
    return refuse(kExitInterrupted, "interrupted before the first write; nothing was written");
  }
  std::vector<int> missing;
  std::vector<int> unexpected;
  std::set_difference(
    expected.begin(), expected.end(), live.begin(), live.end(), std::back_inserter(missing));
  std::set_difference(
    live.begin(), live.end(), expected.begin(), expected.end(), std::back_inserter(unexpected));
  if (!missing.empty() || !unexpected.empty()) {
    const std::string found = "census: the source has " + spaced(expected) + ", the bus answers " +
      (live.empty() ? std::string("nothing") : spaced(live));
    if (missing.size() != 1 || unexpected.size() != 1) {
      return refuse(kExitCannotResolve, found + "; ambiguous; resolve by hand with scan");
    }
    // a stray id is moved back only when it is the missing servo's model: bytes 0, 1, 3 and 4
    const int m = missing[0];
    const int u = unexpected[0];
    for (const int reg : {0, 1, 3, 4}) {
      const int value = read_byte(bus, u, reg);
      if (value == kUnreadable || value != source.servos.at(m).reg(reg)) {
        return refuse(
          kExitCannotResolve, found + "; " + id_text(u) + " reg " + std::to_string(reg) +
          " reads " + value_text(value) + " where the missing " + id_text(m) + " had " +
          value_text(source.servos.at(m).reg(reg)) + "; ambiguous; resolve by hand with scan");
      }
    }
    report.moved_from = u;
    report.moved_to = m;
  }

  // 2. the bench as it is now, each id read where it answers, and the model check
  std::map<int, ServoRecord> current;
  for (const int id : expected) {
    if (stopped(stop)) {
      return refuse(kExitInterrupted, "interrupted before the first write; nothing was written");
    }
    const int at = (id == report.moved_to) ? report.moved_from : id;
    current[id] = read_servo(bus, at);
    const std::vector<int> holes = unreadable_registers(current[id]);
    if (!holes.empty()) {
      return refuse(
        kExitCannotResolve, "cannot read " + id_text(at) + " register(s) " + spaced(holes) +
        "; nothing was written");
    }
  }
  std::vector<std::string> refusals;
  for (const int id : expected) {
    for (const int reg : {0, 1, 3, 4}) {
      const int want = source.servos.at(id).reg(reg);
      const int now = current[id].reg(reg);
      if (want != now) {
        refusals.push_back(
          id_text(id) + " reg " + std::to_string(reg) + ": the source has " + value_text(want) +
          ", the servo reads " + value_text(now) +
          " -- a different servo model; nothing was written");
      }
    }
  }
  for (const int id : expected) {
    for (const int reg : compared_registers(false)) {
      const int want = source.servos.at(id).reg(reg);
      const int now = current[id].reg(reg);
      if (want != now) {
        Diff diff;
        diff.id = id;
        diff.reg = reg;
        diff.a = value_text(want);
        diff.b = value_text(now);
        report.before.push_back(diff);
      }
    }
  }
  // registers restore never writes: 5 except to move a stray id back, and 6, the link itself
  for (const Diff & diff : report.before) {
    if ((diff.reg == kRegId && diff.id != report.moved_to) || diff.reg == kRegBaud) {
      refusals.push_back(
        id_text(diff.id) + " reg " + std::to_string(diff.reg) + " differs (" + diff.a + " -> " +
        diff.b + "); restore never writes the id register outside an id move, nor the baud " +
        "register; nothing was written");
    }
  }
  // 3. --allow-regs: every difference printed, and zero writes, if one falls outside the list
  if (refusals.empty() && options.limit_regs) {
    for (const Diff & diff : report.before) {
      if (options.allow_regs.count(diff.reg) == 0) {
        refusals.push_back(
          id_text(diff.id) + " reg " + std::to_string(diff.reg) + " differs (" + diff.a + " -> " +
          diff.b + ") and is not in --allow-regs; nothing was written");
      }
    }
  }
  if (!refusals.empty()) {
    report.problems = refusals;
    report.exit = kExitCannotResolve;
    return report;
  }
  if (stopped(stop)) {
    return refuse(kExitInterrupted, "interrupted before the first write; nothing was written");
  }

  // From the first write on, the four signals are held and nothing stops the sequence.
  DeferredSignals signals;
  Writer write(bus, &report.writes, &signals);
  bool failed = false;
  auto fail = [&report, &failed](const std::string & why) {
      report.problems.push_back(why);
      failed = true;
    };

  // 1 (continued). the id move: verified unlock, the id, the new id answers and the old one does
  // not, verified lock where the servo now is
  if (report.moved_from != -1) {
    const int u = report.moved_from;
    const int m = report.moved_to;
    write.byte(u, kRegLock, current[m].reg(kRegLock), 0);
    if (!reads_back(bus, u, kRegLock, false, 0)) {
      fail("could not open the EEPROM lock of " + id_text(u) + "; its id was not written");
      write.byte(u, kRegLock, kUnreadable, 1);
      reads_back(bus, u, kRegLock, false, 1);
    } else {
      write.byte(u, kRegId, u, m);
      const bool at_m = answers_within(bus, m, kMoveWindowMs);
      const bool at_u = pings(bus, u, kCensusAttempts);
      if (!at_m || at_u) {
        fail(
          "the id write did not move " + id_text(u) + " to " + id_text(m) + " (" + id_text(m) +
          (at_m ? " answers" : " is silent") + ", " + id_text(u) +
          (at_u ? " still answers" : " is silent") + "); run scan");
        const int now_at = at_m ? m : u;
        write.byte(now_at, kRegLock, 0, 1);
        reads_back(bus, now_at, kRegLock, false, 1);
      } else {
        write.byte(m, kRegLock, 0, 1);
        if (!reads_back(bus, m, kRegLock, false, 1)) {
          fail("the EEPROM lock of " + id_text(m) + " did not close after the id move");
        }
      }
    }
  }

  // 4. EEPROM bytes 7..39, per id: torque off, verified unlock, each differing field written once
  // and read back, verified lock
  for (const int id : expected) {
    if (failed) {
      break;
    }
    std::vector<int> fields;
    for (int reg = 7; reg <= kEepromLast; reg++) {
      if (source.servos.at(id).reg(reg) != current[id].reg(reg)) {
        const int field = field_of(reg);
        if (fields.empty() || fields.back() != field) {
          fields.push_back(field);
        }
      }
    }
    if (fields.empty()) {
      continue;
    }
    const int torque = read_byte(bus, id, kRegTorque);
    if (torque != 0) {
      write.byte(id, kRegTorque, torque, 0);
      if (!reads_back(bus, id, kRegTorque, false, 0)) {
        fail(
          "could not switch the torque of " + id_text(id) +
          " off; none of its EEPROM was written");
        break;
      }
    }
    write.byte(id, kRegLock, read_byte(bus, id, kRegLock), 0);
    if (!reads_back(bus, id, kRegLock, false, 0)) {
      fail(
        "could not open the EEPROM lock of " + id_text(id) + " (register 55 does not read back 0); "
        "none of its EEPROM was written");
      write.byte(id, kRegLock, kUnreadable, 1);
      reads_back(bus, id, kRegLock, false, 1);
      break;
    }
    for (const int field : fields) {
      const bool word = is_word_field(field);
      const int want =
        word ? word_of(source.servos.at(id), field) : source.servos.at(id).reg(field);
      const int was = word ? word_of(current[id], field) : current[id].reg(field);
      if (word) {
        write.word(id, field, was, want);
      } else {
        write.byte(id, field, was, want);
      }
      if (!reads_back(bus, id, field, word, want)) {
        fail(id_text(id) + " reg " + std::to_string(field) + ": wrote " + std::to_string(want) +
          " and it does not read back");
        break;
      }
    }
    write.byte(id, kRegLock, 0, 1);
    if (!reads_back(bus, id, kRegLock, false, 1)) {
      fail(
        "the EEPROM lock of " + id_text(id) +
        " did not close (register 55 does not read back 1)");
    }
  }

  // 5. SRAM last: the lock as the source had it, then torque -- the goal first, so enabling it
  // never drives a joint toward a stale goal
  for (const int id : expected) {
    if (failed) {
      break;
    }
    const ServoRecord & want = source.servos.at(id);
    const int lock = read_byte(bus, id, kRegLock);
    if (lock != want.reg(kRegLock)) {
      write.byte(id, kRegLock, lock, want.reg(kRegLock));
      if (!reads_back(bus, id, kRegLock, false, want.reg(kRegLock))) {
        fail(id_text(id) + " reg 55 does not read back " + value_text(want.reg(kRegLock)));
        break;
      }
    }
    const int torque = read_byte(bus, id, kRegTorque);
    if (torque == want.reg(kRegTorque)) {
      continue;
    }
    if (want.reg(kRegTorque) != 0) {
      const int mode = want.reg(kRegMode);
      if (mode == 0) {
        const int present = read_word(bus, id, kRegPresentPosition);
        if (present == kUnreadable) {
          fail("cannot read the position of " + id_text(id) + " to set its goal; torque left off");
          break;
        }
        write.word(id, kRegGoalPosition, read_word(bus, id, kRegGoalPosition), present);
        if (!reads_back(bus, id, kRegGoalPosition, true, present)) {
          fail(id_text(id) + ": the goal does not read back " + std::to_string(present) +
            "; torque left off");
          break;
        }
      } else if (mode == 1) {
        write.word(id, kRegGoalSpeed, read_word(bus, id, kRegGoalSpeed), 0);
        if (!reads_back(bus, id, kRegGoalSpeed, true, 0)) {
          fail(id_text(id) + ": the goal speed does not read back 0; torque left off");
          break;
        }
      } else {
        fail(
          id_text(id) + " is in mode " + std::to_string(mode) + ": restore sets a goal only in "
          "modes 0 and 1, so its torque is left off");
        break;
      }
    }
    write.byte(id, kRegTorque, torque, want.reg(kRegTorque));
    if (!reads_back(bus, id, kRegTorque, false, want.reg(kRegTorque))) {
      fail(id_text(id) + " reg 40 does not read back " + value_text(want.reg(kRegTorque)));
      break;
    }
  }
  signals.end();
  report.signal_deferred = !report.writes.empty() && stopped(stop);

  // 6. a fresh snapshot, census included, compared with the source; nothing interrupts it
  bool ignored = false;
  const Snapshot fresh = take_snapshot(bus, expected, true, nullptr, &ignored);
  report.after = compare(source, fresh, false);
  if (!failed && !report.after.empty()) {
    report.problems.push_back(
      "a fresh snapshot still differs from the source in " + std::to_string(report.after.size()) +
      " place(s)");
  }
  report.exit = (failed || !report.after.empty()) ? kExitCannotResolve : kExitOk;
  return report;
}

// ---- blockcheck ----

BlockcheckReport blockcheck(
  ServoBus & bus, const std::vector<int> & ids, int first, int count,
  const volatile std::sig_atomic_t * stop)
{
  BlockcheckReport report;
  if (!bus.is_open()) {
    report.exit = kExitCannotOpen;
    report.unreadable.push_back("the bus is not open");
    return report;
  }
  if (count < 1 || count > kMaxBlockBytes || first < 0 || first + count - 1 > 255) {
    report.exit = kExitUsage;
    return report;
  }
  for (const int id : ids) {
    if (stopped(stop)) {
      report.exit = kExitInterrupted;
      return report;
    }
    // one block read of the whole range (retried once, like every read here), then each register
    // on its own
    std::vector<uint8_t> block(static_cast<std::size_t>(count));
    bool block_read = false;
    for (int attempt = 0; attempt < kReadAttempts && !block_read; attempt++) {
      block_read = bus.Read(
        static_cast<uint8_t>(id), static_cast<uint8_t>(first), block.data(),
        static_cast<uint8_t>(count)) == count;
    }
    if (!block_read) {
      report.unreadable.push_back(
        id_text(id) + " block " + std::to_string(first) + ".." + std::to_string(first + count - 1));
    }
    for (int k = 0; k < count; k++) {
      const int single = read_byte(bus, id, first + k);
      if (single == kUnreadable) {
        report.unreadable.push_back(id_text(id) + " reg " + std::to_string(first + k));
      } else if (block_read && block[static_cast<std::size_t>(k)] != single) {
        report.mismatches.push_back(
          BlockMismatch{id, first + k, block[static_cast<std::size_t>(k)], single});
      }
    }
  }
  report.ok = report.unreadable.empty() && report.mismatches.empty();
  if (!report.unreadable.empty()) {
    report.exit = kExitCannotResolve;
  } else {
    report.exit = report.mismatches.empty() ? kExitOk : kExitBlockMismatch;
  }
  return report;
}

// ---- drift ----

DriftReport drift(ServoBus & bus, int id, double seconds, const volatile std::sig_atomic_t * stop)
{
  DriftReport report;
  if (!bus.is_open()) {
    report.exit = kExitCannotOpen;
    report.problem = "the bus is not open";
    return report;
  }
  const int mode = read_byte(bus, id, kRegMode);
  if (mode != 0) {
    report.exit = kExitCannotResolve;
    report.problem = mode == kUnreadable ?
      "cannot read the mode of " + id_text(id) + "; nothing was written" :
      id_text(id) + " is in mode " + std::to_string(mode) + ": drift measures a position servo " +
      "(mode 0) only; nothing was written";
    return report;
  }
  report.torque_before = read_byte(bus, id, kRegTorque);
  report.goal_before = read_word(bus, id, kRegGoalPosition);
  const int position = read_word(bus, id, kRegPresentPosition);
  if (report.torque_before == kUnreadable || report.goal_before == kUnreadable ||
    position == kUnreadable)
  {
    report.exit = kExitCannotResolve;
    report.problem = "cannot read registers 40, 42-43 and 56 of " + id_text(id) +
      "; nothing was written";
    return report;
  }
  if (stopped(stop)) {
    report.exit = kExitInterrupted;
    report.problem = "interrupted before the first write; nothing was written";
    return report;
  }

  DeferredSignals signals;
  Writer write(bus, &report.writes, &signals);
  bool torque_off = true;
  if (report.torque_before != 0) {
    write.byte(id, kRegTorque, report.torque_before, 0);
    report.torque_written = true;
    torque_off = reads_back(bus, id, kRegTorque, false, 0);
    if (!torque_off) {
      report.problem = "the torque did not switch off (register 40 does not read back 0); "
        "nothing was sampled";
    }
  }
  if (torque_off) {
    const auto t0 = Clock::now();
    const auto window = std::chrono::duration<double>(seconds);
    for (int k = 0; ; k++) {
      const auto due = t0 + std::chrono::milliseconds(static_cast<int64_t>(k) * kDriftSampleMs);
      if (due - t0 > window) {
        break;
      }
      std::this_thread::sleep_until(due);
      report.samples.push_back(read_word(bus, id, kRegPresentPosition));
    }
  }
  // torque back as it was: the goal at the present position first, then register 40, both read
  // back. A servo that had torque off gets no write at all.
  if (report.torque_before != 0) {
    const int present = read_word(bus, id, kRegPresentPosition);
    if (present == kUnreadable) {
      report.problem = "cannot read the position to put the goal at; torque left OFF";
    } else {
      write.word(id, kRegGoalPosition, report.goal_before, present);
      const bool goal = reads_back(bus, id, kRegGoalPosition, true, present);
      write.byte(id, kRegTorque, 0, report.torque_before);
      const bool torque = reads_back(bus, id, kRegTorque, false, report.torque_before);
      report.restored = goal && torque;
      if (!report.restored) {
        report.problem = "the goal or the torque does not read back as written";
      }
    }
  } else {
    report.restored = read_byte(bus, id, kRegTorque) == 0;
  }
  report.goal_after = read_word(bus, id, kRegGoalPosition);
  report.torque_after = read_byte(bus, id, kRegTorque);
  signals.end();
  report.signal_deferred = !report.writes.empty() && stopped(stop);

  // positions unwrapped around the first readable sample, so a sag across 0/4095 is not a jump
  const int64_t last_ms = report.samples.empty() ? 0 :
    static_cast<int64_t>(report.samples.size() - 1) * kDriftSampleMs;
  int base = kUnreadable;
  bool all_read = !report.samples.empty();
  int total_min = INT_MAX;
  int total_max = INT_MIN;
  int last_min = INT_MAX;
  int last_max = INT_MIN;
  for (std::size_t k = 0; k < report.samples.size(); k++) {
    const int sample = report.samples[k];
    if (sample == kUnreadable) {
      all_read = false;
      continue;
    }
    if (base == kUnreadable) {
      base = sample;
    }
    int step = (sample - base) % 4096;
    if (step > 2048) {
      step -= 4096;
    } else if (step < -2048) {
      step += 4096;
    }
    const int unwrapped = base + step;
    total_min = std::min(total_min, unwrapped);
    total_max = std::max(total_max, unwrapped);
    if (static_cast<int64_t>(k) * kDriftSampleMs >= last_ms - 1000) {
      last_min = std::min(last_min, unwrapped);
      last_max = std::max(last_max, unwrapped);
    }
  }
  report.drift_total_ticks = base == kUnreadable ? -1 : total_max - total_min;
  report.drift_last_1s_ticks = base == kUnreadable ? -1 : last_max - last_min;
  if (!all_read && report.problem.empty()) {
    report.problem = "a position sample could not be read";
  }
  report.ok = torque_off && all_read && report.restored;
  report.exit = report.ok ? kExitOk : kExitCannotResolve;
  return report;
}

// ---- read ----

ReadReport read_register(ServoBus & bus, int id, int address, bool word)
{
  ReadReport report;
  if (!bus.is_open()) {
    report.exit = kExitCannotOpen;
    return report;
  }
  report.value = word ? read_word(bus, id, address) : read_byte(bus, id, address);
  report.ok = report.value != kUnreadable;
  report.exit = report.ok ? kExitOk : kExitCannotResolve;
  return report;
}

// ---- the command line ----

namespace
{

constexpr const char * kUsage =
  "usage: hil_eeprom --port P snapshot --ids LIST [--census] --out FILE\n"
  "       hil_eeprom compare A.snap B.snap [--eeprom-only]\n"
  "       hil_eeprom --port P restore --from A.snap [--allow-regs LIST]\n"
  "       hil_eeprom --port P blockcheck --ids LIST --first F --count C\n"
  "       hil_eeprom --port P drift --id I --seconds S\n"
  "       hil_eeprom --port P read --id I --addr A [--word]\n"
  "ids are 0..253; LIST is comma-separated. Exit: 0 ok or equal; 1 port held, not equal or a\n"
  "block mismatch; 2 cannot open; 3 cannot resolve, verify or read; 64 usage; 130 interrupted\n"
  "before the first write.\n";

int finish(std::ostream & out, const std::string & json, int code)
{
  out << "RESULT " << json << std::endl;
  return code;
}

int usage_error(std::ostream & out, const std::string & message, bool compare_result)
{
  out << "hil_eeprom: " << message << "\n" << kUsage;
  return finish(
    out, std::string("{\"") + (compare_result ? "equal" : "ok") +
    "\": false, \"error\": \"usage\", \"message\": " + json_string(message) + "}", kExitUsage);
}

int failure(std::ostream & out, const std::string & error, const std::string & message, int code)
{
  out << "hil_eeprom: " << message << "\n";
  return finish(
    out, "{\"ok\": false, \"error\": " + json_string(error) + ", \"message\": " +
    json_string(message) + "}", code);
}

// The options of one command: `--name value` for each of `valued`, `--name` for each of `flags`,
// each at most once, nothing else. Positional arguments go to `positional`.
struct Options
{
  std::map<std::string, std::string> values;
  std::set<std::string> flags;
  std::vector<std::string> positional;
};

bool parse_options(
  const std::vector<std::string> & args, const std::set<std::string> & valued,
  const std::set<std::string> & flags, Options * options, std::string * error)
{
  for (std::size_t i = 0; i < args.size(); i++) {
    const std::string & arg = args[i];
    if (arg.rfind("--", 0) != 0) {
      options->positional.push_back(arg);
    } else if (valued.count(arg) != 0) {
      if (i + 1 >= args.size()) {
        *error = arg + " needs a value";
        return false;
      }
      if (!options->values.emplace(arg, args[++i]).second) {
        *error = arg + " given twice";
        return false;
      }
    } else if (flags.count(arg) != 0) {
      if (!options->flags.insert(arg).second) {
        *error = arg + " given twice";
        return false;
      }
    } else {
      *error = "unknown option '" + arg + "'";
      return false;
    }
  }
  return true;
}

bool read_text(const std::string & path, std::string * text)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  *text = buffer.str();
  return !in.bad();
}

bool load_snap(const std::string & path, Snapshot * snap, std::string * error)
{
  std::string text;
  if (!read_text(path, &text)) {
    *error = "cannot read '" + path + "'";
    return false;
  }
  std::string why;
  if (!parse_snap(text, snap, &why)) {
    *error = "'" + path + "' is not a usable snapshot: " + why;
    return false;
  }
  return true;
}

int run_compare(const std::vector<std::string> & args, std::ostream & out)
{
  Options options;
  std::string error;
  if (!parse_options(args, {}, {"--eeprom-only"}, &options, &error)) {
    return usage_error(out, error, true);
  }
  if (options.positional.size() != 2) {
    return usage_error(out, "compare takes exactly two snapshot files", true);
  }
  Snapshot a;
  Snapshot b;
  if (!load_snap(options.positional[0], &a, &error) ||
    !load_snap(options.positional[1], &b, &error))
  {
    out << "hil_eeprom: " << error << "\n";
    return finish(
      out, "{\"equal\": false, \"error\": \"unreadable\", \"message\": " + json_string(error) +
      "}", kExitCannotResolve);
  }
  const bool eeprom_only = options.flags.count("--eeprom-only") != 0;
  const std::vector<Diff> diffs = compare(a, b, eeprom_only);
  for (const Diff & diff : diffs) {
    out << diff_text(diff) << "\n";
  }
  const std::string verdict =
    diffs.empty() ? "equal" : std::to_string(diffs.size()) + " difference(s)";
  out << verdict << " (" << (eeprom_only ? "EEPROM and census" : "EEPROM, 40, 55 and census") <<
    ")\n";
  return finish(
    out, std::string("{\"equal\": ") + (diffs.empty() ? "true" : "false") +
    ", \"eeprom_only\": " + (eeprom_only ? "true" : "false") + ", \"diffs\": " +
    diffs_json(diffs) + "}", diffs.empty() ? kExitOk : kExitNotEqual);
}

// Takes the port the way the tools do. On a refusal it has printed its RESULT and returns the
// exit code; 0 means the bus is open.
int open_port(ServoBus & bus, const std::string & port, std::ostream & out)
{
  const OpenResult opened = bus.open(port, kBaudrate, kIoTimeoutMs);
  if (opened) {
    std::vector<int> others;
    for (const int pid : port_holder_pids(port)) {
      if (pid != ::getpid()) {
        others.push_back(pid);
      }
    }
    if (!others.empty()) {
      out << "warning: pid(s) " << spaced(others) << " had '" << port <<
        "' open before hil_eeprom took it; they hold no lock and can still write to the bus\n";
    }
    return 0;
  }
  const bool held = opened.status == BusStatus::LOCK_FAILED ||
    (opened.status == BusStatus::LOCK_OPEN_FAILED && opened.error == EBUSY);
  std::string message = "cannot take '" + port + "': " + to_string(opened.status);
  if (opened.error != 0) {
    message += std::string(": ") + std::strerror(opened.error);
  }
  if (held) {
    std::vector<int> others;
    for (const int pid : port_holder_pids(port)) {
      if (pid != ::getpid()) {
        others.push_back(pid);
      }
    }
    message = "port '" + port + "' is held by another process" +
      (others.empty() ? std::string() : " (pid " + spaced(others) + ")") +
      "; nothing was sent";
  }
  return failure(out, held ? "port_held" : "cannot_open", message,
           held ? kExitPortHeld : kExitCannotOpen);
}

std::string servo_summary(int id, const ServoRecord & servo)
{
  std::ostringstream line;
  const auto position = servo.volatiles.find(kRegPresentPosition);
  line << "id " << id << ": " << servo.readable_eeprom() << "/39 EEPROM bytes, firmware " <<
    value_text(servo.reg(0)) << "." << value_text(servo.reg(1)) << ", model " <<
    value_text(servo.reg(3)) << "/" << value_text(servo.reg(4)) << ", id register " <<
    value_text(servo.reg(kRegId)) << ", baud register " << value_text(servo.reg(kRegBaud)) <<
    ", mode " << value_text(servo.reg(kRegMode)) << ", offset raw " <<
    value_text(word_of(servo, kRegOffset)) << ", torque " << value_text(servo.reg(kRegTorque)) <<
    ", lock " << value_text(servo.reg(kRegLock)) << ", position " <<
    value_text(position == servo.volatiles.end() ? kUnreadable : position->second);
  return line.str();
}

int run_snapshot(
  ServoBus & bus, const std::vector<int> & ids, bool with_census, const std::string & path,
  std::ostream & out, const volatile std::sig_atomic_t * stop)
{
  bool interrupted = false;
  const Snapshot snap = take_snapshot(bus, ids, with_census, stop, &interrupted);
  if (interrupted) {
    return failure(
      out, "interrupted", "interrupted before the snapshot was complete; nothing was written",
      kExitInterrupted);
  }
  if (snap.has_census) {
    out << "census: " << (snap.census.empty() ? "no id answered" : spaced(snap.census)) <<
      " (pinged ids " << kCensusFirstId << ".." << kCensusLastId << ", " << kCensusAttempts <<
      " attempts at " << kCensusTimeoutMs << " ms)\n";
  }
  for (const auto & entry : snap.servos) {
    out << servo_summary(entry.first, entry.second) << "\n";
  }
  std::ofstream file(path, std::ios::trunc);
  file << format_snap(snap);
  file.close();
  if (!file) {
    out << "hil_eeprom: cannot write '" << path << "'\n";
    return finish(out, snapshot_json(snap), kExitCannotResolve);
  }
  out << "wrote " << path << (snap.ok ? "" : " (NOT ok: it holds unreadable bytes, 'x')") << "\n";
  return finish(out, snapshot_json(snap), snap.ok ? kExitOk : kExitCannotResolve);
}

std::string writes_json(const std::vector<Written> & writes)
{
  std::string json = "[";
  for (std::size_t i = 0; i < writes.size(); i++) {
    const Written & w = writes[i];
    json += (i == 0 ? "" : ", ") + std::string("{\"id\": ") + std::to_string(w.id) +
      ", \"reg\": " + std::to_string(w.reg) + ", \"from\": " + json_value(w.from) + ", \"to\": " +
      std::to_string(w.to) + ", \"bytes\": " + std::to_string(w.bytes) + ", \"kind\": \"" +
      (w.eeprom ? "eeprom" : "sram") + "\"}";
  }
  return json + "]";
}

std::string write_text(const Written & w)
{
  return "wrote id " + std::to_string(w.id) + " reg " + std::to_string(w.reg) +
         (w.bytes == 2 ? " (word)" : "") + ": " + value_text(w.from) + " -> " +
         std::to_string(w.to) + (w.eeprom ? " [eeprom]" : " [sram]");
}

int run_restore(
  ServoBus & bus, const Snapshot & source, const RestoreOptions & options, std::ostream & out,
  const volatile std::sig_atomic_t * stop)
{
  const RestoreReport report = restore(bus, source, options, stop);
  for (const Diff & diff : report.before) {
    out << "differs: " << diff_text(diff) << " (source -> bench)\n";
  }
  if (report.moved_from != -1) {
    out << "moved the servo at id " << report.moved_from << " back to id " << report.moved_to <<
      "\n";
  }
  for (const Written & w : report.writes) {
    out << write_text(w) << "\n";
  }
  for (const Diff & diff : report.after) {
    out << "still differs: " << diff_text(diff) << " (source -> bench)\n";
  }
  for (const std::string & problem : report.problems) {
    out << "hil_eeprom: " << problem << "\n";
  }
  if (report.signal_deferred) {
    out << "hil_eeprom: a signal arrived during the restore; it was completed first\n";
  }
  if (report.exit == kExitOk && report.writes.empty()) {
    out << "the bench already matches the source; nothing written\n";
  } else if (report.exit == kExitOk) {
    out << "restored; a fresh snapshot matches the source\n";
  }
  std::string moved = "null";
  if (report.moved_from != -1) {
    moved = "{\"from\": " + std::to_string(report.moved_from) + ", \"to\": " +
      std::to_string(report.moved_to) + "}";
  }
  return finish(
    out, std::string("{\"ok\": ") + (report.exit == kExitOk ? "true" : "false") +
    ", \"exit\": " + std::to_string(report.exit) + ", \"moved\": " + moved + ", \"writes\": " +
    writes_json(report.writes) + ", \"before\": " + diffs_json(report.before) + ", \"after\": " +
    diffs_json(report.after) + ", \"problems\": " + json_strings(report.problems) +
    ", \"signal_deferred\": " + (report.signal_deferred ? "true" : "false") + "}", report.exit);
}

int run_blockcheck(
  ServoBus & bus, const std::vector<int> & ids, int first, int count, std::ostream & out,
  const volatile std::sig_atomic_t * stop)
{
  const BlockcheckReport report = blockcheck(bus, ids, first, count, stop);
  for (const std::string & what : report.unreadable) {
    out << "unreadable: " << what << "\n";
  }
  std::string mismatches = "[";
  for (std::size_t i = 0; i < report.mismatches.size(); i++) {
    const BlockMismatch & m = report.mismatches[i];
    out << "id " << m.id << " reg " << m.reg << ": block " << value_text(m.block) <<
      ", single " << value_text(m.single) << "\n";
    mismatches += (i == 0 ? "" : ", ") + std::string("{\"id\": ") + std::to_string(m.id) +
      ", \"reg\": " + std::to_string(m.reg) + ", \"block\": " + json_value(m.block) +
      ", \"single\": " + json_value(m.single) + "}";
  }
  mismatches += "]";
  out << (report.ok ? "every block equals its single reads" : "blockcheck FAILED") << " (ids " <<
    spaced(ids) << ", registers " << first << ".." << first + count - 1 << ")\n";
  return finish(
    out, std::string("{\"ok\": ") + (report.ok ? "true" : "false") + ", \"first\": " +
    std::to_string(first) + ", \"count\": " + std::to_string(count) + ", \"ids\": " +
    json_ints(ids) + ", \"unreadable\": " + json_strings(report.unreadable) +
    ", \"mismatches\": " + mismatches + "}", report.exit);
}

int run_drift(
  ServoBus & bus, int id, double seconds, std::ostream & out,
  const volatile std::sig_atomic_t * stop)
{
  const DriftReport report = drift(bus, id, seconds, stop);
  if (!report.problem.empty()) {
    out << "hil_eeprom: " << report.problem << "\n";
  }
  if (report.signal_deferred) {
    out << "hil_eeprom: a signal arrived during the drift probe; it was completed first\n";
  }
  out << "id " << id << ": torque_before " << value_text(report.torque_before) << ", " <<
    report.samples.size() << " samples every " << kDriftSampleMs << " ms, drift " <<
    report.drift_total_ticks << " ticks in all, " << report.drift_last_1s_ticks <<
    " in the last second, restored " << (report.restored ? "yes" : "NO") << "\n";
  std::string samples = "[";
  for (std::size_t i = 0; i < report.samples.size(); i++) {
    samples += (i == 0 ? "" : ", ") + json_value(report.samples[i]);
  }
  samples += "]";
  return finish(
    out, std::string("{\"ok\": ") + (report.ok ? "true" : "false") + ", \"id\": " +
    std::to_string(id) + ", \"torque_before\": " + json_value(report.torque_before) +
    ", \"torque_written\": " + (report.torque_written ? "true" : "false") +
    ", \"goal_before\": " + json_value(report.goal_before) + ", \"sample_ms\": " +
    std::to_string(kDriftSampleMs) + ", \"samples\": " + samples + ", \"drift_total_ticks\": " +
    std::to_string(report.drift_total_ticks) + ", \"drift_last_1s_ticks\": " +
    std::to_string(report.drift_last_1s_ticks) + ", \"goal_after\": " +
    json_value(report.goal_after) + ", \"torque_after\": " + json_value(report.torque_after) +
    ", \"restored\": " + (report.restored ? "true" : "false") + ", \"writes\": " +
    writes_json(report.writes) + ", \"problem\": " +
    (report.problem.empty() ? "null" : json_string(report.problem)) + "}", report.exit);
}

int run_read(ServoBus & bus, int id, int address, bool word, std::ostream & out)
{
  const ReadReport report = read_register(bus, id, address, word);
  out << "id " << id << " " << (word ? "word" : "byte") << " at " << address << ": " <<
    value_text(report.value) << "\n";
  return finish(
    out, std::string("{\"ok\": ") + (report.ok ? "true" : "false") + ", \"id\": " +
    std::to_string(id) + ", \"addr\": " + std::to_string(address) + ", \"word\": " +
    (word ? "true" : "false") + ", \"value\": " +
    (report.ok ? std::to_string(report.value) : "null") + "}", report.exit);
}

}  // namespace

int run(
  const std::vector<std::string> & args, std::ostream & out,
  const volatile std::sig_atomic_t * stop)
{
  std::size_t next = 0;
  std::string port;
  bool has_port = false;
  while (next < args.size() && args[next] == "--port") {
    if (has_port || next + 1 >= args.size() || args[next + 1].empty()) {
      return usage_error(out, "--port needs one non-empty value, given once", false);
    }
    port = args[next + 1];
    has_port = true;
    next += 2;
  }
  if (next >= args.size()) {
    return usage_error(out, "no command", false);
  }
  const std::string command = args[next];
  const std::vector<std::string> rest(
    args.begin() + static_cast<std::ptrdiff_t>(next) + 1, args.end());
  if (command == "compare") {
    if (has_port) {
      return usage_error(out, "compare opens no port; give it no --port", true);
    }
    return run_compare(rest, out);
  }

  // Everything is validated before the port is opened: a usage error never touches the bus.
  Options options;
  std::string error;
  std::vector<int> ids;
  int id = 0;
  int first = 0;
  int count = 0;
  int address = 0;
  double seconds = 0.0;
  Snapshot source;
  RestoreOptions restore_options;
  bool parsed = false;
  if (command == "snapshot") {
    parsed = parse_options(rest, {"--ids", "--out"}, {"--census"}, &options, &error);
    if (parsed && (options.values.count("--ids") == 0 || options.values.count("--out") == 0)) {
      error = "snapshot needs --ids and --out";
      parsed = false;
    }
    if (parsed && !parse_int_list(options.values["--ids"], kCensusFirstId, kCensusLastId, &ids)) {
      error = "--ids must be a comma-separated list of ids 0..253";
      parsed = false;
    }
  } else if (command == "restore") {
    parsed = parse_options(rest, {"--from", "--allow-regs"}, {}, &options, &error);
    if (parsed && options.values.count("--from") == 0) {
      error = "restore needs --from";
      parsed = false;
    }
    std::vector<int> regs;
    if (parsed && options.values.count("--allow-regs") != 0) {
      if (!parse_int_list(options.values["--allow-regs"], 0, 255, &regs)) {
        error = "--allow-regs must be a comma-separated list of registers 0..255";
        parsed = false;
      }
      restore_options.limit_regs = true;
      restore_options.allow_regs.insert(regs.begin(), regs.end());
    }
  } else if (command == "blockcheck") {
    parsed = parse_options(rest, {"--ids", "--first", "--count"}, {}, &options, &error);
    if (parsed && (options.values.count("--ids") == 0 || options.values.count("--first") == 0 ||
      options.values.count("--count") == 0))
    {
      error = "blockcheck needs --ids, --first and --count";
      parsed = false;
    }
    if (parsed && (!parse_int_list(options.values["--ids"], kCensusFirstId, kCensusLastId, &ids) ||
      !parse_int(options.values["--first"], 0, 255, &first) ||
      !parse_int(options.values["--count"], 1, kMaxBlockBytes, &count) ||
      first + count - 1 > 255))
    {
      error = "--ids must be ids 0..253, --first 0..255 and --count 1..64 within 255";
      parsed = false;
    }
  } else if (command == "drift") {
    parsed = parse_options(rest, {"--id", "--seconds"}, {}, &options, &error);
    if (parsed && (options.values.count("--id") == 0 || options.values.count("--seconds") == 0)) {
      error = "drift needs --id and --seconds";
      parsed = false;
    }
    char * end = nullptr;
    const std::string & text = options.values["--seconds"];
    seconds = parsed ? std::strtod(text.c_str(), &end) : 0.0;
    if (parsed && (!parse_int(options.values["--id"], kCensusFirstId, kCensusLastId, &id) ||
      end == text.c_str() || *end != '\0' || !(seconds > 0.0 && seconds <= 60.0)))
    {
      error = "--id must be 0..253 and --seconds a number of seconds in (0, 60]";
      parsed = false;
    }
  } else if (command == "read") {
    parsed = parse_options(rest, {"--id", "--addr"}, {"--word"}, &options, &error);
    if (parsed && (options.values.count("--id") == 0 || options.values.count("--addr") == 0)) {
      error = "read needs --id and --addr";
      parsed = false;
    }
    if (parsed && (!parse_int(options.values["--id"], kCensusFirstId, kCensusLastId, &id) ||
      !parse_int(options.values["--addr"], 0, 255, &address)))
    {
      error = "--id must be 0..253 and --addr 0..255";
      parsed = false;
    }
  } else {
    return usage_error(out, "unknown command '" + command + "'", false);
  }
  if (!parsed) {
    return usage_error(out, error, false);
  }
  if (!options.positional.empty()) {
    return usage_error(out, "unexpected argument '" + options.positional[0] + "'", false);
  }
  if (!has_port) {
    return usage_error(out, command + " needs --port", false);
  }
  if (command == "snapshot" || command == "blockcheck") {
    const std::set<int> unique(ids.begin(), ids.end());
    if (unique.size() != ids.size()) {
      return usage_error(out, "--ids names an id twice", false);
    }
  }
  if (command == "restore") {
    // E.1 restore step 0, before the port is opened: a source restore would have to guess from
    // is refused with nothing sent
    if (!load_snap(options.values["--from"], &source, &error)) {
      return failure(out, "unreadable_source", error, kExitCannotResolve);
    }
    const std::vector<std::string> problems = source_problems(source);
    if (!problems.empty()) {
      for (const std::string & problem : problems) {
        out << "hil_eeprom: " << problem << "\n";
      }
      return finish(
        out, "{\"ok\": false, \"error\": \"refused_source\", \"exit\": 3, \"writes\": [], "
        "\"problems\": " + json_strings(problems) + "}", kExitCannotResolve);
    }
  }
  if (stopped(stop)) {
    return failure(
      out, "interrupted", "interrupted before the port was opened; nothing was sent",
      kExitInterrupted);
  }

  ServoBus bus;
  const int refused = open_port(bus, port, out);
  if (refused != 0) {
    return refused;
  }
  if (command == "snapshot") {
    return run_snapshot(
      bus, ids, options.flags.count("--census") != 0, options.values["--out"], out, stop);
  }
  if (command == "restore") {
    return run_restore(bus, source, restore_options, out, stop);
  }
  if (command == "blockcheck") {
    return run_blockcheck(bus, ids, first, count, out, stop);
  }
  if (command == "drift") {
    return run_drift(bus, id, seconds, out, stop);
  }
  return run_read(bus, id, address, options.flags.count("--word") != 0, out);
}

}  // namespace hil_eeprom
}  // namespace waveshare_servos
