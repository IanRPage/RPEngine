#ifndef PROFILER_HPP
#define PROFILER_HPP

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>

class Profiler {
 public:
  static Profiler& instance() {
    static Profiler prof;
    return prof;
  }

  void startTimer(const std::string& name) {
    start_times_[name] = std::chrono::high_resolution_clock::now();
  }

  void endTimer(const std::string& name) {
    auto end = std::chrono::high_resolution_clock::now();
    auto start_it = start_times_.find(name);

    if (start_it != start_times_.end()) {
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                          end - start_it->second)
                          .count();

      // Accumulate timing data
      if (timings_.find(name) == timings_.end()) {
        timings_[name] = {0.0, 0, 0.0, 999999999.0};  // total, count, max, min
      }

      auto& timing = timings_[name];
      timing.total += duration;
      timing.count++;
      timing.max = std::max(timing.max, static_cast<double>(duration));
      timing.min = std::min(timing.min, static_cast<double>(duration));

      start_times_.erase(start_it);
    }
  }

  void printResults() const {
    std::cout << "\n========== PROFILER RESULTS ==========\n";
    // Set up table headers with proper widths
    std::cout << std::left << std::setw(25) << "Function" << std::right
              << std::setw(8) << "Count" << std::setw(14) << "Total(μs)"
              << std::setw(12) << "Avg(μs)" << std::setw(12) << "Min(μs)"
              << std::setw(12) << "Max(μs)" << "\n";
    std::cout << std::string(83, '-') << "\n";

    // Format data rows with consistent alignment
    for (const auto& [name, timing] : timings_) {
      double avg = timing.total / timing.count;

      // Truncate function names if too long
      std::string func_name = name;
      if (func_name.length() > 24) {
        func_name = func_name.substr(0, 21) + "...";
      }

      std::cout << std::left << std::setw(25) << func_name << std::right
                << std::setw(8) << timing.count << std::setw(14) << std::fixed
                << std::setprecision(1) << timing.total << std::setw(12)
                << std::fixed << std::setprecision(2) << avg << std::setw(12)
                << std::fixed << std::setprecision(2) << timing.min
                << std::setw(12) << std::fixed << std::setprecision(2)
                << timing.max << "\n";
    }
    std::cout << std::string(83, '=') << "\n\n";
  }

  void reset() {
    timings_.clear();
    start_times_.clear();
  }

  // Get specific timing data for programmatic access
  double getTotalTime(const std::string& name) const {
    auto it = timings_.find(name);
    return (it != timings_.end()) ? it->second.total : 0.0;
  }

  double getAverageTime(const std::string& name) const {
    auto it = timings_.find(name);
    return (it != timings_.end()) ? it->second.total / it->second.count : 0.0;
  }

  size_t getCallCount(const std::string& name) const {
    auto it = timings_.find(name);
    return (it != timings_.end()) ? it->second.count : 0;
  }

 private:
  struct TimingData {
    double total;
    size_t count;
    double max;
    double min;
  };

  std::unordered_map<std::string,
                     std::chrono::high_resolution_clock::time_point>
      start_times_;
  std::unordered_map<std::string, TimingData> timings_;
};

// RAII timer class for automatic timing
class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& name) : name_(name) {
    Profiler::instance().startTimer(name_);
  }

  ~ScopedTimer() { Profiler::instance().endTimer(name_); }

 private:
  std::string name_;
};

// Conditional profiling - set to 0 to disable all profiling
#ifndef ENABLE_PROFILING
#define ENABLE_PROFILING 1
#endif

#if ENABLE_PROFILING
    // Profiling enabled - normal macros
    #define PROFILE_FUNCTION() ScopedTimer _timer(__FUNCTION__)
    #define PROFILE_SCOPE(name) ScopedTimer _timer(name)
    #define START_TIMER(name) Profiler::instance().startTimer(name)
    #define END_TIMER(name) Profiler::instance().endTimer(name)
    #define PRINT_PROFILE() Profiler::instance().printResults()
    #define RESET_PROFILE() Profiler::instance().reset()
#else
    // Profiling disabled - empty macros (zero overhead)
    #define PROFILE_FUNCTION()
    #define PROFILE_SCOPE(name)
    #define START_TIMER(name)
    #define END_TIMER(name)
    #define PRINT_PROFILE()
    #define RESET_PROFILE()
#endif

// Convenience macros for common profiling scenarios
#define PROFILE_BLOCK(name, code) { PROFILE_SCOPE(name); code; }

#endif
