#include <Profiler.hpp>
#include <Simulator.hpp>
#include <chrono>
#include <iostream>
#include <ui/Renderer.hpp>

int main() {
  std::cout << "Starting Profiled Simulation\n";
  std::cout << "============================\n";

  // Initialize simulation
  Simulator sim({0.0f, 0.0f}, 5.0f, 0.0f, 0.0f, 0.0, IntegrationType::Verlet);

  Renderer renderer(sim, Renderer::Options{60, "RPEngine2D - Profiled"});

  // Profiling state
  auto lastProfileReport = std::chrono::steady_clock::now();
  const auto reportInterval =
      std::chrono::seconds(5);  // Report every 5 seconds
  size_t frameCount = 0;
  auto startTime = std::chrono::steady_clock::now();

  std::cout << "Simulation running. Performance reports every 5 seconds.\n";
  std::cout << "Close the window to see final statistics.\n\n";

  // Main simulation loop
  while (renderer.isOpen()) {
    PROFILE_SCOPE("Main::frame");

    {
      PROFILE_SCOPE("Main::event_handling");
      renderer.pollAndHandleEvents();
    }

    {
      PROFILE_SCOPE("Main::simulation_update");
      sim.update();
    }

    {
      PROFILE_SCOPE("Main::rendering");
      renderer.drawFrame();
    }

    frameCount++;

    // Periodic profiling report
    auto now = std::chrono::steady_clock::now();
    if (now - lastProfileReport >= reportInterval) {
      auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime)
              .count();

      std::cout << "\n" << std::string(60, '=') << "\n";
      std::cout << "PERFORMANCE REPORT - " << elapsed << "ms elapsed\n";
      std::cout << "Frames: " << frameCount
                << " | Particles: " << sim.getParticles().size()
                << " | FPS: " << (frameCount * 1000.0 / elapsed) << "\n";
      std::cout << std::string(60, '=') << "\n";

      // Print profiling data
      Profiler::instance().printResults();

      // Reset profiler for next interval
      Profiler::instance().reset();
      lastProfileReport = now;
    }
  }

  // Final report
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "FINAL SIMULATION STATISTICS\n";
  std::cout << std::string(60, '=') << "\n";

  auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - startTime)
                       .count();

  std::cout << "Total runtime: " << totalTime << " ms\n";
  std::cout << "Total frames: " << frameCount << "\n";
  std::cout << "Average FPS: " << (frameCount * 1000.0 / totalTime) << "\n";
  std::cout << "Final particle count: " << sim.getParticles().size() << "\n";

  // Print final profiling data if any
  if (Profiler::instance().getCallCount("Main::frame") > 0) {
    std::cout << "\nFinal profiling data:\n";
    Profiler::instance().printResults();
  }

  std::cout << "\nSimulation completed successfully!\n";
  return 0;
}
