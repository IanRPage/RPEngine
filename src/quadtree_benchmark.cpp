#include <Profiler.hpp>
#include <dsa/QuadTree.hpp>
#include <iostream>
#include <random>
#include <vector>

// Simple particle structure for testing
struct TestParticle {
  struct {
    float x, y;
  } position;

  TestParticle(float x, float y) : position{x, y} {}
};

class QuadTreeBenchmark {
 public:
  QuadTreeBenchmark(size_t num_particles, float world_size,
                    size_t quad_capacity)
      : world_size_(world_size), quad_capacity_(quad_capacity) {
    // Generate random particles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, world_size);

    particles_.reserve(num_particles);
    for (size_t i = 0; i < num_particles; ++i) {
      particles_.emplace_back(dis(gen), dis(gen));
    }

    std::cout << "Generated " << num_particles << " particles in " << world_size
              << "x" << world_size << " world\n";
    std::cout << "QuadTree capacity: " << quad_capacity << "\n\n";
  }

  void runBenchmark() {
    RESET_PROFILE();  // Clear any previous profiling data

    // Test insertion performance
    benchmarkInsertion();

    // Test query performance
    benchmarkQueries();

    // Print comprehensive results
    PRINT_PROFILE();

    // Print custom analysis
    printAnalysis();
  }

 private:
  std::vector<TestParticle> particles_;
  float world_size_;
  size_t quad_capacity_;

  void benchmarkInsertion() {
    std::cout << "Benchmarking insertion of " << particles_.size()
              << " particles...\n";

    START_TIMER("Total_Insertion");

    AABBf world_bounds({0, 0}, {world_size_, world_size_});
    QuadTree<TestParticle> qtree(world_bounds, quad_capacity_);

    for (auto& particle : particles_) {
      qtree.insert(&particle);
    }

    END_TIMER("Total_Insertion");

    std::cout << "Insertion completed.\n";
  }

  void benchmarkQueries() {
    std::cout << "Benchmarking queries...\n";

    // Rebuild the tree for querying
    AABBf world_bounds({0, 0}, {world_size_, world_size_});
    QuadTree<TestParticle> qtree(world_bounds, quad_capacity_);

    for (auto& particle : particles_) {
      qtree.insert(&particle);
    }

    // Generate test query ranges
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dis(0.0f, world_size_ * 0.8f);
    std::uniform_real_distribution<float> size_dis(world_size_ * 0.05f,
                                                   world_size_ * 0.2f);

    const size_t num_queries = 1000;
    std::vector<TestParticle*> results;

    START_TIMER("Total_Queries");

    for (size_t i = 0; i < num_queries; ++i) {
      float x = pos_dis(gen);
      float y = pos_dis(gen);
      float w = size_dis(gen);
      float h = size_dis(gen);

      AABBf query_range({x, y}, {w, h});
      results.clear();

      qtree.query(results, query_range);
    }

    END_TIMER("Total_Queries");

    std::cout << "Completed " << num_queries << " queries.\n";
  }

  void printAnalysis() {
    std::cout << "\n========== PERFORMANCE ANALYSIS ==========\n";

    auto& profiler = Profiler::instance();

    // Insertion analysis
    double total_insertion = profiler.getTotalTime("Total_Insertion");
    size_t insert_calls = profiler.getCallCount("QuadTree::insert");
    size_t subdivide_calls = profiler.getCallCount("QuadTree::subdivide");

    std::cout << "INSERTION:\n";
    std::cout << "  Total time: " << total_insertion << " μs\n";
    std::cout << "  Per particle: " << (total_insertion / particles_.size())
              << " μs\n";
    std::cout << "  Insert calls: " << insert_calls << "\n";
    std::cout << "  Subdivisions: " << subdivide_calls << "\n";

    if (subdivide_calls > 0) {
      double subdivide_time = profiler.getTotalTime("QuadTree::subdivide");
      std::cout << "  Time in subdivide: " << subdivide_time << " μs ("
                << (subdivide_time / total_insertion * 100) << "%)\n";
    }

    // Query analysis
    double total_queries = profiler.getTotalTime("Total_Queries");
    size_t query_calls = profiler.getCallCount("QuadTree::query");
    size_t leaf_checks = profiler.getCallCount("QuadTree::query_leaf_check");

    std::cout << "\nQUERY:\n";
    std::cout << "  Total query time: " << total_queries << " μs\n";
    std::cout << "  Query calls: " << query_calls << "\n";
    std::cout << "  Leaf checks: " << leaf_checks << "\n";
    std::cout << "  Avg per query: " << (total_queries / 1000) << " μs\n";

    if (leaf_checks > 0) {
      double leaf_time = profiler.getTotalTime("QuadTree::query_leaf_check");
      std::cout << "  Time in leaf checks: " << leaf_time << " μs ("
                << (leaf_time / total_queries * 100) << "%)\n";
    }

    std::cout << "==========================================\n";
  }
};

int main() {
  std::cout << "QuadTree Performance Benchmark\n";
  std::cout << "==============================\n\n";

  // Test different configurations
  struct TestConfig {
    size_t particles;
    float world_size;
    size_t capacity;
    std::string description;
  };

  std::vector<TestConfig> configs = {
      {1000, 1000.0f, 4, "Small dataset (1K particles, cap=4)"},
      {10000, 1000.0f, 4, "Medium dataset (10K particles, cap=4)"},
      {10000, 1000.0f, 8, "Medium dataset (10K particles, cap=8)"},
      {10000, 1000.0f, 16, "Medium dataset (10K particles, cap=16)"},
      {50000, 2000.0f, 4, "Large dataset (50K particles, cap=4)"},
      {50000, 2000.0f, 8, "Large dataset (50K particles, cap=8)"},
      {50000, 2000.0f, 16, "Large dataset (50K particles, cap=16)"}};

  for (const auto& config : configs) {
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "TEST: " << config.description << "\n";
    std::cout << std::string(50, '=') << "\n";

    QuadTreeBenchmark benchmark(config.particles, config.world_size,
                                config.capacity);
    benchmark.runBenchmark();

    std::cout << "\nPress Enter to continue to next test...";
    std::cin.get();
  }

  return 0;
}
