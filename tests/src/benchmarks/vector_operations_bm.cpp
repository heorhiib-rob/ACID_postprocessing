/*
 * Author: Jan Chleboun
 * Date: 17-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <benchmark/benchmark.h>

#include <iostream>
#include <vector>

std::vector<int> fill_vector_pushback(const int n)
{
  std::vector<int> vec;
  vec.reserve(n);
  for (int i = 0; i < n; i++)
  {
    vec.push_back(rand());
  }
  return vec;
}

std::vector<int> fill_vector_emplaceback(const int n)
{
  std::vector<int> vec;
  vec.reserve(n);
  for (int i = 0; i < n; i++)
  {
    vec.emplace_back(rand());
  }
  return vec;
}

std::vector<int> fill_vector_resize(const int n)
{
  std::vector<int> vec;
  vec.resize(n, 0);
  for (int i = 0; i < n; i++)
  {
    vec[i] = rand();
  }
  return vec;
}

std::vector<int> fill_vector_initialize(const int n)
{
  std::vector<int> vec(n, 0);
  for (int i = 0; i < n; i++)
  {
    vec[i] = rand();
  }
  return vec;
}

// benchmark push_back
static void BM_fill_vector_pushback(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::vector<int> v = fill_vector_pushback(state.range(0));
  }
}
BENCHMARK(BM_fill_vector_pushback)->Range(100, 100000)->RangeMultiplier(10);

// benchmark emplace_back
static void BM_fill_vector_emplaceback(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::vector<int> v = fill_vector_pushback(state.range(0));
  }
}
BENCHMARK(BM_fill_vector_emplaceback)->Range(100, 100000)->RangeMultiplier(10);

// benchmark resize
static void BM_fill_vector_resize(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::vector<int> v = fill_vector_resize(state.range(0));
  }
}
BENCHMARK(BM_fill_vector_resize)->Range(100, 100000)->RangeMultiplier(10);

// benchmark init
static void BM_fill_vector_initialize(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::vector<int> v = fill_vector_initialize(state.range(0));
  }
}
BENCHMARK(BM_fill_vector_initialize)->Range(100, 100000)->RangeMultiplier(10);

BENCHMARK_MAIN();
