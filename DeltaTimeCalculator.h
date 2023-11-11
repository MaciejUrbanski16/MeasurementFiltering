#pragma once
#include <chrono>

class DeltaTimeCalculator
{
public:
	DeltaTimeCalculator()
	{
		start = std::chrono::high_resolution_clock::now();
	}

	uint32_t getDurationInMs()
	{
		end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		start = std::chrono::high_resolution_clock::now();

		return static_cast<uint32_t>(duration.count());
	}
private:
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;
};