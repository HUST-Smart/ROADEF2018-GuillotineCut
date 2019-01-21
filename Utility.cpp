#include "Utility.h"

#if _OS_MS_WINDOWS
#include <Windows.h>
#include <Psapi.h>
#else
// EXTEND[szx][9]: get memory usage on *nix.
#endif // _OS_MS_WINDOWS


using namespace std;


namespace szx {

System::MemoryUsage System::memoryUsage() {
    MemoryUsage mu = { 0, 0 };

    #if _OS_MS_WINDOWS
    PROCESS_MEMORY_COUNTERS pmc;
    HANDLE hProcess = GetCurrentProcess();
    if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
        mu.physicalMemory.size = pmc.WorkingSetSize;
        mu.virtualMemory.size = pmc.PagefileUsage;
    }
    CloseHandle(hProcess);
    #else

    #endif // _OS_MS_WINDOWS

    return mu;
}

System::MemoryUsage System::peakMemoryUsage() {
    MemoryUsage mu = { 0, 0 };

    #if _OS_MS_WINDOWS
    PROCESS_MEMORY_COUNTERS pmc;
    HANDLE hProcess = GetCurrentProcess();
    if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
        mu.physicalMemory.size = pmc.PeakWorkingSetSize;
        mu.virtualMemory.size = pmc.PeakPagefileUsage;
    }
    CloseHandle(hProcess);
    #else

    #endif // _OS_MS_WINDOWS

    return mu;
}

}
