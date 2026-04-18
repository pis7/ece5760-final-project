// Simulation main for Verilator --timing mode (CMake flow)
// Replaces --main which only works with --exe

#include "VCGTop_tb.h"
#include "verilated.h"
#include <memory>

int main(int argc, char** argv) {
    const auto contextp = std::make_unique<VerilatedContext>();
    contextp->commandArgs(argc, argv);

    const auto top = std::make_unique<VCGTop_tb>(contextp.get());

    while (!contextp->gotFinish()) {
        top->eval();
        if (!top->eventsPending()) break;
        contextp->time(top->nextTimeSlot());
    }

    top->final();
    return 0;
}
