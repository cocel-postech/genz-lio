#include "core/GenZLIO.hpp"
#include "Check.hpp"
#include <omp.h>
#include <thread>

int main() {
    genz_lio::Config one, two;
    one.max_threads = 1;
    two.max_threads = 2;
    genz_lio::GenZLIO first(one), second(two);
    int observed_first = 0, observed_second = 0, observed_again = 0;
    std::thread worker([&] {
        // Stand in for a ROS 2 executor worker with an unrelated initial limit.
        omp_set_num_threads(4);
        first.registerScan({}, 0, .1, {});
        observed_first = omp_get_max_threads();
        second.registerScan({}, 0, .1, {});
        observed_second = omp_get_max_threads();
        first.reset();
        first.registerScan({}, 0, .1, {});
        observed_again = omp_get_max_threads();
    });
    worker.join();
    GENZ_CHECK(observed_first == 1);
    GENZ_CHECK(observed_second == 2);
    GENZ_CHECK(observed_again == 1);
}
