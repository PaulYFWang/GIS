#include <iostream>
#include <unittest++/UnitTest++.h>

/*
 * This is the main that drives running
 * unit tests.
 */
int main(int /*argc*/, char** /*argv*/) {
    //Run the unit tests
    int num_failures = UnitTest::RunAllTests();

    return num_failures;
}
