# Common tests
This directory contains all the unit tests for the `common` library. 

## Supported platforms
* Linux
* macOs

## Prerequisites
GTest should be installed in the system running the tests.

### Linux
```
apt-get -y install libgtest-dev
```

### macOS
```
brew install googletest
```

## How to run

To execute the unit tests for the first time, run the following commands from `common/test`:
```
mkdir build
cd build
cmake ..
make check
```

If no new files are added, the following unit test executions can be invoked from `common/test/build` with:
```
make check
```

If you want to see detailed output for every test, run this instead from `common/test/build`:
```
./test/runUnitTests
```
