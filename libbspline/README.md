# General Bspline Representation Library (libbspline)

## Installation
libbspline is a header only library, several test nodes are compiled using CMake.

### Setup
```bash
cd <libbspline directory>/cpp
mkdir build && cd build
cmake .. 
make
```

#### Run Comparison Bash file

Run `bash launch_single_and_full.bash` in the `cpp` folder, this is to test the perfromance and speed without going through the ctest checks
- *This can only be launched after you create and compile your **build** folder*
- *Output example is as shown below*
```bash
CALC SINGLE POINT
[3d_bspline_single] query_time: 
0.43182
[bs_utils] test single_1d_bspline: 0.000113589s
[bs_utils] test single_1d_bspline: 4.1747e-05s
[bs_utils] test single_1d_bspline: 2.816e-05s
[3d_bspline_single] time for 3d_bspline_single: 0.241696ms
...
Success
CALC PATH
[bs_utils] test 1d_bspline: 0.00016964s
[bs_utils] test 1d_bspline: 0.00016016s
[bs_utils] test 1d_bspline: 0.000178995s
[3d_bspline] time for 3d_bspline: 0.661326ms
Success
```


#### CTest
To run sample scripts to synchronize MATLAB and C++ output
```bash
make test
```

Where the CTest output in the console should be as follows
```bash
Running tests...
Test project /<directory>/libbspline/cpp/build
    Start 1: m_matrix
1/5 Test #1: m_matrix .........................   Passed    0.00 sec
    Start 2: txt_reader
2/5 Test #2: txt_reader .......................   Passed    0.00 sec
    Start 3: 1d_bspline
3/5 Test #3: 1d_bspline .......................   Passed    0.00 sec
    Start 4: 3d_bspline
4/5 Test #4: 3d_bspline .......................   Passed    0.00 sec
    Start 5: 3d_bspline_single
5/5 Test #5: 3d_bspline_single ................   Passed    0.00 sec

100% tests passed, 0 tests failed out of 5

Total Test time (real) =   0.01 sec
```

#### Include in other projects:
To link this lib properly, add following in the `CMakeLists.txt`
```
find_package(libbspline REQUIRED)
include_directories(${LIBBSPLINE_INCLUDE_DIRS})
```

### References
1. Liu Sikang's Decomp Util https://github.com/sikang/DecompUtil