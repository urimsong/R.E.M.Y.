Build instructions (Windows - PowerShell)

1. Create and enter a build directory:

```powershell
mkdir build
cd build
```

2. Configure with CMake and build:

```powershell
cmake ..
cmake --build . --config Release
```

3. Run tests with CTest:

```powershell
ctest -C Release --output-on-failure
```

Notes:
- Requires CMake 3.14+ and a C++17-capable compiler (MSVC, clang, or GCC).
- On Linux/macOS replace PowerShell commands with the equivalent shell commands.
