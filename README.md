# LC3 VM

This is a simple LC3 virtual machine written in C.

## Usage

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/lc3-vm <program.obj>
```
## References

- [Write your Own Virtual Machine](https://www.jmeiners.com/lc3-vm/#:lc3.c_2)
