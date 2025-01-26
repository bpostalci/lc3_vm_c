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
- [Introduction to Computing Systems: From Bits and Gates to C and Beyond](https://www.amazon.com/Introduction-Computing-Systems-Gates-Beyond/dp/0072467509)
- [2048 LC3 implementation (2048.obj is based on this)](https://github.com/rpendleton/lc3-2048)