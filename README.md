

# dependencies
### for fastdds
```shell
sudo apt install libasio-dev
sudo apt install libssl-dev
```

### for ros1 built from source
```shell
sudo apt install libboost-thread-dev
sudo apt install libboost-filesystem-dev
```


# reference
### pic option
https://stackoverflow.com/questions/38296756/what-is-the-idiomatic-way-in-cmake-to-add-the-fpic-compiler-option

### stack protector
https://stackoverflow.com/questions/48754619/what-are-cmake-build-type-debug-release-relwithdebinfo-and-minsizerel
https://stackoverflow.com/questions/78322480/improve-g-compiler-flags-for-debug-and-release
https://developers.redhat.com/articles/2022/06/02/use-compiler-flags-stack-protection-gcc-and-clang#control_flow_integrity

`-fsplit-stack` may cause crash