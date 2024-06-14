# idl 
generate source files from IDL.
compile source files to library linked to fastrtps and fastcddr

```shell
fastddsgen -replace  ./Message.idl
```

# config


# split dds_handler.h

# variant
```c++


template<typename T>
struct Writer{
    
};


template<typename T>
struct Reader{

};



absl<Writer<Laser>, Writer<Pointcloud>> writers;

absl<Reader<Laser>, Reader<Pointcloud>> readers;

```
