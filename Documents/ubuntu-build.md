# Building and uploading the firmware on Ubuntu

These instructions use Ubuntu/Debian specific package names and ``apt-get``.
Other distributions will be similar.

Once the firmware is built, see [linux-gdb.md](linux-gdb.md) for details on
uploading and running the firmware.

## tl;dr

If you know what you're doing with Ubuntu and building software in general:

```console
$ sudo apt-get -y install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi
$ git clone git@github.com:cuspaceflight/state-estimators && cd state-estimators
$ mkdir build-fw && cd build-fw
$ cmake -DBUILD_FIRMWARE=ON \
	-DCMAKE_TOOLCHAIN_FILE=../Toolchain-arm-none-eabi.cmake \
	..
$ make VERBOSE=1
```

## Initial clone and build

Clone the repository. Since we use git submodules, it is important, albeit
easily forgotten, to use the ``--recursive`` flag:

```console
$ git clone git@github.com:cuspaceflight/state-estimators
$ cd state-estimators
```

Install CMake, the ARM GCC toolchain and standard C library:

```console
$ sudo apt-get -y install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi
```

Use CMake to configure and compile the firmware. I tend to use a ``build-fw``
directory.

```console
$ mkdir build-fw
$ cd build-fw
$ cmake -DBUILD_FIRMWARE=ON \
	-DCMAKE_TOOLCHAIN_FILE=../Toolchain-arm-none-eabi.cmake \
	..
[...output...]
$ make VERBOSE=1
```

I tend to use ``VERBOSE=1`` to see what's going on. It's optional.

> **NOTE:** These steps are also followed by Travis as part of the continuous
> integration (CI) tests. See the ``.travis.yml`` and
> ``scripts/travis-build-firmware.sh`` files.

## Subsequent builds

Assuming you're in the root of the repository, i.e. that the ``pwd`` command
prints something ending in ``state-estimators``, you don't need to call CMake
again to rebuild. One need only use ``make``:

```console
$ make -C build-fw
```



