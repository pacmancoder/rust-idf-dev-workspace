#!/bin/bash

ORIGINAL_CWD=$(pwd)
ORIGINAL_RUSTFLAGS=$RUSTFLAGS

function restore_env()
{
    cd $ORIGINAL_CWD
    export RUSTFLAGS=$ORIGINAL_RUSTFLAGS
}

function terminate()
{
    echo $1 1>&2
    restore_env
    exit 1
}

function print_stage()
{
    echo $1
}

function run_rust_build()
{
    print_stage "Checking environment..."

    LINKER_PATH=$(which xtensa-lx106-elf-gcc)

    echo "Using linker $LINKER_PATH"

    if [[ $? != 0 ]]; then
        terminate "Cant find xtensa toolchain in path";
    fi

    print_stage "Building rust code..."

    export RUSTFLAGS="-C linker="$LINKER_PATH" --emit llvm-bc,link"

    cd .
    xargo build --target xtensa-esp8266-none-elf --release

    if [[ $? != 0 ]]; then
        terminate "Firmware compilation failed"
    fi

    cd $ORIGINAL_CWD
}

function run_rename_symbols()
{
    print_stage "Renaming conflicting symbols..."
    xtensa-lx106-elf-objcopy \
        --redefine-syms=symbol_renaming.map \
        ../target/xtensa-esp8266-none-elf/release/libfirmware.a

    cd $ORIGINAL_CWD
}

function run_build()
{
    run_rust_build
    run_rename_symbols

    print_stage "Building idf shim..."

    mkdir idf-build 1>/dev/null 2>&1
    cd idf-build
    cmake ..

    if [[ $? != 0 ]]; then
        terminate "Firmware compilatiom (CMake invocation) failed"
    fi

    make

    if [[ $? != 0 ]]; then
        terminate "Firmware compilation (idf-shim) failed"
    fi

    cd $ORIGINAL_CWD
}

function run_flash()
{
    print_stage "Flashing device..."

    cd idf-build
    if [[ "$?" != 0 ]]; then
        terminate "Firmware was not build"
    fi

    make flash
    if [[ "$?" != 0 ]]; then
        terminate "Device flash failed"
    fi

    cd $ORIGINAL_CWD
}

function run_menuconfig()
{
    cd idf-build
    if [[ "$?" != 0 ]]; then
        terminate "Firmware was not build"
    fi

    make menuconfig

    cd $ORIGINAL_CWD
}

if [[ "$1" == "build" ]]; then
    run_build
elif [[ "$1" == "rust_build" ]]; then
    run_rust_build
elif [[ "$1" == "flash" ]]; then
    run_flash
elif [[ "$1" == "menuconfig" ]]; then
    run_menuconfig
fi

print_stage "$1 completed successfully"

restore_env
