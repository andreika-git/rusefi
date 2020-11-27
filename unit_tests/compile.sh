#!/bin/bash

echo "Compiling unit tests"
rm -rf .dep/
rm -rf build/
make -r -j4
