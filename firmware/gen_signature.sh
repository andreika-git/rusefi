#!/bin/bash

SHORT_BOARDNAME=$1

echo "Generating signature for ${SHORT_BOARDNAME}"

# generate a unique signature
date=$(date +"%Y.%m.%d")
echo "#define SIGNATURE_DATE $date" > tunerstudio/signature_${SHORT_BOARDNAME}.txt
echo "#define SIGNATURE_BOARD ${SHORT_BOARDNAME}" >> tunerstudio/signature_${SHORT_BOARDNAME}.txt

nanosec=$(date +"%N")
# prevent octal numbers once and for all
nanosec="1$nanosec"
hash=$(($nanosec % 2147483648))
echo "#define SIGNATURE_HASH $hash" >> tunerstudio/signature_${SHORT_BOARDNAME}.txt

exit 0
