#!/bin/sh

if [ $# -eq 0 -o ! -f "$1" ]; then
	echo "Usage: $0 file [initial function] [assert file] [assert line]"
	exit 1
fi

if [ -z "$2" ]; then
	echo "Initial function: "
	read SLICE_INITIAL_FUNCTION
else
	SLICE_INITIAL_FUNCTION="$2"
fi

if [ -z "$3" ]; then
	echo "Assert file: "
	read SLICE_ASSERT_FILE
else
	SLICE_ASSERT_FILE="$3"
fi

if [ -z "$4" ]; then
	echo "Assert line: "
	read SLICE_ASSERT_LINE
else
	SLICE_ASSERT_LINE="$4"
fi

echo "Slicing file $1, starting at function $SLICE_INITIAL_FUNCTION with assert on\
 ${SLICE_ASSERT_FILE}:$SLICE_ASSERT_LINE"

export SLICE_INITIAL_FUNCTION SLICE_ASSERT_FILE SLICE_ASSERT_LINE
export LD_LIBRARY_PATH=../src:$LD_LIBRARY_PATH

opt -load LLVMSlicer.so -create-hammock-cfg -slice-inter $1\
	-simplifycfg -o ${1%%.*}.sliced.${1##*.}
