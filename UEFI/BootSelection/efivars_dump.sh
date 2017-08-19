#!/bin/bash

if [ -f ./efi_vars ]; then
	echo "./efi_vars already exists" >&2
	exit 1
fi

for var in /sys/firmware/efi/efivars/*; do
	echo $var >> ./efi_vars
	hexdump -C $var >> ./efi_vars
	echo >> ./efi_vars
	
	echo $var
done
