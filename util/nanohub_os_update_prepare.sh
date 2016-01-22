#!/bin/bash

#
# Copyright (C) 2016 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

terminate() { #cleanup and exit
	rm -rf $stage
	exit $1
}

usage () { #show usage and bail out
	echo "USAGE:" >&2
	echo "    $1 <PRIV_KEY_FILE> <PUB_KEY_FILE> < nanohub.update.bin > nanohub.update.signed.bin" >&2
	terminate -1
}

putchar() {
	hexch="0123456789abcdef"
	h=$[$1/16]
	l=$[$1%16]
	h=${hexch:$h:1}
	l=${hexch:$l:1}
	e="\x"$h$l
	echo -ne $e
}

printhex() {
	w3=$[$1/16777216]
	t=$[$w3*16777216]
	a=$[$1-$t]

	w2=$[$a/65536]
	t=$[$w2*65536]
	a=$[$a-$t]

	w1=$[$a/256]
	w0=$[$a%256]

	putchar $w0
	putchar $w1
	putchar $w2
	putchar $w3
}

#create temp dir
stage=$(mktemp -dt "$(basename $0).XXXXXXXXXX")


#sanity checks (on the user)
if [ -t 1 ]
then
	usage $0
fi

if [ -t 0 ]
then
	usage $0
fi

#handle signing
if [ $# -ne 2 ]
then
	usage $0
fi
priv1="$1"
pub1="$2"


#save update to file in dir
cat > "$stage/raw"

#pad update to 4 byte boundary
t=$(du -b "$stage/raw" | cut -f1)
while [ $[$t%4] -ne 0 ]
do
	echo -ne "\0" >> "$stage/raw"
	t=$(du -b "$stage/raw" | cut -f1)
done

#get and save the file size
signed_sz=$(du -b "$stage/raw" | cut -f1)

#create the header (with the marker set for signing
echo -ne "Nanohub OS\x00\xFE" > "$stage/hdr"
printhex $signed_sz >> "$stage/hdr"

#concat the data to header
cat "$stage/hdr" "$stage/raw" > "$stage/with_hdr"

#create the signature
nanoapp_sign sign "$priv1" "$pub1" < "$stage/with_hdr" > "$stage/sig"

#insert proper upload marker
echo -ne "\xff" | dd bs=1 seek=11 count=1 conv=notrunc of="$stage/with_hdr" 2>/dev/null

#produce signed output
cat "$stage/with_hdr" "$stage/sig" "$pub1"

terminate 0

