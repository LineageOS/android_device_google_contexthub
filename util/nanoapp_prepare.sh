#!/bin/bash

terminate() { #cleanup and exit
	rm -rf $stage
	exit $1
}

usage () { #show usage and bail out
	echo "USAGE:" >&2
	echo "    $1 [-e <ENCR_KEY_NUM> <ENCR_KEY_FILE>] [-s <PRIV_KEY_FILE> <PUB_KEY_FILE> [<SIG_TO_CHAIN_1> [<SIG_TO_CHAIN_2> [...]]]] < app.napp > app.final.napp" >&2
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

#save args and create temp dir
stage=$(mktemp -dt "$(basename $0).XXXXXXXXXX")
args=( "$@" )

#sanity checks (on the user)
if [ -t 1 ]
then
	usage $0
fi

if [ -t 0 ]
then
	usage $0
fi


#get encryption key if it exists & encrypt app
encr_key_num=""
if [ ${#args[@]} -ge 1 ]
then
	if [[ ${args[0]} = "-e" ]]
	then
		if [ ${#args[@]} -lt 3 ]
		then
			usage $0
		fi
		encr_key_num=${args[1]}
		encr_key_file=${args[2]}
		args=("${args[@]:3}")

		if [ ! -f "$encr_key_file" ]; then
			usage $0
		fi

		nanoapp_encr encr "$encr_key_num" "$encr_key_file" > "$stage/postencr"
	fi
fi


#if app is not encrypted, just copy it to staging area
if [ ! -f "$stage/postencr" ]; then
	cat > "$stage/postencr"
fi

#handle signing
if [ ${#args[@]} -ge 1 ]
then
	if [[ ${args[0]} = "-s" ]]
	then
		if [ ${#args[@]} -lt 3 ]
		then
			usage $0
		fi
		priv1=${args[1]}
		pub1=${args[2]}

		#make sure files exist
		i=1
		while [ $i -lt ${#args[@]} ]
		do
			if [ ! -f "${args[$i]}" ]; then
				usage $0
			fi
			i=$[$i+1]
		done

		#get and save file size
		signed_sz=$(du -b "$stage/postencr" | cut -f1)

		nanoapp_sign sign "$priv1" "$pub1" < "$stage/postencr" > "$stage/sig"

		#pad data to 16 bytes
		t=$signed_sz
		while [ $[$t%16] -ne 0 ]
		do
			echo -ne "\0" >> "$stage/postencr"
			t=$(du -b "$stage/postencr" | cut -f1)
		done

		#produce signed output
		cat "$stage/postencr" "$stage/sig" "$pub1" > "$stage/signed"

		#append remaining chunks
		i=3
		while [ $i -lt ${#args[@]} ]
		do
			cat "${args[$i]}" >> "$stage/signed"
			i=$[$i+1]
		done

		#create header
		num_sigs=$[${#args[@]}-2]

		echo -n SigndApp > "$stage/finished"
		printhex $signed_sz >> "$stage/finished"
		printhex $num_sigs >> "$stage/finished"
		echo -ne "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0" >> "$stage/finished"
		cat "$stage/signed" >> "$stage/finished"
	else
		usage $0
	fi
fi

#if app is not signed, just copy it to staging area
if [ ! -f "$stage/finished" ]; then
	mv "$stage/postencr" "$stage/finished"
fi

#produce output
cat "$stage/finished"

terminate 0

