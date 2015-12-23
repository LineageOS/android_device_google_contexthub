#!/bin/bash


infile="$1"
shift
outfile="$1"
shift

retcode=0

echo -e "\n\nChecking '$infile' for forbidden symbols\n\n" >&2

for var in "$@"
do
	look_for=$(echo "$var" | sed 's/\([^=]*\)=\(.*\)/\1/g')
	if echo "$var" | grep = >/dev/null
	then
		explanation=$(echo "$var" | sed 's/\([^=]*\)=/ /g')
	else
		explanation=""
	fi

	explanation="Forbidden function '$look_for' found. This is a build error.$explanation"


	if ${CROSS_COMPILE}nm -a "$infile" |grep -e "[0-9a-f]\{8\} [Tt] $look_for" >/dev/null
	then
		echo $explanation >&2
		retcode=-1
	fi
done

if [ $retcode -eq 0 ]
then
	echo "Symcheck found nothing bad. Proceeding" >&2
	cp "$infile" "$outfile"
fi

exit $retcode


