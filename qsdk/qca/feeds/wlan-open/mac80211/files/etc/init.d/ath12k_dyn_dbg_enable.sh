#!/bin/sh /etc/rc.common
#
# Copyright (c) 2020 The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

START=03
STOP=94

get_front_end_mode() {
	config_load "ecm"
	config_get front_end global acceleration_engine "auto"

	case $front_end in
	auto)
		echo '0'
		;;
	nss)
		echo '1'
		;;
	sfe)
		echo '2'
		;;
	ppe)
		echo '3'
		;;
	nss-sfe)
		echo '4'
		;;
	ppe-sfe)
		echo '5'
		;;
	*)
		echo 'uci_option_acceleration_engine is invalid'
	esac
}

boot()
{
	ath12k="/etc/modules.d/ath12k"
	if [ -e $ath12k ];
	then
		# Read first line only
		content=$(head -n 1 $ath12k)

		# If first line does not have "dyndbg" substr in it
		if [[ $content != *"dyndbg"* ]];
		then
			sed -i '1s/ath12k/ath12k dyndbg=+p/' $ath12k
		fi
	fi

	ecm="/etc/modules.d/ecm"
	if [ ! -f $ecm ];
	then
		touch $ecm
	fi
	echo "ecm front_end_selection=$(get_front_end_mode)" > $ecm

}
