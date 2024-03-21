#!/bin/sh
#
# Copyright (c) 2015, 2020, The Linux Foundation. All rights reserved.

# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.

# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

. /lib/functions.sh

is_ftm_conf_supported() {
	local board=ap$(echo $(board_name) | awk -F 'ap' '{print$2}')

	case "$board" in
	ap-mi*|ap-al02-c4*)
		;;
	*)
		echo "ftm.conf file is not supported for $board " > /dev/console
		rm -rf /ini/ftm.conf
		;;
	esac
}

is_ftm_conf_supported

create_cfg_caldata() {
	local brd=ap$(echo $(board_name) | awk -F 'ap' '{print$2}')

	awk -F ',' -v apdk='/tmp/' -v mtdblock=$1 -v ahb_dir=$2 -v pci_dir=$3 -v is_wkk=$4 -v board=$brd '{
		if ($1 == board) {
			print $1 "\t" $2 "\t" $3 "\t" $4 "\t" $5
			if ($3 == 0) {
				print "Internal radio"
				cmd ="stat -Lc%s /lib/firmware/" ahb_dir "/bdwlan.b" $2 " 2> /dev/null"
				cmd | getline BDF_SIZE
				close(cmd)
				if(!BDF_SIZE) {
					print "BDF file for Board id " $2 " not found. Using default value"
					BDF_SIZE=131072
				}
				cmd = "dd if="mtdblock" of=" apdk ahb_dir "/caldata.bin bs=1 count=" BDF_SIZE " skip=" $4
				system(cmd)
				cmd = "cp " apdk ahb_dir "/caldata.bin /lib/firmware/" ahb_dir "/"
				system(cmd)
			} else {
				print "PCI radio"
				cmd ="stat -Lc%s /lib/firmware/" pci_dir "/bdwlan.b" $2 " 2> /dev/null"
				cmd | getline BDF_SIZE
				close(cmd)
				if(!BDF_SIZE) {
					print "BDF file for Board id " $2 " not found. Using default value"
					if (is_wkk == 1)
						BDF_SIZE=184320
					else
						BDF_SIZE=131072
				}
				cmd = "dd if="mtdblock" of=" apdk pci_dir "/caldata_" $3 ".b" $2 " bs=1 count=" BDF_SIZE " skip=" $4
				system(cmd)
				cmd = "cp " apdk pci_dir "/caldata_" $3 ".b" $2 " /lib/firmware/" pci_dir "/"
				system(cmd)
			}
		}
	}' /ini/ftm.conf

	[ -f /lib/firmware/$2/caldata.bin ] || touch /lib/firmware/$2/caldata.bin
}

do_load_ipq4019_board_bin()
{

    local board=ap$(echo $(board_name) | awk -F 'ap' '{print$2}')
    local mtdblock=$(find_mtd_part 0:ART)

    local apdk="/tmp"

    if [ -z "$mtdblock" ]; then
        # read from mmc
        mtdblock=$(find_mmc_part 0:ART)
    fi

    [ -n "$mtdblock" ] || return

    # load board.bin
    case "$board" in
            ap-dk0*)
                    mkdir -p ${apdk}
                    dd if=${mtdblock} of=${apdk}/wifi0.caldata bs=32 count=377 skip=128
                    dd if=${mtdblock} of=${apdk}/wifi1.caldata bs=32 count=377 skip=640
            ;;
            ap16* | ap148*)
                    mkdir -p ${apdk}
                    dd if=${mtdblock} of=${apdk}/wifi0.caldata bs=32 count=377 skip=128
                    dd if=${mtdblock} of=${apdk}/wifi1.caldata bs=32 count=377 skip=640
                    dd if=${mtdblock} of=${apdk}/wifi2.caldata bs=32 count=377 skip=1152
            ;;
            ap-hk14 | ap-hk01-c6)
                    [ -f /lib/firmware/IPQ8074/caldata.bin ] && return
                    FILESIZE=131072
                    mkdir -p ${apdk}/IPQ8074
                    dd if=${mtdblock} of=${apdk}/IPQ8074/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ8074/caldata.bin /lib/firmware/IPQ8074/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
            ;;
            ap-hk01-*)
                    [ -f /lib/firmware/IPQ8074/caldata.bin ] && return
                    HK_BD_FILENAME=/lib/firmware/IPQ8074/bdwlan.bin
                    mkdir -p ${apdk}/IPQ8074
                    if [ -f "$HK_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$HK_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ8074/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ8074/caldata.bin ] || \
                    cp ${apdk}/IPQ8074/caldata.bin /lib/firmware/IPQ8074/caldata.bin
            ;;
            ap-hk10-*)
                    [ -f /lib/firmware/IPQ8074/caldata.bin ] && return
                    FILESIZE=131072
                    mkdir -p ${apdk}/IPQ8074
                    dd if=${mtdblock} of=${apdk}/IPQ8074/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ8074/caldata.bin /lib/firmware/IPQ8074/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
                    cp ${apdk}/qcn9000/caldata_2.bin /lib/firmware/qcn9000/caldata_2.bin
	    ;;
            ap-hk* | ap-ac* | ap-oa*)
                    [ -f /lib/firmware/IPQ8074/caldata.bin ] && return
                    HK_BD_FILENAME=/lib/firmware/IPQ8074/bdwlan.bin
                    mkdir -p ${apdk}/IPQ8074
                    dd if=${mtdblock} of=${apdk}/wifi1.caldata bs=1 count=12064 skip=208896
                    if [ -f "$HK_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$HK_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ8074/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ8074/caldata.bin ] || \
                    cp ${apdk}/IPQ8074/caldata.bin /lib/firmware/IPQ8074/caldata.bin
            ;;
            ap-cp01-c3*)
                    [ -f /lib/firmware/IPQ6018/caldata.bin ] && return
                    CP_BD_FILENAME=/lib/firmware/IPQ6018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ6018
                    if [ -f "$CP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$CP_BD_FILENAME")
                    else
                        FILESIZE=65536
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ6018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ6018/caldata.bin ] || \
                    cp ${apdk}/IPQ6018/caldata.bin /lib/firmware/IPQ6018/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    FILESIZE=131072
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
            ;;
            ap-cp01-c5*)
                    [ -f /lib/firmware/IPQ6018/caldata.bin ] && return
                    CP_BD_FILENAME=/lib/firmware/IPQ6018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ6018
                    if [ -f "$CP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$CP_BD_FILENAME")
                    else
                        FILESIZE=65536
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ6018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ6018/caldata.bin ] || \
                    cp ${apdk}/IPQ6018/caldata.bin /lib/firmware/IPQ6018/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    FILESIZE=131072
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
                    cp ${apdk}/qcn9000/caldata_2.bin /lib/firmware/qcn9000/caldata_2.bin
            ;;
            ap-mp02.1*)
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi

                    #FTM Daemon compresses the caldata and writes the lzma file in ART Partition
                    dd if=${mtdblock} of=${apdk}/virtual_art.bin.lzma
                    lzma -fdv --single-stream ${apdk}/virtual_art.bin.lzma || {
                            # Create dummy virtual_art.bin file of size 512K
                            dd if=/dev/zero of=${apdk}/virtual_art.bin bs=1024 count=512
                    }

                    dd if=${apdk}/virtual_art.bin of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096

                    mkdir -p ${apdk}/qcn6122
                    # Read after 154KB
                    dd if=${apdk}/virtual_art.bin of=${apdk}/qcn6122/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    # Read after 304KB
                    dd if=${apdk}/virtual_art.bin of=${apdk}/qcn6122/caldata_2.bin bs=1 count=$FILESIZE skip=311296

                    ln -s ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin
                    ln -s ${apdk}/qcn6122/caldata_1.bin /lib/firmware/qcn6122/caldata_1.bin
                    ln -s ${apdk}/qcn6122/caldata_2.bin /lib/firmware/qcn6122/caldata_2.bin
            ;;
            ap-mp03.1)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    mkdir -p ${apdk}/IPQ5018
                    FILESIZE=131072

                    if [ -e /sys/firmware/devicetree/base/compressed_art ]
                    then
                        #FTM Daemon compresses the caldata and writes the lzma file in ART Partition
                        dd if=${mtdblock} of=${apdk}/virtual_art.bin.lzma
                        lzma -fdv --single-stream ${apdk}/virtual_art.bin.lzma || {
                        # Create dummy virtual_art.bin file of size 512K
                        dd if=/dev/zero of=${apdk}/virtual_art.bin bs=1024 count=512
                        }

                        dd if=${apdk}/virtual_art.bin of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096

                        mkdir -p ${apdk}/qcn9000
                        # Read after 154KB
                        dd if=${apdk}/virtual_art.bin of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    else
                        dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096

                        mkdir -p ${apdk}/qcn9000
                        dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    fi

                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
            ;;
            ap-mp03.1-* | ap-mp03.6*)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
            ;;
            ap-mp03.5*)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin

                    mkdir -p ${apdk}/qcn6122
                    dd if=${mtdblock} of=${apdk}/qcn6122/caldata_1.bin bs=1 count=$FILESIZE skip=157696

                    cp ${apdk}/qcn6122/caldata_1.bin /lib/firmware/qcn6122/caldata_1.bin

                    dd if=${mtdblock} of=${apdk}/qcn6122/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn6122/caldata_2.bin /lib/firmware/qcn6122/caldata_2.bin
            ;;
            ap-mp03.3*)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin

                    mkdir -p ${apdk}/qcn6122
                    dd if=${mtdblock} of=${apdk}/qcn6122/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn6122/caldata_1.bin /lib/firmware/qcn6122/caldata_1.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn9000/caldata_2.bin /lib/firmware/qcn9000/caldata_2.bin
            ;;
            ap-mp03.4*)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn9000/caldata_2.bin /lib/firmware/qcn9000/caldata_2.bin
            ;;
            ap-mp*)
                    [ -f /lib/firmware/IPQ5018/caldata.bin ] && return
                    MP_BD_FILENAME=/lib/firmware/IPQ5018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ5018
                    if [ -f "$MP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$MP_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ5018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ5018/caldata.bin ] || \
                    cp ${apdk}/IPQ5018/caldata.bin /lib/firmware/IPQ5018/caldata.bin
            ;;
            ap-cp*)
                    [ -f /lib/firmware/IPQ6018/caldata.bin ] && return
                    CP_BD_FILENAME=/lib/firmware/IPQ6018/bdwlan.bin
                    mkdir -p ${apdk}/IPQ6018
                    if [ -f "$CP_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$CP_BD_FILENAME")
                    else
                        FILESIZE=65536
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ6018/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ6018/caldata.bin ] || \
                    cp ${apdk}/IPQ6018/caldata.bin /lib/firmware/IPQ6018/caldata.bin
            ;;
            ap-al02-c13*)
                    [ -f /lib/firmware/IPQ9574/caldata.bin ] && return
                    AL_BD_FILENAME=/lib/firmware/IPQ9574/bdwlan.bin
                    mkdir -p ${apdk}/IPQ9574
                    if [ -f "$AL_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$AL_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ9574/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ9574/caldata.bin ] || \
                    cp ${apdk}/IPQ9574/caldata.bin /lib/firmware/IPQ9574/caldata.bin

                    # PCI0 IS QCN9000 Scan radio
                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin

                    WKK_FILESIZE=184320
                    mkdir -p ${apdk}/qcn9224
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_2.bin bs=1 count=$WKK_FILESIZE skip=362496
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_3.bin bs=1 count=$WKK_FILESIZE skip=567296
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_4.bin bs=1 count=$WKK_FILESIZE skip=772096
                    cp ${apdk}/qcn9224/caldata_2.bin /lib/firmware/qcn9224/caldata_2.bin
                    cp ${apdk}/qcn9224/caldata_3.bin /lib/firmware/qcn9224/caldata_3.bin
                    cp ${apdk}/qcn9224/caldata_4.bin /lib/firmware/qcn9224/caldata_4.bin
            ;;
            ap-al02-c6*|ap-al02-c7*|ap-al02-c8*|ap-al02-c9*|ap-al02-c10*|ap-al02-c11*|ap-al02-c12*|ap-al02-c14*|ap-al02-c15*|ap-al02-c16*|ap-al03-c1*|ap-al03-c2*)
                    [ -f /lib/firmware/IPQ9574/caldata.bin ] && return
                    AL_BD_FILENAME=/lib/firmware/IPQ9574/bdwlan.bin
                    mkdir -p ${apdk}/IPQ9574
                    if [ -f "$AL_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$AL_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ9574/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ9574/caldata.bin ] || \
                    cp ${apdk}/IPQ9574/caldata.bin /lib/firmware/IPQ9574/caldata.bin

                    WKK_FILESIZE=184320
                    mkdir -p ${apdk}/qcn9224
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_1.bin bs=1 count=$WKK_FILESIZE skip=157696
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_2.bin bs=1 count=$WKK_FILESIZE skip=362496
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_3.bin bs=1 count=$WKK_FILESIZE skip=567296
                    dd if=${mtdblock} of=${apdk}/qcn9224/caldata_4.bin bs=1 count=$WKK_FILESIZE skip=772096
                    cp ${apdk}/qcn9224/caldata_1.bin /lib/firmware/qcn9224/caldata_1.bin
                    cp ${apdk}/qcn9224/caldata_2.bin /lib/firmware/qcn9224/caldata_2.bin
                    cp ${apdk}/qcn9224/caldata_3.bin /lib/firmware/qcn9224/caldata_3.bin
                    cp ${apdk}/qcn9224/caldata_4.bin /lib/firmware/qcn9224/caldata_4.bin
            ;;
            ap-al02-c4*)
                    [ -f /lib/firmware/IPQ9574/caldata.bin ] && return
                    mkdir -p ${apdk}/IPQ9574
                    mkdir -p ${apdk}/qcn9224

                    create_cfg_caldata "${mtdblock}" "IPQ9574" "qcn9224" "1"
            ;;
            ap-al02*)
                    [ -f /lib/firmware/IPQ9574/caldata.bin ] && return
                    AL_BD_FILENAME=/lib/firmware/IPQ9574/bdwlan.bin
                    mkdir -p ${apdk}/IPQ9574
                    if [ -f "$AL_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$AL_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ9574/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ9574/caldata.bin ] || \
                    cp ${apdk}/IPQ9574/caldata.bin /lib/firmware/IPQ9574/caldata.bin

                    mkdir -p ${apdk}/qcn9000
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_1.bin bs=1 count=$FILESIZE skip=157696
                    dd if=${mtdblock} of=${apdk}/qcn9000/caldata_2.bin bs=1 count=$FILESIZE skip=311296
                    cp ${apdk}/qcn9000/caldata_1.bin /lib/firmware/qcn9000/caldata_1.bin
                    cp ${apdk}/qcn9000/caldata_2.bin /lib/firmware/qcn9000/caldata_2.bin
            ;;
            ap-al*)
                    [ -f /lib/firmware/IPQ9574/caldata.bin ] && return
                    AL_BD_FILENAME=/lib/firmware/IPQ9574/bdwlan.bin
                    mkdir -p ${apdk}/IPQ9574
                    if [ -f "$AL_BD_FILENAME" ]; then
                        FILESIZE=$(stat -Lc%s "$AL_BD_FILENAME")
                    else
                        FILESIZE=131072
                    fi
                    dd if=${mtdblock} of=${apdk}/IPQ9574/caldata.bin bs=1 count=$FILESIZE skip=4096
                    [ -L /lib/firmware/IPQ9574/caldata.bin ] || \
                    cp ${apdk}/IPQ9574/caldata.bin /lib/firmware/IPQ9574/caldata.bin
            ;;
            ap-mi01.1*|ap-mi01.2*|ap-mi01.4*|ap-mi01.6*|ap-mi01.9*|ap-mi02.1*)
                    [ -f /lib/firmware/IPQ5332/caldata.bin ] && return
                    mkdir -p ${apdk}/IPQ5332
                    mkdir -p ${apdk}/qcn9224

                    create_cfg_caldata "${mtdblock}" "IPQ5332" "qcn9224" "1"
            ;;
            ap-mi01.3*|ap-mi04.1*)
                    [ -f /lib/firmware/IPQ5332/caldata.bin ] && return
                    mkdir -p ${apdk}/IPQ5332
                    mkdir -p ${apdk}/qcn6432

                    create_cfg_caldata "${mtdblock}" "IPQ5332" "qcn6432"
            ;;
            ap-mi*)
                    [ -f /lib/firmware/IPQ5332/caldata.bin ] && return
                    mkdir -p ${apdk}/IPQ5332
                    create_cfg_caldata "${mtdblock}" "IPQ5332"
            ;;
   esac
}

