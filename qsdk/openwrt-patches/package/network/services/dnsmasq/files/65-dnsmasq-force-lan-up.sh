#!/bin/sh

uci -q batch <<-EOT
	set dhcp.lan.force='1'
	commit dhcp
EOT
exit 0
