# Recipe extension for package : rp-pppoe-relay

RP-PPPOE:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))

define rp-pppoe-relay_install_append
	$(INSTALL_DIR) $(1)/etc/hotplug.d/iface/
	$(INSTALL_DATA) $(RP-PPPOE)/files/pppoe.hotplug $(1)/etc/hotplug.d/iface/65-pppoe
endef

define rp-pppoe-common_install_append
	echo "config pppoe_relay" >> $(1)/etc/config/pppoe
	echo -e "\tlist server_interface wan" >> $(1)/etc/config/pppoe
	echo -e "\tlist client_interface lan" >> $(1)/etc/config/pppoe
	echo -e "\toption maxsessions 64" >> $(1)/etc/config/pppoe
	echo -e "\toption timeout 60" >> $(1)/etc/config/pppoe
endef

Package/rp-pppoe-relay/install += $(newline)$(rp-pppoe-relay_install_append)
Package/rp-pppoe-common/install += $(newline)$(rp-pppoe-common_install_append)
