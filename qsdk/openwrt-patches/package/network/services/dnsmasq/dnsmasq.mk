# Recipe extension for package : dnsmasq

DNSMASQ:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))

define dnsmasq_install_append
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) $(DNSMASQ)/files/65-dnsmasq-force-lan-up.sh $(1)/etc/uci-defaults
endef

Package/dnsmasq/install += $(newline)$(dnsmasq_install_append)
