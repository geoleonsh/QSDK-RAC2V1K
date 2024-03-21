# Recipe extension for package : mwan3

MWAN3:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))

define mwan3_install_append
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_DATA) $(MWAN3)/files/mwan3.defaults $(1)/etc/uci-defaults/65-mwan3
endef

Package/mwan3/install += $(newline)$(mwan3_install_append)
