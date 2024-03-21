include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=qca-nss-cfi
PKG_SOURCE_PROTO:=git
PKG_BRANCH:=master
PKG_RELEASE:=2

include $(INCLUDE_DIR)/local-development.mk
ifeq ($(DUMP)$(PKG_VERSION),)
  PKG_VERSION:=$(shell git ls-remote $(PKG_SOURCE_URL) $(PKG_BRANCH) | cut -b -7)
endif

include $(INCLUDE_DIR)/package.mk

CFI_OCF_DIR:=ocf/v2.0
CFI_CRYPTOAPI_DIR:=cryptoapi/v2.0

define KernelPackage/qca-nss-cfi-cryptoapi
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Devices
  DEPENDS:=@TARGET_ipq95xx +kmod-qca-nss-crypto +kmod-crypto-authenc
  TITLE:=Kernel driver for NSS cfi
  FILES:=$(PKG_BUILD_DIR)/$(CFI_CRYPTOAPI_DIR)/qca-nss-cfi-cryptoapi.ko
  AUTOLOAD:=$(call AutoLoad,59,qca-nss-cfi-cryptoapi)
endef

define KernelPackage/qca-nss-cfi-ocf
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Devices
  DEPENDS:=@TARGET_ipq95xx +kmod-qca-nss-crypto +PACKAGE_kmod-crypto-ocf:kmod-crypto-ocf
  TITLE:=Kernel driver for NSS cfi
  FILES:=$(PKG_BUILD_DIR)/$(CFI_OCF_DIR)/qca-nss-cfi-ocf.ko

ifdef CFI_IPSEC_DIR
  FILES+=$(PKG_BUILD_DIR)/$(CFI_IPSEC_DIR)/qca-nss-ipsec.ko
  AUTOLOAD:=$(call AutoLoad,61,qca-nss-cfi-ocf qca-nss-ipsec)
else
  AUTOLOAD:=$(call AutoLoad,61,qca-nss-cfi-ocf)
endif
endef

define Build/InstallDev/qca-nss-cfi
	$(INSTALL_DIR) $(1)/usr/include/qca-nss-cfi
	$(CP) $(PKG_BUILD_DIR)/$(CFI_CRYPTOAPI_DIR)/../exports/* $(1)/usr/include/qca-nss-cfi
	$(CP) $(PKG_BUILD_DIR)/include/* $(1)/usr/include/qca-nss-cfi
endef

define Build/InstallDev
	$(call Build/InstallDev/qca-nss-cfi,$(1))
endef

define KernelPackage/qca-nss-cfi/Description
This package contains a NSS cfi driver for QCA chipset
endef

EXTRA_CFLAGS+= \
	-DCONFIG_NSS_DEBUG_LEVEL=4 \
	-I$(LINUX_DIR)/crypto/ocf \
	-I$(STAGING_DIR)/usr/include/qca-nss-crypto \
	-I$(STAGING_DIR)/usr/include/crypto \
	-I$(STAGING_DIR)/usr/include/qca-nss-drv

#TODO: qca-nss-clients shouldn't be needed by ipq95xx
ifneq (, $(findstring $(subtarget), "ipq95xx" "ipq95xx_32" "ipq807x" "ipq807x_64" "ipq60xx" "ipq60xx_64" "ipq50xx" "ipq50xx_64"))
EXTRA_CFLAGS+= -I$(STAGING_DIR)/usr/include/qca-nss-clients
endif

# Build individual packages if selected
ifneq ($(CONFIG_PACKAGE_kmod-qca-nss-cfi-cryptoapi),)
MAKE_OPTS+=cryptoapi=y
endif

ifneq ($(CONFIG_PACKAGE_kmod-qca-nss-cfi-ocf),)
MAKE_OPTS+=ocf=y
endif

define Build/Compile
	$(KERNEL_MAKE) -C "$(LINUX_DIR)" $(strip $(MAKE_OPTS)) \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		M="$(PKG_BUILD_DIR)" \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		CC="$(TARGET_CC)" \
		CFI_CRYPTOAPI_DIR=$(CFI_CRYPTOAPI_DIR) \
		CFI_OCF_DIR=$(CFI_OCF_DIR) \
		CFI_IPSEC_DIR=$(CFI_IPSEC_DIR) \
		SoC=$(CONFIG_TARGET_BOARD) \
		modules
endef

$(eval $(call KernelPackage,qca-nss-cfi-cryptoapi))
$(eval $(call KernelPackage,qca-nss-cfi-ocf))
