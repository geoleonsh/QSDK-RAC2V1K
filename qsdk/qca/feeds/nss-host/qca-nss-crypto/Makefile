include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=qca-nss-crypto
PKG_SOURCE_PROTO:=git
PKG_BRANCH:=master
PKG_RELEASE:=1

include $(INCLUDE_DIR)/local-development.mk
ifeq ($(DUMP)$(PKG_VERSION),)
  PKG_VERSION:=$(shell git ls-remote $(PKG_SOURCE_URL) $(PKG_BRANCH) | cut -b -7)
endif

include $(INCLUDE_DIR)/package.mk

NSS_CRYPTO_DIR:=v2.0

define KernelPackage/qca-nss-crypto/Default
SECTION:=kernel
CATEGORY:=Kernel modules
SUBMENU:=Network Devices
DEPENDS:=@TARGET_ipq95xx TARGET_ipq95xx:kmod-qca-nss-drv-lite
endef

define KernelPackage/qca-nss-crypto
  $(call KernelPackage/qca-nss-crypto/Default)
  TITLE:=Kernel driver for NSS crypto driver
  FILES:=$(PKG_BUILD_DIR)/$(NSS_CRYPTO_DIR)/src/qca-nss-crypto.ko \
	$(PKG_BUILD_DIR)/$(NSS_CRYPTO_DIR)/tool/qca-nss-crypto-tool.ko
  AUTOLOAD:=$(call AutoLoad,52,qca-nss-crypto)
endef

define KernelPackage/qca-nss-crypto/Description
This package contains a NSS crypto driver for QCA chipset
endef

define Build/InstallDev/qca-nss-crypto
	$(INSTALL_DIR) $(1)/usr/include/qca-nss-crypto
	$(CP) $(PKG_BUILD_DIR)/$(NSS_CRYPTO_DIR)/include/* $(1)/usr/include/qca-nss-crypto
endef

define Build/InstallDev
	$(call Build/InstallDev/qca-nss-crypto,$(1))
endef

EXTRA_CFLAGS+= \
	-DCONFIG_NSS_DEBUG_LEVEL=4 \
	-I$(STAGING_DIR)/usr/include/qca-nss-crypto \
	-I$(STAGING_DIR)/usr/include/qca-nss-drv \
	-I$(PKG_BUILD_DIR)/$(NSS_CRYPTO_DIR)/include \
	-I$(PKG_BUILD_DIR)/$(NSS_CRYPTO_DIR)/src

define Build/Compile
	$(KERNEL_MAKE) -C "$(LINUX_DIR)" \
		CC="$(TARGET_CC)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		M="$(PKG_BUILD_DIR)" \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		NSS_CRYPTO_DIR=$(NSS_CRYPTO_DIR) \
		SoC="$(CONFIG_TARGET_BOARD)" \
		modules
endef

$(eval $(call KernelPackage,qca-nss-crypto))
