# Makefile for PPE RULE module

ccflags-y := -I$(obj)/../exports -I$(obj)/..
ccflags-y += -DPPE_DRV_BUILD_ID="$(BUILD_ID)"
ccflags-y += -Wall -Werror

ifeq ($(PPE_RULE_IPQ53XX), y)
ccflags-y += -DNSS_PPE_RULE_IPQ53XX
endif

obj-m += qca-nss-ppe-rule.o
qca-nss-ppe-rule-objs := ppe_rule.o

ifeq ($(PPE_RFS_ENABLED), y)
qca-nss-ppe-rule-objs += ppe_rfs/ppe_rfs.o
qca-nss-ppe-rule-objs += ppe_rfs/ppe_rfs_stats.o
ccflags-y += -DPPE_RFS_DEBUG_LEVEL=3
endif

ifeq ($(PPE_POLICER_ENABLED), y)
qca-nss-ppe-rule-objs += ppe_policer/ppe_policer.o
qca-nss-ppe-rule-objs += ppe_policer/ppe_policer_stats.o
ccflags-y += -DPPE_POLICER_DEBUG_LEVEL=3
endif

ifeq ($(PPE_ACL_ENABLED), y)
qca-nss-ppe-rule-objs += ppe_acl/ppe_acl.o
qca-nss-ppe-rule-objs += ppe_acl/ppe_acl_stats.o
ccflags-y += -DPPE_ACL_DEBUG_LEVEL=3
endif

ifeq ($(PPE_MIRROR_ENABLED), y)
qca-nss-ppe-rule-objs += ppe_mirror/ppe_mirror.o
qca-nss-ppe-rule-objs += ppe_mirror/ppe_mirror_stats.o
ccflags-y += -DPPE_MIRROR_DEBUG_LEVEL=3
endif
