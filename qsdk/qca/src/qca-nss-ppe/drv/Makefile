# Makefile for the PPE drivers

ccflags-y := -I$(obj) -I$(obj)/.. -I$(obj)/../..

ifneq ($(ppe-drv),)
obj-m += ppe_drv/
endif

ifneq ($(ppe-qdisc),)
obj-m += ppe_qdisc/
endif

ifneq ($(ppe-vp),)
obj-m += ppe_vp/
endif

ifneq ($(ppe-tun),)
obj-m += ppe_tun/
endif

ifneq ($(ppe-ds),)
obj-m += ppe_ds/
endif

ifneq ($(ppe-rule),)
obj-m += ppe_rule/
endif

obj ?= .
