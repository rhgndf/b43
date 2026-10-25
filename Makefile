# SPDX-License-Identifier: GPL-2.0

ifneq ($(KERNELRELEASE),)

ifneq ($(CONFIG_B43),m)
$(error Target kernel must be configured with CONFIG_B43=m)
endif

b43-y				+= main.o
b43-y				+= bus.o
b43-$(CONFIG_B43_PHY_G)		+= phy_g.o tables.o lo.o wa.o
b43-$(CONFIG_B43_PHY_N)		+= tables_nphy.o
b43-$(CONFIG_B43_PHY_N)		+= radio_2055.o
b43-$(CONFIG_B43_PHY_N)		+= radio_2056.o
b43-$(CONFIG_B43_PHY_N)		+= radio_2057.o
b43-y				+= phy_common.o
b43-$(CONFIG_B43_PHY_N)		+= phy_n.o
b43-$(CONFIG_B43_PHY_LP)	+= phy_lp.o
b43-$(CONFIG_B43_PHY_LP)	+= tables_lpphy.o
b43-$(CONFIG_B43_PHY_HT)	+= phy_ht.o
b43-$(CONFIG_B43_PHY_HT)	+= tables_phy_ht.o
b43-$(CONFIG_B43_PHY_HT)	+= radio_2059.o
b43-$(CONFIG_B43_PHY_LCN)	+= phy_lcn.o tables_phy_lcn.o
b43-$(CONFIG_B43_PHY_AC)	+= phy_ac.o
b43-y				+= sysfs.o
b43-y				+= xmit.o
b43-y				+= dma.o
b43-y				+= pio.o
b43-y				+= rfkill.o
b43-y				+= ppr.o
b43-$(CONFIG_B43_LEDS)		+= leds.o
b43-$(CONFIG_B43_SDIO)		+= sdio.o
b43-$(CONFIG_B43_DEBUG)		+= debugfs.o

obj-m				+= b43.o

else

KDIR ?= /lib/modules/$(shell uname -r)/build
MODULE_DIR := $(CURDIR)

.PHONY: all modules modules_install clean
all: modules

modules modules_install clean:
	$(MAKE) -C "$(KDIR)" M="$(MODULE_DIR)" $@

endif
