#
# Copyright (C) 2008-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
# mkimage -A mips -O linux -T kernel -C gzip -a 0x80000000 -e 0x80000000 -n "NiceTry" -d vmlinux_orig.gz vmlinux.bix
include $(TOPDIR)/rules.mk

ARCH:=mips
CPU_TYPE:=4kec
BOARD:=rtl838x
BOARDNAME:=Realtek MIPS
FEATURES:=squashfs

KERNEL_PATCHVER:=4.19

define Target/Description
	Build firmware images for Realtek RTL838x  based boards.
endef

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += swconfig uboot-envtools

$(eval $(call BuildTarget))
